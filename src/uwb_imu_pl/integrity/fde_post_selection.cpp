// C3 implementation: §8.4 profile-likelihood evidence, §8.5 guarantee groups and
// the §8.3 bridge/IMU decision order.

#include "uwb_imu_pl/integrity/fde_post_selection.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace uwb_imu_pl {

const char* toString(FaultUnitKind unit) {
  switch (unit) {
    case FaultUnitKind::UwbRangeMeters:
      return "uwb_range_m";
    case FaultUnitKind::ImuAccelMps2:
      return "imu_accel_mps2";
    case FaultUnitKind::ImuGyroRadps:
      return "imu_gyro_radps";
    case FaultUnitKind::Unknown:
      break;
  }
  return "unknown";
}

ProfileEvidence profileLikelihoodEvidence(const ProfileEvidenceInput& input) {
  ProfileEvidence out;
  const Eigen::Index dimension = input.fault_gram.rows();
  if (dimension == 0 || input.fault_gram.cols() != dimension) {
    out.reason = "fault Gram must be a non-empty square matrix";
    return out;
  }
  if (input.t.size() != dimension) {
    out.reason = "t_h does not match the fault Gram dimension";
    return out;
  }
  if (!input.fault_gram.allFinite() || !input.t.allFinite()) {
    out.reason = "profile evidence inputs must be finite";
    return out;
  }
  if (input.parameter_dim != 0 &&
      input.parameter_dim != static_cast<std::size_t>(dimension)) {
    out.reason = "declared parameter dimension does not match the Gram";
    return out;
  }
  if (!std::isfinite(input.channel_current_residual) ||
      input.channel_current_residual < 0.0 || !std::isfinite(input.kappa_b) ||
      input.kappa_b < 0.0) {
    out.reason = "residual/constant terms must be finite and non-negative";
    return out;
  }
  // Gamma_{h,full}^dagger through its small SVD (raw quantities only: no C2
  // weight/Lambda scaling may enter the evidence).
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(input.fault_gram, Eigen::ComputeThinU);
  const Eigen::VectorXd singular = svd.singularValues();
  const double largest = singular.size() > 0 ? singular(0) : 0.0;
  if (!(largest > 0.0)) {
    out.reason = "fault Gram is identically zero";
    return out;
  }
  const double floor = 1e-12 * largest;
  int kept = 0;
  for (int index = 0; index < singular.size(); ++index) {
    if (singular(index) > floor) ++kept;
  }
  out.rank = kept;
  out.condition_proxy = kept > 0 ? largest / singular(kept - 1)
                                 : std::numeric_limits<double>::infinity();
  // explained = t' Gamma^dagger t = sum_i (u_i . t)^2 / sigma_i
  double explained = 0.0;
  for (int index = 0; index < kept; ++index) {
    const double projection = svd.matrixU().col(index).dot(input.t);
    explained += projection * projection / singular(index);
  }
  out.explained_energy = explained;
  out.t_norm = input.t.norm();
  out.constant_used = input.kappa_b;  // audited raw constant
  // J = ||r_c||^2 + kappa_b - t' Gamma^dagger t.  A negative profile value is
  // impossible in exact arithmetic; clamp only at the numerical floor and record
  // nothing hidden (the caller sees the clamped value).
  const double raw = input.channel_current_residual + input.kappa_b - explained;
  out.j_profile = raw;
  out.valid = true;
  return out;
}

CandidateRanking rankStructuredCandidates(
    const std::vector<StructuredCandidate>& candidates) {
  CandidateRanking out;
  if (candidates.empty()) {
    out.reason = "candidate pool is empty";
    return out;
  }
  std::set<FaultUnitKind> units;
  std::set<std::size_t> dimensions;
  for (const auto& candidate : candidates) {
    units.insert(candidate.unit);
    dimensions.insert(candidate.parameter_dim);
    if (candidate.unit == FaultUnitKind::Unknown) {
      out.reason =
          "candidate has no declared physical unit: profiles from different "
          "units may not be compared";
      return out;
    }
    if (!std::isfinite(candidate.j_profile)) {
      out.reason = "candidate profile value is not finite";
      return out;
    }
  }
  out.mixed_units = units.size() > 1;
  out.mixed_dimensions = dimensions.size() > 1;
  if (out.mixed_units || out.mixed_dimensions) {
    out.reason =
        "candidate pool mixes physical units or parameter dimensions; ranking "
        "requires an explicit declaration";
    return out;
  }
  out.order.resize(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    out.order[index] = index;
  }
  std::stable_sort(out.order.begin(), out.order.end(),
                   [&](std::size_t left, std::size_t right) {
                     return candidates[left].j_profile <
                            candidates[right].j_profile;
                   });
  out.valid = true;
  return out;
}

GuaranteeGroupResult buildGuaranteeGroups(
    const std::vector<ActionGuarantee>& actions, double available_budget) {
  GuaranteeGroupResult out;
  if (actions.empty()) {
    out.reason = "no candidate actions to guarantee";
    return out;
  }
  if (!std::isfinite(available_budget) || available_budget <= 0.0) {
    out.reason = "selection-risk budget is not positive";
    return out;
  }
  auto triangleResidual = [](const ActionGuarantee& action) {
    const Eigen::Vector3d expected =
        action.L_reference_m +
        (action.p_action_m - action.p_reference_m).cwiseAbs();
    const double scale = std::max(
        1.0, action.L_action_m.norm() + (action.p_action_m - action.p_reference_m).norm());
    return (action.L_action_m - expected).norm() / scale;
  };
  for (const auto& action : actions) {
    if (!std::isfinite(action.epsilon_budget) || action.epsilon_budget < 0.0) {
      out.reason = "action budget must be finite and non-negative";
      return out;
    }
    if (!action.L_reference_m.allFinite() || !action.L_action_m.allFinite() ||
        !action.p_action_m.allFinite() || !action.p_reference_m.allFinite()) {
      out.reason = "guarantee vectors must be finite";
      return out;
    }
  }
  // Grouping: only actions that share ALL evidence flags, the same reference
  // certificate and satisfy the triangle-transfer identity
  //   L_{a,d} = L_{ref,d} + |p_{a,d} - p_{ref,d}|
  // may share the reference failure event.  Everything else is a singleton and
  // is charged on its own.
  const double triangle_tolerance = 1e-9;
  std::map<std::uint64_t, std::size_t> group_of_reference;
  for (const auto& action : actions) {
    const double residual = triangleResidual(action);
    const bool shareable = action.shared_reference_certificate &&
                           action.shared_accepted_event && action.shared_time &&
                           action.shared_output_quantity &&
                           action.triangle_transfer_evidence &&
                           action.reference_certificate_id != 0 &&
                           residual <= triangle_tolerance;
    if (shareable) {
      const auto found = group_of_reference.find(action.reference_certificate_id);
      if (found == group_of_reference.end()) {
        group_of_reference.emplace(action.reference_certificate_id,
                                   out.groups.size());
        GuaranteeGroup group;
        group.group_id = action.reference_certificate_id;
        group.reference_certificate_id = action.reference_certificate_id;
        out.groups.push_back(group);
        ++out.shared_groups;
      }
      auto& group =
          out.groups[group_of_reference.at(action.reference_certificate_id)];
      group.action_ids.push_back(action.action_id);
      group.charged_budget = std::max(group.charged_budget, action.epsilon_budget);
      group.worst_triangle_residual =
          std::max(group.worst_triangle_residual, residual);
    } else {
      GuaranteeGroup group;
      group.group_id = action.action_id;
      group.reference_certificate_id = action.reference_certificate_id;
      group.action_ids.push_back(action.action_id);
      group.charged_budget = action.epsilon_budget;
      group.worst_triangle_residual = residual;
      out.groups.push_back(group);
      ++out.singleton_groups;
    }
  }
  // Union budget over the groups: every group that could be published is
  // charged, not only the winner.  A group with more than one member is a
  // shared failure event; everything else is a singleton (charged on its own).
  out.shared_groups = 0;
  out.singleton_groups = 0;
  for (const auto& group : out.groups) {
    if (group.action_ids.size() > 1) ++out.shared_groups;
    else ++out.singleton_groups;
  }
  out.total_charged_budget = 0.0;
  for (const auto& group : out.groups) {
    out.total_charged_budget += group.charged_budget;
  }
  if (out.total_charged_budget > available_budget) {
    out.reason =
        "selection-risk budget does not close over the publishable action set";
    out.valid = false;
    return out;
  }
  out.valid = true;
  return out;
}

CandidateDecision decideCandidateHandling(const StructuredCandidate& candidate) {
  CandidateDecision out;
  // §8.3 order: a validated random/bounded model enters the reference estimate
  // (with the dual-channel envelope); anything usable only for the estimate
  // centre needs a valid reference and a transfer bound; otherwise the result
  // can only be an explicitly unprotected diagnostic.
  if (candidate.bounded_model) {
    out.disposition = CandidateDisposition::UseInReferenceEstimate;
    out.integrity_available = true;
    out.uses_bounded_model = true;
    out.reason =
        "validated random/bounded model: reference estimate + dual envelope";
    return out;
  }
  if (candidate.centre_only) {
    if (!candidate.has_valid_reference) {
      out.disposition = CandidateDisposition::UnprotectedDiagnosticOnly;
      out.integrity_available = false;
      out.reason =
          "centre-only candidate without a valid reference: no protection can "
          "be transferred, unprotected diagnostic only";
      return out;
    }
    if (!candidate.propagation_bound_available) {
      out.disposition = CandidateDisposition::UnprotectedDiagnosticOnly;
      out.integrity_available = false;
      out.reason =
          "centre-only candidate at a different time/quantity without a "
          "transfer bound: unprotected diagnostic only";
      return out;
    }
    out.disposition = CandidateDisposition::TransferToValidReference;
    out.integrity_available = true;
    out.reason =
        "protection transferred from a valid reference at the same instant "
        "with a documented bound";
    return out;
  }
  out.disposition = CandidateDisposition::UnprotectedDiagnosticOnly;
  out.integrity_available = false;
  out.reason = "no validated model and no usable reference: diagnostics only";
  return out;
}

}  // namespace uwb_imu_pl
