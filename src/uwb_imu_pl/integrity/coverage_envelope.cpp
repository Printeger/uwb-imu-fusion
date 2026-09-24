#include "uwb_imu_pl/integrity/coverage_envelope.hpp"

#include "uwb_imu_pl/estimation/integrity_window_snapshot.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace uwb_imu_pl {
namespace {

bool effectiveBasisUsable(const FaultModeBasis& mode) {
  return mode.effective_basis_certified &&
      mode.effective_parameter_dimension > 0 &&
      mode.effective_parameter_basis.rows() == mode.parameter_dimension &&
      mode.effective_parameter_basis.cols() ==
          mode.effective_parameter_dimension;
}

int modeDimension(const FaultModeBasis& mode) {
  return effectiveBasisUsable(mode) ? mode.effective_parameter_dimension
                                    : mode.parameter_dimension;
}

// A mode's whitened fault map, restricted to the factor groups it touches.
// This is the same compact form the evidence path uses; the builder is a
// consumer of the window and never modifies it.
struct CompactMap {
  struct Span {
    Eigen::Index row_offset = 0;
    Eigen::Index rows = 0;
    Eigen::Index block_offset = 0;
    std::size_t piece = 0;
  };
  std::vector<Span> spans;  // ascending row_offset
  Eigen::MatrixXd block;    // Sum(rows) x dimension, whitened
  bool valid = false;
  std::string reason;
};

CompactMap compactMapOf(const LinearizedIntegrityWindow& window,
                        const FaultModeBasis& mode) {
  CompactMap map;
  std::map<std::uint64_t, Eigen::Index> block_index;
  std::vector<Eigen::Index> block_offsets(window.blocks.size(), 0);
  Eigen::Index row_offset = 0;
  for (std::size_t index = 0; index < window.blocks.size(); ++index) {
    block_index.emplace(window.blocks[index].group_id.value(), index);
    block_offsets[index] = row_offset;
    row_offset += window.blocks[index].residual_whitened.size();
  }
  const int dimension = modeDimension(mode);
  if (dimension <= 0) {
    map.reason = "degenerate fault mode dimension";
    return map;
  }
  std::vector<Eigen::MatrixXd> pieces;
  for (const auto& item : mode.raw_group_maps) {
    const auto found = block_index.find(item.first.value());
    if (found == block_index.end()) {
      map.reason = "mode references an unknown factor group";
      return map;
    }
    const auto& block = window.blocks[found->second];
    if (item.second.rows() != block.residual_raw.size() ||
        item.second.cols() != mode.parameter_dimension) {
      map.reason = "mode map does not match the factor group shape";
      return map;
    }
    const Eigen::MatrixXd whitened = block.whitener * item.second;
    pieces.push_back(effectiveBasisUsable(mode)
                         ? whitened * mode.effective_parameter_basis
                         : whitened);
    CompactMap::Span span;
    span.row_offset = block_offsets[found->second];
    span.rows = item.second.rows();
    span.block_offset = 0;  // assigned after the sort
    span.piece = 0;
    map.spans.push_back(span);
  }
  // Keep the piece association while sorting by window row.
  std::vector<std::size_t> order(map.spans.size());
  for (std::size_t index = 0; index < order.size(); ++index) order[index] = index;
  std::sort(order.begin(), order.end(), [&](std::size_t left, std::size_t right) {
    return map.spans[left].row_offset < map.spans[right].row_offset;
  });
  std::vector<CompactMap::Span> sorted;
  sorted.reserve(map.spans.size());
  Eigen::Index compact_rows = 0;
  for (const auto index : order) {
    CompactMap::Span span = map.spans[index];
    span.block_offset = compact_rows;
    span.piece = index;
    compact_rows += span.rows;
    if (!sorted.empty() &&
        span.row_offset < sorted.back().row_offset + sorted.back().rows) {
      map.reason = "mode maps the same factor group twice";
      return map;
    }
    sorted.push_back(span);
  }
  map.spans = std::move(sorted);
  map.block.resize(compact_rows, dimension);
  for (const auto& span : map.spans) {
    map.block.middleRows(span.block_offset, span.rows) = pieces[span.piece];
  }
  map.valid = map.block.allFinite();
  if (!map.valid) map.reason = "mode map is not finite";
  return map;
}

// A_i^T A_j over the rows both modes write (the padded rows contribute zero).
Eigen::MatrixXd compactCross(const CompactMap& left, const CompactMap& right) {
  Eigen::MatrixXd value = Eigen::MatrixXd::Zero(left.block.cols(),
                                                right.block.cols());
  std::size_t left_index = 0;
  std::size_t right_index = 0;
  while (left_index < left.spans.size() && right_index < right.spans.size()) {
    const auto& left_span = left.spans[left_index];
    const auto& right_span = right.spans[right_index];
    if (left_span.row_offset + left_span.rows <= right_span.row_offset) {
      ++left_index;
      continue;
    }
    if (right_span.row_offset + right_span.rows <= left_span.row_offset) {
      ++right_index;
      continue;
    }
    const Eigen::Index begin =
        std::max(left_span.row_offset, right_span.row_offset);
    const Eigen::Index end = std::min(left_span.row_offset + left_span.rows,
                                      right_span.row_offset + right_span.rows);
    const Eigen::Index rows = end - begin;
    value += left.block
                 .middleRows(left_span.block_offset + begin - left_span.row_offset,
                             rows)
                 .transpose() *
        right.block.middleRows(right_span.block_offset + begin - right_span.row_offset,
                               rows);
    ++left_index;
    ++right_index;
  }
  return value;
}

// Independent least-squares reference for the transform T with
// A_leaf ~= A_group * T (same normal equations as the reference fixtures).
struct ModeBound {
  bool valid = false;
  double slope = std::numeric_limits<double>::infinity();
  Eigen::MatrixXd gram;
  Eigen::MatrixXd protected_response;
};

ModeBound boundOf(const LinearizedIntegrityWindow& window,
                  const CompactMap& map, const Eigen::MatrixXd& gram) {
  ModeBound bound;
  if (!window.numerics || !map.valid || gram.rows() != gram.cols()) return bound;
  // H^T A on the compact rows, then the frozen solve for C (H^T H)^-1 H^T A.
  Eigen::MatrixXd normal_cross =
      Eigen::MatrixXd::Zero(window.H.cols(), map.block.cols());
  for (const auto& span : map.spans) {
    normal_cross += window.H.middleRows(span.row_offset, span.rows).transpose() *
        map.block.middleRows(span.block_offset, span.rows);
  }
  Eigen::MatrixXd rhs(window.H.cols(), map.block.cols());
  rhs = normal_cross;
  const Eigen::MatrixXd covariance = solveFrozenInformation(
      window.square_root.get(), *window.numerics, window.base_information, rhs);
  if (covariance.rows() != window.H.cols() ||
      covariance.cols() != rhs.cols() || !covariance.allFinite()) {
    return bound;
  }
  bound.protected_response = window.protected_state_map * covariance;
  bound.gram = gram;
  const GramResponseCertificate certificate =
      certifyGramAndProtectedResponse(
          gram, bound.protected_response,
          window.numerics->numerical_contract.rank_tolerance,
          frozenWindowNumericalProofIdentity(window, *window.numerics),
          map.block.norm());
  // Coverage is a production protection claim.  Rank-zero is finite only when
  // the same exact structural G*ker certificate used by detector/evidence/PL
  // proves it harmless; any nonzero remainder is unbounded and fails closed.
  if (!certificate.valid ||
      certificate.nullspace_class == GramNullspaceClass::Dangerous) {
    return bound;
  }
  bound.slope = certificate.protected_slopes.maxCoeff();
  bound.valid = std::isfinite(bound.slope);
  return bound;
}

}  // namespace

bool envelopeDominanceAccepts(double envelope_slope, double leaf_slope,
                              double relative_tolerance) {
  if (!std::isfinite(envelope_slope) || !std::isfinite(leaf_slope)) return false;
  if (leaf_slope <= 0.0) return true;  // no monitorable leaf direction
  const double tolerance = std::isfinite(relative_tolerance)
      ? std::max(0.0, relative_tolerance) : 0.0;
  return envelope_slope >= leaf_slope * (1.0 - tolerance);
}

double envelopeDominanceMargin(double envelope_slope, double leaf_slope) {
  if (!std::isfinite(envelope_slope) || !std::isfinite(leaf_slope) ||
      leaf_slope <= 0.0) {
    return 0.0;
  }
  return envelope_slope / leaf_slope - 1.0;
}

const char* toString(CoverageLabel label) {
  switch (label) {
    case CoverageLabel::Exact:
      return "EXACT";
    case CoverageLabel::UpperEnvelope:
      return "UPPER_ENVELOPE";
    case CoverageLabel::Uncovered:
      return "UNCOVERED";
  }
  return "UNCOVERED";
}

CoverageCertificate buildExactCoverageCertificate(
    const std::vector<FaultModeBasis>& modes) {
  CoverageCertificate certificate;
  certificate.hypothesis_count = modes.size();
  certificate.exact_count = modes.size();
  certificate.complete = true;
  certificate.reason = "exact traversal of the frozen registry";
  return certificate;
}

CoverageCertificate buildGroupedCoverageCertificate(
    const LinearizedIntegrityWindow& window,
    const std::vector<FaultModeBasis>& modes,
    const std::vector<FaultModeBasis>& groups,
    const CoverageCapacity& capacity) {
  CoverageCertificate certificate;
  certificate.hypothesis_count = modes.size();
  certificate.complete = false;
  if (!window.numerics || !window.numerics->valid) {
    certificate.reason = "window numerics unavailable";
    return certificate;
  }
  if (modes.size() <= capacity.max_group_members && groups.empty()) {
    certificate.exact_count = modes.size();
    certificate.complete = true;
    certificate.reason = "no envelope groups requested; exact traversal";
    return certificate;
  }
  // Exact traversal: every mode is a leaf unless a group proves that it spans
  // it.  A mode whose map cannot be materialized is UNCOVERED and makes the
  // certificate incomplete (protected output unavailable).
  struct Leaf {
    const FaultModeBasis* mode = nullptr;
    CompactMap map;
  };
  std::vector<Leaf> leaves;
  for (const auto& mode : modes) {
    Leaf leaf;
    leaf.mode = &mode;
    leaf.map = compactMapOf(window, mode);
    if (!leaf.map.valid) {
      certificate.uncovered_modes.push_back(mode.id);
      continue;
    }
    leaves.push_back(std::move(leaf));
  }
  if (groups.empty()) {
    certificate.exact_count = leaves.size();
    certificate.uncovered_count = certificate.uncovered_modes.size();
    certificate.complete = certificate.uncovered_modes.empty();
    certificate.reason = certificate.complete
        ? "no envelope groups requested; exact traversal"
        : "coverage incomplete: protected output unavailable";
    return certificate;
  }

  std::uint64_t next_envelope_id = 1;
  std::set<std::uint64_t> enveloped;
  for (const auto& group : groups) {
    if (certificate.envelopes.size() >= capacity.max_envelopes) {
      certificate.capacity_exceeded = true;
      break;
    }
    const CompactMap group_map = compactMapOf(window, group);
    CoverageEnvelope envelope;
    envelope.id = next_envelope_id;
    envelope.group = group.id;
    envelope.dominance_margin = std::numeric_limits<double>::infinity();
    envelope.dominance_ratio = std::numeric_limits<double>::infinity();
    bool envelope_ok = group_map.valid;
    const Eigen::MatrixXd gram_group =
        envelope_ok ? compactCross(group_map, group_map) : Eigen::MatrixXd();
    std::size_t members = 0;
    for (auto& leaf : leaves) {
      if (!envelope_ok) break;
      if (leaf.mode->id.value() == group.id.value()) continue;
      if (enveloped.count(leaf.mode->id.value()) != 0) continue;
      if (members >= capacity.max_group_members) {
        certificate.capacity_exceeded = true;
        break;
      }
      if (leaf.map.block.cols() > group_map.block.cols()) continue;
      // T = argmin ||A_leaf - A_group T|| (normal equations on the compact
      // blocks); the identity is verified before the envelope may be used.
      const Eigen::MatrixXd cross = compactCross(group_map, leaf.map);
      const Eigen::MatrixXd transform =
          gram_group.ldlt().solve(cross.transpose()).transpose();
      if (!transform.allFinite()) continue;
      CoverageInclusionProof proof;
      proof.leaf = leaf.mode->id;
      proof.group = group.id;
      proof.transform = transform;
      const Eigen::MatrixXd reconstructed = [&]() {
        Eigen::MatrixXd value =
            Eigen::MatrixXd::Zero(leaf.map.block.rows(), leaf.map.block.cols());
        std::size_t group_index = 0;
        std::size_t leaf_index = 0;
        while (group_index < group_map.spans.size() &&
               leaf_index < leaf.map.spans.size()) {
          const auto& group_span = group_map.spans[group_index];
          const auto& leaf_span = leaf.map.spans[leaf_index];
          if (group_span.row_offset + group_span.rows <= leaf_span.row_offset) {
            ++group_index;
            continue;
          }
          if (leaf_span.row_offset + leaf_span.rows <= group_span.row_offset) {
            ++leaf_index;
            continue;
          }
          const Eigen::Index begin =
              std::max(group_span.row_offset, leaf_span.row_offset);
          const Eigen::Index end =
              std::min(group_span.row_offset + group_span.rows,
                       leaf_span.row_offset + leaf_span.rows);
          const Eigen::Index rows = end - begin;
          value.middleRows(leaf_span.block_offset + begin - leaf_span.row_offset,
                           rows) =
              group_map.block
                  .middleRows(group_span.block_offset + begin - group_span.row_offset,
                              rows) *
              transform;
          ++group_index;
          ++leaf_index;
        }
        return value;
      }();
      const double denominator =
          std::max(1e-300, leaf.map.block.cwiseAbs().maxCoeff());
      proof.relative_residual =
          (leaf.map.block - reconstructed).cwiseAbs().maxCoeff() / denominator;
      proof.verified = proof.relative_residual <= capacity.inclusion_tolerance;
      if (!proof.verified) {
        proof.reason = "inclusion identity A_leaf = A_group * T not satisfied";
        envelope.reason = "leaf " + std::to_string(leaf.mode->id.value()) +
            " fails the inclusion proof (relative residual " +
            std::to_string(proof.relative_residual) + ")";
        envelope.proofs.push_back(std::move(proof));
        envelope_ok = false;
        break;
      }
      // Dominance: the envelope's bound in the leaf parameterization must not
      // be below the leaf's own exact bound.
      const Eigen::MatrixXd gram_leaf = compactCross(leaf.map, leaf.map);
      const ModeBound leaf_bound = boundOf(window, leaf.map, gram_leaf);
      const Eigen::MatrixXd gram_envelope_in_leaf =
          transform.transpose() * gram_group * transform;
      const ModeBound envelope_bound =
          boundOf(window, leaf.map, gram_envelope_in_leaf);
      if (!leaf_bound.valid || !envelope_bound.valid) {
        // A zero/indeterminate envelope Gram with a nonzero protected response
        // is an under-covering envelope, not a harmless numerical omission.
        // Preserve the one-sided rejection attribution while failing closed.
        envelope.reason = leaf_bound.valid && !envelope_bound.valid
            ? "envelope bound is below the leaf exact bound (unbounded or uncertified)"
            : "dominance obligation could not be evaluated";
        envelope_ok = false;
        break;
      }
      // One-sided relative dominance: ratio >= 1 - tolerance.  The tolerance
      // is only a binary64 noise floor for the shared identity; it never
      // permits a systematic under-covering of the leaf.
      if (!envelopeDominanceAccepts(envelope_bound.slope, leaf_bound.slope,
                                    capacity.dominance_tolerance)) {
        envelope.reason = "envelope bound is below the leaf exact bound";
        envelope_ok = false;
        break;
      }
      if (leaf_bound.slope > 0.0) {
        const double margin =
            envelopeDominanceMargin(envelope_bound.slope, leaf_bound.slope);
        const double ratio = margin + 1.0;
        envelope.dominance_ratio =
            members == 0 ? ratio : std::min(envelope.dominance_ratio, ratio);
        envelope.dominance_margin =
            members == 0 ? margin : std::min(envelope.dominance_margin, margin);
      }
      envelope.proofs.push_back(std::move(proof));
      envelope.covered_modes.push_back(leaf.mode->id);
      ++members;
    }
    envelope.accepted = envelope_ok && !envelope.covered_modes.empty();
    // Every covered leaf passed the inclusion proof *and* the dominance
    // obligation (a failure breaks out of the loop above), so an accepted
    // envelope is dominant by construction.
    envelope.dominant = envelope.accepted;
    if (envelope.accepted) {
      for (const auto& covered : envelope.covered_modes) {
        enveloped.insert(covered.value());
      }
      certificate.enveloped_count += envelope.covered_modes.size();
      ++next_envelope_id;
    } else if (envelope.reason.empty()) {
      envelope.reason = envelope_ok
          ? "no leaf is provably spanned by this group"
          : "group map is unavailable";
    }
    certificate.envelopes.push_back(std::move(envelope));
  }
  for (const auto& leaf : leaves) {
    if (enveloped.count(leaf.mode->id.value()) == 0) ++certificate.exact_count;
  }
  certificate.uncovered_count = certificate.uncovered_modes.size();
  certificate.complete =
      certificate.exact_count + certificate.enveloped_count +
          certificate.uncovered_count >= modes.size() &&
      certificate.uncovered_modes.empty() && !certificate.capacity_exceeded;
  certificate.reason = certificate.complete
      ? "every hypothesis is enumerated exactly or served by a verified envelope"
      : "coverage incomplete: protected output unavailable";
  return certificate;
}

CoverageLabel coverageLabelFor(const CoverageCertificate& certificate,
                               FaultModeId mode) {
  for (const auto& envelope : certificate.envelopes) {
    if (!envelope.accepted) continue;
    for (const auto& covered : envelope.covered_modes) {
      if (covered.value() == mode.value()) {
        return CoverageLabel::UpperEnvelope;
      }
    }
  }
  for (const auto& uncovered : certificate.uncovered_modes) {
    if (uncovered.value() == mode.value()) return CoverageLabel::Uncovered;
  }
  return CoverageLabel::Exact;
}

std::uint64_t coverageEnvelopeIdFor(const CoverageCertificate& certificate,
                                    FaultModeId mode) {
  for (const auto& envelope : certificate.envelopes) {
    if (!envelope.accepted) continue;
    for (const auto& covered : envelope.covered_modes) {
      if (covered.value() == mode.value()) return envelope.id;
    }
  }
  return 0;
}

}  // namespace uwb_imu_pl
