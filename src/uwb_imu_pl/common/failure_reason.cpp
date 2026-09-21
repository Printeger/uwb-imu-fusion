#include "uwb_imu_pl/common/failure_reason.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>

namespace uwb_imu_pl {
namespace {

std::string lower(const std::string& text) {
  std::string out = text;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return out;
}

bool contains(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}

}  // namespace

const char* toString(FailureReason value) {
  switch (value) {
    case FailureReason::None: return "NONE";
    case FailureReason::StateOutputUnobservable: return "STATE_OUTPUT_UNOBSERVABLE";
    case FailureReason::DangerousFaultNullspace: return "DANGEROUS_FAULT_NULLSPACE";
    case FailureReason::NumericalCertificateUnresolved:
      return "NUMERICAL_CERTIFICATE_UNRESOLVED";
    case FailureReason::LinearizationUncertified: return "LINEARIZATION_UNCERTIFIED";
    case FailureReason::HistorySummaryInvalid: return "HISTORY_SUMMARY_INVALID";
    case FailureReason::HistoryCapacityExceeded: return "HISTORY_CAPACITY_EXCEEDED";
    case FailureReason::RiskBudgetInfeasible: return "RISK_BUDGET_INFEASIBLE";
    case FailureReason::FaultModelUnsupported: return "FAULT_MODEL_UNSUPPORTED";
    case FailureReason::PostFdeUncertified: return "POST_FDE_UNCERTIFIED";
    case FailureReason::OutputIdentityMismatch: return "OUTPUT_IDENTITY_MISMATCH";
    case FailureReason::DeadlineMissed: return "DEADLINE_MISSED";
    case FailureReason::ResourceLimitExceeded: return "RESOURCE_LIMIT_EXCEEDED";
    case FailureReason::UnknownText: return "UNKNOWN_TEXT";
  }
  return "UNKNOWN_TEXT";
}

std::vector<FailureReason> allFailureReasons() {
  return {
      FailureReason::StateOutputUnobservable,
      FailureReason::DangerousFaultNullspace,
      FailureReason::NumericalCertificateUnresolved,
      FailureReason::LinearizationUncertified,
      FailureReason::HistorySummaryInvalid,
      FailureReason::HistoryCapacityExceeded,
      FailureReason::RiskBudgetInfeasible,
      FailureReason::FaultModelUnsupported,
      FailureReason::PostFdeUncertified,
      FailureReason::OutputIdentityMismatch,
      FailureReason::DeadlineMissed,
      FailureReason::ResourceLimitExceeded,
  };
}

std::string failureReasonCatalogJson() {
  std::ostringstream out;
  out << '{';
  bool first = true;
  for (const auto reason : allFailureReasons()) {
    if (!first) out << ", ";
    first = false;
    out << '"' << toString(reason) << '"'
        << ": {\"check_implemented\": "
        << (reason == FailureReason::LinearizationUncertified ||
                    reason == FailureReason::DeadlineMissed ||
                    reason == FailureReason::ResourceLimitExceeded
                ? "false" : "true")
        << '}';
  }
  out << '}';
  return out.str();
}

FailureReason classifyFailureReason(const std::string& text) {
  if (text.empty()) return FailureReason::None;
  const std::string value = lower(text);
  if (contains(value, "deadline")) return FailureReason::DeadlineMissed;
  if (contains(value, "resource limit") || contains(value, "capacity"))
    return FailureReason::ResourceLimitExceeded;
  if (contains(value, "history_summary") || contains(value, "history summary"))
    return FailureReason::HistorySummaryInvalid;
  if (contains(value, "historyprovenance") || contains(value, "history provenance") ||
      contains(value, "history prior contaminated") ||
      contains(value, "historical") || contains(value, "maturity"))
    return FailureReason::HistorySummaryInvalid;
  if (contains(value, "risk budget") || contains(value, "risk_budget"))
    return FailureReason::RiskBudgetInfeasible;
  if (contains(value, "unsupported") || contains(value, "fault_model") ||
      contains(value, "fault model"))
    return FailureReason::FaultModelUnsupported;
  if (contains(value, "identity")) return FailureReason::OutputIdentityMismatch;
  if (contains(value, "rank deficient") || contains(value, "rank loss") ||
      contains(value, "unmonitorable") || contains(value, "state is rank") ||
      contains(value, "no residual dof") ||
      contains(value, "rank/condition gate") ||
      contains(value, "rank and condition"))
    return FailureReason::DangerousFaultNullspace;
  if (contains(value, "condition")) return FailureReason::NumericalCertificateUnresolved;
  if (contains(value, "certificate") || contains(value, "spd") ||
      contains(value, "solve residual"))
    return FailureReason::NumericalCertificateUnresolved;
  if (contains(value, "step gate") || contains(value, "linearization step") ||
      contains(value, "linearization"))
    return FailureReason::LinearizationUncertified;
  if (contains(value, "post-fde") || contains(value, "post fde"))
    return FailureReason::PostFdeUncertified;
  if (contains(value, "observable") || contains(value, "monitorable output"))
    return FailureReason::StateOutputUnobservable;
  return FailureReason::UnknownText;
}

std::string joinFailureReasons(const std::vector<FailureReason>& reasons) {
  std::set<FailureReason> unique(reasons.begin(), reasons.end());
  unique.erase(FailureReason::None);
  std::ostringstream out;
  bool first = true;
  for (const auto reason : unique) {
    if (!first) out << ';';
    first = false;
    out << toString(reason);
  }
  return out.str();
}

}  // namespace uwb_imu_pl
