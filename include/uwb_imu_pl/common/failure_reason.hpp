#pragma once

#include <string>
#include <vector>

namespace uwb_imu_pl {

// Frozen failure taxonomy (roadmap section 4.5).  These codes are additive:
// every code exists from this point on, even when the producing check is not
// implemented yet, so that reports can mark a check NOT_EVALUATED instead of
// silently claiming success.
enum class FailureReason {
  None = 0,
  StateOutputUnobservable,
  DangerousFaultNullspace,
  NumericalCertificateUnresolved,
  LinearizationUncertified,
  HistorySummaryInvalid,
  HistoryCapacityExceeded,
  RiskBudgetInfeasible,
  FaultModelUnsupported,
  PostFdeUncertified,
  OutputIdentityMismatch,
  DeadlineMissed,
  ResourceLimitExceeded,
  UnknownText
};

const char* toString(FailureReason value);

// Maps existing human-readable reason text (detector/window/candidate/FDE/PL
// strings) onto the frozen taxonomy.  Unmapped text returns UnknownText rather
// than silently passing.
FailureReason classifyFailureReason(const std::string& text);

std::vector<FailureReason> allFailureReasons();

// Stable, machine-readable catalog used by run_manifest.json and
// validation-manifest.json.
std::string failureReasonCatalogJson();

// ";"-joined helpers for diagnostics export.
std::string joinFailureReasons(const std::vector<FailureReason>& reasons);

}  // namespace uwb_imu_pl
