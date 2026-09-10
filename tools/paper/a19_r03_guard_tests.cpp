#include "a18_optimizer.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void Require(bool condition, const std::string& reason) {
  if (!condition) throw std::runtime_error(reason);
}

uifgo::InexactHandoffAudit FrozenOuter15Guard() {
  uifgo::InexactHandoffAudit h;
  h.enabled = true;
  h.accepted_update_count = 2;
  h.last_accepted_generic_convergence = true;
  h.last_accepted_stationarity_valid = true;
  h.last_accepted_scaled_navigation_step = 2.3246922409272282e-8;
  h.last_accepted_scaled_navigation_gradient = 2.3495763237235678e-6;
  h.last_accepted_gradient_roundoff_allowance = 1.8541120693007066e-10;
  h.scaled_step_tolerance = 1e-6;
  h.objective_increase_allowance = 2.235406498993357e-12;
  h.last_accepted_values_identity = "frozen-r02-outer15-call2-bitwise";
  // Bounds are the archived certified endpoints.  The complete independent
  // replay checks the exact 12 rows; this executable exercises the production
  // guard with their extrema and count.
  for (size_t i = 0; i < 12; ++i) {
    uifgo::InexactHandoffTrialAudit t;
    t.trial_index = i + 1;
    t.certificate_valid = true;
    t.certificate_status = "REJECT";
    t.predicted_lo = i == 0 ? 2.3568871183e-16 : 1.9e-17;
    t.predicted_hi = i == 0 ? 2.3568871185e-16 : 2.0e-17;
    t.actual_decrease_lo = -7e-15;
    t.actual_decrease_hi = -1e-15;
    t.scaled_navigation_step =
        i == 0 ? 1.2452159737816888e-9 : 1e-10;
    h.last_lambda_search_trials.push_back(t);
  }
  return h;
}

}  // namespace

int main() {
  try {
    const std::string exhausted = "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED";
    auto valid = FrozenOuter15Guard();
    Require(r03InexactHandoffGuard(valid, exhausted, 0, true),
            "FROZEN_OUTER15_GUARD_DID_NOT_PASS");

    size_t negative = 0;
    const auto Rejects = [&](uifgo::InexactHandoffAudit h,
                             const std::string& failure =
                                 "CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED",
                             size_t unresolved = 0,
                             bool state_valid = true) {
      Require(!r03InexactHandoffGuard(h, failure, unresolved, state_valid),
              "NEGATIVE_GUARD_ACCEPTED");
      ++negative;
    };
    auto h = valid; h.accepted_update_count = 0; Rejects(h);
    h = valid; h.last_accepted_generic_convergence = false; Rejects(h);
    h = valid; h.last_accepted_scaled_navigation_step = 2e-6; Rejects(h);
    h = valid; h.last_lambda_search_trials[0].scaled_navigation_step = 2e-6;
    Rejects(h);
    h = valid; h.last_lambda_search_trials[0].predicted_hi =
        2.0 * h.objective_increase_allowance; Rejects(h);
    h = valid; h.last_lambda_search_trials[0].certificate_valid = false;
    Rejects(h);
    h = valid; h.last_lambda_search_trials[0].certificate_status =
        "NUMERIC_REDUCTION_UNRESOLVED"; Rejects(h);
    h = valid; Rejects(h, "LINEAR_SOLVE_FAILED");
    h = valid; Rejects(h, exhausted, 1);
    h = valid; Rejects(h, exhausted, 0, false);
    h = valid; h.last_lambda_search_trials.clear(); Rejects(h);
    h = valid; h.last_lambda_search_trials[0].actual_decrease_hi = 1e-20;
    Rejects(h);

    Require(negative == 12, "NEGATIVE_CASE_COUNT");
    std::cout << "{\"schema\":\"A19_R03_GUARD_TEST_V1\","
                 "\"frozen_outer15_guard\":true,"
                 "\"archived_trial_count\":12,"
                 "\"negative_cases\":" << negative
              << ",\"inner_converged\":false,"
                 "\"returned_state\":\"LAST_CERTIFICATE_ACCEPTED\"}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
