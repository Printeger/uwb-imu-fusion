# REFACTOR-GATE-06R-A common initialization repair report

Date: 2026-09-14  
Scope: initialization only  
Locked recording: SFUISE Walk1, `state_step=1`, all estimator-usable correlation representatives  
Truth boundary: GT, ATE, truth and oracle were not loaded

## Outcome

The full-recording open-loop IMU seed has been replaced in the common paper
preparation path by a causal, UWB-aided progressive initializer. The code and
its deterministic engineering tests pass, and a failed prefix cannot return
the remaining open-loop trajectory as valid Initial Values.

The locked Walk1 audit does not complete, however. Prefixes 1--50 are accepted;
the solve at frontier state 51 is rejected after the one permitted same-graph,
same-parameter continuation. Its maximum scaled navigation gradient is
`1.0741484355758502e-05`, above the frozen initialization qualification
`1e-5`. The initializer therefore returns `INITIALIZATION_FAILED` with an empty
result `Values`. No full-sequence post-repair state or residual distribution
exists, and the Gate06 final estimator remains blocked.

## Old and new flows

The old flow was:

1. `Initializer::Run` produced a valid first pose, velocity and IMU bias using
   only sensor data.
2. `GraphBuilder::Build` predicted all remaining 912 states open loop through
   the recording's PIM/Combined IMU chain.
3. Those 913 states were directly passed to the production batch solver.

On Walk1 that seed reached a position-norm median/max of approximately
`162.809/1188.033 m`, a velocity-norm median/max of
`15.370/63.037 m/s`, median absolute UWB residual `162.397 m`, median
standardized residual `1082.645`, and median final-Cauchy weight `4.853e-6`.

The new flow is:

1. Preserve the existing valid first-state initialization.
2. Build the unchanged final physical graph and its open-loop values only as
   physical factor/PIM input to the initializer.
3. Advance exactly one recorded state at a time. Bind the already accepted
   previous `X/V/B` as a constant, predict the new state with the physical
   `CombinedImuFactor` PIM, and form an initialization-only local graph.
4. Add only physical UWB factors attached to the current state, cloned with an
   initialization-only Huber wrapper.
5. Accept and commit the new state only after finite-state, graph/key,
   objective and navigation-stationarity qualification.
6. Return a full common `X/V/B Values` only if every frontier is accepted.
   Otherwise return no values and stop.

This is not an EKF, iSAM2, spline, continuous-time or independent range
estimator.

## Implementation surface

The Gate06R-A implementation changes are confined to:

- `include/uifgo/common_initializer.h` and `src/common_initializer.cpp`:
  `CommonInitializer::Run`, its prefix/result records, causal local factor
  construction and fail-closed return.
- `include/uifgo/estimator_core.h` and `src/estimator_core.cpp`:
  call the common initializer after physical graph construction and replace
  `initial_values` only after full success.
- `include/uifgo/config.h`, `src/config.cpp`, `src/logger.cpp` and
  `test/test_config.cpp`: the sole new engineering parameter,
  `initialization.progression_horizon_s`.
- `include/uifgo/nlos_solver_utils.h` and `src/nlos_solver_utils.cpp`:
  `RunCheckedConditionalLmRetainingTerminalForInitialization`, an additive
  initialization-only entry that reuses the existing checked LM implementation
  while retaining the last accepted terminal for explicit qualification. It
  does not alter the production solver policy.
- `tools/paper/run_ie_app.cpp`: thin common-preparation identity/statistics
  wiring only.
- `tools/run_all_uwb_optimization_diagnosis.cpp`: initialization-only,
  truth-free audit mode and failure artifact output.
- `test/test_common_initializer.cpp`,
  `test/test_refactor_gate05_architecture.py`, and `CMakeLists.txt`: focused
  tests, architecture guard and build registration.
- `doc/ie_sprint/METHOD_CONTRACT.md` and
  `doc/ie_sprint/EXPERIMENT_CONTRACT.md`: pre-implementation scope and
  implementation-period amendments.

## Causality and schedule

For frontier `k`, the local optimization contains the physical IMU interval
`k-1 -> k`, the exact accepted state `k-1` as a fixed boundary, and UWB factors
whose state timestamp is the frontier timestamp. No factor associated with a
later state is included. Committed states are never reopened. Consequently a
measurement after frontier `k` cannot change the accepted output at `k`; the
deterministic future-independence test checks this directly.

The cadence is exactly one state per frontier. The sole new parameter
`initialization.progression_horizon_s=0.25 s` is a fail-closed upper bound on
the adjacent timestamp gap. Frontier placement does not depend on observation
validity, selected-count, sigma, residual, NLOS status, GT or ATE.

The initially registered `1.0 s` bound caused the deterministic synthetic drift
fixture's second local block to exhaust the existing 100-call budget. Before
any Walk1, GT or ATE audit, it was narrowed once to `0.25 s`; there was no
parameter sweep and it was not changed in response to Walk1.

## Initialization robust model and qualification

Local UWB factors retain their physical Gaussian sensor noise and receive the
existing `PaperRobustNoise` Huber wrapper with fixed parameter `1.345`. This
wrapper exists only on cloned local initialization factors. The final baseline
continues to use its existing Cauchy `2.3849` model.

The local solve uses the existing checked LM configuration (`lm_max_iter`,
relative tolerance and absolute tolerance), existing navigation scales and
binary64 roundoff treatment. Initialization qualification uses the frozen
`1e-5 objective/normalized-coordinate` stationarity tolerance. If the first
checked-LM block ends with finite last-accepted values but is not qualified,
the initializer permits at most one continuation from those exact values on
the identical graph with identical parameters. It records that continuation
as a retry. There is no second retry, parameter relaxation or open-loop
fallback.

## Fail-closed behavior

Any invalid timeline/key/factor, excessive state gap, local graph construction
error, non-finite value/objective, key mismatch or unqualified local solve
returns `INITIALIZATION_FAILED`. On failure, `CommonInitializationResult::values`
remains empty. The error records frontier state/time, evaluated and accepted
prefix counts, failure/retry count, stationarity reason and dominant key.

## Locked Walk1 initialization audit

The target physical construction remains 913 states, 2276 UWB factors and 3191
total factors. The following table distinguishes the frozen Gate06D open-loop
input from the only valid Gate06R-A output. Since Gate06R-A rejects frontier 51
and exports no full `Values`, full-sequence post-repair metrics are unavailable
rather than computed from a failed partial iterate.

| Diagnostic | Frozen Gate06D open-loop seed | Gate06R-A result |
|---|---:|---:|
| Full returned state count | 913 | `NOT_AVAILABLE_INITIALIZATION_FAILED` |
| Position norm median / P95 / max | 162.809176 / 1015.068510 / 1188.032980 m | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Velocity norm median / P95 / max | 15.369829 / 56.251833 / 63.036904 m/s | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Consecutive displacement mean / median / P95 / max | 1.326344 / 0.930132 / 3.972095 / 7.531671 m | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Raw UWB residual mean / median / P95 / max | 299.043 / 162.397 / 1010.496 / 1184.936 m (absolute) | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Standardized absolute UWB residual median / P95 / max | 1082.644782 / 6736.636805 / 7899.571710 | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Final-Cauchy weight P05 / median / P95 | `1.253297e-7` / `4.852632e-6` / `0.609538`; 94.332% `<0.5`; 81.371% `<1e-3` | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Per-anchor residual distribution | Frozen values below | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Full initial-state finite audit | finite but physically bad open-loop seed | `NOT_AVAILABLE_NO_FULL_INITIAL_VALUES` |
| Evaluated / accepted prefixes | not applicable | 51 / 50 |
| Failure / retry count | not applicable | 1 / 1 |
| Failing frontier | not applicable | state 51, time `1664959680.3226264 s` |
| Failure reason | not applicable | `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`, `NOT_STATIONARY`, dominant `x51` |
| Maximum scaled gradient | not applicable | `1.0741484355758502e-05` (required `<=1e-5`) |
| Initializer runtime | not applicable | `0.066330588999999995 s` |

Frozen Gate06D per-anchor absolute raw residuals, shown as
count / mean / median / P95 / max metres, were:

| Anchor | Count | Mean | Median | P95 | Max |
|---:|---:|---:|---:|---:|---:|
| 7475 | 479 | 291.245961 | 150.905156 | 1001.368532 | 1181.860578 |
| 9524 | 433 | 291.457427 | 160.070967 | 981.673173 | 1152.244766 |
| 10548 | 450 | 315.467405 | 186.304332 | 1030.634474 | 1180.256103 |
| 15155 | 464 | 298.289598 | 154.185488 | 1023.422499 | 1184.935757 |
| 20276 | 450 | 298.992609 | 162.945364 | 995.737065 | 1180.139285 |

Gate06R-A has no corresponding per-anchor table because it did not return a
full common seed.

Evidence:

- `experiments/icra2027/dev/COMMON_INITIALIZATION_REPAIR_WALK1/audit_final/command.txt`
- `experiments/icra2027/dev/COMMON_INITIALIZATION_REPAIR_WALK1/audit_final/failure.json`
- Earlier implementation failures are preserved separately as
  `attempt1_failed_multistate` through `attempt8_failed_evidence_label`; none is
  treated as the final audit.

The audit command exits 1 by design because the initializer fails. It records
`gt_truth_oracle_read=false`, `final_estimator_run=false`, and
`ie_stages_run=false`. Gate06 final LM, solver certificate, trajectory export,
evaluator and ATE are all `NOT_RUN`.

## Tests and build results

- Focused target build after the final evidence-label change:
  `cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target test_common_initializer uwb_imu_fgo_all_uwb_optimization_diagnosis -j2`:
  exit 0.
- Production paper-runner target after final initialization-statistics wiring:
  `cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target uwb_imu_fgo_paper_runner -j2`:
  exit 0 (only GCC's variable-tracking-size informational retry was emitted).
- Initializer deterministic suite:
  `.../test_common_initializer`: 6/6 passed. It covers causal
  future-independence, ledger/selected-ID preservation, physical sigma
  preservation, no open-loop fallback, synthetic drift correction and final
  physical graph immutability.
- Configuration test `ConfigLoader.CommonInitializationProgressionHorizon`:
  passed as part of the core suite.
- Architecture guard:
  `python3 -B test/test_refactor_gate05_architecture.py`: exit 0.
- Full core regression:
  `catkin test uwb_imu_fgo --no-deps --summarize`: exit 0,
  502 tests, 0 errors, 0 failures, 0 skipped. Catkin reported one existing
  build-warning package, not a test failure.
- Whitespace validation: `git diff --check`: exit 0.

## Final graph and IE invariance

`CommonInitializer::Run` receives the production physical graph by const
reference. The boundary Combined IMU view and Huber UWB clones are created only
inside a local graph. The physical factor's residual, PIM, noise model and
current-state Jacobians are delegated without modification; accepted values
are the only possible output. The focused graph-immutability test records and
compares every physical factor pointer and key list before and after a run.

No raw ledger, validity, repeat grouping, observation/state association,
anchor, lever arm, fixed sensor sigma, IMU model, final factor definition,
production Cauchy setting, final LM/certificate, Stage-1, PL/CUSUM, Stage-2,
recoverability, recover/suppress or final inference formula was changed by
this repair. Because the locked initialization failed, none of those later
stages ran.

## Acceptance decision

The engineering requirements for causality, no open-loop fallback, focused
tests, core regression, architecture and scientific-algorithm isolation pass.
The required full-sequence finite initialization and the post-repair residual/
Cauchy-capture conditions do not pass because no full common Initial Values
are returned. The acceptance rule is not weakened.

`COMMON_INITIALIZATION_REPAIR_FAIL`

Narrowest remaining failure: the one-state causal local solve at Walk1
frontier state 51 remains marginally non-stationary after the sole permitted
continuation under the frozen checked-LM policy and `1e-5` qualification.
Gate06 rerun is not unblocked.
