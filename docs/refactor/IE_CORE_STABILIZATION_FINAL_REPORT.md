# IE Core Stabilization Final Report

Date: 2026-09-14  
Branch: `feature/uwb-imu-fusion-ie-postprocessing`  
Final verdict: `IE_CORE_STABILIZATION_FAILED`

## 1. Outcome and stop reason

The initialization and final-solver contracts were separated in code and documentation, and the focused initializer,
architecture, and catkin core suites passed. The required full CTest gate did not pass: 33 of 36 tests passed and three
runner contract tests failed because their synthetic state cadence is 1 s while the frozen fixed-lag horizon is exactly
0.25 s. Each failure was fail-closed before estimation with
`INITIALIZATION_FAILED:STATE_CADENCE_EXCEEDS_FIXED_LAG_HORIZON`.

The task explicitly requires an immediate stop on any regression or architecture failure. Consequently the locked
913-state Walk1 initialization, Gate06, the one authorized Huber-to-Cauchy fallback, and all GT evaluation were not run.
No lag, threshold, solver parameter, or test fixture was changed after the failure.

## 2. Why the former per-window stationarity contract was wrong

The former initializer reused the final scientific navigation-stationarity threshold as a per-window acceptance gate.
That conflated two different roles. A bounded causal initializer only has to produce finite, structurally consistent,
physically local and measurement-compatible Values in the final optimizer's capture basin. It is not itself a final
scientific solve. Rejecting an otherwise usable local terminal iterate solely because its scaled navigation gradient was
above `1e-5` made an engineering seed generator claim the obligations of a certified estimator and caused the known
frontier-125 stop.

Final scientific outputs still require strict stationarity. The correction therefore changes initializer acceptance,
not scientific success criteria.

## 3. Final initializer contract implemented

`INITIALIZATION_SEED_QUALITY_V1` is distinct from `PAPER_SOLVER_CERTIFICATE_V1`.

The existing formulation is retained unchanged:

- fixed lag exactly 0.25 s;
- earliest state in the window fixed and later X/V/B states jointly optimized;
- no future measurements;
- physical CombinedImu and UWB factors with the existing representatives and fixed sensor sigma;
- initialization-only Huber scale 1.345;
- existing checked-LM parameters and single continuation policy.

A local terminal seed is accepted only when graph/key/time structure is valid, X/V/B and objective are finite, the
objective is not materially worse than the entering seed beyond a binary64 allowance, the update remains local under
configured `max_range`, `v_max`, and the fixed horizon, and median UWB residual quality is not catastrophically worse
than the entering seed. Invalid support, construction errors, non-finite values/objectives, graph/key mismatch, or state
explosion still fail closed. Raw LM termination and the complete stationarity audit remain diagnostic fields. Maximum
iterations, lambda-search exhaustion, or gradient above `1e-5` alone no longer aliases seed rejection.

After a complete sequence, one truth-free audit checks finite states and UWB predictions, position/velocity tails,
consecutive displacement, raw and standardized UWB residuals, and production Cauchy weights without changing Values.
Its documented physical bounds use existing configured quantities. It rejects an almost-silent graph when at least 90%
of production UWB weights are below 0.5. This gate was implemented but was not exercised on Walk1 because the earlier
required regression gate failed.

## 4. Solver Certificate was not weakened

No Solver Certificate source, threshold, or final-solver rule was modified. The new unit pair demonstrates the separation:

- `NonstationaryWindowMayBeValidInitializationSeed` accepts finite seed-quality-qualified local Values while retaining a
  failing stationarity diagnostic;
- `SolverCertificateStillRejectsNonstationarySeed` presents the same nonstationary Values as a final estimate and obtains
  `CERTIFIED_FAILURE` from the existing certificate.

The initializer never labels such a seed as converged or certified.

## 5. Tests and commands

| Gate | Command / evidence | Result |
|---|---|---|
| Focused initializer | `test_common_initializer` | PASS, 11/11 |
| Required old-one-state versus fixed-lag regression | `CommonInitializer.AuthoritativeFrontier51FormulationRegression` | PASS; old `1.0741484355758502e-5 > 1e-5`, fixed-lag `4.9057538777930176e-6 <= 1e-5` |
| Gate05 architecture | `python3 test/test_refactor_gate05_architecture.py` | PASS, 1/1 |
| Catkin core regression | `catkin test uwb_imu_fgo --no-deps --summarize` | PASS, 512 tests, 0 errors, 0 failures, 0 skipped; exit 0 |
| Full CTest | `ctest --output-on-failure` from `build/uwb_imu_fgo` | FAIL, 33/36; exit 8 |

Full CTest failures:

| Test | Failure |
|---|---|
| `test_t04_runner_contract` | `STATE_CADENCE_EXCEEDS_FIXED_LAG_HORIZON`, frontier state 1, time 1 s |
| `test_t06_runner_contract` | same fail-closed reason |
| `test_t08_runner_contract` | same fail-closed reason |

All other 33 CTest entries passed, including the initializer and Gate05 architecture entries. This does not satisfy the
required zero-failure gate.

## 6. Locked Walk1 initialization diagnostics

`NOT_RUN` due to the mandatory regression STOP. Therefore no new 913-state completion status, window termination
histogram, stationarity distribution, state-tail distribution, residual/standardized-residual distribution, production
Cauchy-weight distribution, per-anchor distribution, finite audit, or runtime exists for this task. Historical failed
audits were not reused as if they were this run.

No GT, ATE, truth, or oracle was read.

## 7. Gate06 and fallback

- Unchanged-Cauchy Gate06: `NOT_RUN` because the prerequisite regression gate failed before the full seed audit.
- Huber 1.345 warm start to unchanged Cauchy: `NOT_RUN`; its preconditions were never reached.
- Solver Certificate and valid trajectory export: `NOT_RUN` / no trajectory exported.
- ATE RMSE, P50, P95: `NOT_RUN`; GT evaluation was never authorized by a certified export.
- `ALL_UWB_CORRECTNESS_PASS`: not issued.

## 8. Files changed by this task

- `include/uifgo/common_initializer.h`
- `src/common_initializer.cpp`
- `test/test_common_initializer.cpp`
- `tools/run_all_uwb_optimization_diagnosis.cpp`
- `doc/ie_sprint/METHOD_CONTRACT.md`
- `doc/ie_sprint/EXPERIMENT_CONTRACT.md`
- `doc/ie_sprint/STATUS.md`
- `paper/CLAIM_EVIDENCE.md`
- `docs/refactor/IE_CORE_STABILIZATION_FINAL_REPORT.md`

The worktree already contained extensive unrelated and earlier uncommitted work. Those files and historical results were
preserved; the list above is the scope of this task, not a claim that the entire worktree is otherwise clean.

## 9. Proof that IE mathematics remained unchanged

The implementation changes are confined to common-initializer seed auditing, initializer diagnostics, and their tests.
No PL/CUSUM, support discovery, bias refit, recoverability, recover/suppress, final inference, raw ledger, repeat handling,
association, anchors, lever arm, sensor/IMU noise, final robust kernel, final LM policy, or Solver Certificate mathematics
was changed. The 512-test core suite and Gate05 architecture guard passed. Since the workflow stopped at the regression
gate, no IE scientific experiment was started and no result or claim was upgraded.

## 10. Closure

The contract redesign is implemented but cannot be accepted as stabilized while required runner contracts fail. The next
work must reconcile sub-horizon synthetic state cadences with the fixed-lag initialization contract without tuning the
frozen 0.25 s Walk1 formulation or weakening fail-closed behavior.

`IE_CORE_STABILIZATION_FAILED`
