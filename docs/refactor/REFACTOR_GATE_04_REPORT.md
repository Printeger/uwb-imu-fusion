# Refactor Gate 04 Report — Solver Certificate

Status: `PASS / SOLVER_CERTIFICATE_ESTABLISHED / GATE_05_READY`

Scope: Prompt 4 in `doc/v3/v3.md`. Gate 03 prerequisites were satisfied before
this work. This gate changes solver-success semantics and its audit artifacts
only. It does not alter LM damping or stopping parameters, initialization, IMU
covariance, UWB sigma, robust kernels, NLOS discovery/refit/recoverability,
final-factor mathematics, or scientific thresholds. No formal-data or
full-matrix experiment was run.

## Old and new success semantics

Previously, Base FGO accepted `CheckedLmResult::converged` as sufficient after
basic graph/key checks, set `valid=true/status=OK`, and allowed trajectory
export. A generic GTSAM small-cost-change termination could therefore be
reported as successful even when the independent navigation-stationarity audit
failed. Final IE had detailed refit/factor audits but no single authoritative
post-solve qualification at the validity/export boundary.

Gate 04 separates two records:

1. **Termination status** preserves whether and why LM/GTSAM stopped. It is
   diagnostic evidence and is never silently renamed.
2. **Solver certificate** is the fail-closed scientific-use decision. Only
   `CERTIFIED_SUCCESS` permits `valid_estimate` and trajectory/bias export;
   every failed predicate yields `CERTIFIED_FAILURE` with the first precise
   failure reason.

The shared policy is `PAPER_SOLVER_CERTIFICATE_V1`. Its request API contains no
GT, ATE, truth, oracle, trajectory-error, or reference-trajectory input.

## Authoritative shared certificate

`CertifySolverResult` in `src/nlos_solver_utils.cpp` evaluates the same final
physical graph/`Values` pair used by the output path. It requires all of:

- a successful raw optimizer termination, while retaining its exact reason;
- nonempty, finite supported states (`x`, `v`, `b`, and the existing `a`, `l`,
  `c`, `z` state families), failing closed on unknown state types;
- a finite final graph objective;
- exact graph/`Values` key equality and no null factor;
- the production caller's factor-integrity audit;
- when a state timeline is required, finite strictly increasing timestamps,
  contiguous `X(k)` existence, and equal timestamp/pose counts;
- production-solver-specific checks already owned by that path; and
- a valid and passing existing `AuditNavigationStationarity` result.

The stationarity implementation is reused, not duplicated. It uses the
existing physical navigation scales, `1e-6` objective/normalized-coordinate
tolerance, and binary64 roundoff safety factor `8`. Final IE also preserves its
existing objective-nonincreasing, scaled-step, nonnegative-KKT, final-factor,
and state-stationarity requirements as solver-specific prerequisites.

`max_position_norm_m` is finite and emitted only as a diagnostic. It is not a
threshold and cannot make a solution pass or fail. This avoids replacing the
old false-success rule with a dataset-tuned position or ATE heuristic.

## Production integration

### Base FGO

`RunPaperBaseline` applies the shared certificate to `all_range`,
`robust_huber`, `robust_cauchy`, and `fixed_rejection` after their final solve
and factor metadata audit. The runner supplies the paper keyframe timestamps.
On certificate failure, status is `CERTIFIED_FAILURE`, final `Values` are
cleared, and no trajectory or bias file is written. On success, status is
`CERTIFIED_SUCCESS`.

The runner writes `solver_certificate.json` and records raw termination plus
certificate policy/status/reason separately in `run_status.json` and the
baseline manifest.

### Final IE inference

The final engine certifies the recovery refit after the existing exact factor
audit. A failed recovery certificate triggers only the already-authorized
single suppress-all fallback. The fallback is independently certified; if it
also fails, no final graph/`Values` is published as a valid estimate. The
selected certificate is required by `InferenceResult::valid_estimate()` and is
written with the final summary and attempt diagnostics.

Final content identity is upgraded to `t08_final_content_identity_v3`, the
summary to `t08_inference_summary_v3`, and affected T09 final-audit policies to
`*_FINAL_AUDIT_V2_SOLVER_CERTIFICATE`. Historical termination-only artifacts
therefore cannot be silently reused as new certified evidence.

## Regression requirements A--D

| Requirement | Regression and result |
|---|---|
| A — good solve still succeeds | A stationary one-pose graph obtains `CERTIFIED_SUCCESS`; existing Base FGO variants and both recovery/fallback final-IE fixtures also certify. PASS. |
| B — small-change bad state fails | A graph whose state is 10 m away from a tight pose prior is presented with synthetic raw termination `CONDITIONAL_LM_CONVERGED_SMALL_OBJECTIVE_CHANGE`; the raw termination remains successful but stationarity is `NOT_STATIONARY`, so the certificate is `CERTIFIED_FAILURE`. PASS. |
| C — termination remains independent | The failure regression asserts the exact termination reason remains present alongside `CERTIFIED_FAILURE`; JSON artifacts expose both fields. PASS. |
| D — certificate does not access GT | The shared request/API has no GT/ATE/truth/oracle input and derives its decision only from final graph/`Values`, timestamps, declared factor integrity, and existing solver checks. The artifact explicitly records this boundary. PASS. |

The real-UWB 100-sigma baseline regression from Gate 03 remains passing: plain
Gaussian moves by more than 4 m while direct Cauchy stays within 0.1 m, and both
are certified only because each returned state is stationary for its own final
objective. This distinguishes a scientifically meaningful method stress case
from a numerically unfinished state; the certificate is not an accuracy judge.

## D2 artifact decision

The old D2 directory is present, but it stores trajectory/CSV/summary outputs
only. It does not store the final GTSAM graph and typed `Values` needed to run
the new certificate on the identical final solve. Re-running D2 would be a new
full-recording estimator run, not a cheap artifact replay, and is outside this
engineering-only gate. Therefore:

```text
D2_CERTIFICATE_REPLAY=NOT_RUN
reason=OLD_ARTIFACT_HAS_NO_REPLAYABLE_FINAL_GRAPH_VALUES_AND_FRESH_DENSE_RUN_IS_OUT_OF_SCOPE
```

The deterministic nonstationary unit regression supplies the required failure
coverage without fabricating dataset evidence.

## Verification

Commands and evidence:

```text
catkin build uwb_imu_fgo --no-deps --summarize
exit: 0
result: all requested packages succeeded (existing warnings only)
logs: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/build.make.204.log
      /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/build.symlink.179.log

ctest -R '(_ctest_uwb_imu_fgo_gtest_test_(paper_methods|nlos_inference)|test_t09_(runner_contract|rereview_fix))' --output-on-failure
exit: 0
result: 4/4 test executables/scripts passed
detail: test_paper_methods 9/9; test_nlos_inference 25/25

catkin run_tests uwb_imu_fgo
exit: 0
result: 488 tests, 0 errors, 0 failures, 0 skipped
result log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/test.results.040.log
test log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/test.make.044.log

python3 -m json.tool /tmp/ie0911-2653-4c4b/solver_certificate.json
python3 -m json.tool /tmp/ie0911-2653-4c4b/final_inference_summary.json
exit: 0 for both
```

The JSON example is a deterministic test artifact, not a scientific run. No
test failure was waived. No solver tuning, dependency upgrade, dense replay,
formal dataset run, accuracy comparison, complete experiment matrix, commit,
or push was performed.

## Files changed by Gate 04

- `include/uifgo/nlos_solver_utils.h`, `src/nlos_solver_utils.cpp`
- `include/uifgo/paper_methods.h`, `src/paper_methods.cpp`
- `include/uifgo/nlos_inference.h`, `src/nlos_inference.cpp`
- `src/nlos_inference_io.cpp`, `tools/run_ie_paper.cpp`
- `test/test_paper_methods.cpp`, `test/test_nlos_inference.cpp`
- `test/test_t09_runner_contract.py`, `test/test_t09_rereview_fix.py`
- `doc/ie_sprint/METHOD_CONTRACT.md`,
  `doc/ie_sprint/EXPERIMENT_CONTRACT.md`, `doc/ie_sprint/STATUS.md`
- `paper/CLAIM_EVIDENCE.md`, `docs/refactor/IE_CORE_V2_CONTRACT.md`, and this
  report

Other dirty-worktree files and existing result directories are user-owned and
were not overwritten.

## Gate decision

Gate 04 acceptance criteria are satisfied. A small-objective-change termination
with a nonstationary final state can no longer be exported as a valid
scientific `CONVERGED` result in Base FGO or final IE. Gate 05 may begin only
after a new explicit user instruction; Gate 05 has not been started here.
