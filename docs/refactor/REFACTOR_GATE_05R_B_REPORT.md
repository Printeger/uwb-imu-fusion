# REFACTOR-GATE-05R-B fixed-sigma runner contract recovery

Date: 2026-09-13  
Verdict: **GATE05R_PASS**  
Gate-05 status: **STRUCTURAL_REFACTOR_ACCEPTED_AFTER_GATE05R**  
Next prompt: **PROMPT_6_UNBLOCKED** (not executed)

## Scope and decision

This task changed runner tests and test-only fixture generation, not production
estimator behavior. Each existing T04/T06/T08 CTest now runs both a small
deterministic `FIXED_SENSOR_SIGMA_V2` success case through the real CLI and the
unchanged historical sim-circle input as an explicit fail-closed regression.
The three existing CTest registrations remain unchanged.

The repair follows the Gate-05R-A diagnosis: the old success expectations were
made stale by the intentional Gate-02 fixed-sensor-sigma transition. Adaptive
sigma was not restored, no production threshold or solver option changed, and
`MAX_ITERATIONS` was not reclassified as success.

## Old and new runner contracts

| Runner | Old contract | Recovered contract |
|---|---|---|
| T04 | Used the historical sim-circle case for successful Stage-2 and CSV/JSON serialization; fixed sigma now fails its first conditional solve. | The analytic fixed-sigma case must converge through oracle support and preserve CSV/JSON/segment-to-observation linkage. The full historical case must remain `CONDITIONAL_LM_FAILED / CONDITIONAL_LM_MAX_ITERATIONS`, export no valid trajectory/certificate/final result, and retain its input/graph identity. |
| T06 | Used the historical sim-circle case as the automatic-discovery success/partial-success fixture. | The analytic fixed-sigma case must run the real automatic provider twice, produce byte/value-identical frozen partition identity, complete Stage 2 under all joint stop conditions, and retain all existing invalid-config, Stage-1, qualification, empty-partition and refit-failure checks. The historical case must fail in `AUTOMATIC_DISCOVERY`, leave Stage 2 unrun and publish no valid scientific result. |
| T08 | Used the historical sim-circle case for Stage2, recoverability, final inference, covariance and strict artifact checks. | The analytic fixed-sigma case must exercise the complete oracle-candidate → Stage2 → score → recover/suppress → final graph/Values path, pass strict schemas/content identity, covariance checks and `PAPER_SOLVER_CERTIFICATE_V1`. The historical case must remain a Stage-2 conditional-LM failure with scoring/final inference unrun and no success artifacts. |

## Positive fixture provenance

The shared fixture is generated at test runtime by
`test/runner_fixed_sigma_fixture.py`; no synthetic result is checked in or used
as paper evidence. It was added only after existing T07 deterministic caches
were inspected and found unsuitable as success fixtures under the current
contract.

- Origin: closed-form static tag at `[0, 0, 3]` m with the repository's eight
  `[-5/5, -5/5, 1/5]` m anchors; identity orientation, zero angular rate and
  `[0, 0, 9.81]` m/s² IMU samples.
- Determinism: no random input, GT, ATE, parameter search or fitted quantity.
  Stable observation IDs use the same source-message/source-range FNV identity
  rule as production. RSSI payloads vary deterministically by message ordinal
  only to make the production exact-repeat classification unambiguous; their
  FP/RX difference remains 3 dB.
- Input size: 401 IMU rows, five UWB messages, 40 raw/valid/planned UWB
  observations, zero stale repeats and five keyframe states.
- Sensor model: every planned UWB observation has configured
  `sigma_range = 0.1 m` and exported fixed-sigma strategy `1`
  (`FIXED_SENSOR_SIGMA_V2`).
- Graph size: 47 factors: seven non-UWB factors and 40 UWB likelihood terms.
  In the structured runs these are 36 raw-range and four segment-range factors.
- Candidate definition: link `1:1` has a closed-form `+0.4 m` range excess at
  message times 1, 2, 3 and 4 seconds. T04/T08 receive that interval through
  oracle support; T06 discovers it through the real automatic provider. The
  resulting segment owns four observations.
- T04 expected condition: `OK`, `CONVERGED`, and
  `ALL_JOINT_STOP_CONDITIONS_SATISFIED`, including the quoted/newline segment-ID
  CSV/JSON round trip.
- T06 expected condition: automatic discovery and Stage 2 complete with
  `ALL_JOINT_STOP_CONDITIONS_SATISFIED`; two independent executions have the
  same complete `partition.json` and partition hash. T06 is an intermediate
  development mode and does not claim a final scientific certificate.
- T08 expected condition: Stage 2 `CONVERGED`, final inference `OK`, valid
  covariance status and `PAPER_SOLVER_CERTIFICATE_V1 / CERTIFIED_SUCCESS` with
  termination, factor integrity and navigation stationarity passing and
  `gt_ate_truth_oracle_inputs=false`.

## Historical negative fixtures

All three negative subcases still load the original 627-observation sim-circle
window without removing hard observations. They assert 64 planned fixed-sigma
observations, eight states, input-plan identity
`sha256:cab61419...58795f67`, initial Values identity
`t08values-sha256:51a986...6bc46bd`, and physical graph-linearization identity
`t08graphlin-sha256:a029fb...85709e7`. The last fingerprint preserves the
diagnosed 74-factor/64-UWB construction.

- T04 and T08 retain `CONDITIONAL_LM_FAILED /
  CONDITIONAL_LM_MAX_ITERATIONS` before a completed refit outer iteration.
  T08 additionally requires Stage 3 and Stage 4 durations to remain absent.
- T06 retains `AUTOMATIC_DISCOVERY / CONDITIONAL_LM_FAILED /
  CONDITIONAL_LM_MAX_ITERATIONS`, with `stage2_refit_run=false`.
- Every negative case requires `status=FAILED`, no valid-estimate claim, no
  `trajectory.tum`, no successful solver certificate, and no final inference
  summary. Assertions do not depend on wall-clock timing or an incidental
  iteration count.

## Files changed by Gate-05R-B

Test and fixture files:

- `test/runner_fixed_sigma_fixture.py`
- `test/test_t04_runner_contract.py`
- `test/test_t06_runner_contract.py`
- `test/test_t08_runner_contract.py`

Governance and reports:

- `doc/ie_sprint/METHOD_CONTRACT.md`
- `doc/ie_sprint/EXPERIMENT_CONTRACT.md`
- `doc/ie_sprint/STATUS.md`
- `paper/CLAIM_EVIDENCE.md`
- `docs/refactor/REFACTOR_GATE_05R_B_REPORT.md`
- `docs/refactor/REFACTOR_GATE_05_REPORT.md` (append-only resolution)

No file under `src/`, `include/`, `tools/` or `config/` was changed by this
task. No CMake registration was changed.

## Production-behavior preservation

The historical subcases are executable before/after sentinels, not weakened
expectations. For identical `obs_id`s they require the same fixed 0.1 m sigma,
627/64 raw/planned counts, eight states, input-plan hash, initial Values hash,
physical graph-linearization hash and diagnosed failure stage/reason. The
positive subcases independently assert the same fixed-sigma materialization.

The focused tests preserve Gate-02 observation/state separation, Gate-03
integrity and robust-baseline scope, refit/FDE/recoverability/final-inference
mathematics, and Gate-04 certificate behavior. The static architecture guard
preserves the Gate-05 one-way dependency direction. Since only tests and
documentation changed, there is no production estimator delta to reconcile.

## Validation results

Commands were run from `/home/mint/ws_fusion_uwb/build/uwb_imu_fgo` in the
required order:

```text
ctest -R '^test_t04_runner_contract$' --output-on-failure
  1/1 PASS

ctest -R '^test_t06_runner_contract$' --output-on-failure
  1/1 PASS

ctest -R '^test_t08_runner_contract$' --output-on-failure
  1/1 PASS

ctest -R '^test_t0(4|6|8)_runner_contract$' --output-on-failure
  3/3 PASS

ctest -R '^_ctest_uwb_imu_fgo_gtest_test_(paper_input|nlos_refit|nlos_fde|nlos_recoverability|nlos_inference|paper_methods)$' --output-on-failure
  6/6 PASS

ctest -R '^test_refactor_gate05_architecture$' --output-on-failure
  1/1 PASS

ctest --output-on-failure
  35/35 PASS, zero failures, 36.94 s
```

The registration count remains 35. No dense-state experiment, formal dataset
matrix, sweep, tuning, Prompt 6, commit or push was performed.

## Final decision

`GATE05R_PASS`

Gate-05 is `STRUCTURAL_REFACTOR_ACCEPTED_AFTER_GATE05R` and
`PROMPT_6_UNBLOCKED`. Prompt 6 was not executed.
