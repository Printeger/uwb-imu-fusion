# REFACTOR-GATE-05 report

Date: 2026-09-13  
Verdict: **NOT PASSED — regression gate blocked**  
Next prompt: **DO NOT START Prompt 6**

## Scope and result

Gate 05 performed only structural extraction. It did not intentionally change
observation semantics, robust noise, support discovery, PL/CUSUM, bias refit,
recoverability, gate thresholds, recover/suppress policy, solver parameters or
certificate rules.

Implemented changes:

- replaced the 5,432-line `tools/run_ie_paper.cpp` with a five-line entry that
  delegates to `RunIePaperApplication`;
- moved application orchestration and compatibility behavior to
  `tools/paper/run_ie_app.cpp` with a small public application header;
- extracted source-neutral input-plan/initialization/graph preparation to the
  production library as `uifgo::PrepareEstimatorCore`;
- made the application consume the returned physical graph, initial `Values`,
  UWB factor indices and factor metadata;
- retained historical T04/T06/T08 strings as compatibility aliases while core
  code uses semantic provenance names;
- redirected the two existing include-based forensic binaries to the new
  application location, preserving their previous diagnostic access;
- added a structural dependency regression test.

The architecture and dependency rules are documented in
`docs/refactor/IE_CORE_V2_ARCHITECTURE.md`.

## Behavior-preservation evidence

### Deterministic core regression

The focused core set passed 7/7 before runner-contract execution: paper input,
NLOS refit, FDE, recoverability, final inference, paper methods / solver
certificate, and the Gate-05 architecture dependency guard. These tests cover
stable observation/selection semantics, factor construction,
candidate/support identities, refit values, recoverability values, final factor
decisions, objective/trajectory validity and certificate behavior on their
deterministic fixtures.

### Before/after runner snapshot

The pre-refactor runner was executed once with
`config/paper/sim_circle_t08_oracle_development.yaml`, output
`/tmp/uifgo_gate05_before.mmpDKe/gate05`. It exited 1 at Stage 2 with
`CONDITIONAL_LM_FAILED / CONDITIONAL_LM_MAX_ITERATIONS`.

The rebuilt post-refactor runner used the same input and run ID at
`/tmp/uifgo_gate05_after.D4Qou1/gate05`. It also exited 1 with the same stage,
solver status, reason, zero refit outer iterations, and no valid estimate.

The following ten artifacts are byte-identical before and after:

```text
capability_status.json
common_preparation.json
config_effective.yaml
config_original.yaml
imu_covariance_model.txt
input_manifest.json
observations.csv
oracle_support.yaml
refit_iterations.csv
stage2_refit_iterations.csv
```

`stage2_refit_status.json` is also byte-identical. `run_status.json` differs
only in `elapsed_seconds` and `stage2_seconds`, which are explicitly wall-clock
measurements. Thus the available before/after snapshot preserves observation
IDs, selected observations, state/factor counts, input/graph identities, refit
trace and failure decision. It cannot supply successful final trajectory,
recoverability or final-certificate comparison because the pre-refactor run
already failed before those stages; those paths are covered by deterministic
unit/integration fixtures.

## Commands and actual outcomes

```text
python3 -B test/test_refactor_gate05_architecture.py
  exit 0, PASS

catkin build uwb_imu_fgo --no-deps --summarize
  exit 0, package built successfully
  unrelated devel symlink hash warnings remained for three existing ROS libraries

ctest --output-on-failure -R <Gate-05 focused set>
  exit 8, 7/10 passed

ctest --output-on-failure
  exit 8, 32/35 CTest targets passed
```

The three failures are:

| Test | Failure |
|---|---|
| `test_t04_runner_contract` | Stage-2 support refit reaches `CONDITIONAL_LM_MAX_ITERATIONS` |
| `test_t06_runner_contract` | automatic discovery reaches `CONDITIONAL_LM_MAX_ITERATIONS` |
| `test_t08_runner_contract` | Stage-2 support refit reaches `CONDITIONAL_LM_MAX_ITERATIONS` |

All other 32 CTest targets passed, including the new architecture guard and all
core C++ test targets. The T08 failure was reproduced in the saved pre-refactor
snapshot, so it is not evidence of changed Gate-05 mathematics. Nevertheless,
the Prompt-5 acceptance criterion is unambiguous: all selected regressions must
pass. Altering solver tolerances, success rules or test expectations to clear
these failures would violate this gate's scientific freeze.

## Acceptance decision and stop

The source separation and one-way dependency goals are implemented, the build
passes, and the available snapshot shows behavior preservation. Gate 05 is
still **NOT PASSED** because three runner regressions fail. Prompt 6 is a dense
scientific rerun and must not start until a separately authorized diagnosis
restores those runner contracts without silently changing scientific behavior,
or the governing contract is explicitly amended.

No dense-state experiment, formal dataset matrix, sweep, tuning, commit or push
was performed.

## 2026-09-13 Gate-05R resolution (append-only)

The historical Gate-05 result above remains the record of the original gate:
structural extraction passed, while three runner contracts blocked acceptance.
The separately authorized Gate-05R-A diagnosis subsequently established that
those contracts first became stale at the earlier intentional Gate-02
`FIXED_SENSOR_SIGMA_V2` transition; Gate 05 was not causal.

Gate-05R-B restored current-semantic coverage without modifying production
code, configuration, sigma, solver parameters, thresholds, mathematics or
certificate rules. Each existing T04/T06/T08 runner CTest now includes both a
deterministic fixed-sigma success path through the real application and the
unchanged sim-circle fixed-sigma failure as a fail-closed negative regression.
The Gate-05 focused core set passed 6/6, the architecture dependency guard
passed 1/1, and final full CTest passed 35/35 with zero failures.

Resolution: `STRUCTURAL_REFACTOR_ACCEPTED_AFTER_GATE05R`.
Gate-05R verdict: `GATE05R_PASS`.
Next state: `PROMPT_6_UNBLOCKED`; Prompt 6 was not executed here.

See `REFACTOR_GATE_05R_B_REPORT.md` for fixture provenance, negative semantics,
changed files and the ordered validation record.
