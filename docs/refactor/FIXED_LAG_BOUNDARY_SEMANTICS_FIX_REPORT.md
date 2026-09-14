# Fixed-Lag Boundary Semantics Fix Report

Date: 2026-09-14  
Branch: `feature/uwb-imu-fusion-ie-postprocessing`  
Final verdict: `IE_CORE_STABILIZATION_FAILED`

## Outcome

The narrow boundary semantics fix is implemented and its initializer tests pass, but the required runner regression gate
does not. After the affected runner was fully relinked, T04, T06, and T08 no longer fail on state cadence. Their historical
sim-circle subcase instead fails its frozen graph-linearization identity check. Per the task instruction, test expectations
were not changed and execution stopped at the first genuine new regression. Walk1, Gate06, fallback, and GT evaluation
were not run.

## Corrected semantics

At frontier `k`, the initializer now finds the maximal suffix of recorded states satisfying
`t_k - t_i <= 0.25 s`. State zero remains the immutable initial anchor. The immediate predecessor of the operational
free suffix is included as the fixed boundary. Its age is not constrained by the lag, and the physical CombinedImuFactor
from that boundary to the first free state is retained even when its duration exceeds 0.25 s.

Consequently:

- dense cadence produces a multi-state free suffix;
- 1 Hz cadence with a 0.25 s lag produces one free current state and the previous state as fixed boundary;
- lag controls reoptimization membership, not dataset sampling cadence;
- `STATE_CADENCE_EXCEEDS_FIXED_LAG_HORIZON` is no longer a failure path;
- non-increasing timestamps, missing IMU factors, nonpositive/nonfinite PIM duration, invalid keys/Values, and numerical
  seed-quality failures remain fail-closed.

The identity was versioned to `PAPER_BOUNDED_FIXED_LAG_CAUSAL_INITIALIZER_V4` with
`boundary=PREDECESSOR_OF_MAXIMAL_FREE_SUFFIX_FIXED_V2`. Diagnostic artifacts now distinguish the total local graph span,
the constrained free-window span, and the possibly longer boundary-bridge duration.

## Frozen behavior

No lag value, state cadence, fixture data, Huber scale, sensor sigma, LM setting, continuation count,
`INITIALIZATION_SEED_QUALITY_V1`, Solver Certificate, final estimator, physical graph, or IE method mathematics was
changed. The physical graph is still passed as const and its factors are not replaced by the initializer.

## Required initializer tests

The rebuilt `test_common_initializer` ran 13 tests: 13 passed, 0 failed.

Coverage includes:

1. cadence greater than lag is accepted rather than rejected solely for cadence;
2. 1 s cadence yields previous fixed boundary, current free state, one physical IMU bridge, zero free span, and a 1 s
   bridge span;
3. 0.1 s dense cadence with a 0.25 s lag creates a multi-state free suffix;
4. states that have exited the lag remain immutable;
5. future UWB measurements do not enter an earlier frontier;
6. the physical IMU bridge from an older boundary is present;
7. missing CombinedImuFactor and reset/zero-duration PIM both fail closed;
8. physical graph pointers/keys, ledger IDs, selected IDs, and fixed UWB sigma remain unchanged;
9. nonstationary seed acceptance remains distinct from unchanged Solver Certificate rejection.

An existing prefix-comparison assertion was corrected to compare only states already outside the lag. Its former last
state was still free when the next frontier arrived and was therefore contractually allowed to be revised. The separate
frontier-level future-factor test remains unchanged and passes.

## Build note

The first targeted runner invocation used a stale `uwb_imu_fgo_paper_runner` binary after the public initializer result
structure gained diagnostic fields, producing `SIGSEGV` during result destruction. A read-only GDB trace identified the
ABI mismatch. Relinking the affected runner removed the crash without a source change; this stale-build artifact is not
treated as the new estimator failure.

## First genuine new regression

Command:

```text
ctest --output-on-failure -R 'test_t0(4|6|8)_runner_contract'
```

Result after complete relink: 0/3 passed, exit 8. All three fail at the same historical sim-circle assertion after
initialization rather than at the former cadence check.

First failing test: `test_t04_runner_contract`.

```text
expected graph_linearization_sha256:
t08graphlin-sha256:a029fb09830efd4d17d24900cc594e25bdef7c317b5dea78602af2ccb85709e7

actual graph_linearization_sha256:
t08graphlin-sha256:cd46d08fca3a104e35fae843fbca68288dfa2e07e07039c6e6f071fa28d6cced
```

The actual preparation reports initializer V4, 7 prefixes, 7 accepted prefixes, and zero initializer failures. Thus the
requested cadence failure is removed, but the new seed changes the frozen graph-at-Values linearization identity used by
the historical runner contract. The test helper reports this as `historical factor construction changed`. T06 and T08
report the identical expected/actual hash mismatch.

Changing the frozen expected hash, redefining the identity, or forcing the new initializer to reproduce the earlier
Values would be an additional compatibility decision outside this narrow fix. None was done automatically.

## Remaining required gates

| Gate | Result |
|---|---|
| Initializer tests | PASS, 13/13 |
| T04 runner contract | FAIL, graph-linearization identity mismatch |
| T06 runner contract | FAIL, same mismatch |
| T08 runner contract | FAIL, same mismatch |
| Full core regression | `NOT_RUN_AFTER_RUNNER_IDENTITY_REGRESSION` |
| Gate05 architecture guard | `NOT_RUN_AFTER_RUNNER_IDENTITY_REGRESSION` |
| Full CTest | `NOT_RUN_AFTER_RUNNER_IDENTITY_REGRESSION` |

## Scientific execution

- Locked 913-state Walk1 initialization: `NOT_RUN`.
- Full seed-quality gate: `NOT_RUN`.
- Unchanged-Cauchy Gate06: `NOT_RUN`.
- Huber 1.345 to unchanged-Cauchy fallback: `NOT_RUN`.
- Solver Certificate / trajectory export: `NOT_RUN`; no trajectory exported.
- GT evaluation and ATE/P50/P95: `NOT_RUN`; GT was not read.
- IE scientific experiments: `NOT_RUN`.

## Files changed by this task

- `include/uifgo/common_initializer.h`
- `src/common_initializer.cpp`
- `test/test_common_initializer.cpp`
- `tools/run_all_uwb_optimization_diagnosis.cpp`
- `doc/ie_sprint/METHOD_CONTRACT.md`
- `doc/ie_sprint/EXPERIMENT_CONTRACT.md`
- `doc/ie_sprint/STATUS.md`
- `paper/CLAIM_EVIDENCE.md`
- `docs/refactor/FIXED_LAG_BOUNDARY_SEMANTICS_FIX_REPORT.md`

The repository already contained extensive uncommitted work; unrelated changes and historical evidence were preserved.
No commit or push was performed.

`IE_CORE_STABILIZATION_FAILED`
