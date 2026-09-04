# Integrity V2 gate evidence index

This file is an implementation evidence index, not a certification claim.

| Gate | Implementation evidence | Verification status |
|---|---|---|
| A | baseline commit, scope ADR, handbook | complete |
| B | frozen `EpochTransaction`, prevalidated one-update commit, zero-update discard, historical remove/add receipt | first-round functional implementation and unit tests complete |
| C | enriched factor ledger, frozen slot identity/accounting, unified frozen values, 20-interval detector window plus recovery provenance catalog | first-round functional implementation and unit tests complete |
| D | deterministic stacked Woodbury remove/add, independent dense oracle, numerical gates | first-round functional implementation and unit tests complete |
| E | epoch-independent, arbitrary-onset persistent-constant and affine-ramp single-anchor modes; canonical union actions | functional chain complete; statistical boundary-sweep evidence pending |
| F | six interval-constant IMU axes for each detector-window interval, analytic/raw-sample sensitivity with central finite-difference contract | functional chain complete; statistical evidence pending |
| G | independent CV/constant-attitude prediction, analytic 9D bridge Jacobian, separate 6D bias continuity, deterministic box propagation | functional chain complete; calibration evidence pending |
| H | post-action detector/PL, physical-source cardinality and complete plausible-set action coverage | functional chain complete; Monte Carlo coverage verification pending |
| I | active-history UWB replacement and IMU bridge recovery in one commit, ledger/catalog synchronization, health, bridge timeout and controlled reinitialization | first-round active-window recovery chain complete; checkpoint/replay and marginalized-prior recovery remain unavailable by scope |
| J | calibrated priors/noise/bridge evidence and independent review | not started |

## Reproducible verification snapshot

- Baseline: commit `1c547f3`.
- Release: 17/17 CTest suites, 0 failures; all generated GoogleTest XML reports have 0 errors, 0 failures and 0 disabled tests.
- Debug: 17/17 CTest suites, 0 failures; all generated GoogleTest XML reports have 0 errors, 0 failures and 0 disabled tests.
- The Integrity V2 suite contains 15 focused tests, including compact fault-mode generation, analytic bridge Jacobians, unresolved-removal rejection, controlled reinitialization and atomic historical UWB replacement.
- Release tests were isolated from the workspace's older shared library with the Release artifact preloaded; this avoids an ABI-false-failure caused by catkin's cached underlay order.
- A 30-epoch Release smoke recorded complete per-candidate wall time at p50 77.051 ms, p95 170.314 ms, p99 177.621 ms and max 193.360 ms (`391.76 s` total, `2,132,572 KiB` peak RSS). This is an observation only, not a 40 ms acceptance result.

The full Monte Carlo boundary sweep, calibrated 20 Hz timing inventory, formal
40 ms candidate latency acceptance, and independent review are not complete.
Machine-readable manifests therefore keep
`gates_a_to_i_complete=false`, `gate_j_complete=false`, and
`formal_eligible=false`. Runtime status remains `IMPLEMENTED_UNVERIFIED`.
