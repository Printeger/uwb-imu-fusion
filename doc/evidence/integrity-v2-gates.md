# Integrity V2 gate evidence index

This file is an implementation evidence index, not a certification claim.

| Gate | Implementation evidence | Verification status |
|---|---|---|
| A | baseline commit, scope ADR, handbook | complete |
| B | `EpochTransaction`, atomic commit/discard receipts, backend slot audit | Release/Debug unit and regression tests pass |
| C | factor ledger, trusted-prefix 20-interval window, provenance gates | Release/Debug unit and dense-oracle tests pass |
| D | one-time factorization, Woodbury evaluator, dense oracle | Release/Debug unit tests pass |
| E | joint squared-parity detector, physical-anchor hypotheses, union actions | unit tests pass; boundary-sweep MC pending |
| F | six interval-constant IMU axes, raw-sample central finite difference | unit tests and schedule smoke pass; MC pending |
| G | 9D CV/constant-attitude bridge, bias continuity, deterministic box | unit tests pass; calibration pending |
| H | post-action detector and PL, full plausible-set coverage | unit tests pass; MC coverage verification pending |
| I | persistent source health, quarantine/recovery counters, bridge timeout | unit/scenario tests pass; active-window atomic replacement remains fail-closed |
| J | calibrated priors/noise/bridge evidence and independent review | not started |

## Reproducible verification snapshot

- Release: 17/17 CTest suites; 162 tests, 0 errors, 0 failures, 0 skipped.
- Debug: 17/17 CTest suites; 162 tests, 0 errors, 0 failures, 0 skipped.
- Python syntax checks pass and all seven IMU fault-schedule smoke cases load.
- The pre-existing simulator-library symlink/hash warnings remain unchanged.

The full Monte Carlo boundary sweep, calibrated 20 Hz timing inventory, and
independent review are not complete. Machine-readable manifests therefore keep
`gates_a_to_i_complete=false`, `gate_j_complete=false`, and
`formal_eligible=false`. Runtime status remains `IMPLEMENTED_UNVERIFIED`.
