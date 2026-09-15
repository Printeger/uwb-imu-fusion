# Integrity V2 gate evidence index

This file is an implementation evidence index, not a certification claim.

| Gate | Implementation evidence | Verification status |
|---|---|---|
| A | baseline commit, scope ADR, handbook | complete |
| B | frozen `EpochTransaction`, prevalidated one-update commit, zero-update discard, historical remove/add receipt | first-round functional implementation and unit tests complete |
| C | enriched factor ledger, frozen slot identity/accounting, unified frozen values, 20-interval detector window plus recovery provenance catalog | first-round functional implementation and unit tests complete |
| D | shared base covariance/logdet solve, low-rank Woodbury correction factors, residual energy/determinant identities, guarded dense oracle slow path, deterministic four-worker merge | round-two implementation complete; smoke fails the latency requirement and formal 3×12,000-epoch timing was therefore not promoted |
| E | epoch-independent, arbitrary-onset persistent-constant and affine-ramp single-anchor modes; canonical union actions | functional chain complete; statistical boundary-sweep evidence pending |
| F | six interval-constant IMU axes for each detector-window interval, analytic/raw-sample sensitivity with central finite-difference contract | functional chain complete; statistical evidence pending |
| G | independent CV/constant-attitude prediction, analytic 9D bridge Jacobian, separate 6D bias continuity, deterministic box propagation | functional chain complete; calibration evidence pending |
| H | post-action detector/PL, physical-source cardinality and complete plausible-set action coverage | functional chain complete; Monte Carlo coverage verification pending |
| I | active-history UWB replacement and IMU bridge recovery in one commit, ledger/catalog synchronization, health, bridge timeout and controlled reinitialization | first-round active-window recovery chain complete; checkpoint/replay and marginalized-prior recovery remain unavailable by scope |
| J | calibrated priors/noise/bridge evidence and independent review | not started |

## First-round checkpoint

- Baseline: checkpoint commit `61ff769` (parent implementation snapshot `1c547f3`).
- Release: 17/17 CTest suites, 0 failures; all generated GoogleTest XML reports have 0 errors, 0 failures and 0 disabled tests.
- Debug: 17/17 CTest suites, 0 failures; all generated GoogleTest XML reports have 0 errors, 0 failures and 0 disabled tests.
- The Integrity V2 suite contains 15 focused tests, including compact fault-mode generation, analytic bridge Jacobians, unresolved-removal rejection, controlled reinitialization and atomic historical UWB replacement.
- Release tests were isolated from the workspace's older shared library with the Release artifact preloaded; this avoids an ABI-false-failure caused by catkin's cached underlay order.
- A 30-epoch Release smoke recorded complete per-candidate wall time at p50 77.051 ms, p95 170.314 ms, p99 177.621 ms and max 193.360 ms (`391.76 s` total, `2,132,572 KiB` peak RSS). This is an observation only, not a 40 ms acceptance result.

Before checkpointing, both clean Debug and clean Release were rerun and each
reported `172 tests, 0 errors, 0 failures, 0 skipped`. The Release XML inventory
contained 16 files and no error, failure, disabled or skipped test.

## Round-two protocol implementation

The frozen protocol is `config/integrity_round2_protocol.json`. Its SHA-256,
the code SHA and canonical cells determine the campaign directory. The runner
supports `prepare`, `build`, `calibrate`, `pilot`, `run`, `analyze`, `finalize`
and `status`; formal `build`/`run` reject a dirty checkout. Raw outcomes are
resumable by canonical cell/domain/ordinal/seed, use no more than four process
workers, and force common numeric libraries to one thread.

The C++ scenario runner generates 200 Hz IMU and 20 Hz UWB from motion truth,
adds noise/faults before the unchanged production preintegration path, and
runs both full-history and fixed-lag-200 configurations. A nominal
counterfactual with the same counter-RNG stream supplies the frozen directional
fault Gram used for boundary scaling. Python analysis uses exact
Clopper–Pearson intervals and refuses to turn missing cells, invalid checksums,
dirty SHA or replay mismatches into a scientific `FAIL`.

Typical smoke preparation (never formally eligible):

```bash
python3 tools/run_integrity_round2.py prepare --limit-cells 1
python3 tools/run_integrity_round2.py build
python3 tools/run_integrity_round2.py calibrate
python3 tools/run_integrity_round2.py pilot --limit-cells 1
python3 tools/run_integrity_round2.py status
```

Omit `--limit-cells` only on the final committed clean SHA. Run every stage in
the registered order; `finalize` keeps `formal_eligible=false` and
`gate_j_complete=false` even if Gates A–I pass.

The full Monte Carlo boundary sweep, calibrated 20 Hz timing inventory, formal
40 ms candidate latency acceptance, and independent review are not complete.
Machine-readable manifests therefore keep
`gates_a_to_i_complete=false`, `gate_j_complete=false`, and
`formal_eligible=false`. Runtime status remains `IMPLEMENTED_UNVERIFIED`.

## Round-two pre-commit verification

- Release: `178 tests, 0 errors, 0 failures, 0 skipped` after the shared-base
  log-determinant correction.
- The preregistered analytic engine completed 24,000,000 H0 and 64,000,000
  noncentral trials over 72 cells; its global parametric-bootstrap p-value was
  0.9528047195 with zero Holm rejections. This was a pre-commit implementation
  verification run, not formal evidence for the final SHA.
- The fixed-lag-200 Gate D smoke produced all 128 candidates in all 40 measured
  epochs. Four-worker full-epoch latency was p50 1015.75 ms, p95 1636.45 ms,
  p99 1705.42 ms and max 1743.52 ms; the one-worker p99 was 2221.89 ms. The
  machine result is correctly `FAIL` against the 40 ms requirement. The raw
  smoke artifacts stay outside the repository.

## Round-three closure implementation

Round three keeps the numerical thresholds, sample counts and seed domains from
round two.  The frozen extension is
`config/integrity_round3_protocol.json`; `tools/run_integrity_round3.py` refuses
to start the formal matrix while `formal_matrix_authorized=false`.

The pilot/formal execution path writes exactly four deterministic JSONL task
shards and feeds each shard to one long-lived C++ `--bulk` worker.  Every output
row carries and hashes the code SHA, protocol SHA, cell, domain, ordinal, seed
and shard.  Resume accepts only an exact output prefix; malformed trailing bytes
are removed, while a complete identity/checksum mismatch invalidates the run.
The analyzer streams those shards and never materializes the 1,347,332 formal
outcomes or one future per outcome.

Gate B/C verdicts now come only from named transaction/window/ledger tests.
The shared analytic H0/noncentral campaign is labelled calibration prerequisite
and cannot produce a B/C PASS.  Gate H's machine field is produced by replaying
the same raw stream twice and comparing hypothesis census, plausible count,
selected action and selected sources; it is no longer a constant.  Gate F/I
interval faults modify all ten 200 Hz samples in each affected 20 Hz interval,
including continuous and 3-fault/2-clean schedules.

The committed gap snapshot is `doc/evidence/round3_gap_audit.json` and readiness
is `doc/evidence/round3_readiness.json`.  They deliberately retain
`IMPLEMENTED_UNVERIFIED`, `formal_eligible=false`, `gate_j_complete=false` and
`gates_a_to_i_complete=false`.
