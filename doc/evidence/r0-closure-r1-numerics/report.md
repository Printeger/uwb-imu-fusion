# R0 closure and R1 numerics development report

- Date: 2026-09-16 (Asia/Shanghai)
- Status: `DEVELOPMENT_ONLY / PARTIALLY_CLOSED`
- Formal eligibility: `false`
- Gate D: **not evaluated and not declared PASS**
- Source HEAD: `e6ded3069bbc4cd5cf98f14257e96f61066685b8`
- Worktree: dirty by design; the pre-existing roadmap and `doc/evidence/r0-r1/`
  were preserved.
- Implementation content identity: SHA-256
  `2905fbc228487681d27b472c674f1f3d78e0c78574179c984817723abcdcb649`
  over the 35 modified/untracked implementation files, excluding this report,
  old evidence, results, and the user's roadmap.
- Scenario manifest identity: SHA-256
  `f556003bbe3dfc5c8f3991dc0bf5543cdddc258b4b00f42b776dbe03fcbd37e4`.
- Release runner SHA-256:
  `601a3d86cc73a546554137c51b043064ccbcceb45798055b56a5a6dea61fb0bb`.
- Final Release core library SHA-256:
  `3406574ff799700e8360626e0d073f01ffe07279a8981386831328b76177d8a1`.
- Toolchain: Release, GCC 9.4.0, Eigen 3.3.7, GTSAM 4.2a5, Linux
  6.18.33.2-microsoft-standard-WSL2.

This is development evidence, not a formal campaign. No risk, detector, alert
limit, rank/condition, 0.25 step, 20-epoch window, or 128-action limit was
relaxed.

## Outcome

| Batch | Development outcome | Status |
|---|---|---|
| A: boundary consistency | Epochs 19--22 have exclusive slot ownership; at epoch 21 the first-state UWB group is explicit and the crossing IMU group is boundary input. Default-ramp nominal raw stream produced finite PL for all 30 epochs, including after the first marginal window boundary. | PASS (development) |
| B: ramp and coverage | Window-effective ramp basis removed only certified raw-map null directions while retaining physical dimension/risk. A single raw UWB fault selected a safe union containing the correct `uwb:1` source and committed exactly once. It was not a unique UWB-only isolation. | PARTIAL |
| C: IMU/union/recovery | Calibrated accel-x injection selected generic bridge after three rejected attempts and committed once. The combined UWB+IMU injection later selected only the IMU source, so a true joint/union positive was not demonstrated. Continuous bridge timeout/recovery was not run. | PARTIAL |
| D: shared numerics | Canonical SVD/LLT context, numerical-contract fingerprint, residual/error fallback, counters, analytic/oracle split, dense candidate comparison, identity invalidation, and 1/4-worker regressions are implemented and pass. The 220+ real-pipeline run and fixed-condition performance smoke were not run because batch C did not close. | PARTIAL |

## Confirmed failure causes and fixes

1. **Boundary ownership mismatch.** The old explicit-window construction used
   factors after the oldest state while UWB hypothesis generation independently
   scanned epochs at or after it. The first-state UWB was therefore monitored
   without an explicit row. `FrozenWindowFactorInventory` is now the authority
   for explicit, boundary, pending, and unrecoverable-history ownership. Group
   keys determine ownership and each slot is required to have exactly one
   disposition.
2. **Artificial ramp rank failure.** A newest-onset `[bias,slope]` ramp has raw
   map `[1,0]`. A per-mode thin-SVD now constructs a window-effective basis only
   for directions certified null in both raw measurement and protected-state
   response. Physical dimension, source identity, and risk allocation remain
   unchanged; the basis is rebuilt every window. ADR 0003 records the contract.
3. **Floating risk allocation overshoot.** Equal binary allocations could sum
   approximately `1.58e-18` above the available risk. The common
   `RiskBudgetAudit` uses high-precision accumulation and adjusts an equal share
   downward by one ULP when necessary. A true excess still fails the exact
   upper-bound check. Selected-path margin was `2.6081e-21` and nonnegative.
4. **Coverage bookkeeping defects.** Numerically identical actions now merge
   all covered modes/sources only when removal, additions, bridge semantics, and
   added-block content identity agree. Conflicting replacements remain
   distinct. Plausible strict supersets no longer create an additional action
   duty when their minimal plausible subset is already covered; all hypotheses
   remain in risk/post-PL processing. Candidate modes whose complete
   post-action measurement map is certified zero are removed only from that
   candidate residual census.
5. **Recovery provenance loss.** Historical replacement groups were formerly
   copied to unrelated records, and a selected current bridge was later renamed
   back to the original IMU group. Commit now attaches replacement groups only
   to the record owning the replaced group, filters the current catalog to the
   current transaction, and carries an actually selected bridge forward. The
   combined historical-UWB/current-bridge regression builds the next window
   with complete provenance.
6. **Repeated base linear algebra and production finite differences.** The
   frozen context now supplies one canonical SVD solution/residual/spectrum and
   one LLT selected-RHS facility to detector, evidence, KEEP_ALL and rank-update
   base. Production IMU sensitivity is analytic; the 12 reintegrations occur
   only through the explicit development oracle.

The boundary mismatch explains the deterministic loss of PL at epoch 21. It
was not the only blocker: ramp representation, candidate residual census, risk
rounding, coverage merging, and post-recovery provenance each had independent
minimal regressions.

## Raw-stream results

All scenarios use the covariance-mean raw stream, seed `20260901`, 200 Hz IMU,
20 Hz UWB, eight anchors, truth velocity `(0.45,0.45,0) m/s`, and
`formal_eligible=false`. Raw artifacts are under the ignored directory
`results/r0-closure-r1-numerics/final/`.

After the four-scenario replay, the numerical contract's already-hashed fixed
tolerances were also stored as explicit immutable fields. This changed only the
core-library binary identity. The final core was revalidated with a new
22-epoch default-ramp replay and the full calibrated IMU replay under
`results/r0-closure-r1-numerics/final-contract/`: respectively 22/22 finite PL
with one SVD/LLT/solve per window and the same generic-bridge selection,
HPL/VPL, counters, and zero contract mismatches. The older four-scenario core
SHA-256 was
`1703f9b2e5153cc7ffc4e8b01430e13a80ce670f0736e7915cec019952ddc49c`;
it is retained here as provenance, not presented as the final binary.

| Scenario | Raw injection | Decisions / commit | PL result | Contract result |
|---|---|---|---|---|
| A default ramp, 30 epochs | none | 30/30 KEEP_ALL commits; no alarm/reinit | finite 30/30; HPL/VPL exceed AL in this development geometry | normal flow across epoch 21 confirmed |
| C UWB, 28 epochs | anchor 1, +2.25 m, exactly one sample at 1.25 s | selected source `imu_accel:0:interval:25;uwb:1`; removed groups `23002;24002;25001;25002`; post detector `4.37316e-05 <= 266.153`; one backend update | HPL 3.79879 m, VPL 2.98875 m; finite but HPL > 2 m AL | correct UWB source is excluded in a conservative safe union; unique UWB-only isolation not shown |
| D IMU, 28 epochs | accel-x +1582 m/s² on ten raw samples, 1.205--1.250 s | three zero-update rejects, then source `imu_accel:0:interval:25`, generic bridge, post detector `6.12128e-04 <= 269.932`, one backend update | HPL 3.94488 m, VPL 3.26066 m; both exceed AL | bridge path executed; legitimate unavailable by AL |
| E joint, 28 epochs | same ten IMU samples plus anchor 1 +2.25 m at 1.25 s | three zero-update rejects; eventual selected source is IMU only; one backend update | same finite bridge PL as D, above AL | **joint positive not achieved**; the faulted UWB batch was rejected, not represented by the final recovery action |

The IMU absolute value is the frozen 1.25x point from the nominal-counterfactual
scan with approximate boundary 1265.6 m/s² and ratios
`0.75,1.0,1.25,1.5,2.0`. It is a sensor-reading fault. Bridge envelope checks
continue to use truth velocity/acceleration/angular rate, not this injected
reading.

Prepare/discard had zero backend updates and every commit had exactly one in
the transaction logs. Scenario C also exercised the unchanged stress contract:
three subsequent epochs each executed all 128 candidate kernels (412 total
kernel evaluations, 411 PL evaluations). It did not reinitialize or report
history contamination after recovery.

## Numerical work

The pre-change source audit found a normal base path of SVD x2, LLT x2, QR x1,
and three base-state solves per frozen window. The nominal final run reports one
base SVD, one base LLT, one base-state solve, and zero detector-reference QR per
window (30 each for 30 windows). It also reports 30 spectral/covariance RHS
solves and zero numerical-contract mismatches. Candidate/reference work is
separately counted and is not presented as base work.

| Scenario | base SVD / LLT / solve | QR ref | candidate SVD / inner LLT | covariance / spectral RHS | oracle reintegrations |
|---|---:|---:|---:|---:|---:|
| A | 30 / 30 / 30 | 0 | 29 / 0 | 30 / 30 | 0 |
| C | 28 / 28 / 28 | 0 | 254 / 1152 | 352 / 352 | 0 |
| D | 28 / 28 / 28 | 0 | 27 / 33 | 49 / 49 | 0 |
| E | 28 / 28 / 28 | 0 | 24 / 9 | 38 / 38 | 0 |

The explicit finite-difference tests execute exactly 12 reintegrations per
oracle invocation and cover stationary, constant velocity, acceleration, yaw,
3D rotation, multiple durations, nonzero bias, and a long-pending input.
Production calls report zero reintegrations regardless of pending length.

## Tests

Final command:

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl -j2
cd /home/mint/ws_fusion_uwb/build/uwb_imu_pl
ctest --output-on-failure
```

Result: **13/13 CTest targets passed**, including 42 Integrity V2/Gate D C++
tests, the two dense-oracle tests, realtime incremental tests, Python tools,
and the ROS visualization test. The four original raw directories and both
final-core replay directories also pass
`tools/validate_run_schema.py` (`uwb-imu-pl/v5` main schema; coverage attachment
v6 remains backward compatible with v1--v5 readers). `git diff --check` passed.

Important regression coverage includes prepare/discard/commit transaction
counts, boundary ownership, correlated-UWB principal-covariance rewhitening,
history replacement plus current bridge provenance, ambiguity/history
fail-closed paths, frozen graph/order/noise/linpoint/content identity mutation,
contract mismatch, dense action oracle at ADR tolerances, 1/4-worker agreement,
risk true-excess rejection, and analytic/oracle separation.

## Development timing observations

These are small, unpinned samples from the four-scenario behavior runs before
the explicit-contract-fields-only rebuild. They are not a
same-condition before/after benchmark; p99 is only an observation and RSS was
not captured. No performance delta is claimed.

| Scenario | n | core p50 / p95 / p99 / max ms | outer p50 / p95 / p99 / max ms |
|---|---:|---:|---:|
| A | 30 | 740.753 / 6307.591 / 6500.029 / 6549.880 | 741.054 / 6308.427 / 6501.040 / 6550.940 |
| C | 28 | 1597.610 / 134858.800 / 136810.120 / 137362.000 | 1598.250 / 134876.150 / 136827.120 / 137379.000 |
| D | 28 | 1374.685 / 3958.271 / 5465.192 / 5974.010 | 1375.385 / 3959.059 / 5466.409 / 5975.370 |
| E | 28 | 1297.940 / 3958.337 / 4237.611 / 4256.530 | 1298.805 / 3959.129 / 4238.719 / 4257.770 |

The same-condition baseline/performance comparison, fixed CPU affinity run,
RSS series, successful joint timing, rejection/timeout timing, and mature
220+ timing are `NOT_RUN`. In particular, no percent improvement is inferred
from the table and stage quantiles are not added or subtracted.

## Reproduction

```sh
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/r0_r1_development
CFG=config/realtime_uwb_imu_pl_research.yaml
ROOT=results/r0-closure-r1-numerics/replay
OMP_NUM_THREADS=1 "$BIN" "$CFG" "$ROOT/A_default_ramp" 30 A_default_ramp
OMP_NUM_THREADS=1 UWB_IMU_PL_DEV_UWB_BIAS_M=2.25 \
  "$BIN" "$CFG" "$ROOT/C_uwb_union" 28 C_uwb_fde
OMP_NUM_THREADS=1 UWB_IMU_PL_DEV_ACCEL_X_MPS2=1582 \
  "$BIN" "$CFG" "$ROOT/D_imu_calibrated_1_25" 28 D_imu_bridge
OMP_NUM_THREADS=1 UWB_IMU_PL_DEV_UWB_BIAS_M=2.25 \
  UWB_IMU_PL_DEV_ACCEL_X_MPS2=1582 \
  "$BIN" "$CFG" "$ROOT/E_union_calibrated" 28 E_union
for d in "$ROOT"/*/; do python3 tools/validate_run_schema.py "$d"; done
```

Each directory contains resolved configuration, run/scenario manifests, raw
fault truth, per-attempt numerical/risk/action diagnostics, coverage audit,
transactions, timing, states, integrity, candidates, and summary.

The final-core identity checks are reproducible by changing `ROOT` to
`results/r0-closure-r1-numerics/replay-contract` and executing the A command
with 22 epochs plus the D command unchanged.

## Remaining decisions and work

- Find a geometrically/model-valid joint injection whose final selected action
  explicitly covers both injected physical sources. The current E replay is a
  retained minimal negative result, not a pass.
- Demonstrate twenty consecutive bridge commits, the 21st timeout with zero
  update, controlled reinitialization, and clean recovery.
- Add the requested independent full frozen-graph boundary rebuild comparison;
  current boundary ownership plus candidate dense-oracle tests do not by
  themselves constitute that dedicated oracle.
- Only after the short joint and timeout prerequisites close, run the 220+
  real-pipeline marginalization scenario and post-marginalization joint fault.
- Run the fixed-affinity, four-worker, numeric-thread=1 before/after performance
  set with RSS capture. Formal 3x12000 and the E--I matrix remain outside this
  development round.

No R2 eligibility precheck/pruning or R3 sparse/local solver refactor was
introduced. No commit, merge, or push was performed.
