# SC6 results (in progress)

W0 baseline saved; W1–W4 pending. No SC6 capability is awarded yet.

## Baseline and frozen scope

Actual entry HEAD: `20831ec47888c6d96c9ac73571254b0efd8e9b52`.
GTSAM 4.2a5 `/usr/local/lib/libgtsam.so.4` and unstable; Eigen 3.3.7; TBB 2020.1.
Actual project DSO `/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/libuwb_imu_pl.so`.
Release `-O3 -march=native`, proof sources `-frounding-math -ffp-contract=off`, no fast-math.
One TBB solver thread. `config/sc6.yaml` freezes D0/D1, trajectories, geometry, seeds and episodes before candidate results. Original sigma, initial tangent 15D law, RW, global birth q, epsilon=1e-6 and production publication switches stay unchanged.

Baseline runner SHA256 `fd00dfe97d1d613f5621b2ccb7d6fe8b4e2e03b3af5270312bd6eb17da6cf726`; project DSO `5faa46cc2d41492a55c65e9972a7bbe1f2a1b1773d791034ebdf119ec5954189`.

|Old entry|scheduled/native/raw|tag RMSE m|HPL mean/max m|VPL mean/max m|core40/full50 misses|prior repairs|slots max|
|---|---|---|---|---|---|---|---|
|B0|250/250/250|0.050868|4.333362/4.600195|6.827268/7.307554|0/0|0|284|
|B1|400/400/400|0.068410|4.485941/5.413660|7.094618/8.559925|2/0|2|453|

B1 actual repair epochs 61/321 and actual IMU rejoin 80–83/340–343. These old times include GT core scoring and omit final gate/I/O; they are not equivalent speed measurements of the new service.

Dependency graph: ranges→tag→p/R; dynamics→p/v; RW→bias; no existing measurement→bias contraction. Recovery metric consumes raw capsule p/v/R/bias midpoints, so its changes need A comparisons. M uses the identical actual commit center and source event; P fixes the full new mathematical/selection algorithm.

Slot cause: local GTSAM `ISAM2::marginalizeLeaves` appends `LinearContainerFactor` via `nonlinearFactors_.push_back`, bypassing `findUnusedFactorSlots`; later removes summarized indices. Recycling is already enabled for normal updates. Slot compaction cannot merely flip that option or clear history. Active factors/ledger remain bounded in baseline; tombstones grow.

Unified service boundary will start at arrived measurement verification/allocation, include prepare/FDE/raw/commit/proof/cleanup/payload/gate and end at receiving sink. Scoring follows delivery. Independent producer releases at 5ms/50ms from RUN_READY; measurement-to-sink is separate from service/core.

## Reproduction and Git

Temporary output exclusively `.cache/sc6-20261014`. B0/B1 commands: original `epoch_raim_runner config/sc5_open.yaml CSV OE_HEALTHY|OE_MIXED open-rolling on 250|400`. No old campaign rerun. W0 validation 650/650 native/raw, no abort; production defaults unchanged.
