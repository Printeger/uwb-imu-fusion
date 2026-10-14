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

## Simulation/source separation (W1)

`Sc6Service::step` receives an arrived envelope, the frozen physical domain,
initial observation and original law/config. It has no provider, trajectory,
GT, draws or episode manifest. Provider-owned draw-based certification is
**privileged SIMULATION_SOURCE_CERTIFIED**, not an online health detector or
hardware law. Only its Boolean numerical qualification and sealed range
roundoff are passed; it never selects healthy rows. Per-epoch certification
is executed inside measured service and uses only arrived samples, with RW
roundoff carried forward. No future maximum or source prequalification is
subtracted from runtime. Initial tangent source is independently checked by
directed exponential/V Taylor remainder; intervals contain the represented
initial observation's numerical residue. Noise, bias and injection values
stay in provider/scorer.

The independent 100-digit scalar oracle checks fixed recipes at extrema,
seams and late times; it does not generate expected results using the
contractor. Motion proof is analytic: M1 quintic q=10u³−15u⁴+6u⁵ has
max |q''|=10/√3, max |q'''|=60. A 4s travel of d=(.8,.4,.2), two seconds
rest, return and rest has A<.331, J<.860, continuous p/v/a including seams.
M2/M3 amplitudes (.8,.6,.4) and frequencies (.6,.5,.4) give A<.332, J<.191.
R=Rx(.3 sin(.4t))Ry(.4 sin(.3t))Rz(.5 sin(.2t)), body→world; its gyro is
RzᵀRyᵀex*x'+Rzᵀey*y'+ez*z'. Thus norm rate≤.34 and alpha<.181,
including noncommuting axes. These all fit the frozen D1 1/1/2/2 domain;
M0 retains D0. Force Lipschitz follows J+Omega*(g+A), if needed. Neither
trajectory formulas nor actual derivative values enter online propagation.
OpenFaultPool, RollingIntegrity and actual bridge config consume D1.
D1 is a new conditional research scope, not an extension granted to old
production defaults.

W1 debug failures retained in accounting: one compile-time const-config
interface error; one new-target Make reconfiguration failure (runner and
legacy check had linked); one direct legacy-check invocation resolved an old
DSO because LD_LIBRARY_PATH was omitted. Stable tools enforce the private
DSO. Initial W1 traces (30+250+400 rows) exposed an Eigen lazy expression
bug in tag serialization/scoring. Their native states/proofs were valid;
reported tag/accuracy fields are invalid and excluded from capability
claims. Only affected scoring/adapter cases are rerun under new labels.

W1 stable checks PASS: 100-digit trajectory/seam/late scalar inclusion,
privileged prefix source qualification, future data mutation leaves actual
state/action/PL unchanged; legacy open check with correct DSO PASS.
D0 old/new raw measurements, initial observation and range numerical evidence
compare **exactly equal** for 65 epochs; healthy B0 terminal state/PL max
absolute difference 1.207e-11. Stable B0 250/250, B1 400/400 native/raw;
B1 repairs61/321 and rejoin80–83/340–343 are preserved. B1 recovered full state
has material differences (max serialized component 1.686 before certificate
adapter refinement); compatibility of recovered numerical states is OPEN,
not called a pure performance difference. Exact input identity and recovery
semantics alone do not close this gap. Capsule-midpoint recovery/SO(3) metric
conditioning must be evaluated in W2 A comparisons. Stable new S1/S2 paced
90/90 each: real prior repair61, rejoin80–83, no source gap/abort or PE>PL;
core40 misses1/7 and measurement-to-sink50 misses0/1 are retained.
W1 scorer evaluates actual online terminal states and post-delivery GT,
including body SE(3) RPE Δ1/20, velocity/rotation/bias and stage error.
