# ADR-0001: UWB–IMU Integrity V2 research scope

- Status: accepted for research implementation
- Date: 2026-09-04
- Baseline: `feature/realtime-uwb-imu-pl@ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`

## Decision

Integrity V2 protects the body-origin position in the world frame and reports
`PL_x`, `PL_y`, `PL_z`, HPL, and VPL.  Its first monitored set contains a
single physical UWB anchor bias, interval-constant additive faults on each of
the three accelerometer and three gyroscope axes, and one-anchor-plus-one-IMU-
axis combinations.  Two-UWB faults, IMU ramps, bias jumps, scale factors, time
synchronization faults, and a formal dynamics bridge are outside this claim.

The detector uses a 20-epoch joint window.  Fixed-lag operation uses a
10-epoch recovery margin and therefore requires `fixed_lag_epochs > 30`.
Continuity uses a conservative union bound over 1000 tests.  Maximum monitored
cardinality is two and maximum candidate count is 128.

Ambiguity is resolved only by an action covering the complete plausible set;
otherwise integrity is unavailable.  IMU exclusion uses an independent,
bounded constant-velocity/constant-attitude bridge, limited to 20 epochs or
one second.  Active-window history may be atomically removed and replaced;
suspect information already absorbed into an unrecoverable boundary produces
`HISTORY_PRIOR_CONTAMINATED`.

The maturity-delay policy is selected for the first implementation.  The
estimator may commit a valid best-effort plan when calibrated integrity is
unavailable.  A discarded transaction republishes only the last committed
state at its original timestamp and explicitly marks it stale/reinitializing.

## Release boundary

All Gate A–I outputs remain `IMPLEMENTED_UNVERIFIED` with
`formal_eligible=false`.  `FORMAL_WINDOWED_UWB_IMU_FDE` is prohibited until
noise/risk/bridge calibration identifiers, the complete Gate J evidence set,
and independent equation and code reviews are present and valid.

Placeholder combination priors do not assume independence and use the
conservative Frechet upper bound `min(component priors)`.  Formal estimator
weights remain fixed; GNC is shadow-only.

## D round status (2026-09-22)

The declared scope is unchanged.  The D acceptance round measured the whole
pipeline against the 40 ms normal-frame target and recorded a blocker instead
of narrowing the claim: with the shipped `fixed_lag_epochs: 200` the boundary
history grows until fixed-lag marginalization engages, and the C1 history
summary extraction (stage `window_boundary_provenance`) costs O(boundary
rows ^ ~4).  Measured at 20 Hz replay: ~10 s/frame at epoch 100 and ~26 s at
epoch 114 on an 18-core workstation, and `core_total` crosses 40 ms at epoch
13 of the benchmark trajectory.  `formal_eligible` remains false; the
performance finding is recorded in the D-round evidence
(`d-round-report.md` section 11, `proof-obligations.md`).

