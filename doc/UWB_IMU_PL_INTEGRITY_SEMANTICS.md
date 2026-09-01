# UWB–IMU integrity semantics

Status: `IMPLEMENTED_UNVERIFIED`. This is research software, not a certified navigation safety function.

The protected state is the body origin position in the configured world frame. Snapshot monitoring covers a single physical-anchor range bias in the current snapshot. Fusion monitoring covers a single physical-anchor range bias in the current UWB group, conditioned on an IMU/history prior that excludes that group.

The snapshot detector uses the post-fit whitened residual and `DOF = measurement_rows - rank(H)`. `Regularizer` rows may constrain an estimator but never enter the formal statistic or DOF. The conditional detector uses innovation dimension as DOF because the independent prior constrains the state.

`PL_xyz` is the per-world-axis sum of the nominal Gaussian tail component and the maximum monitored single-anchor failure-slope component. `HPL_box = max(PL_x, PL_y)` and `VPL = PL_z`. Availability requires a passing detector, valid rank/conditioning/linearization, finite PL, `HPL_box <= HAL`, `VPL <= VAL`, and an unmonitored-risk budget no larger than the configured allocation.

An HMI event for evaluation is `available && protected_position_error exceeds the corresponding alert limit`. A failed detector isolates the whole current UWB group and publishes the IMU/history prior with `ALERT`; invalid mathematical assumptions, insufficient measurements, unmonitorable modes or non-finite quantities publish infinite PL and `UNAVAILABLE`.

Formal results are labelled `FORMAL_LOCAL_CURRENT_FAULT_ONLY`. That label does not cover persistent historical contamination, more than one simultaneous fault, anchor-map faults, nonlinear remainder, calibration-domain escape or non-Gaussian overbound validity. Until the deferred validation ledger is executed, the repository-level delivery status remains `IMPLEMENTED_UNVERIFIED`.
