# UWB–IMU integrity semantics

Status: research software, not a certified navigation safety function. Each result
starts as `IMPLEMENTED_UNVERIFIED` and is promoted to
`FORMAL_LOCAL_CURRENT_FAULT_ONLY` only after the applicable formal gate passes.

The protected state is the body origin position in the configured world frame. Snapshot monitoring covers a single physical-anchor range bias in the current snapshot. Fusion monitoring covers a single physical-anchor range bias in the current UWB group, conditioned on an IMU/history prior that excludes that group.

The snapshot detector uses the post-fit whitened residual and `DOF = measurement_rows - rank(H)`. `Regularizer` rows may constrain an estimator but never enter the formal statistic or DOF. The conditional detector uses innovation dimension as DOF because the independent prior constrains the state.

`PL_xyz` is the per-world-axis sum of the nominal Gaussian tail component and the maximum monitored single-anchor failure-slope component. `HPL = hypot(PL_x, PL_y)` and `VPL = PL_z`.

For the current batch, the allocated HMI upper bound is
`3 * nominal_axis_tail + p_nm + sum(prior_probability_bound * missed_detection_allocation)`.
The strict loader checks this against `risk.p_hmi_total` using the complete anchor
map, and the monitor repeats the check for the physical-anchor hypotheses in the
current batch. The reported `allocated_hmi_risk`, `hmi_risk_requirement` and
`risk_budget_valid` expose this decision.

`AVAILABLE` requires a passing detector, valid capability/model and formal gate,
finite PL, `HPL <= HAL`, `VPL <= VAL`, and a valid risk budget. A valid detector
alarm is `ALERT`; all other failures are `UNAVAILABLE`. Exceeding an alert limit
or the risk requirement does not reject a detector-passing UWB batch. A missing
capability, invalid model or detector alarm does reject the current batch.

An HMI event for evaluation is `available && protected_position_error exceeds the corresponding alert limit`. A failed detector isolates the whole current UWB group and publishes the IMU/history prior with `ALERT`; invalid mathematical assumptions, insufficient measurements, unmonitorable modes or non-finite quantities publish infinite PL and `UNAVAILABLE`.

The snapshot formal gate requires complete physical fault mapping, correct
whitening, valid rank/condition/linearization, finite covariance and a complete
risk allocation. The conditional gate additionally requires a 15x15 SPD
pre-measurement prior that explicitly excludes the current UWB group, consistent
graph/linearization versions, non-blind linearization, complete row/factor
provenance and an SPD innovation. Gate failures report infinite PL,
`UNAVAILABLE`, `formal_eligible=false` and an explicit reason. A mathematically
valid detector alarm retains the formal label but reports infinite PL and
`ALERT`.

`FORMAL_LOCAL_CURRENT_FAULT_ONLY` does not cover persistent historical
contamination, more than one simultaneous fault, anchor-map faults, nonlinear
remainder, calibration-domain escape or non-Gaussian overbound validity. The
unexecuted Monte Carlo, full ROS-topic simulation and performance items in the
test ledger remain outside the deterministic acceptance scope.
