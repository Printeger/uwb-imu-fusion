# Locked PL conditional detector contract

Source commit: `ae54fb8ca55dfbfaf64fe45615b6bcd106548a93` in sibling repository
`uwb-imu-fusion-pl`. This document records source behavior; it does not claim that the IE adapter has passed.

## Lifecycle and prior

`RealtimeIntegrityPipeline::processUwbBatch` calls, in order,
`validateUwbBatch`, `predictTo`, `preMeasurementSnapshot`, conditional evaluation, and then exactly one of
`commitUwbBatch` or `rejectUwbBatch`. `predictTo` adds the causal Combined IMU factor and new `X,V,B` values but
does not add the current UWB group. `preMeasurementSnapshot` calls `currentJointMarginal()` for the current
`X,V,B` 15-dimensional tangent state and asserts `excludes_current_uwb`. The locked implementation obtains this
joint marginal from the current iSAM2 Bayes-tree clique/ancestor closure. The current group is committed only after
the detector passes and the PL model gate is valid; the original PL behavior rejects the entire alarm group.

## Physical and whitened equations

For each current measurement the snapshot stores physical innovation `nu = z-h` and the physical Jacobian
`H = d(h-z)/dx`. With physical covariance `R=LL^T`, it forms `W=L^-1`, `nu_w=W nu`, and `H_w=W H`.
The locked conditional detector is

```text
S_w = H_w P H_w^T + I
T   = nu_w^T S_w^-1 nu_w
dof = current group measurement count
threshold = chi_square_inverse_cdf(1-p_fa, dof)
p_fa = 1e-5
PASS iff T <= threshold
```

This is equivalent to physical `S=HPH^T+R`. Cholesky/LDLT, dimensions, finite values, whitening parity, prior
version, provenance, full-rank/SPD prior and model checks fail closed. The PL source also computes diagonal local
scores and posterior diagnostics, but neither changes the detector statistic above.

## Hypotheses and prohibited reinterpretations

`currentAnchorHypotheses` groups physical fault incidence by anchor ID. The source uses those hypotheses to compute
conditional protected-position failure slopes and protection levels. Failure slope is not an IE detector statistic,
not an IE isolation score, and must never rank an isolation winner. The IE adaptation instead evaluates every
leave-one-anchor-out subset against the same pre-measurement prior and accepts an isolation only when exactly one
subset passes. Truth, injected amplitude and trajectory RMSE are not detector inputs.

## Source map and hashes

- `src/uwb_imu_pl/estimation/incremental_estimator.cpp`: `predictTo`, `currentJointMarginal`,
  `preMeasurementSnapshot`, `commitUwbBatch`, `rejectUwbBatch`.
- `src/uwb_imu_pl/integrity/integrity_monitor.cpp`: `chiSquareThreshold`, `evaluateConditional`,
  `RealtimeIntegrityPipeline::processUwbBatch`.
- `config/realtime_uwb_imu_pl_research.yaml`: locked `p_fa: 1.0e-5`, fixed-lag and incremental settings.

Exact source hashes and extracted fixture hashes are sealed in the preflight output root. The IE core parity
tolerance is `abs_error <= 1e-12 + 1e-10*scale`; discrete fields must match exactly.

## IE preflight extraction note and verdict

The IE shadow reads the physical residual from the actual locked-input `ExpressionFactor`. The linked GTSAM build
asserts when that factor is asked for a generic derivative vector, so the adapter evaluates the same factor under
central `Pose3::retract` perturbations to obtain the six `X(k)` columns of `H`; velocity and IMU-bias columns are
zero for the raw range factor. This is an extraction implementation detail, not a changed detector equation.

The sealed Walk1 preflight did not pass admission: clean produced zero retained support, while all 30 affected
injected groups remained below threshold and produced zero alarms. Verdict:
`PL_CONDITIONAL_PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM`. Consequently no production provider, cache admission,
Stage2/final run, or E2E result was created.
