# P1-06 repair investigation v9

Status: `FROZEN / NONREALTIME_ACCEPTANCE_BLOCKED`.

This repair pass did not change the sensor model, noise, priors, fault scope,
risk allocation, thresholds, dof, coverage, or any prescribed sample count.
It did not rerun or relabel the final-v8 denominator.  The final-v8 failures
remain evidence.

## History-root failure and bounded repair

The original 512-attempt reproduction first failed at attempt 511 with
`H_o=finite,H_b=finite,A=non_finite,z=non_finite`.  Added checks established
that the raw historical factor linearizations, fault columns, and augmented
Jacobian factors were finite before the hierarchical history-root cache.  The
invalid values therefore came from the repeatedly updated compact tree, not
from the frozen raw rows or the full-row summary.

The implementation now uses the already-prescribed certified full-row
orthogonal rebuild only when the optimized hierarchical root is invalid.  A
valid full-row result is returned and the suspect tree is discarded; an
invalid full-row result still fails closed.  There is no clamp, noise
injection, row removal, dof change, or invalid-value masking.

Focused results:

- `test_history_fault_summary`: 14/14 PASS, including correlated raw oracle,
  Schur/objective/Gram/kappa/dof, elimination-order and invalid-input tests.
- `HistorySummaryPipeline.P103ProductionClosureAudit` and
  `CarrierInvariantsUnderRowPermutation`: 2/2 PASS; the production oracle
  reported 30 checks, zero mismatches, maximum relative error `1.537e-12`.
- The same IMU-order1 nominal seed was replayed for 520 attempts.  Attempts
  511 through 520 all report `HISTORY_SUMMARY_VALID`; all 520 attempts are
  terminal `EXECUTED` with backend epoch advancing once per attempt.  The
  compact raw evidence is in `repair-history-root-fallback-520/`.

## Quality blocker: body pose is not identifiable from the fixed sensor model

The UWB factor observes the antenna point

`a = p + R l`,

not body origin `p` and attitude `R` separately.  For any alternative
attitude `R'`, choosing `p' = a - R' l` produces exactly the same antenna
point and therefore exactly the same range prediction to every anchor.  The
per-epoch rotational range Jacobian is

`J_theta(i,:) = u_i^T R [-l]_x`,

so it has the exact null direction `l`.  On the frozen off-profile geometry,
an independent calculation gives singular values
`[5.30363677e-1, 2.94025636e-1, 4.06934640e-17]`, numerical rank 2, and
`||J_theta l|| = 5.32121243e-18`.

The raw final-v8 output independently confirms that this is the active mode,
not a quaternion, timestamp, or scoring defect:

| Campaign | body-origin position RMSE | antenna-point RMSE |
|---|---:|---:|
| 20 x 1200 seeds, mean | 0.044497 m | 0.004363 m |
| 3 x 12000 seeds, mean | 0.209013 m | 0.004368 m |
| 12000 seed 20260902 | 0.355874 m | 0.004339 m |

The antenna estimate remains millimetric while body-origin position and yaw
separate along the unobservable/weakly observable decomposition.  The fixed
configuration has a single body-to-tag lever arm and no absolute heading
measurement; its initial rotation prior sigma is already `0.1 rad`, equal to
the required attitude-RMSE limit.  The observed 20-seed result (4/20 below
`0.1 rad`) and 12000-attempt result (0/3 below `0.1 rad`, position 2/3) cannot
be repaired by a mathematically equivalent implementation.

The minimum new authority must select at least one contract change: add a
second non-collinear tag/antenna observation, add an absolute heading sensor
or heading prior with a stated accuracy contract, redefine the protected
position as the antenna point, or revise the quality threshold/model.  All
are outside the present Goal.

## Section 2.4 strict comparison

The three original strict comparisons remain `FAIL` and were not rewritten.
They mix three categories: authorized post-golden P0-07 identity/dof/config
changes, the added final-packet publication boundary, and exact comparison of
wall-derived `watchdog_sensor_lag_ns` across separate executions.  The latter
is not a deterministic numerical quantity.  No ignore set was widened and no
strict result was replaced.  A future acceptance needs an explicitly
versioned semantic mapping for the authorized fields while retaining the old
strict result, or a deterministic-clock golden campaign under a separately
approved baseline procedure.

Because the fixed-model quality gate remains impossible and strict golden
equivalence remains failed, no new full 38x20/FGO/long-run/Section 2.4/CTest/
sanitizer acceptance campaign was started after the source identity changed.
Doing so cannot close the blocker and would create another ineligible mixed
identity evidence set.
