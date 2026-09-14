# REFACTOR-GATE-06R-E: bounded fixed-lag common initializer

## Verdict

`COMMON_INITIALIZATION_FIXED_LAG_FAIL`

The requested formulation was implemented, but the single locked SFUISE Walk1
initialization-only audit failed closed at frontier state 125. Consequently no
full 913-state Initial Values exist and Gate06 remains blocked.

## Formulation repair

The previous production initializer fixed every accepted state and optimized
only the new frontier `X/V/B`. Gate06R-D established that this one-state
conditional formulation stopped at frontier 51 with a maximum scaled
navigation gradient of `1.0741484355758502e-05`, despite a full-rank local
problem and an identical continuation.

The replacement is explicitly a **BOUNDED FIXED-LAG CAUSAL INITIALIZER**, not a
strict zero-lag initializer. At frontier `k` it:

1. chooses the earliest recorded boundary for which
   `t_k - t_boundary <= 0.25 s`;
2. fixes the boundary `X/V/B` and jointly optimizes every later `X/V/B` through
   `k`;
3. uses only the corresponding physical adjacent `CombinedImuFactor`s and UWB
   representatives associated with states no later than `k`;
4. clones only the local UWB factors to apply the existing Huber `1.345`
   initialization wrapper, leaving their measurement and fixed sensor sigma
   unchanged;
5. applies the existing checked LM settings, the existing one identical
   continuation, and the unchanged `1e-5` navigation-stationarity qualification;
6. atomically commits all free window states only after the complete window
   qualifies. States before or at the advancing boundary are never reopened.

Any rejected window returns `INITIALIZATION_FAILED` with empty full Values. It
does not restore the full-recording IMU-propagated tail.

The existing backward-compatible configuration field
`initialization.progression_horizon_s` remains the sole initializer horizon;
its actual meaning is now the fixed-lag window bound. The locked effective
configuration omits the key and therefore uses its frozen `0.25 s` default.
There is no second horizon and no sweep.

## Files changed for this task

- `include/uifgo/common_initializer.h`: fixed-lag window audit fields and
  explicit causal semantics.
- `src/common_initializer.cpp`: bounded window construction, joint solve and
  atomic window commit.
- `include/uifgo/config.h`: compatibility-field semantic documentation only.
- `test/test_common_initializer.cpp`: deterministic causality, lag boundary,
  factor-membership, immutability, fail-closed and graph/sigma tests.
- `tools/run_all_uwb_optimization_diagnosis.cpp`: success-path per-window CSV
  and summary/frontier-51 reporting; no algorithmic logic.
- `doc/ie_sprint/METHOD_CONTRACT.md`,
  `doc/ie_sprint/EXPERIMENT_CONTRACT.md`, `doc/ie_sprint/STATUS.md`, and
  `paper/CLAIM_EVIDENCE.md`: pre-registered scope and final failed outcome.

No final-graph, Cauchy, final LM, Solver Certificate, PL/CUSUM, support,
bias-refit, recoverability, recover/suppress, or final-inference algorithm was
changed by this task.

## Deterministic engineering evidence

The initializer target and initialization-audit executable built successfully.
The focused test binary ran 8 tests and reported 8 passed, 0 failed. Covered
behavior includes future-measurement invariance, a bounded multi-state window,
lag-exited state immutability, unchanged ledger/selected IDs, unchanged physical
UWB sigma, fail-closed empty Values, drift correction in a synthetic fixture,
and physical graph immutability.

The dedicated synthetic requirement that independently reproduces the exact
one-state stationarity failure and then passes in fixed-lag form was not added
before the locked audit's mandatory STOP. Its authoritative integration
counterpart remains Gate06R-D. Thus the requested Test 3 is not claimed as a
new unit-test PASS. The Gate05 architecture guard and full core regression were
also `NOT_RUN_AFTER_LOCKED_AUDIT_FAILURE`; running further gates after the first
later-frontier failure would contradict the task's STOP instruction.

Commands actually run:

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target uwb_imu_fgo_all_uwb_optimization_diagnosis test_common_initializer -j2
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/test_common_initializer --gtest_color=no
```

Both exited `0`. `git diff --check` also exited `0` before reporting.

## Locked Walk1 initialization-only audit

The only effective audit invocation was:

```text
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_all_uwb_optimization_diagnosis experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/science/effective_configs/t09-2bbed3b2f76a412380b021a6c6e49fcf.yaml experiments/icra2027/dev/COMMON_INITIALIZATION_FIXED_LAG_WALK1/audit_final initialization
```

It built the locked `state_step=1` physical problem with 913 target states,
2276 estimator-usable UWB factors and 3191 total factors. It evaluated 125
frontiers, accepted 124, and then exited `1` as designed:

| Field | Result |
|---|---:|
| First failing frontier | 125 |
| Frontier timestamp | `1664959685.0827963 s` |
| Dominant navigation key | `x125` |
| Maximum scaled gradient | `0.0011026163512872778` |
| Required maximum | `1e-5` |
| Initialization failures | 1 |
| Total continuation count at failure | 2 |
| Runtime to failure | `0.37897320299999998 s` |
| Returned full Initial Values | no (empty/fail closed) |
| GT/truth/oracle read | false |
| Final estimator run | false |
| IE stages run | false |

Narrowest observed mechanism:
`PREFIX_SOLVE_REJECTED / CONDITIONAL_LM_STATIONARITY_NOT_REACHED`, with the
dominant scaled navigation gradient at `x125`. No claim is made about how a
parameter change would affect it, because such a test was forbidden.

Because preparation deliberately rejects the first unqualified window before
the success-path audit writer receives full Values, the following requested
full-sequence metrics are unavailable rather than computed from a partial
iterate: maximum window-state count and complete window-duration distribution;
position/velocity/consecutive-displacement distributions; raw and standardized
UWB residual distributions; final-baseline Cauchy-weight distribution;
per-anchor residual distributions; maximum stationarity gradient over all 912
accepted windows; and a full finite-state audit. The absence of these metrics
is direct evidence that partial initialization was not presented as a valid
913-state seed.

Machine evidence is isolated at
`experiments/icra2027/dev/COMMON_INITIALIZATION_FIXED_LAG_WALK1/audit_final/`:
`command.txt` records the invocation and `failure.json` records the fail-closed
status and the flags excluding GT, final FGO and IE.

An earlier attempted invocation had its arguments in the wrong order and
aborted before loading data or entering initialization; it produced no output
and is not counted as a scientific audit.

## Frontier 51 comparison

| Formulation | Window | Maximum scaled gradient | Qualification |
|---|---|---:|---|
| Previous one-state production | state 50 fixed; state 51 free | `1.0741484355758502e-05` | FAIL |
| Gate06R-D fixed-lag diagnostic | state 48 fixed; states 49--51 free | `4.905753878e-06` | PASS |
| This locked fixed-lag run | frontier 51 is within the 124 accepted frontiers | per-frontier value not persisted after the later fail-closed exception | accepted |

The current run therefore crossed frontier 51 under the same locked settings,
but the overall initializer is not accepted: it failed later at frontier 125.
No threshold, retry count, sigma, Huber constant, LM setting, or lag was changed.

## Acceptance decision

The mandatory conditions fail because no complete 913-state Values were
returned, not every frontier qualified, full-tail/residual/weight/finite audits
cannot be performed, the dedicated deterministic Test 3 is incomplete, and
core/architecture gates were not run after STOP. No ATE criterion was used.

`COMMON_INITIALIZATION_FIXED_LAG_FAIL`

`GATE06_RERUN_BLOCKED`
