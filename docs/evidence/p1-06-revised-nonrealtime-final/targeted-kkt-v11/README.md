# P1-06 targeted graph/KKT and history fallback v11

Status: `FROZEN / NO IMPLEMENTATION DEFECT PROVEN / QUALITY GATE FAIL`.

This pass is deliberately limited to seed `20260902`, the 1200- and
12000-attempt nominal off-profile replays, and the requested current-instance
history-root mutation.  It does not run or relabel the 38x20 matrix, FGO,
complete CTest, sanitizers, or Section 2.4.  The three final-v8 Section 2.4
comparisons remain `FAIL`; this source identity is `NOT_RUN` for Section 2.4.
No model, noise, prior, threshold, risk allocation, dof, coverage, sample
count, denominator, or publication rule was changed.

## Versioned read-only graph snapshot

`realtime_performance_benchmark` has an explicit diagnostic-only environment
switch, `UWB_IMU_PL_KKT_DIAGNOSTIC`.  It takes a read-only
`uwb-imu-pl/read-only-graph-snapshot/v1` copy at the actual maximum position
and attitude error attempts.  The snapshot contains the active fixed-lag
factors (including the marginal `LinearContainerFactor`), stored
linearization point, production estimate, production 15x15 current marginal,
graph/order/noise/linpoint versions, attempt/epoch, truth, and a hash of the
linearized graph at both the stored linpoint and production estimate.  Access
uses a private friend and adds no public method, data member, or virtual entry.
The golden-P1-05 header/client against the current DSO passes.

Both final replay directories contain 100% of their states, truth rows and
terminal packets: 1200/1200 and 12000/12000.  No row was removed.

| Replay | Actual worst attempt | rank | sigma min | condition | production objective | batch objective | production/batch current-state L2 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1200 | 1192 | 450/450 | 0.08176797 | 1.092e6 | 0.2773545 | 0.2662801 | 0.0233153 |
| 12000 | 8126 | 450/450 | 0.08478980 | 1.053e6 | 0.2493209 | 0.2297037 | 0.0270067 |

The independent batch LM reduces the local objective and KKT gradient, so the
incremental solution is not bitwise/full-MAP equivalent at these nonlinear
windows.  The scale-normalized gradient
`||g||/(sigma_max*sqrt(2*objective))` changes from `1.016e-3` to `2.695e-6`
at 1200 and from `1.525e-3` to `3.532e-6` at 12000.  That difference is not
the cause of the quality failure:

| Replay | production position / attitude error | batch position / attitude error |
|---|---:|---:|
| 1200 | 0.101474 m / 0.388997 rad | 0.095544 m / 0.366497 rad |
| 12000 | 0.519993 m / 2.500541 rad | 0.517611 m / 2.474511 rad |

The batch solution therefore retains essentially the same failed long-run
attitude/body-origin quality.  The active graphs are full rank, so repair-v9's
isolated single-epoch-UWB structural-nullspace explanation remains disproved.
The production covariance agrees with an independent marginal of the same
active graph at the stored production linpoint to `2.14e-11` (1200) and
`9.71e-12` (12000).  Its 0.197/0.223 difference from a marginal relinearized
at the current nonlinear estimate is the expected incremental-linpoint
approximation and does not establish a selected-inverse or marginal bug.

No implementation defect that explains the quality failure was isolated.
Changing relinearization policy, priors, sensors, noise, protected quantity,
or quality limits would be tuning or a contract/model change and was not done.
The fixed-model quality gate remains `FAIL`; no full campaign is recommended
from this pass.

## Current-instance tree mutation

A private, instance-scoped one-shot mutation damages only the optimized
hierarchical root after its normal update.  The immutable raw request rows are
left complete and finite.  On that same frozen attempt:

- tree invalid + independent full-row oracle valid is observed with fallback
  count 1;
- the suspect tree is reset, and the control instance is forced through a
  cold full rebuild;
- summary dimensions, carrier effective dof, kappa/Gram/response invariants,
  detector, PL, generated/evaluated action census, selected winner, commit
  boundary and immutable final packet all agree;
- fallback and hierarchical rebuild retain distinct representation version
  digests, as required because the digest binds exact orthogonal bytes, while
  their scope and every consumed invariant agree;
- the existing oracle-invalid mutation remains covered by
  `HistoryFaultSummary.P103OwnerCollisionTamperAndStaleCacheFailClosed`.

The focused mutation test passes and reports:

```text
fallback_count=1 reset=1 representation_identity_distinct=1
summary=PASS carrier=PASS dof=PASS gram=PASS response=PASS
pl=PASS action=PASS winner=PASS receipt=PASS packet=PASS
```

Peak RSS is 36,600 KiB for 1200, 43,544 KiB for 12000, and 140,328 KiB for
the two-pipeline mutation test, all below 1 GiB.  Commands, exit codes, raw
records, `/usr/bin/time -v` outputs, source/binary/DSO hashes and artifact
hashes are retained beside this report.  The first diagnostic write failure
and the two pre-final-identity diagnostic runs are historical failed/stale
iterations and are not acceptance evidence.  In particular,
`tree-mutation-pass.{exit,log,time.txt}` is the current mutation evidence and
has exit 0.  The earlier `tree-mutation-final.*` set has exit 1, is stale, and
is deliberately excluded from the pause checkpoint.
