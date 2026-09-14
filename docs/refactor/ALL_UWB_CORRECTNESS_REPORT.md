# REFACTOR-GATE-06 all-UWB correctness report

Date: 2026-09-13  
Task: `REFACTOR-GATE-06 / ALL-UWB CORRECTNESS VALIDATION`  
Role: correctness gate only; not a publication ablation

## Verdict

`ALL_UWB_CORRECTNESS_FAIL`

The repaired frontend built the intended all-usable-UWB graph correctly, fixed
sensor sigma was preserved, exact repeats did not multiply likelihoods, and the
23.94 m glitch did not dominate the attempted robust objective.  The sole run
nevertheless exhausted the frozen 100-call conditional-LM budget while it was
still making large state updates.  Its optimizer termination was
`CONDITIONAL_LM_MAX_ITERATIONS`, the solver certificate remained
`NOT_EVALUATED`, and the runner correctly exported no trajectory.

The narrowest evidenced remaining failure mechanism is therefore:

`ROBUST_CONDITIONAL_LM_DID_NOT_CONVERGE_WITHIN_THE_FROZEN_100_ITERATIONS`

This wording does not claim that merely increasing the budget would fix the
problem; that intervention was not run and is outside this gate.

## Locked configuration and provenance

- Branch: `feature/uwb-imu-fusion-ie-postprocessing`
- Commit: `e9d821e960f725e064ca32fd9e475e529ac5f501`
- Dirty-worktree status SHA-256 at lock: `7532b5058fbe87d8396f65faaeaf8a819e29df746c8e5213f30c7d3bd4b82eef`
- Runner SHA-256: `a325298990087e1ea3c6d1bca1b9b22d528300d65ecc3c356d0a94daa83850d5`
- Linked `libuwb_imu_fgo.so` SHA-256: `81e67a1ce9ce61436117fa596c33acb3d6448d4b83f8e74476ebca62279b04cd`
- Source config SHA-256: `4cd9da0df33407e15c75de3bafc1075c5d096d2a55d7a68962b99b4936710820`
- Dataset: the same SFUISE Walk1 clean t07 cache used by DEV-DENSE-01
- `keyframe.step=1`; no interpolation, B-spline, or continuous-time state
- Method: existing `robust_cauchy`, direct common-initial-Values path
- Cauchy standardized-residual scale: 2.3849, inherited without tuning
- Sensor sigma: fixed 0.15 m (`FIXED_SENSOR_SIGMA_V2`)
- Solver: existing LM settings, including `lm_max_iter=100`
- Attempts: exactly one; no retry, sweep, tuning, or runtime replicate
- Diagnostic instrumentation: `UIFGO_BASELINE_DIAGNOSTIC=1`; it records calls
  and factor errors but does not alter solver decisions.  Runtime below is the
  instrumented runtime.

The estimator sandbox mounted the runner, linked dependencies, effective config,
and locked measurement cache only.  GT/truth/oracle mounts were empty.  The old
DEV-DENSE-01 tree hash was unchanged before and after the run.

## Required audit outputs

The distinction between a constructed factor graph, a terminal optimizer iterate,
and a certified final estimate is essential here.  No certified final estimate
exists, so final-state quantities are reported as `NA`; available terminal-iterate
diagnostics are labelled explicitly and are not promoted to final results.

| Required output | Result |
|---|---:|
| Raw UWB observations | 4,850 |
| Source-valid | 4,266 |
| Estimator-usable | 4,266 |
| All-ledger correlation groups | 2,860 |
| Estimator-usable/selected correlation groups | 2,276 |
| Repeat-bearing groups | 1,990 |
| Rows marked `STALE_REPEAT` | 1,990 |
| Instantiated UWB factors in attempted graph | 2,276 |
| States | 913 |
| Total graph factors | 3,191 |
| Per-anchor UWB factors | 7475: 479; 9524: 433; 10548: 450; 15155: 464; 20276: 450 |
| Sensor sigma distribution | n=2,276; min/median/max=0.15/0.15/0.15 m; one unique value |
| Final robust residual statistics | `NA_NO_CERTIFIED_FINAL_VALUES` |
| Maximum final standardized UWB residual | `NA_NO_CERTIFIED_FINAL_VALUES` |
| Final objective | `NA_NO_CERTIFIED_FINAL_VALUES` |
| LM calls/iterations | 100 |
| Solver termination | `CONDITIONAL_LM_MAX_ITERATIONS` |
| Solver certificate | `PAPER_SOLVER_CERTIFICATE_V1 / NOT_EVALUATED` |
| Maximum final position norm | `NA_NO_CERTIFIED_FINAL_VALUES` |
| Runner elapsed | 8.043338118 s, instrumented |
| Process wall time | 8.099658470 s, instrumented |
| Peak RSS | 102,956 KiB = 100.542969 MiB |
| Aligned ATE RMSE / P50 / P95 | `NA_NOT_EVALUABLE_NO_CERTIFIED_TRAJECTORY` |

The orchestration command itself returned zero because it successfully recorded
and sealed a terminal failed cell.  The only estimator attempt returned exit 1;
`run_status.json` records `ESTIMATION_FAILED` and
`valid_estimate_exported=false`.

### Uncertified terminal-iterate diagnostics

The 100th call reduced the robust graph objective from 56,832.304628094505 to
56,797.576034342019, with accepted-Values delta norm 5.7393612335542858.  Thus
the optimizer was neither falsely promoted nor silently treated as stationary.
The sum of all captured terminal factor contributions equals
56,797.576034342019 exactly at the recorded precision.

For the 2,276 UWB factors, inverting the frozen Cauchy loss gives these absolute
standardized-residual diagnostics at that uncertified iterate:

| Statistic | `abs(q)` | Cauchy weight |
|---|---:|---:|
| Mean | 1,915.407320 | 0.321791221 |
| Median | 1,001.619905 | 0.00000566946 |
| P95 | 6,724.565282 | — |
| P99 / weight P01 | 7,605.009026 | 0.0000000983426 |
| Max / min weight | 7,889.417153 | 0.0000000913799 |

The UWB robust factors account for 56,682.08327670502, or 99.7966590%, of the
uncertified terminal objective.  These widespread extreme residuals support the
observed global non-convergence mechanism; they are not final residual claims.
Signed residuals are unavailable because the fail-closed runner did not export
the terminal Values.

## The 3.48 / 23.94 / 23.94 / 3.41 region

The exact four-row pattern remains at anchor 15155, source messages 277--280.
The audit also includes messages 276 and 281 to show both adjacent repeat pairs.

| Msg | Raw range (m) | Integrity | In estimator | Factor | Final q/weight | Uncertified terminal `abs(q)` / weight | Objective fraction |
|---:|---:|---|---:|---:|---|---|---:|
| 277 | 3.480000019 | `STALE_REPEAT` of msg 276 | no | — | NA | — | — |
| 278 | 23.940000534 | `USABLE`, group representative | yes | 921 | NA | 135.000482 / 0.000311985 | 0.0404196% |
| 279 | 23.940000534 | `STALE_REPEAT` of msg 278 | no | — | NA | — | — |
| 280 | 3.410000086 | `USABLE`, group representative | yes | 930 | NA | 1.653903 / 0.675253 | 0.0019661% |

All four source observations remain in provenance with distinct stable `obs_id`s.
The two 23.94 m rows share correlation group/representative
`17215179282077361602`; exactly one likelihood entered the graph.  The preceding
3.48 m pair and following 3.41 m pair obey the same rule.

There is no certified final standardized residual or final robust weight to
report.  At the captured terminal iterate, the sole 23.94 m likelihood contributes
22.9573274290013 objective units, 0.000404195549 of the total.  It therefore does
not dominate even this failed iterate under the preregistered strict-majority
definition.  The failure is global robust-LM non-convergence, not duplicate
amplification or domination by this single glitch.

## Correctness questions A--F

| Question | Answer |
|---|---|
| A. Numerically stable? | **No pass.** Values were not certified/exported; LM exhausted 100 calls while the last update remained large. |
| B. Solver certificate passes? | **No.** It is `NOT_EVALUATED` because raw termination failed. |
| C. Single glitch stops dominating? | **Yes at the uncertified terminal iterate**, where the 23.94 m representative is 0.0404% of objective; no final claim is possible. |
| D. Repeats prevented from multiplying information? | **Yes.** 4,266 usable rows became 2,276 groups/factors, with maximum one instantiated likelihood per group. |
| E. Persistent-NLOS information still available to IE? | **Interface preserved, IE execution not exercised.** 336 selected representatives retain `suspected_nlos=1`; this baseline run deliberately did not execute Stage1/Stage2/final IE. |
| F. Physically plausible? | **Not established.** No certified trajectory or maximum position norm exists, so GT evaluation was correctly skipped. |

## OLD PIPELINE vs REPAIRED PIPELINE

This table is context only.  The methods, sigma semantics, repeat policy, state
density, factor count, objective function, success semantics, and instrumentation
all differ.  It is not a controlled causal state-density ablation.

| Item | OLD PIPELINE D0 | REPAIRED PIPELINE Gate06 |
|---|---:|---:|
| State step / states | 4 / 229 | 1 / 913 |
| UWB factors | 1,074 | 2,276 correlation representatives |
| UWB model | plain Gaussian old path | Cauchy, scale 2.3849 |
| LM status / calls | `CONDITIONAL_LM_CONVERGED` / 94 | `CONDITIONAL_LM_MAX_ITERATIONS` / 100 |
| Certificate | historical termination-only result | `NOT_EVALUATED`; fail closed |
| Objective | 128.934602, old graph/loss | final NA; terminal diagnostic 56,797.576034 |
| Max position norm | 4.466275 m | NA |
| Aligned ATE RMSE / P95 | 0.164026 / 0.299122 m | NA / NA |
| Runtime / peak RSS | 1.365219 s / 41.3828 MiB | 8.043338 s / 100.5430 MiB, instrumented |

The old `DENSE_STATE_RECOMMENDATION=NOT_WORTH_IT` is not reused.  Gate06 instead
fails strictly because the repaired all-UWB configuration did not produce a
certified result.  Conversely, the repaired success boundary did prevent the
failed iterate from becoming another false-success trajectory.

## Failure accounting and artifact index

- Formal estimator attempts: 1
- Successful estimates: 0
- Failed estimates: 1 (`FINAL_LM_FAILED:CONDITIONAL_LM_MAX_ITERATIONS`)
- Solver certificate success: 0
- Fallback/retry/tuning: `NOT_RUN`
- Stage1/Stage2/final IE: `NOT_RUN_BASELINE_CORRECTNESS_PATH`
- GT evaluation: `NOT_EVALUABLE_NO_CERTIFIED_TRAJECTORY`; GT was not opened
- Paper claims/figures/tables: unchanged
- Old DEV-DENSE-01 evidence: byte-hash unchanged

Primary evidence:

- Lock: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/scientific_lock.json`
- Batch and single attempt: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/science/batch_manifest.json`
- Science seal: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/science_seal.json`
- Run status and certificate: `science/runs/t09-2bbed3b2f76a412380b021a6c6e49fcf/{run_status.json,solver_certificate.json}`
- Observation/integrity ledger: `science/runs/t09-2bbed3b2f76a412380b021a6c6e49fcf/observations.csv`
- Instrumented LM/factor trace: `science/runs/t09-2bbed3b2f76a412380b021a6c6e49fcf/stdout.log`
- Aggregate audit: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/audit_metrics.json`
- Problem region: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/problematic_region_audit.csv`
- Terminal residual audit: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/robust_residuals_terminal_uncertified.csv`
- Verification: `experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/verification.json`

Commands actually executed:

```text
python3 -B experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/run_all_uwb_correctness.py
python3 -B experiments/icra2027/dev/ALL_UWB_CORRECTNESS_WALK1/postprocess_gate06.py
```

No follow-up fix was implemented.  This gate stops at the failure report as
required.
