# 0911 Oracle-support recovery backend sanity check

Status: `DONE / RECOVERY_BACKEND_NOT_OPERATIONAL`. Authorized by the user on 2026-09-11. The sole code
baseline is HEAD `15b32f5afa01c5a0f8bc432bbac78c53bfa36420`; the sole data baseline is
the existing locked SFUISE Walk1 semi-synthetic injection manifest and its
immutable artifacts under
`/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01`.

This is an oracle-support, backend-only scientific diagnostic. The oracle may
provide only the locked tag, anchor, interval and actual planned affected
`obs_id` values. It must not provide the injected amplitude, GT pose, an
initialization, or any estimated quantity. The estimator consumes the existing
`OracleSupportProvider -> SupportPartition -> SegmentRefitter -> recoverability
-> final inference` production chain. Injection amplitude `0.5 m` and GT are
evaluation-only. All results are labelled oracle/development diagnostics and
must not be reported as end-to-end detector performance.

Freeze the current FDE implementation, threshold/probability/grouping, Stage 2
formulation and stopping rules, `R_c`, local sigma, `kappa=2`, LCB and full
correction formulas, final factors, optimizer tolerances, injection recipe and
GT evaluator. Do not tune parameters or implement a detector. A correctness or
plumbing repair is allowed only after a concrete failure is demonstrated; it
must be recorded before implementation and may not change those definitions.

Run the locked Walk1 normal-redundancy injected dataset with one identical
oracle partition for `suppress_all`, `structured_debias`, `lcb_fixed_full` and
`lcb_partial`, plus `all_range` and `robust_cauchy`. Reuse locked clean metrics
as the control. Require nonempty, link-exact planned support; a real nonempty
Stage 2 solve; recorded objective, stationarity, KKT, `c_hat`, `R_c` rank and
conditioning, local sigma, fixed corrections, and real nonempty-support final
optimizer calls/factor audits. Trace estimator file access and audit effective
configuration, partition, Stage 2 inputs and final factors for absence of the
true amplitude. Use the unchanged locked evaluator for trajectory metrics.

The normal result is classified exactly as follows. Successful Stage 2,
reasonable `c_hat`, available `R_c`/sigma, nonzero LCB and successful final
solve establish `RECOVERY_BACKEND_OPERATIONAL`; if LCB also has lower RMSE than
suppress, the final classification is
`RECOVERY_BACKEND_LOCALIZATION_BENEFIT_OBSERVED`, otherwise
`RECOVERY_BACKEND_OPERATIONAL_BUT_NO_LOCALIZATION_BENEFIT`. Failure of Stage 2,
`R_c`, usable LCB or final recovery gives `RECOVERY_BACKEND_NOT_OPERATIONAL`.
No detector work follows any outcome.

Only if normal is operational may the already selected Walk1 low-redundancy
subset `[9524,15155,20276]` be materialized from the locked normal injection
recipe and tested with the same oracle support and methods. The subset, link,
window, seed and injection recipe may not be reselected. Existing clean-control
failure remains visible.

All estimator process trees have a 1800-second limit and are not retried after
algorithm failure. Outputs use a fresh isolated evidence root; historical
results and untracked `doc/v2/ie_0911/` remain untouched. Deliver
`doc/ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md`, machine-readable support,
Stage2/Rc/LCB/final/metric audits, commands and all failures. Update STATUS and
CLAIM_EVIDENCE, choose one required final classification, and stop. No commit or
push is authorized by this task.

After completion, the user separately authorized committing and pushing this
delivery. The protected untracked `doc/v2/ie_0911/` material remains excluded.

## Completion record

The normal oracle support mapped 30/30 locked planned observations to the sole
target link. The nonempty Stage 2 ran 50 conditional optimizer calls (110 LM
iterations and 359 inner trials) and did not take `SUCCESS_EMPTY`. Objective,
step and nonnegative KKT predicates passed at the terminal iterate, but the
navigation stationarity metric remained `0.0750243858624` against the unchanged
`1e-6` tolerance plus `2.2355735248e-11` roundoff allowance. Stage 2 returned
`MAX_REFIT_ITERATIONS` and exported no valid Values or `c_hat`; no cache, `R_c`,
sigma, correction or candidate final could follow. There was no algorithm retry
or tolerance/policy change. The required classification is
`RECOVERY_BACKEND_NOT_OPERATIONAL`; the low-redundancy check is
`NOT_RUN_NORMAL_BACKEND_NOT_OPERATIONAL`. See the
[final result](../ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md).
