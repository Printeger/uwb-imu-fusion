# 0911 Stage2 stationarity forensic and correctness repair

Status: `COMPLETE / RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX / LOCALIZATION_BENEFIT_OBSERVED`.
Authorized by the user on 2026-09-11. The sole code
baseline is actual HEAD `3db4f3175fc647ac5a24e3de771e7e41ceddf4e7`; the
sole experimental baseline is
[`ORACLE_SUPPORT_BACKEND_RESULT.md`](../ie_0911/ORACLE_SUPPORT_BACKEND_RESULT.md)
and its immutable Walk1 normal-redundancy oracle artifacts. The locked input is
seed 911, link `27956:20276`, closed 8 s constant `+0.5 m` injection and the same
30 planned oracle-support observations.

This task diagnoses why the nonempty Stage2 reaches objective, scaled-step and
nonnegative-`C` KKT predicates but retains navigation stationarity
`0.0750243858624` against the unchanged `1e-6` tolerance. Before changing the
solver, use the production `AuditNavigationStationarity` definition to compare
clean and injected raw-reference final Values with the authoritative Stage2
terminal Values; identify the dominant key/native coordinate/category and its
factorwise absolute-gradient sum/roundoff allowance; and use the existing
finite-difference diagnostic implementation on that coordinate and additional
navigation coordinates.

Instrument development-only Stage2 diagnostics to preserve, for at least the
last 15 outer iterations, the conditional fixed-old-`C` state immediately
before the exact amplitude update and the joint state afterward: `C`, objective,
navigation gradient/dominant coordinate, conditional LM status/iterations/
lambda/step, amplitude gradient/KKT and full scaled step. Continue the same
terminal process diagnostically to outer 100 and 200 with all acceptance
criteria unchanged. These diagnostic results are nonconsumable: they publish no
cache, `c_hat`, score, `R_c` or final result.

Freeze FDE/detector, oracle support, injection, Stage2 objective/model/factors,
UWB and IMU models, `R_c`, local sigma, `kappa=2`, LCB, final factor semantics,
GT evaluator and every existing objective/step/KKT/stationarity tolerance. Do
not accept `PAPER_STAGE2_INEXACT_HANDOFF_V1` as convergence and do not raise an
iteration budget merely because a longer diagnostic becomes smaller.

After evidence is sealed, select exactly one repair: D1 audit/derivative
correctness; D2 existing conditional LM numerical convergence behavior if the
fixed-`C` subproblem itself is nonstationary; or D3 one joint active-set polish
only if the fixed-`C` solve is stationary, the exact `C` update reintroduces the
gradient, and the outer loop plateaus. A D3 polish must optimize the unchanged
joint graph from the alternating terminal Values, retain positive interior `C`
as a free scalar, fix attempted negative `C` at zero without a prior, repeat to
a stable active set, and accept only finite/key-exact/non-increasing objective,
navigation stationarity, interior amplitude gradient, boundary KKT and
nonnegative amplitude checks. Success/failure reasons are
`JOINT_ACTIVE_SET_POLISH_CONVERGED` and
`JOINT_ACTIVE_SET_POLISH_FAILED:<reason>`.

Add analytic coupled toy, interior, boundary, graph-identity and affected
regression tests; run full CTest/GTest. Then rerun exactly one locked normal
oracle batch. Only a valid Stage2 result may proceed through production
`c_hat -> R_c -> sigma -> LCB/full -> suppress/structured/full/LCB final` and the
unchanged evaluator. Truth amplitude remains evaluator-only. The final result
must select exactly `RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX`, with a
separate localization-benefit observation, or
`RECOVERY_BACKEND_NOT_OPERATIONAL_FINAL`. Stop without a new phase, detector
work, parameter tuning, commit or push.

## Sealed diagnosis and selected repair (before production solver change)

The independent pre-fix run is
`/home/mint/ws_fusion_uwb/res/stage2_stationarity_repair_20260911_01/forensics_sealed_v2`.
Its clean raw, injected raw and oracle Stage2 terminal gradients are respectively
`0.13528201598437639`, `3.9520210700762921` and
`0.075024385862432652`, so the strict Stage2 audit is inconsistent with the
base solver's generic success floor (`STATIONARITY_CONTRACT_INCONSISTENT_WITH_BASE_SOLVER`).
The Stage2 dominant coordinate is translation coordinate 4 of `x186`; its
native/scaled gradient is `0.075024385862432652`, physical scale is one,
absolute factor-gradient sum is `9.5174948299773163`, and coordinate roundoff
allowance is `1.1039922973978534e-11`. Central finite differences from `1e-2`
through `3e-7` agree at the declared diagnostic tolerance; sampled navigation
coordinates agree at their numerically meaningful steps. Therefore D1 is
rejected.

For outer iterations 34--50, and unchanged continuation through 100 and 200,
`C_before=C_after=0.46875451327633244`, `delta_C=0`, and the before/after
navigation gradient is identically `0.075024385862432652`. The conditional LM
returns with zero accepted updates while the fixed-C gradient remains real.
Thus `BLOCK_COORDINATE_COUPLING_CONFIRMED` is rejected and D3 is forbidden;
the continuation is `BLOCK_COORDINATE_COUPLING_PLATEAU` only as an observed
outer-loop outcome caused upstream by the conditional solver.

The unique diagnosis is D2. Existing V2, with all objective/model/tolerances
unchanged, reduces the terminal fixed-C gradient from `0.0750243858624` to
`7.1399474678290886e-4` at 100 updates, `9.1811478379624778e-5` at 200, and
passes at update 340 with `9.6541993599430498e-7`. In the V2 alternating probe,
outer 2 reaches lambda upper bound at `1.0306268252158191e-6`; restarting the
same V2 on the exact same fixed-C graph and last accepted Values, with the
unchanged default lambda initialization, passes after six accepted updates at
`7.4037526535952747e-7`. This is direct evidence for the already existing
fixed-checkpoint recovery behavior.

The selected minimal repair is therefore a bounded checked-LM wrapper used only
when the preceding outer iteration has passed objective, step and C-KKT while
failing navigation stationarity. It applies to the nonempty production Stage2
conditional solve and the common no-C/fixed-offset production refit. It uses the existing
`GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2`; after only
`CONDITIONAL_LM_STATIONARITY_NOT_REACHED` or
`CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`, or the V2 block's existing
`CONDITIONAL_LM_MAX_ITERATIONS`, it recreates that same optimizer on
the identical fixed-C graph and exact last accepted Values. Lambda resets to
the same linked-GTSAM default; objective, graph, Values keys, scales, all
tolerances, linear solver and LM parameters are unchanged. A checkpoint is
never handed to the C update. Success requires the ordinary V2 generic AND
navigation-stationarity result. Recovery is bounded by the existing Stage2
outer cap and by the pre-existing total nominal conditional-call budget
`max_refit_iterations * lm_max_iterations`; budget exhaustion is explicit.
No inexact-handoff state is accepted. The policy and recovery limits enter the
Stage2 cache identity. D1 and D3 are not combined with this repair.

The authoritative rerun and evaluation are documented in
[`STAGE2_STATIONARITY_REPAIR_RESULT.md`](../ie_0911/STAGE2_STATIONARITY_REPAIR_RESULT.md).
The unchanged joint contract converged at outer 42 with navigation gradient
`7.10213015446548e-7`, `c_hat=0.4713457858280814 m`, finite
`sigma_c=0.07507370718106174 m`, and LCB correction
`0.3211983714659579 m`. All four recovery finals converged without fallback.
LCB RMSE `0.1700862485306654 m` is below suppress RMSE
`0.1816368784331689 m` by `0.0115506299025035 m` (`6.3592%`). This is a
single locked oracle-backend development observation, not detector E2E evidence;
T10/T11 and C1--C3 remain unchanged.

After task completion, the user separately authorized committing and pushing
this delivery to the current Git branch. The protected untracked
`doc/v2/ie_0911/` material remains outside that delivery.
