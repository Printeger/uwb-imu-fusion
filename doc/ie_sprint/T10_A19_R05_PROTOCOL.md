# T10-A19-R05 shared development identity and complete development protocol

Registered UTC: `2026-09-10T08:32:23Z`. This protocol precedes every R05
source change, engineering execution, scientific ticket, estimator execution,
and evaluation-truth read. The dirty worktree and every A19--R04 artifact,
failure record, ticket, process directory, and result remain immutable.

## Scope and fixed identity contract

R05 fixes only the demonstrated Stage-1 development identity handshake. The
unchanged payload keeps the exact schema
`A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`; R05 does not create another
schema. The runner and producer shall share the schema, role
`development`, provider `development_nonconsumable`, Stage-1 policy
`PAPER_CERTIFIED_PAIR_REDUCTION_V1`, implementation/context identity equality,
required callback, and conditional policy validation. Outputs remain
default-off, `consumable=false`, and rejected by formal cache readers.
Implementation, executable, loaded-library, run, ticket, and output identities
shall describe the new build truthfully.

No solver threshold, lambda rule, 333-bit certificate precision, native
direction/retract, R03 inexact handoff guard, IMU covariance model,
initialization, segmentation, score definition, gate working point, or science
budget may change.

## Engineering admission gates

Before a science ticket, the CMake-built runner shall load the complete frozen
raw input, construct its graph and Values, construct the exact request used by
the science mode through the shared factory, and invoke the same producer
contract validator. Prepare-only must record acceptance and zero Stage-1 and
conditional optimizer calls. Negative checks cover unknown schema, wrong
role/provider/policy, implementation/context mismatch, missing callback, and
formal-reader rejection, with a nonzero executed-test count.

A small real C++/GTSAM engineering input must pass the direct chain using the
same request/identity construction path: Stage-1 producer, actual immutable
partition/context, Stage-2, scoring, frozen decision, final inference, and an
independent evaluator consuming the emitted artifacts and fixture-only truth.
Every boundary records schema, provider, policy, solver identity, and parent
relations. These artifacts remain engineering-only and cannot substitute for
the complete scientific input.

Only checks affected by the identity/source/library change are rerun. R04's
preserved crash stack and ABI diagnosis remain historical evidence; current
build identity and direct handshakes must be verified again where changed.

## One fresh complete development run

After every gate passes, exactly one newly issued ticket permits a fresh run of
the frozen A10 P1 step seed10101 complete raw input and original initialization.
It must not read a checkpoint, oracle support, GT, or evaluation truth. Stage-1
has at most 500 outer iterations and 50 iterations per conditional block;
Stage-2 has at most 200 outer iterations. The automatic Stage-1 -> Stage-2 ->
score process tree has a hard 900-second limit.

The first failure after algorithm entry is preserved and stops its dependent
scientific branch, with no retry and no parameter, identity, tolerance,
precision, or budget adjustment. An unknown execution stage is not treated as
zero calls.

## Fixed decisions, finals, and evaluation

Only if Stage-2's unchanged objective/step/KKT/navigation four-way AND passes
and at least one score is eligible, all decision-time artifacts are frozen
before final inference. The five preregistered policies are:

1. `suppress_all`;
2. `structured_debias`;
3. `fit_only`, with `gamma <= 1`;
4. `s_fit`, with `s <= 0.10 m` and `gamma <= 1`;
5. `full_gate`, with `eta >= 0.10`, `s <= 0.10 m`, and `gamma <= 1`.

Each final process tree has a hard 900-second limit and at most the existing
single contractual fallback. The five-final budget is 4500 seconds and the
automatic-plus-final science budget is 5400 seconds. Equivalent frozen
decisions may be reused only under the contract's full equivalence conditions
and must be marked `SAME_DECISION`. Final graphs use original ranges and live
accepted `c_s`; no corrected pseudo-range is added. Each trajectory, bias,
residual, and covariance comes from its own single final graph/Values.

After every runnable decision/final artifact is frozen, an independent
evaluator may read evaluation-only synthetic truth. It reports full-record and
predeclared `[3,6] s` raw-frame ATE RMSE/P95/match count and deltas from
`suppress_all`; decision-time and final-time bias-field error; accepted bias
RMSE; bad-correction rate using `epsilon_bad=0.20 m`; good-correction rejection;
candidate and eligible coverage; overall retained fraction; failures,
fallbacks, stage time, and total cost. Zero acceptance has accepted risk
`UNDEFINED`. Unavailable values remain `UNAVAILABLE`. This is one development
record: it does not lock a formal gate, establish generalization or statistical
significance, prove an eta increment when decisions coincide, prove LOS
non-degradation, or upgrade C1--C3.
