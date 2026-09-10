# T10-A19-R04 crash root-cause and complete development protocol

Registered UTC: `2026-09-10T07:28:15Z`. This protocol precedes every R04
diagnostic execution, rebuild, scientific ticket, and evaluation-truth read.
R03 and all earlier dirty-worktree artifacts remain immutable.

## Scope and diagnostic budget

R04 first preserves the exact R03 runner, loaded core/GTSAM/MPFR/GMP
identities, build IDs, maps, core compile/link flags, and recorded R03 argv.
It then permits one original-binary GDB run, hard limited to 60 seconds, with
SIGSEGV stop, all-thread backtrace, registers/instruction/maps and a breakpoint
at the resolved Stage1 entry. Reaching Stage1 stops the diagnostic. Only if
this stack is insufficient may one targeted symbol/ASan or explicit
stage-marker run be made, hard limited to 120 seconds. These diagnostic runs
cannot be relabeled as the scientific pilot.

Only the root cause supported by stack and ABI/lifetime evidence may be fixed.
No solver threshold, lambda rule, 333-bit precision, IMU model, initialization
mathematics, partition, scoring definition, input, or budget may change. If
the issue is build compatibility, the runner must use the project's existing
CMake target and imported dependency settings instead of another handwritten
compile/link recipe.

## Engineering gates

The final runner must expose an explicit prepare-only mode that executes the
complete frozen raw load, input plan, materialization, initializer, graph and
Values construction, and content-identity check, then emits an atomic stage
record with Stage1 and conditional optimizer call counts exactly zero. One
post-fix prepare-only run is allowed with a 120-second hard limit. Wrong argv
must fail closed, and the original fault must have a regression check.

Focused R03 guard/refit filters must execute a nonzero test count. The five
final policies must each have an independent process-tree 900-second limit,
retain completed results after another policy fails or times out, and share a
4500-second final budget; automatic processing retains its own 900-second
limit and total science retains 5400 seconds. Before a science ticket, one
same-build mixed-graph chain must write real score/decision/final artifacts
and an independent evaluator must consume those exact artifacts and
fixture-only truth. Engineering fixture costs remain separate from science.

## Scientific run and truth boundary

After every engineering gate passes, one new R04 ticket permits one fresh
P1-step seed10101 run from complete raw input and original initialization:
Stage1 outer <=500, each conditional block <=50, Stage2 outer <=200, automatic
Stage1->Stage2->score <=900 seconds. The first algorithm failure stops its
dependent branch without retry or parameter changes.

If Stage2 satisfies the unchanged joint objective/step/KKT/navigation AND and
at least one eligible score exists, all decisions are frozen before five
independent final runs: `suppress_all`, `structured_debias`,
`fit_only(gamma<=1)`, `s_fit(s<=0.10m,gamma<=1)`, and
`full_gate(eta>=0.10,s<=0.10m,gamma<=1)`. Each may use the existing single
fallback. Only after all runnable final outputs are frozen may an independent
evaluator read evaluation-only synthetic truth. It reports full and `[3,6]s`
raw-frame ATE RMSE/P95/matches, paired deltas from `suppress_all`, decision and
final bias errors, accepted RMSE, bad-correction rate, good rejection,
candidate/eligible/accepted coverage, retained fraction, failures, fallback,
and timing; `epsilon_bad=0.20m`. Zero accepted risk is `UNDEFINED`.

All R04 outputs remain development-only and `consumable=false`; no formal
validation/test gate or C1--C3 claim is upgraded.
