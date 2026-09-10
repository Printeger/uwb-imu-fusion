# T10-A19-R06 complete live-C output and unblinded development protocol

Registered UTC: `2026-09-10T09:25:30Z`. This registration precedes every R06
source change, engineering execution, new estimator ticket, estimator run, and
new evaluation-truth access. The dirty worktree, all A19--R05 artifacts, the
R05 partial `final_values.csv`, failures, tickets, process records, and
`EVALUATION_TRUTH_BOUNDARY_INCIDENT.json` remain immutable.

## Scope and evidence label

R06 keeps the exact shared schema
`A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`, role `development`, provider
`development_nonconsumable`, default-off behavior, and `consumable=false`.
It changes only the Stage-2 output bundle and inexact-handoff audit. No solver
threshold, lambda rule, 333-bit certificate precision, native direction or
retract, handoff guard, accepted Values, IMU model, initialization,
segmentation, score definition, gate workpoint, or scientific budget may
change.

The R05 truth incident is permanent: this seed has already exposed its
generation manifest and part of its bias truth to the implementing agent.
Every downstream result in R06 is therefore labelled
`UNBLINDED_DEVELOPMENT`. A new ticket does not restore blindness. The estimator
still must open zero GT/truth/oracle inputs. This run cannot be called formal
validation, blind evaluation, eta generalization evidence, or support for
C1--C3.

## Output and audit correctness gates

The production Stage-2 writer must support the complete actual Values domain:
X/Pose3, V/Vector3, B/ConstantBias, and C/double. It records stable key, type,
dimension, coordinate, hexadecimal binary64 text, and raw 64-bit pattern. It
must reject missing keys, wrong types, unsupported values, non-finite values,
and I/O failures. A production reader must reconstruct every key and compare
type, dimension, and every binary64 component. The graph/Values content
identity must match before and after the round trip.

The writer prevalidates the graph/Values key set and the complete factor
metadata, writes temporary files, verifies their content, and publishes the
success manifest last by atomic rename. A failed export remains explicitly
incomplete and cannot masquerade as a consumable `final_values`. Runtime state
distinguishes `solver_converged`, `export_complete`, and `score_complete`.

Inexact-handoff JSON must be strict: unavailable guard numbers are JSON null
with an explicit `guard_evaluated` state/reason; a genuinely evaluated
non-finite value is an error. `inner_converged` comes from the actual
`CheckedLmResult`; `qualified`, guard evaluation, and handoff status remain
separate. This serialization may not change guard decisions or optimization.

Before a science ticket, one real C++/GTSAM fixture containing nonzero C, a
zero-boundary C, and multiple segments must invoke the same production
writer/reader used by the runner, compare X/V/B/C bit-for-bit, verify graph and
Values identity, and re-audit objective/KKT/gradient on the same graph. It must
exercise missing key, wrong type, unsupported value, non-finite value, and
write-failure rejection. A strict parser checks inner-converged, qualified,
guard-rejected, and guard-not-evaluated audit examples against block status.
The same fixture result must then follow production export -> score -> decision
-> final -> independent fixture evaluation. Only affected build/tests and the
still-applicable R05 identity/startup evidence are rerun.

## One fresh complete scientific run and frozen comparison

After all engineering gates pass, exactly one new ticket permits a fresh
`A10_FROZEN_P1_STEP_SEED10101` run from complete raw input and original config,
with `ORIGINAL_RAW_NO_CHECKPOINT`. Stage 1 has at most 500 outer iterations and
50 iterations per conditional block; Stage 2 has at most 200 outer iterations;
the automatic process tree has a hard 900-second limit. The first failure after
algorithm entry stops its dependent branch with no retry or parameter change.

Only a converged Stage 2 with a verified complete export enters scoring. If at
least one group is eligible, freeze all decisions and run exactly the five
preregistered policies: `suppress_all`, `structured_debias`, `fit_only`
(`gamma<=1`), `s_fit` (`s<=0.10 m`, `gamma<=1`), and `full_gate`
(`eta>=0.10`, `s<=0.10 m`, `gamma<=1`). Each final has one 900-second process
tree budget including at most the existing contractual fallback; finals total
4500 seconds and automatic plus finals total 5400 seconds. Equivalent results
may be reused only under the full contract and are labelled `SAME_DECISION`.

All final graphs use original ranges and live accepted C states, with no
corrected pseudo-range. Every trajectory, bias, residual, covariance, and audit
comes from the same final graph/Values. Failure, fallback, unavailable score,
and zero acceptance remain explicit.

Only after all runnable decisions and finals and their method/code/config
hashes are frozen may the independent evaluator read evaluation-only truth.
It records that access time and the frozen manifest, and reports full-record
and `[3,6] s` raw-frame ATE RMSE/P95/matches and deltas from `suppress_all`,
decision/final bias-field error, accepted bias RMSE, bad-correction rate at
`epsilon_bad=0.20 m`, good-correction rejection, candidate/eligible coverage,
overall retained fraction, failures, fallback, timing, and cost. Completed
zero acceptance has risk `UNDEFINED`; not-run evaluation is
`UNAVAILABLE_NOT_RUN`. Thresholds are never selected from truth and no sweep,
new scenario, new seed, or held-out study is authorized here.
