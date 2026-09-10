# T10-A19-R08 verification

Status: `LOCAL_DELIVERY_COMPLETE / LIMITED_SYNTHETIC_VALIDATION_MATRIX_EXECUTED_WITH_RECORDED_FAILURES / NO_GATE_LOCK`

R08 opened only the pre-registered validation envelope for `a10_val_turn_01/seed20101` and `a10_val_turn_02/seed20102`, P1 and six fixed scenarios. It preserved the legacy/default paths, all old A19/R01--R07 evidence, the permanent R05 truth event, and every new ticket/failure. It did not generate, read, or run test reservations 30101--30103 and did not return to seed10101.

## Implementation and engineering gate

The implementation parameterizes the existing synthetic generator, binds validation role/split/ancestry/content hashes through the real C++ producer, Stage2 request, decision manifest and final identity, adds a validation-only CMake runner, and evaluates each frozen scenario through its generation manifest. It reuses `AutomaticSupportProvider -> SegmentRefitter -> ScoreRefitRecoverability` and the existing final inference path; it does not add an estimator or alter solver thresholds, precision, lambda, gate values, or budgets.

Relevant current sources are `tools/paper/generate_synthetic_input.py`, `tools/paper/a19_r08_pipeline.cpp`, `tools/paper/a19_r08_prepare.py`, `tools/paper/a19_r08_run_matrix.py`, `tools/paper/a19_r08_evaluate.py`, `tools/paper/a19_r08_aggregate.py`, `tools/paper/a17_graph_io.h`, `include/uifgo/nlos_{discovery,refit,inference}.h`, `src/nlos_{discovery,refit,inference}.cpp`, `src/nlos_inference_io.cpp`, tests, and `CMakeLists.txt`. The exact runner source/binary used after the one shared no-C dispatch fix are frozen under `source_frozen_v2/` and `a19_r08_pipeline_v2`; current run-source hashes still match that snapshot. Runner SHA-256 is `b11f832830edb045825baf2e71978f0724607397d1661642beda488b6b5e8d60`.

The actual engineering gate passed:

- CMake ABI build exit 0.
- Focused discovery validation identity 2/2, refit 27/27, inference/mixed/zero/fallback 20/20.
- Real runner no-C recovery and controlled single fallback fixture passed.
- Cross-role, wrong parent and truth CLI negatives each exited 2.
- Both complete raw bases prepared through the production entry with 371 factors/123 values and zero optimizer calls.
- Generator scenario/obs_id semantics and legacy development invocation passed. The first legacy compatibility failure is preserved and was corrected before the matrix.

The first LOS science row exposed a shared dispatch defect: the empty automatic partition was sent to the nonempty-C handoff request. The ticket remains failed and was never retried. The repair selects the existing strict certified no-C request and forbids handoff; the real runner fixture and complete LOS raw prepare passed before the remaining unrun rows continued. See `ENGINEERING_GATE.json`, `ENGINEERING_GATE_V2.json`, `engineering/`, `source_frozen*/`, and the per-attempt `command.json`/ticket/log/resource files.

## Actual matrix and stopping boundaries

All 12 registered cells received exactly one ticket and began at most once. Eight completed Stage1, Stage2, scoring and all five final attempts. Three retained their first algorithm failure and one retained the system interruption:

- turn01 LOS: Stage1 `NO_CANDIDATES`, then the now-fixed dispatch failure; ticket not retried.
- turn02 LOS: Stage1 and strict no-C Stage2 converged, then score preparation rejected `support/refit segment coverage mismatch`; ordinary algorithm result, no tuning/retry.
- turn02 ramp_gentle: ticket consumed and the process reached final computation, then the host crashed; execution stage was not treated as pre-algorithm and it was not retried.
- turn02 ramp_steep: Stage1 converged, then Stage2 rejected `conditional range metadata does not match graph`; first algorithm failure retained, no tuning/retry.

Turn01/step2 reached terminal `SCORED` output and five valid finals before its outer orchestrator was interrupted; `RUN_RESULT_RECOVERED.json` records the existing terminal evidence and explicitly forbids retry. No result directory was deleted or reused. `SYSTEM_CRASH_RECOVERY_AUDIT.json` and `RUN_MATRIX_V5.json` show that post-crash continuation contained only the never-ticketed? no: correction: it contained only the never-ticketed turn02/ramp_steep cell.

The fixed matrix outcome, candidate/segment/group counts, Stage1/2 costs and exact failure reasons are in `INPUT_RESULTS.csv`. All eight scored rows have one eligible, numerically available group; no scored row contains a short or boundary segment. Turn02/ramp_steep's failed partition contains four segments/32 candidates including one short segment.

The exact timed/reconstructed scientific-process wall is about 736.061 s, engineering/generation/prepare gate cost is 138.350 s, and eight independent evaluator calls total 1.952 s, for 876.363 s of attributable work. The durable continuation used a deliberately conservative 1600 s carried-consumption value and ended at 1652.690 s before evaluation; either accounting is below the 10800 s scheduled limit and no part of the 3600 s reserve was automatically consumed. Per-cell outer timeout was 700 s, inside the registered per-input caps; every final retained its 900 s cap.

## Freeze and truth boundary

Before evaluation, `PRE_EVALUATION_FREEZE.json` froze 1320 present score/scoring/decision/final, partition/Stage2 identity, code, config, context and runtime payloads. `FINAL_CONSISTENCY.json` independently rehashed all 1320 with zero mismatch. `PRE_EVALUATION_FREEZE_SCOPE_CLARIFICATION.json` was written before truth opened and records that a scored input with failed finals remains evaluable with those finals reported `UNAVAILABLE`.

Estimator `strace` records across all 12 attempts contain zero evaluation-truth opens. The independent evaluator first opened truth at `2026-09-10T12:56:16.814205+00:00`, after all decisions/finals and per-cell freeze manifests. Eight evaluator processes exited 0. Generator source and generation-manifest metadata had necessarily been inspected during scenario/truth-separation engineering; no evaluation CSV payload was opened before the freeze. This disclosure is retained in `TRUTH_ACCESS_AUDIT.json`.

The Stage2 production export manifest retains its conservative legacy `evaluation_label=UNBLINDED_DEVELOPMENT`. The actual role-bearing diagnostic/request/decision identities and every final content identity include the validation context hash, and every product remains `consumable=false`. The old field is disclosed and was not relabeled after execution.

## Results

`RESULTS.md` gives the readable matrix. Machine-readable primary tables are `INPUT_RESULTS.csv`, `GROUP_RESULTS.csv` and `PAIRED_RESULTS.csv`; per-cell evaluation JSON retains full-record and historical RMSE/P95, match counts, decision-time/final-time bias fields, accepted risk, coverage, fallback and cost.

On all eight scored cells, fit_only, s_fit and full_gate made identical Use/Suppress decisions. Turn01 step1/2/3 passed eta, s and gamma and used 16/32/48 candidates. Both turn01 ramps and all three turn02 steps failed only gamma and suppressed all candidates. The current matrix therefore does not distinguish the three gate policies and provides no eta-increment comparison.

Among seven cells with both a valid suppress_all reference and structured_debias final, structured historical `[3,6] s` RMSE improved once and worsened six times; P95 worsened seven times. The only RMSE improvement was turn02/step1 by 0.219 mm while its P95 worsened 0.257 mm. The largest harm was turn01/ramp_steep: +28.183 mm RMSE and +43.759 mm P95. The three gate policies used corrections on turn01 step1/2/3 and were worse than suppress_all there by 0.453/0.746/0.553 mm RMSE; elsewhere their valid suppress decisions matched the reference. Turn01/ramp_gentle has a valid structured trajectory but all four suppressing finals exhausted their one fallback, so paired benefit is unavailable.

No LOS cell produced a valid final, so LOS cost versus all_range is `UNAVAILABLE`; the registered 0.05 m RMSE/0.10 m P95 tolerance was not tested. Zero-accepted accepted-risk is `UNDEFINED`, not zero. Bias error is never translated into trajectory harm/benefit. These two bases do not establish generalization, statistical significance, gate safety, eta value, or LOS non-degradation.

## Outcome and next blocker

R08 reached the requested real matrix rather than stopping at engineering checks. Its evidence does not support locking any gate or upgrading C1--C3. The next single blocking item is a validation-correct zero-candidate/LOS path that preserves empty support consistently through Stage2 score and all_range final, followed by the already specified finite cache selection and held-out test; this R08 run does not authorize rerunning these validation tickets or revisiting seed10101.
