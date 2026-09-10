# T10-A19-R08 limited synthetic validation results

This record contains the fixed P1, fixed-work-point matrix registered in `T10_A19_R08_PROTOCOL.md`. It is limited synthetic validation characterization on two base trajectories. It does not lock a gate, use test reservations, establish significance, or upgrade C1--C3.

## Matrix completion

| Base | Scenario | Outcome | Candidates / segments | Groups eligible/unavailable | Final outcome |
|---|---|---|---:|---:|---|
| turn01/20101 | LOS | FAILED | 0 / 0 | 0/0 | original no-C dispatch defect; no score/final; no retry |
| turn01/20101 | step1 | SCORED | 16 / 1 | 1/0 | 5/5 valid |
| turn01/20101 | step2 | SCORED | 32 / 2 | 1/0 | 5/5 valid; wrapper interrupted after terminal output |
| turn01/20101 | step3 | SCORED | 48 / 3 | 1/0 | 5/5 valid |
| turn01/20101 | ramp_gentle | SCORED | 32 / 2 | 1/0 | structured valid; four suppressing finals failed after their single fallback |
| turn01/20101 | ramp_steep | SCORED | 32 / 2 | 1/0 | 5/5 valid |
| turn02/20102 | LOS | FAILED | 0 / 0 | 0/0 | support/refit coverage mismatch after Stage2; no score/final; no retry |
| turn02/20102 | step1 | SCORED | 16 / 1 | 1/0 | 5/5 valid |
| turn02/20102 | step2 | SCORED | 32 / 2 | 1/0 | 5/5 valid |
| turn02/20102 | step3 | SCORED | 48 / 3 | 1/0 | 5/5 valid |
| turn02/20102 | ramp_gentle | INTERRUPTED | unavailable | unavailable | system crash after ticket consumption; no retry |
| turn02/20102 | ramp_steep | FAILED | 32 / 4 (1 short) | 0/0 | conditional range metadata mismatch in Stage2; no score/final; no retry |

Eight of twelve cells completed automatic Stage1 -> Stage2 -> scoring and attempted all five finals. Three cells retained their first algorithm failure and one retained the system interruption. All twelve estimator file-open traces contain zero evaluation-truth opens.

## Frozen group scores and gate decisions

| Base | Scenario | eta | s (m) | max gamma | eta/s/gamma pass | fit_only | s_fit | full_gate |
|---|---|---:|---:|---:|---|---|---|---|
| turn01 | step1 | 0.739456 | 0.014536 | 0.940884 | 1/1/1 | Use | Use | Use |
| turn01 | step2 | 0.585940 | 0.016330 | 0.940734 | 1/1/1 | Use | Use | Use |
| turn01 | step3 | 0.584180 | 0.016354 | 0.941327 | 1/1/1 | Use | Use | Use |
| turn01 | ramp_gentle | 0.586077 | 0.016328 | 2.100073 | 1/1/0 | Suppress | Suppress | Suppress |
| turn01 | ramp_steep | 0.586408 | 0.016323 | 15.908708 | 1/1/0 | Suppress | Suppress | Suppress |
| turn02 | step1 | 0.748218 | 0.014451 | 1.863506 | 1/1/0 | Suppress | Suppress | Suppress |
| turn02 | step2 | 0.582791 | 0.016374 | 1.865205 | 1/1/0 | Suppress | Suppress | Suppress |
| turn02 | step3 | 0.582488 | 0.016378 | 1.864812 | 1/1/0 | Suppress | Suppress | Suppress |

The three gates produced identical Use/Suppress vectors on every scored input. Their reason strings differ on Use rows because full_gate names all three predicates while fit_only/s_fit name their required subsets; that is not a decision difference. This matrix therefore does not distinguish full_gate from s_fit or fit_only.

## Historical [3,6] s paired trajectory result

Values are raw-frame ATE RMSE/P95 in metres. Delta is structured_debias minus suppress_all; positive is worse.

| Base | Scenario | suppress_all RMSE/P95 | structured RMSE/P95 | structured delta RMSE/P95 | gate decision and delta |
|---|---|---|---|---|---|
| turn01 | step1 | 0.007941890 / 0.012582909 | 0.008395210 / 0.013161652 | +0.000453320 / +0.000578743 | Use; same as structured |
| turn01 | step2 | 0.008725342 / 0.013711892 | 0.009471582 / 0.014577266 | +0.000746240 / +0.000865374 | Use; same as structured |
| turn01 | step3 | 0.009535727 / 0.014740264 | 0.010088651 / 0.015294114 | +0.000552924 / +0.000553851 | Use; same as structured |
| turn01 | ramp_gentle | UNAVAILABLE | 0.012751905 / 0.018657076 | UNAVAILABLE | Suppress; recovery and fallback failed |
| turn01 | ramp_steep | 0.008725342 / 0.013711892 | 0.036908749 / 0.057470598 | +0.028183407 / +0.043758707 | Suppress; 0 / 0 delta |
| turn02 | step1 | 0.020860165 / 0.026389773 | 0.020640761 / 0.026646567 | -0.000219404 / +0.000256794 | Suppress; 0 / 0 delta |
| turn02 | step2 | 0.019090101 / 0.024270043 | 0.019410106 / 0.025679921 | +0.000320005 / +0.001409877 | Suppress; 0 / 0 delta |
| turn02 | step3 | 0.019049380 / 0.024564727 | 0.019164702 / 0.025354058 | +0.000115322 / +0.000789331 | Suppress; 0 / 0 delta |

Among seven comparable structured_debias cells, historical RMSE improved in one and worsened in six; P95 worsened in all seven. The single RMSE improvement on turn02/step1 was 0.219 mm while its P95 worsened by 0.257 mm. Retaining the ramp_steep corrections on turn01 caused the largest degradation: +28.183 mm RMSE and +43.759 mm P95, with 7/16 accepted observations over the 0.20 m bad-correction threshold (rate 0.4375). The step accepted sets had zero bad corrections, yet most still slightly worsened trajectory error; bias-field accuracy is therefore not labeled trajectory benefit.

Zero-accepted policies have `accepted_bias_rmse` and bad-correction risk `UNDEFINED`. Four suppressing finals in turn01/ramp_gentle each used their single allowed fallback and failed; no value is imputed. Full-record metrics, 41/16 match counts, final-time bias metrics, all coverage denominators, final costs, and fallback records are in `PAIRED_RESULTS.csv` and the per-cell evaluator JSON.

## LOS result

No LOS cell reached a valid final graph, so LOS cost relative to all_range is `UNAVAILABLE`. The registered +0.05 m RMSE/+0.10 m P95 tolerance was not evaluated and cannot be described as passed. This is the main missing evidence from the matrix.

The Stage2 production export manifest still carries its conservative legacy `evaluation_label=UNBLINDED_DEVELOPMENT`; the role-bearing diagnostic manifest, decision manifest, request validation, and final content identities are validation-context bound and all products remain `consumable=false`. This legacy label is disclosed rather than relabeled after the run.
