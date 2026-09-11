# PL conditional RAIM/FDE locked Walk1 preflight result

Date: 2026-09-12

Verdict: **`PL_CONDITIONAL_PREFLIGHT_FAIL_MISSED_AFFECTED_GROUP_ALARM`**

Production detector: **`NOT_RUN_PREFLIGHT_FAILED`**

Backend: **`NOT_RUN_PREFLIGHT_FAILED`**

Localization: **`NOT_EVALUABLE`**

This is a development preflight failure, not an E2E localization result and not a formal integrity claim.

## Locked inputs and process boundary

- IE start commit: `d82d794e79f2d2550d6815ed687dff170af0e593`.
- PL source commit: `ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`.
- Frozen recovery reference: `cf287b4fec9bc5689317f302ccf4cdd927bfba81`.
- Locked manifest SHA-256: `1660f7a1e8d1bd32bfabda394adf9e69675fb849917b028b66fff7ec0e87e820`.
- Walk1 target: tag/anchor `27956:20276`, closed interval
  `[1664959678.3077347,1664959686.3077347]`, constant `+0.5 m`, seed 911, 30 planned IDs.
- Evidence root:
  `/home/mint/ws_fusion_uwb/res/pl_conditional_raim_preflight_20260912_01`.
- Reserved E2E root remained empty:
  `/home/mint/ws_fusion_uwb/res/pl_conditional_raim_e2e_20260912_01`.

The clean and injected detector runs were separate processes with the truth, general data, and prior truth roots
hidden by tmpfs mounts. Neither runner accepts a truth argument. Their file-open traces contain zero successful
truth/oracle/GT opens. Detector artifacts were sealed first; only then did the independent evaluator open the
injection truth. The detector seal is
`sha256:f709379166fb3b67556ad196e8d0b57d0c17d9dbe2d79d4fa01a080abd8e1693`.
The final initializer was explicitly cropped at keyframe 4 time `1664959678.0012374`, using 85 IMU samples and
five UWB keyframes; its audit records `bootstrap_future_input_count=0`.

## Four serial gates

| Gate | Result | Locked observation |
|---|---|---|
| 1. clean support is zero | `PASS` | 0 retained segments |
| 2. at least one affected-group alarm | `FAIL` | 30/30 IDs covered in 30 groups; 0 alarms |
| 3. target unique isolation | `NOT_RUN_PREVIOUS_GATE_FAILED` | diagnostic count 0; not admitted as a gate |
| 4. persistent target overlap | `NOT_RUN_PREVIOUS_GATE_FAILED` | diagnostic count 0; not admitted as a gate |

Across the affected groups, the largest statistic was `4.860523050243609` at keyframe 33, below that group's
`30.856189940445919` threshold. The minimum threshold among the 30 affected groups was
`28.473255424015775`. No threshold, `p_fa`, temporal parameter, interval, amplitude, grouping, or bootstrap rule
was adjusted after observing this result.

For completeness, clean and injected each processed 224 post-bootstrap groups with zero omnibus alarms, zero unique
isolations, zero prior-degradation events, and zero retained segments. This single clean run does not establish a
calibrated false-alarm guarantee.

## Implementation and engineering evidence

The preflight-only delivery contains a source-neutral conditional core, deterministic LOAO isolation and temporal
partitioning, an independent locked-PL equation fixture, a truth-blind iSAM2/fixed-lag shadow runner, and a separate
truth evaluator. No config switch, production provider, cache admission, production replay, or six-input manifest
entry was added.

The independent fixture, which does not link the IE core, produced
`T=0.10726853294093962`, DoF 3, threshold `25.901749745671491`, PASS. The direct parity/core tests compare
physical and whitened innovations/Jacobians, physical and whitened innovation covariance, statistic, threshold,
DoF, and inclusive decision with tolerance `1e-12 + 1e-10*scale` and passed.

- Target build: exit 0.
- `test_pl_conditional_raim`: 8/8 tests, 0 failures.
- Reference fixture: exit 0.
- Sealed clean shadow: exit 0.
- Sealed injected shadow: exit 0.
- Independent evaluator: exit 2, the expected non-pass exit for the verdict above.
- Full package build/test count greater than the historical 384: `NOT_RUN_PREFLIGHT_FAILED`.
- Stage2/Rc/sigma/LCB/final/fallback/v4 production regressions: `NOT_RUN_PREFLIGHT_FAILED`.

Seven non-final bring-up/audit attempts are retained under `attempts/`: an initial fail-closed IMU-gap placeholder,
an unsupported `strace openat2` filter, a GTSAM `ExpressionFactor` derivative-vector assertion, and a full-input
initializer run discarded by the causality audit, followed by a run discarded because its new-state seed came from
the batch open-loop initialization rather than the current shadow estimate. The sixth pair was superseded after
making numerical-invalid groups explicitly record a prior-degradation event; the seventh was an output-directory
precreation launch rejected by the runner's overwrite guard. The first was
corrected to the locked PL controlled-reinitialization behavior; Walk1 has one `0.021364450454711914 s` gap at
keyframe 110, after the target interval. The third was corrected by taking the actual factor residual and a central
`Pose3::retract` numerical derivative for its six pose columns. No scientific gate parameter changed.

## Eight requested answers

1. **Was the locked PL detector extracted?** Yes. The source snapshot and hashes are sealed; the core uses the
   locked conditional statistic, DoF, `p_fa=1e-5`, and inclusive PASS comparison.
2. **Does the conditional prior exclude current UWB?** Yes. Every audited group had zero intersection between
   committed IDs and current IDs; detection occurs before current-group commit.
3. **Was the preflight causal and truth-blind?** Yes for the detector processes: keyframes 0--4 are bootstrap,
   detection begins at keyframe 5, file-open forbidden count is zero, and truth was read only by the post-seal evaluator.
4. **Did clean pass?** Yes, narrowly under the preregistered criterion: retained support was zero. The run also had
   zero raw alarms and zero degradation events, but is not called a general clean guarantee.
5. **Did injected alarm on the target interval?** No. All 30 planned affected IDs were evaluated, but none of their
   groups alarmed; this is the stopping failure.
6. **Was the target uniquely isolated and retained?** Not evaluated as serial gates after gate 2 failed; diagnostic
   counts were zero for both target unique isolation and retained overlap.
7. **Were production, recovery backend, and localization evaluated?** No. Provider registration, cache plumbing,
   Stage2 and all six methods are `NOT_RUN_PREFLIGHT_FAILED`; localization benefit is `NOT_EVALUABLE`.
8. **Did this change frozen science or claims?** No. The four recovery backend files differ by zero from `cf287b4`;
   paper structure and roadmap hashes remain frozen. T10=C2-C, T11=C, and C1--C3 do not change.

## Evidence index

- `preflight_evaluation.json` (`sha256:195fc47410e3471ee706fdf3408072a09701668e7317c7439da09fe6778b8bd3`)
- `sealed_detector_hashes.sha256`
- `clean/` and `injected/`: status, epochs, isolation, candidate, prior-audit, and support artifacts
- `clean_file_access.trace`, `injected_file_access.trace`
- `pl_locked_source/`, `pl_locked_source_hashes.sha256`
- `pl_reference_fixture.json` (`sha256:5e76d4ab9cdd429ef2f0a5e85d8885fbff34f15eafade08f7825dc599e0df689`)
- `test_pl_conditional_raim.log` (`sha256:3525356005a927232c68506b5b69a80a03a8118bb0cddcb971e2683a197bfcf8`)
- `attempts/`: retained non-sealed bring-up failures

There is intentionally no `PL_CONDITIONAL_RAIM_E2E_RESULT.md`, because production admission was never reached.
