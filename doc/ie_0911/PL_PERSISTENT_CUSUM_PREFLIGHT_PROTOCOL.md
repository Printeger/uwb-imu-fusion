# PL per-anchor persistent CUSUM production-admission preflight protocol

Status: `LOCKED_BEFORE_SPLIT_STATISTICS / IN_PROGRESS`.

Date: 2026-09-12. Start HEAD:
`af0394a1b03edddd3e94d45a24c0c8d9a4a9c6b1`. Locked PL source:
`ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`. Frozen conditional-signal evidence root:
`/home/mint/ws_fusion_uwb/res/pl_threshold_signal_audit_20260912_01`; its statistics seal SHA-256 is
`df78c6594034255c6e8c05de4f415ee65e3f96afe3d4b01f6c9e768b275addfb`.

## 1. Scope and immutable science

This is a single development admission task. The only detector signal is the valid per-observation PL Gaussian
`conditional_z`, with physical innovation `nu=z-h-beta`; positive excess range therefore has positive
`conditional_z`. The detector state is independent for each `(tag_id,anchor_id)` and obeys exactly

```text
G_0 = 0
G_k = max(0, G_{k-1} + conditional_z_k - 0.5).
```

`kappa=0.5` is a fixed development CUSUM reference value aimed at an approximately +1-sigma persistent positive
mean shift. It is a detector design choice, not a formal integrity calibration. It shall not change in this task.
Group statistic, chi-square threshold/DoF, omnibus PASS/alarm, LOAO isolation, RSSI, truth, GT, oracle labels,
injection interval/amplitude, and future observations are absent from the decision input. The old group detector is
retained as historical/diagnostic code and is not deleted or retuned.

This task adds no production provider or `nlos.mode`, changes no estimator decisions/weights/commit/cache, and runs
no Stage2, Rc, final inference, ATE, or RMSE experiment. Passing only permits a separately authorized production
integration task.

## 2. Source-neutral CUSUM lifecycle

Input rows contain timestamp, group/keyframe/source order, tag/anchor/obs identity, `conditional_z`, and diagnostic
validity. Input source order is checked per link before global stable sorting by
`(timestamp,keyframe_id,source_order,obs_id)`; reverse time fails closed. An invalid/nonfinite row cannot alarm,
sets that link's `G=0`, clears an unfinished excursion, and records `NUMERICAL_INVALID_RESET`. For consecutive valid
rows of one link, `delta_t>1.0s` first resets state and records `TIME_GAP_RESET`; equality does not reset.

Threshold crossing is inclusive, `G_k>=h_locked`. An excursion begins on a `0 -> positive` transition. When the
excursion first crosses, its same-link rows since the last zero are retrospectively candidates; later rows remain
candidates while `G>0`. A row returning `G` to zero is not a candidate and ends the segment. Gap/invalid reset also
ends it. An excursion that never crosses has no candidates. `estimated_onset_time` is the last-zero change-point
estimate and is distinct from first alarm time. No truth backfill occurs.

Detector identity binds algorithm `UIFGO_PL_PERSISTENT_CUSUM_V1`, signal definition
`PL_GAUSSIAN_CONDITIONAL_Z_V1`, binary64 `kappa/h`, gap/reset semantics, inclusive comparison, last-zero backfill,
termination rule, calibration artifact hash, and split-manifest hash. Frozen and dynamic paths use the same core and
identity.

## 3. Clean split and calibration order

Before any CUSUM statistic, Process A reads only the sealed clean conditional CSV. Let valid-row extrema be
`t_min,t_max`, duration `d=t_max-t_min`, and `t_split=t_min+0.60d`. The frozen partitions are

```text
calibration: timestamp <= t_split - 0.5 s
validation:  timestamp >= t_split + 0.5 s
guard:       the open interval between those boundaries, excluded.
```

Process A writes `clean_split_manifest.json` with input SHA-256, bounds, counts, and per-link counts, then its
SHA-256 is sealed. Process B may then read only the clean CSV plus the sealed split and computes unthresholded CUSUM
on calibration rows with independent per-link zero states. It defines, without rounding or override,

```text
G_calibration_max = max G over valid calibration rows and links
h_locked = max(5.0, G_calibration_max + 1.0).
```

`cusum_calibration.json` binds counts, source/current commits, split/input hashes, rule, `kappa`, maximum and
threshold, then is sealed. Process B may not read validation statistics, injected input, truth, target identity, or
injection metadata. Only after both seals may Process C start held-out clean validation with all link states reset to
zero and Process D run the frozen injected signal from zero. Processes A--D accept no truth/GT/oracle/target/injection
arguments and must have zero successful forbidden opens in separate file traces.

## 4. Frozen outputs and gates

Each detector run writes deterministic trace, candidate, support/compatibility partition, and status artifacts.
Status records `group_statistic_used_for_decision=false`, `truth_access_count=0`, `production=false`, identity,
hashes, counts, maxima, links, segments, and resets. After outputs are sealed, Process E alone reads locked truth and
evaluates target `27956:20276`, interval `[1664959678.3077347,1664959686.3077347]`, and the fixed 30 affected IDs.

- **F0 Integrity:** old input seal and schemas valid; split sealed before calibration; calibration sealed before
  validation/injected; exact `kappa/rule`; roles separated; group statistic absent from decisions; truth access/open
  counts zero; all 30 target IDs present.
- **F1 Held-out clean validation:** zero alarms and zero segments.
- **F2 Frozen target detection:** first alarm lies inside the injection interval and at affected index 1--15.
- **F3 Frozen specificity:** zero non-target alarm links and zero non-target candidate segments.
- **F4 Frozen support:** target has at least one segment, healthy segments zero, recall and precision each >=0.80,
  where TP/FP/FN use target-link candidate obs IDs and the 30-ID truth set. Also report onset error, alarm latency,
  end overshoot, and candidate duration.

Failure produces the exact first applicable frozen verdict and stops full dynamic admission. No rescue tuning is
permitted.

## 5. Production-commit-policy dynamic shadow

Only after F0--F4 pass, a dedicated diagnostic executable replays the same locked raw Walk1 configs through the same
IE loader, initializer, input plan, graph factors, IMU prediction, fixed-lag iSAM2 update, and current-group
detect-before-commit signal location. Unlike the historical PL alarm-controlled preflight, production commit policy
here always commits every planned current UWB row. The CUSUM observer only copies the computed `conditional_z`; it
cannot alter a factor, weight, optimizer/update call, accept/reject decision, state, cache, or trajectory.

For both clean and injected, CONTROL runs the commit pipeline without the observer and SHADOW runs the identical
pipeline with it. Both write runtime/measurement/state commit logs and scientific trajectory/state outputs. Discrete
logs must match exactly and committed numerical state max absolute difference must be <=1e-12; byte equality is used
where deterministic. Dynamic clean may process full raw data for initialization, but observer state is absent before
`validation_start` and starts from zero there using the already sealed calibration. Dynamic injected starts detector
state at zero at the first post-bootstrap conditional row. No dynamic recalibration occurs.

- **D0 Non-interference:** control/shadow decisions and update sequence exact; states <=1e-12; scientific outputs
  equal.
- **D1 Dynamic held-out clean:** zero alarms and zero segments.
- **D2 Dynamic target:** all 30 IDs present; first alarm inside injection interval at affected index 1--15.
- **D3 Dynamic specificity:** zero non-target alarm links and non-target candidate segments.
- **D4 Dynamic support:** target segment(s) >=1, healthy segments zero, precision and recall each >=0.80.

Frozen/dynamic conditional signals need not be equal. A post-seal evaluator reports paired/missing counts, mean/
median/max-absolute delta, sign agreement, Pearson/Spearman, target positive fraction/cumulative Z, and frozen/dynamic
detection/support metrics. These diagnostics cannot alter the verdict.

## 6. Final verdict and evidence

The only success is `CUSUM_PREFLIGHT_PASS_FOR_PRODUCTION_ADMISSION` after F0--F4 and D0--D4 all pass. Otherwise
the first failed gate's exact verdict is sealed. This is not a P_FA, integrity, cross-dataset, recovery, or localization
claim. No sensitivity sweep is planned.

Evidence root is the reject-overwrite directory
`/home/mint/ws_fusion_uwb/res/pl_persistent_cusum_preflight_20260912T021111Z`. It must contain start state, protocol
snapshot/hash, split/calibration seals, all process traces, frozen and dynamic artifacts, non-interference and
frozen/dynamic reports, test/build logs, detector seal, final evaluation/result snapshots and hashes, and final diff.
All historical failures and `NOT_RUN` branches remain visible.
