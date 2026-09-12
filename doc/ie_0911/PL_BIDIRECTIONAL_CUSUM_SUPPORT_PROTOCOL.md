# PL bidirectional CUSUM offline support-closure protocol

Status: `LOCKED_BEFORE_BACKWARD_STATISTICS / IN_PROGRESS`.

Date: 2026-09-12. Start HEAD:
`63b0062664c7fb4af372da1609128e8aff2b646a`. Previous immutable forward
evidence root:
`/home/mint/ws_fusion_uwb/res/pl_persistent_cusum_preflight_20260912T021111Z`.
New reject-overwrite evidence root:
`/home/mint/ws_fusion_uwb/res/pl_bidirectional_cusum_support_20260912T024240Z`.

## 1. Scope and immutable forward detector

Previous verdict is `CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`: F0--F3 passed and
F4 failed solely because forward `G>0` termination retained 37 post-injection
target rows. Forward CUSUM remains the only alarm/onset detector. Its signal
definition, conditional covariance mathematics, `kappa=0.5`,
`h=7.0234689587858723`, recurrence, 1.0 s gap/invalid reset, inclusive
crossing, last-zero onset, first alarm, clean split/calibration, identity and
sealed artifacts are immutable. Existing forward reverse-time fail-closed is
preserved. Group statistic, chi-square, RSSI, truth, oracle and injection
metadata remain absent from all detector/closure decisions.

This task adds only an offline, non-causal backward support-closure estimator.
It cannot create an alarm or standalone detection. Final candidate membership
is exactly same-link/same-`obs_id` intersection:

```text
final_candidate = forward_candidate AND backward_candidate.
```

There is no padding, tolerance, dilation, erosion, merge, truth clipping,
duration prior or fallback to one direction. The method is offline
post-processing and makes no causal/real-time endpoint claim.

## 2. Backward core

Each `(tag_id,anchor_id)` is processed independently in descending
`(timestamp,keyframe_id,source_order,obs_id)` order. With `x=conditional_z` as
the sole scientific input:

```text
B_0 = 0
B_j = max(0, B_{j-1} + x_reverse_j - 0.5).
```

`kappa_backward=0.5`. A physical time gap uses absolute time difference and
resets only when greater than 1.0 s, recording `BACKWARD_TIME_GAP_RESET`.
Invalid/nonfinite input resets without alarm and records
`BACKWARD_NUMERICAL_INVALID_RESET`. Crossing is inclusive `B>=h_backward`.
A reverse excursion begins at `0 -> positive`; if it crosses, rows from its
reverse last-zero start through crossing are backfilled and later reverse rows
remain candidates while `B>0`. A zero/gap/invalid ends it. When mapped to
ascending physical time, this is a non-causal offset/support-closure interval.

Backward identity binds algorithm/signal versions, direction, binary64
`kappa/h/gap`, reset/crossing/backfill/termination rules, original split/input
hashes and calibration hash. It is distinct from, and cannot weaken, the
forward identity.

## 3. Calibration, process boundary and outputs

The previous sealed `clean_split_manifest.json` is reused byte-for-byte.
Process A sees only locked clean conditional rows in the original calibration
interval, processes links in descending time with zero initial states, and
computes unthresholded `B_calibration_max`. The only threshold rule is

```text
h_backward = max(5.0, B_calibration_max + 1.0).
```

It is sealed before Process B held-out validation or Process C injected replay.
Process B starts `B=0` at validation end and must independently produce zero
alarms/segments. Process C runs all injected links without truth. Process D
reads only immutable forward candidates and sealed backward output, intersects
exact `(tag_id,anchor_id,obs_id)`, and writes bidirectional support. Processes
A--D accept/read no truth, GT, oracle, target or injection metadata and require
zero successful forbidden opens. Only after their outputs are sealed may
Process E read the locked 30-ID truth for evaluation.

Identity/version is `UIFGO_PL_BIDIRECTIONAL_CUSUM_SUPPORT_V1`; final output
records forward identity, backward identity and exact AND rule.

## 4. Frozen gates

- **B0 Integrity:** previous forward evidence seal verifies; forward identity,
  `kappa/h`, split and artifacts unchanged; backward calibration uses only
  calibration and is sealed before later runs; exact backward rule and
  `kappa=0.5`; exact obs-ID AND; zero forbidden opens; 30 truth IDs available
  only to evaluator. Failure:
  `BIDIRECTIONAL_SUPPORT_FAIL_INTEGRITY`.
- **B1 Forward regression:** existing/replayed forward artifacts are byte/hash
  identical and retain first alarm index 15, TP/FP/FN 30/37/0, max G
  17.77901339290359 and zero healthy alarm links. Failure:
  `BIDIRECTIONAL_SUPPORT_FAIL_FORWARD_REGRESSION`.
- **B2 Backward held-out clean:** zero alarms and zero segments. Failure:
  `BIDIRECTIONAL_SUPPORT_FAIL_BACKWARD_CLEAN_VALIDATION`.
- **B3 Frozen target support:** exact intersection has at least one target
  segment, precision >=0.80 and recall >=0.80. Failure:
  `BIDIRECTIONAL_SUPPORT_FAIL_TARGET_SUPPORT`.
- **B4 Frozen specificity:** zero healthy final segments. Failure:
  `BIDIRECTIONAL_SUPPORT_FAIL_SPECIFICITY`.

The first formal injected result is final. No adaptive rescue or parameter,
threshold, reset, intersection, padding, duration, target or injection change
is permitted.

## 5. Conditional dynamic shadow

Only after B0--B4 all pass, clean/injected raw CONTROL and SHADOW replay the
same production always-commit pipeline. At the future production signal
location SHADOW copies dynamically computed `conditional_z`, runs the unchanged
forward core, and after recording completes runs the same sealed backward core
and exact intersection. The observer never changes measurement/factor weights,
accept/reject or commit sequence, optimization/update, state, cache or
trajectory.

Dynamic gates require: exact discrete logs and state difference <=1e-12 (D0);
clean validation forward/backward zero alarms and final zero segments (D1);
forward target alarm inside injection and index <=15 (D2); final precision and
recall >=0.80 with zero healthy segments (D3). Frozen/dynamic paired signal and
support diagnostics are post-seal explanatory outputs only and cannot tune the
method.

## 6. Verdict and prohibited work

Only B0--B4 and all dynamic gates passing permits
`BIDIRECTIONAL_CUSUM_SUPPORT_PASS_FOR_PRODUCTION_ADMISSION`. Any first failure
is sealed and stops later stages. Even on pass this task registers no production
provider/mode, feeds nothing into the estimator, and runs no Stage2, Rc, final
inference, ATE or RMSE. Production integration requires a separate task.
