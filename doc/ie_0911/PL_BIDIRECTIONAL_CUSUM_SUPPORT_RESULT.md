# PL Bidirectional CUSUM Support Admission Result

Verdict:
`BIDIRECTIONAL_CUSUM_SUPPORT_PASS_FOR_PRODUCTION_ADMISSION`

Frozen bidirectional preflight:
PASS

Dynamic shadow replay:
PASS

Production integration:
NOT_RUN

Stage2 / recovery:
NOT_RUN

Localization accuracy:
NOT_EVALUATED

Previous verdict: `CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`

Failure source: forward G-zero termination tail

Forward detector: UNCHANGED

New component: non-causal backward support closure

The source HEAD for the locked run was
`63b0062664c7fb4af372da1609128e8aff2b646a`. Evidence is under
`/home/mint/ws_fusion_uwb/res/pl_bidirectional_cusum_support_20260912T024240Z`.
This is an offline/post-processing development admission result, not a causal
online endpoint detector or a formal integrity/calibrated false-alarm claim.

## Locked design and calibration

The forward detector was not scientifically changed: `conditional_z` remains
its sole input, `kappa_forward=0.5`, `h_forward=7.0234689587858723`, and its
recurrence, reset, inclusive crossing, last-zero onset backfill, alarm identity,
split and calibration are unchanged. Five regenerated forward artifacts are
byte-identical to the previous sealed evidence. The first alarm remains affected
observation 15 at `1664959681.9125128`, latency `3.6047780513763428 s`.

Backward calibration reused the original sealed clean calibration partition in
descending per-link time. It produced
`B_calibration_max=6.0234689587858714` and therefore, by the only allowed rule,
`h_backward=max(5,B_calibration_max+1)=7.0234689587858714`; kappa is 0.5. The
artifact was sealed before held-out validation or injected replay. Backward
held-out clean validation produced 0 alarms and 0 segments (maximum B
`6.1316015833455326`).

## Frozen result

Forward support was TP/FP/FN = 30/37/0, precision
`0.44776119402985076`, recall 1.0, and ended at `1664959696.549906`,
`10.242171287536621 s` after injection end. Backward target support ranged from
`1664959678.3077347` through `1664959686.289086`; relative to truth it was
TP/FP/FN = 30/0/0, with no pre-injection leakage and signed offset error
`-0.018648624420166016 s`.

The exact `(tag_id,anchor_id,obs_id)` Boolean intersection had one target segment,
TP/FP/FN = 30/0/0, precision 1.0, recall 1.0, onset error 0, signed offset error
`-0.018648624420166016 s`, and post-injection overshoot 0. No healthy-anchor
segment was produced. Backward alone cannot create a detector alarm or final
candidate; forward alarm time and latency remain authoritative.

## Dynamic shadow result

The diagnostic runner processed the same raw Walk1 configurations with the same
initialization, fixed-lag state and raw-UWB commit sequence. CONTROL had no CUSUM
observer. SHADOW used a read-only pre-commit conditional signal tap, unchanged
forward CUSUM, and post-recording backward closure. Both roles committed every
planned raw UWB observation; historical group chi-square remained diagnostic
and influenced neither candidate nor commit.

For clean and injected runs, CONTROL versus SHADOW runtime commits, measurement
decisions, state commits, trajectories, conditional rows and epoch logs were
all SHA-256 exact. Maximum numerical state difference was 0. Dynamic held-out
clean produced forward 0 alarms, backward 0 alarms and final 0 segments.

Dynamic injected first alarm remained affected observation 15 with latency
`3.6047780513763428 s`. Dynamic final support was TP/FP/FN = 30/0/0,
precision 1.0 and recall 1.0, with zero healthy segments. Frozen versus dynamic
paired all 1051 conditional rows with zero missing rows, maximum absolute
`conditional_z` delta 0, sign agreement 1.0, Pearson 1.0 and Spearman 1.0.

## Required questions

1. Forward scientific parameters changed: no.
2. Forward first alarm remains affected #15: yes.
3. Forward frozen artifacts regression-free: yes; five artifacts byte-identical.
4. `B_calibration_max`: `6.0234689587858714`.
5. `h_backward`: `7.0234689587858714`.
6. Backward held-out clean: 0 alarms, 0 segments.
7. Backward injected target range: `[1664959678.3077347,1664959686.289086]`.
8. Forward TP/FP/FN/precision/recall: 30/37/0/0.44776119402985076/1.0.
9. Backward truth overlap/leakage: 30/30 overlap, 0 FP, 0 pre-leakage, 0 post-overshoot.
10. Final TP/FP/FN/precision/recall: 30/0/0/1.0/1.0.
11. Final onset error: 0 s.
12. Final signed offset error: `-0.018648624420166016 s`.
13. Post-injection overshoot fell from `10.242171287536621 s` to 0 s.
14. Healthy final segments: 0.
15. Successful forbidden truth/GT/oracle opens in truth-blind processes: 0.
16. Dynamic CONTROL/SHADOW non-interfering: yes; exact scientific artifacts, max state delta 0.
17. Dynamic clean passed: yes.
18. Dynamic first alarm index: 15.
19. Dynamic final precision/recall: 1.0/1.0.
20. Production integration admission: yes, in a separate authorized task only.

## Scope boundary

No production `nlos.mode` or provider was added, no CUSUM decision fed back to
the estimator, and no Stage2, Rc, final inference, recovery or localization
accuracy experiment ran. This result only admits a later production integration
task; it does not establish general-dataset performance or localization benefit.
