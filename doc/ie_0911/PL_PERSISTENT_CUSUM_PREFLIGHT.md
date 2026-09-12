# PL persistent per-link CUSUM preflight result

Verdict:
`CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`

Frozen conditional preflight:
FAIL

Dynamic shadow replay:
NOT_RUN_DUE_TO_EARLIER_FAILURE

Production integration:
NOT_RUN

Stage2 / recovery:
NOT_RUN

Localization accuracy:
NOT_EVALUATED

## Scope and sealed inputs

The protocol was sealed before split or CUSUM statistics at
`cd2511580aee3f7b3de717e459f4a154cb1592687b80ef2ebb0d2aa8315551a5`.
The starting HEAD was `af0394a1b03edddd3e94d45a24c0c8d9a4a9c6b1`; the only
pre-existing worktree entry was protected, untracked `doc/v2/ie_0911/`, which
was not read or included. The locked conditional evidence manifest hashes to
`df78c6594034255c6e8c05de4f415ee65e3f96afe3d4b01f6c9e768b275addfb`;
all 28 listed payloads passed SHA-256 verification.

Detector processes accepted only `conditional_z`, row identity and validity.
The source-neutral input type has no group statistic, chi-square DoF, omnibus
alarm, truth, RSSI, target or injection field. Group chi-square therefore
remains historical/diagnostic and cannot affect this CUSUM decision.

Evidence root:
`/home/mint/ws_fusion_uwb/res/pl_persistent_cusum_preflight_20260912T021111Z`

## Clean split and calibration

Locked-clean valid timestamps span
`[1664959678.3077347, 1664959736.0829639]`. Process A fixed
`split_timestamp=1664959712.9728723` and sealed the manifest at
`sha256:4611aedf457c119aace020d0ee417be28ad6c80c75a59186f7de43f708ab283f`
before statistics. Calibration is
`[1664959678.3077347, 1664959712.4728723]` with 620 rows; held-out validation
is `[1664959713.4728723, 1664959736.0829639]` with 412 rows. The 19 rows in
between form the excluded 1.0 s guard band.

Process B selected only calibration rows. Under
`G_k=max(0,G_{k-1}+conditional_z_k-0.5)`, it found
`G_calibration_max=6.0234689587858723`, hence the sole allowed rule gave
`h_locked=7.0234689587858723`. Calibration was sealed at
`sha256:02587e4441185c886bb558103484ef82f1a9b2ab1086101f6451613089529f63`
before validation/injected replay. `kappa=0.5` is the fixed development
reference for an approximately one-sigma persistent positive mean shift, not
a formal integrity or false-alarm calibration.

## Frozen gates

| Gate | Result | Evidence |
|---|---:|---|
| F0 integrity | PASS | locked hashes valid; split/calibration sealed in order; exact threshold rule; same identity; 30/30 target IDs; zero forbidden opens |
| F1 held-out clean validation | PASS | 0 alarms, 0 segments, max G 6.1316015833455317 |
| F2 target detection | PASS | first crossing 1664959681.9125128, affected observation 15, latency 3.6047780513763428 s |
| F3 anchor specificity | PASS | zero non-target alarm links and zero healthy segments |
| F4 support quality | **FAIL** | TP=30, FP=37, FN=0, precision=0.44776119402985076, recall=1.0 |

The last-zero onset estimate was `1664959678.3077347`, exactly injection start,
and all 30 affected observations were covered. However, `G` did not return to
zero at injection end. The excursion continued through 37 subsequent target
observations to `1664959696.549906`; its 18.24217128753662 s duration overshot
injection end by 10.242171287536621 s. Target max G was
`17.77901339290359`. This is the preregistered precision failure, not a missed
detection or healthy-anchor specificity failure.

No locked setting was changed and no sensitivity sweep altered the verdict.

## Dynamic and downstream disposition

The sealed protocol permits dynamic admission only after F0--F4 all pass.
Since F4 failed, dynamic clean/injected CONTROL and SHADOW were not run.
D0--D4, frozen/dynamic drift, non-interference and dynamic support metrics are
`NOT_RUN_DUE_TO_EARLIER_FAILURE`. No production provider or `nlos.mode` was
added, no CUSUM decision fed back into an estimator, and Stage2, Rc, final
localization, ATE and RMSE were not run.

## Required evaluator answers

1. `conditional_z` was the sole scientific input: **yes**.
2. Group chi-square exited candidate decisions: **yes**.
3. Calibration/validation were temporally disjoint: **yes**.
4. Split sealed before statistics: **yes**.
5. Calibration read no validation statistic: **yes**.
6. `kappa=0.5` is the locked development reference for an approximately one-sigma persistent positive shift; it is not formal calibration.
7. `G_calibration_max=6.0234689587858723`.
8. `h_locked=7.0234689587858723`.
9. Held-out validation had 0 alarms/0 segments: **yes**.
10. Frozen target first alarm: affected observation **15**.
11. Frozen first-alarm latency: **3.6047780513763428 s**.
12. Frozen support: **TP=30, FP=37, FN=0, precision=0.44776119402985076, recall=1.0**.
13. Frozen healthy: **0 alarm links / 0 segments**.
14. Dynamic estimator commit impact: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
15. CONTROL/SHADOW scientific equality: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
16. Dynamic held-out clean: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
17. Dynamic target alarm index: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
18. Dynamic first-alarm latency: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
19. Dynamic support metrics: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
20. Dynamic healthy metrics: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
21. Frozen/dynamic signal change: **NOT_RUN_DUE_TO_EARLIER_FAILURE**.
22. Old PL conditional tests: **13/13 passed**.
23. Processes A--D successful forbidden opens: **0**.
24. Production admission satisfied: **no; F4 failed**.

## Engineering verification and disposition

The new target built. CUSUM tests passed 12/12 and existing conditional tests
passed 13/13, including analytic conditional covariance cases. Frozen CLI
replay completed. Full repository `ctest` and dynamic non-interference were not
run because the scientific stop gate had failed; this is not an engineering
PASS claim.

The sole verdict is `CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY`. Preserve the
evidence and stop. Any later analysis of precision/termination needs separate
authorization and cannot retroactively change this admission.
