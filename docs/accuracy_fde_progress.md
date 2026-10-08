# Accuracy / FDE development — 2026-10-09

Active goal: requirements `UWB_IMU_Accuracy_FDE_Requirements.md`, WP-A–F.
Branch `feature/realtime-uwb-imu-pl`; initial HEAD
`f9222c1d4fd551139f295212370573ce578f933a`. Initial tracked diff empty;
user-owned untracked `docs/requirements/` preserved. Old P1-06 remains failed.

## Baseline and plan

- B: current common fixes, original per-cache nominal config, CV/epoch lag,
  original bootstrap (`run_nominal_accuracy_study --stage covariance`).
- B-exp: causal/IMU/LM/5s/0.01 experimental overlay. Never substitute for B.
- Preserve baseline ELF/DSO in `results/accuracy_fde_20261009/baseline/`.
- D1 Walk1, D2 own_vicon obstacle, D3 simulation; freeze C1 Walk2,
  C2 Walk3, C3 own_vicon no_obstacle, C4 starloc loop-3d_s3 before evaluation.
- WP-A: supplement world displacement, fixed sensor coverage, frame diagnostics;
  reuse trajectories only with explicit historical identity.
- WP-D: inspect existing production stages/counters; collect mature scenarios.
- WP-B: isolated actual-time bootstrap, motion classification, seed/prior and
  explicit integration covariance ablations; deterministic tests first.
- WP-C: normal/failed local batch oracle; retain separate Gaussian/Huber scope.
- WP-E: independent High review of eligibility consumers before pruning;
  implement measured safe optimization with exhaustive comparison.
- WP-F: freeze useful candidates, one confirmation set and core safety regressions;
  High integration review; report remaining accuracy/realtime gaps.

## Current evidence / decisions

Existing report: current common-fix Walk mean 0.3505 m; experimental 0.3178 m,
Huber 0.2272 m (historical identities, not new measurements). own_vicon
experimental regression and SFUISE online/history mismatch require diagnosis.
Initializer source confirms 0.2s nearest-node assignment, IMU-only stationarity,
and posterior bias sigma fallback into later preintegration. Contributions
remain unmeasured. No current test PASS claimed yet.

## Blockers and next work

Hardware qualification stays closed, `formal_eligible=false`. Slow frozen
historical campaigns retain FAIL/NOT_RUN rather than being rewritten.

## WP-A — implemented / measured

B ELF `5e188b6b...408c`, DSO `c49c0394...ccea` exactly match all seven retained
execution manifests/config/cache hashes. Full manifest is in
`results/accuracy_fde_20261009/baseline/manifest.json`; compact metrics:
`docs/benchmark/accuracy_fde_baseline_20261009.csv`. New results never overwrite
old runs. Added world position increments, both GT-fitted fixed rotation
diagnostics, gravity direction, fixed sensor-grid coverage, initialization
delay and missing-output duration/gap. Original APE and SE(3) RPE retained.
`test_dataset_benchmark.py`: 10/10 PASS (90° left/right rotation, q/-q,
lever arm, delayed start/early stop). Grids are offline evaluations of output
sequences, not exact same-receipt online comparisons.

| Frame chain | Actual adapter contract | Qualification |
|---|---|---|
| SFUISE IMU/body | source specific force m/s², gyro rad/s; identity body axes | device frame label only |
| SFUISE tracker→tag | tracker p + R_tracker·[0.1,-0.025,0] | body/tracker rotation not independently documented; FRAME_UNVERIFIED |
| SFUISE nav→anchor world | algorithm q_nav_uwb left-multiplies body orientation and rotated tag position | evolving estimated calibration; no exact input receipt |
| own_vicon IMU→body | frozen GT-assisted C_body_from_imu applied to acc and gyro; acc g×9.80665 | interface fit, not independent hardware calibration |
| own_vicon body→tag | frozen body lever [0.0083583,-0.0018726,0.0445150] m; time shift +0.0031147 s | GT-assisted joint family calibration |
| simulation body→tag | analytic world body pose plus rotated configured lever | synthetic frame verified |
| STAR-Loc IMU→rig | C_r_i=C_c_rᵀ C_c_i; official rig orientation/tag position | external reference axes not independently qualified |

All exported quaternions use xyzw; estimator represents world-from-body.
No fitted rotation is fed back to estimation. D2 B raw APE 0.746763 m versus
aligned 0.008237 m / world increment 0.012532 m. Its 171.385° attitude APE
has 0.280–0.307° residual after a fixed left/right fit; short low-motion
data cannot distinguish missing extrinsics from unobservable startup yaw.
Walk1 fixed-fit residual remains 54–57°, so a single constant rotation does
not explain its entire attitude error.

One new SFUISE Walk1 replay (970/970 received epochs, upstream unchanged)
produced four views at the same 591 timestamps:

| State / calibration | APE RMSE m |
|---|---:|
| online / then-current | 0.589461 |
| online / final | 0.598068 |
| revised history / then-current | 0.225897 |
| revised history / final | 0.157121 |

Thus on this run knot revision is the larger source of historical improvement;
final calibration alone does not improve online states. All four fixed-grid
coverage values 99.662%; cutoff quality WINDOW_BOUNDARY_ONLY.
Evidence: `accuracy_fde_sfuise_four_views_20261009.json`.

## WP-B / WP-C — tests and isolated hypotheses

15/15 nominal C++ tests PASS. New opt-in flags preserve old configs:
actual range-time nodes and split IMU intervals; local UWB slope/geometry speed
uncertainty for static/dynamic/unresolved classification; weak independent
origin regularizer instead of range-derived translation prior; enforced
below-anchor branch only with declared physical geometry. Explicit integration
sigmas stay [0.1,0.1,0.1,0.01,0.01,0.01], independent of returned posterior bias
prior. No joint-prior API expansion: covariance still DIAGONAL_UPPER_BOUND;
necessity/benefit of full covariance not established.

| Isolated evidence | Result / decision |
|---|---|
| D2 causal + fixed integration | APE 3.537878 m, reproduces wrong upper branch |
| D2 actual-time graph | 3.537909 m; time error fixed, not this error's cause |
| D2 seed-only prior | 3.537900 m; local optimizer stays in wrong branch |
| D2 declared below-anchor branch | 0.750192 m; restores B-level result, not an improvement over B 0.746763 m |
| D1 UWB motion confirmation | speed upper 0.278 m/s → MOTION_UNRESOLVED; dynamic graph APE 0.408990 vs causal 0.407729; no precision gain claimed |
| D1 relinearization 0.01 only | 0.376763 vs B 0.396252 m; small gain, not promoted |
| D1 original B + existing Huber only | 0.244599 m / max 0.638048 vs B max 4.008583; independent UNPROTECTED candidate for confirmation |

No further time/seed parameter search: two explanations failed to explain D2;
known geometry identified the branch issue. Local same-active-graph LM at
20.7046/34.2379 s lowers objective 29285→13533 / 93.235→43.333; GT error
4.187→5.604 / 0.309→0.302 m. Failed-window objective reduction does not improve
position. Current LM stopping gradients are 0.00330 / 0.00168 scaled; this is
not a certified fully converged or global optimum reference. Further online
iterations and O-raw are not justified by this evidence; qualification of
O-local remains open. Existing one-update transaction contract preserved.

## WP-D / WP-E — current measurements and safe boundaries

Five original-model 35-epoch profile/scenario replays complete. S0 and S1 share
the noiseless replay (epoch 10/32); recovery scenarios use existing one-epoch
fault at attempt 12. Fixed lag 30, integrity window 10, unchanged scopes.
Original joint-order2 core median 3349.9 ms, maximum 30339.1 ms; candidate stage
63854 ms / evidence 41815 ms / integrity window 21077 ms summed over 35 attempts.
Other profiles chiefly spend time in window boundary provenance. Nested
boundary/window timers and worker PL timers are not additive wall-clock totals.

Strict S0–S5 scenario qualification is NOT_MET: all original runs unprotected;
UWB/joint fault alarms lack valid recovery, IMU example does not alarm. These
remain unavailable samples, not successful recovery benchmarks. Normal and
history windows, bridges, heavy order-two work and fallback are still useful
diagnostics; risk qualification and several unbounded modes remain open.

High review rejected simple action eligibility pruning: any candidate exception
currently fails the whole attempt, even if selection-ineligible. No top-K,
early cardinality stop, PL lower-bound pruning or PROVEN_INELIGIBLE claim added.

Implemented two independent optimizations: (1) production skips only extractor
rank QR with zero consumers, returns rank=-1, keeps every original row and all
downstream certification; reference env UWB_IMU_PL_EXHAUSTIVE_BOUNDARY_RANK.
(2) per-candidate raw fault mode response cache, retains all hypotheses, root
solves, rank/nullspace/PSD checks and boundary fallback; reference env
UWB_IMU_PL_EXHAUSTIVE_MODE_RESPONSES. Cache tests 4/4 PASS, extractor 3/3 PASS.

Initial cache tests 4/4 FAILED before reaching caching: fixed/dynamic Eigen
Gram arithmetic disagreed with the exact raw-factor certificate in a 6-column
fixture. A shared repair computes protected covariance using the certificate's
actual dynamic factor operation, without loosening any gate. Old failures kept;
new reference/optimized use the same repaired model. Production extent remains
to be measured; no claim that this explains all old infinite PL values.

Same-model paired profile replay and optional in-scope cache microcomparison
are in progress. Baseline selected CTest 31/32 PASS (259.45 s); failure is
test_simulation_calibrations stale source binding. Four frozen P0-07 process
campaigns and ROS visualization NOT_RUN this round; their historical FAIL is
retained. Final related regression and frozen candidate confirmation pending.
