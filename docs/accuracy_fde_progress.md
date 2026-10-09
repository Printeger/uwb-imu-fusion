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

## Current disposition

WP-A/B are implemented with isolated flags and preserved reference identities.
WP-C retains Gaussian baseline plus an independent Huber experiment; it does
not promote extra online iterations or claim a fully converged local oracle.
WP-D/E measured five profile streams containing normal/history/fault/fallback
snapshots; strict S0–S5 protected-recovery qualification remains NOT_MET.
WP-F confirmation, default-path measurement and core safety regressions
are complete. Hardware qualification stays closed, formal_eligible=false.

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

Walk1 output comparison (same sensor interval; offline evaluation grids,
not authenticated identical information deadlines):

| Output | APE / P95 / max m | SE(3) translation RPE m | World increment m | Fixed coverage | Init s |
|---|---:|---:|---:|---:|---:|
| Discrete Gaussian online | 0.396 / 0.496 / 4.009 | 1.035 | 0.524 | 96.28% | 2.064 |
| Discrete Huber online (unprotected) | 0.245 / 0.413 / 0.638 | 0.915 | 0.238 | 96.28% | 2.064 |
| SFUISE online/current calibration | 0.589 / 1.471 / 1.817 | 0.771 | 0.394 | 99.66% | 0.100 |
| SFUISE revised history/final calibration (offline) | 0.157 / 0.293 / 0.363 | 0.728 | 0.084 | 99.66% | 0.100 |

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

Same-model 35-attempt reference/experimental-cache pairs all complete:

| Scenario | Reference mean ms | Cache + QR mean ms | Total speedup |
|---|---:|---:|---:|
| S0/S1 normal + history | 301.227 | 299.758 | 1.005 |
| S2 UWB fault | 534.923 | 538.966 | 0.992 |
| S3 IMU fault | 366.207 | 365.922 | 1.001 |
| S4 joint order1 | 632.480 | 631.035 | 1.002 |
| S5 joint order2 | 3471.212 | 3282.112 | 1.058 |

Measured repaired-model bottlenecks over 35 attempts (not additive nesting):
normal core 10.543s: window 5.868s (boundary nested 5.400s), evidence 1.517s,
candidate stage 2.852s; base factorization only 7.63ms. Order2 core 121.492s:
window 20.197s (boundary nested 19.728s), evidence 37.536s, candidate 54.551s,
model generation 6.328s, action generation 1.549s, FDE decision 0.577s,
commit 61.5ms. Worker post/PL timers are nested in candidate stage and not
summed as wall time. Order2 max 61,104 hypotheses/405 actions, 439 generated
and kernel/post-evaluated actions, 67 PL-evaluated, zero protected selections;
base SVD/LLT 35 each, candidate reference SVD 1,340, inner LLT 1,317, fault Gram
eigen 1,987,209. Primary proposed directions were redundant boundary work and
repeated per-mode responses; neither window growth nor root base solve dominates.
Counters and complete stage totals are in the same-model JSON.

All five comparisons PASS for state, selected action, fault coverage, risks,
PL status/values, commit and publication identities. Each of the 350 terminal
packets separately passes the existing publication/transaction/timing binding
verifier. Finish-wall reason/checksum may differ; protected binding fields are
still checked. Initial strict worker comparison FAIL is retained. S5 attempt12
fault_gram_eigen rises 733917→736396: cached provisional boundary certificates
fall back to original products and are certified again. Three scratch reuse
counts differ. Work changes are reported separately, never called identical.
Evidence: accuracy_fde_same_model_20261009.csv/.json; full comparison reports
under results/accuracy_fde_20261009/fde_same_model.

High integration review identified unresolved exact PL tie/alert-boundary
rounding equivalence. Therefore mode cache is NOT default: explicitly opt in
with UWB_IMU_PL_MODE_RESPONSE_CACHE=1; EXHAUSTIVE overrides. Payload limit 16MiB
is per root, not the entire batch. No protected-recovery or general 50% gain
claim. Default retains every original per-hypothesis product. Default extractor
QR removal is bit-identical for served rows. New carrier SVD requests only V;
U never feeds σ/V in the installed Eigen implementation. Three deterministic
tests PASS, including adjacent rank thresholds and incremental tree proof IDs.
Reference UWB_IMU_PL_EXHAUSTIVE_CARRIER_U retains the original U+V path.

## WP-F — frozen precision confirmation and integration

Seven sequences replayed once per Gaussian/Huber frozen candidate, then evaluated.
Gaussian trajectories/metrics exactly reproduce B. Huber Walk1/2/3 APE
0.244599/0.213287/0.227860 m, mean 0.228582 vs Gaussian 0.350485 m (34.8% lower).
Walk1/3 max errors 4.008583/3.944723→0.638048/0.702365 m. No output is deleted:
fixed-grid coverage and initialization delay match the Gaussian candidate.
own_vicon obstacle is unchanged 0.746763 m; no_obstacle improves
1.249486→1.191427 m; simulation 0.049685→0.049715 m (0.06% change).
STAR-Loc regresses 0.600472→0.874786 m (45.7%), with world increment
0.421980→0.590414 m. This is a real confirmation failure, not an excuse to tune
that sequence using GT. Huber downweights range constraints in a weakly
initialized trajectory; whether calibration/yaw or weight loss causes the
regression is unresolved. Keep Huber an independent UNPROTECTED experiment;
recommend Gaussian for broad reproducible use. Full APE/P95/max, both RPE,
world increment, coverage and initialization: accuracy_fde_confirmation_20261009.csv.

Recommended overlay config/accuracy_fde_v2/gaussian_nominal.yaml uses original
bootstrap/CV/epoch lag and explicit B bias integration sigmas. Experimental
bootstrap changes are not promoted on theory alone. Bias covariance semantics
checked against installed CombinedImuFactor.h sha256 31128962…c14285 and local
CombinedImuFactor.cpp sha256 7dc20a6a…3402a: biasAccOmegaInt enters white-noise
velocity/rotation propagation (and cross blocks), while bias random walks use
dt*biasAccCovariance and dt*biasOmegaCovariance. This is local source inspection,
not authentication of an external binary's build. Tests vary initialization
prior by ×100 without changing preintegration covariance under explicit sigmas.

Added safety_decision_latency, analysis_completion_latency, core_compute
alongside arrival_to_publish. This synchronous runner emits the definitive
safety result at publish_call; aliases are not additive timers. Recovery starts
at the first affected IMU ingest (or UWB processing for range-only fault), ends
at the first protected publication, and otherwise emits null/RIGHT_CENSORED.
Unavailable is never counted as successful recovery. A short asynchronous
40ms input-stream acceptance is NOT_RUN because measured computation still
requires seconds; no stale PL or unfinished proof is reused to claim realtime.

Baseline selected CTest 31/32 PASS (259.45 s); failure is stale calibration
source binding. Four frozen P0-07 process campaigns and ROS visualization
NOT_RUN this round; historical failures unchanged. Corrected PYTHONPATH
invocation: dataset 10/10, round3 10/10, Gate-D 14/14 PASS. Initial import-error
invocations retained. Final core safety CTest 25/25 suites PASS (212.59s), including publication,
atomic transactions, IMU exclusion, history lifecycle and bridge/certificate
checks. Optional carrier microbenchmark is NOT_RUN_NO_CAPTURE; this stream never
produces a carrier with both dimensions >=200. This is not counted as a timing
PASS. Core safety suites and the three carrier bit-identity tests did run. No test that was not executed is marked PASS.

Local commits: f56a83a metrics/SFUISE, 8a1c294 bootstrap experiments,
9f251c9 exact dynamic-factor Gram repair, 7245c6c default safe decomposition
removal / opt-in cache / timing and comparison interfaces. No push. Git auto-GC reported an empty
loose object 01c67b7906b9860f93ff0b138babc0e1150dc5e6; fsck failed. Current
refs/commits work and rev-list does not reference this object. No destructive
repair attempted; subsequent local commits disable automatic GC.

Next justified research: profile/batch dirty historical-root recomputation
(current per-leaf updates repeat shared ancestors), and establish calibrated
Gaussian innovation/outlier models plus independently verified frame/yaw
initialization. Any promotion needs final σ/V/proof/selection equivalence,
measured wall-clock gain and valid normal/recovery snapshots. Current evidence
does not justify top-K, relaxed rank/thresholds, raw-history oracle expansion,
or further uninformed bootstrap/relinearization parameter searches.

Frozen default integration, same ELF/DSO reference and optimized, audit OFF:
normal mean 296.281→296.444ms (-0.055% reduction); joint-order2 mean
3448.477→3404.297ms (1.281% reduction, 1.013×). Both complete 35 attempts and
PASS decision/coverage/risk/transaction/publication-binding comparison. No
protected recovery observed: recovery_latency null/RIGHT_CENSORED; normal
NO_FAULT_INPUT. Evidence accuracy_fde_integrated_20261009.json. Single paired
measurement supports only a small potential saving, not a statistically
established broad gain or the 50% target. No more timing repeats justified.

Independent same-production-snapshot audit at attempts 10/12/32: 13 compared
batches / 39 roots pass numeric/discrete/hypothesis/proof-payload and within-batch
ranking comparison; 40 empty batches are explicitly NOT_RUN_NO_POST_PASSED_ROOTS.
Raw PL batches total 15747.876→10513.445ms (33.24% reduction, diagnostic scope);
714555 hypothesis products reuse 12645 unique mode products, with 2479 original
fallback recomputations retained. This explains why candidate substage gain does
not imply a 50% pipeline gain. Parent timing, counters and cache state include
audit overhead and are excluded from production comparisons. Full diagnostic
records retained; compact summary accuracy_fde_production_audit_20261009.json.

Final disposition: a runnable, reproducible research integration is delivered;
WP-A–F have implementations or explicit evidence-based stopping decisions.
Walk <0.20m target, 50% FDE target, fully qualified O-local convergence, strict
S0–S5 recovery snapshots and hardware/formal integrity qualification are NOT_MET
or OPEN, never relabeled PASS. Stop additional blind tuning/repeats. The next
research needs the frame/calibration and historical-root evidence described
above, not relaxed safety gates. Original P1-06 FAIL and stale-calibration FAIL
remain intact. User-owned requirements and old evidence remain untouched.


## Next stage — structural compute and nominal generalization (2026-10-09)

Active scope follows the new six-step objective; previous WP-A–F results above
are reference evidence, not rerun requirements. Initial HEAD 27a339c, clean
worktree. Work remains on feature/realtime-uwb-imu-pl, managed by commits. No new
Git worktree. User explicitly rejects redundant large archives: initial bundle/
tar copies were stopped/deleted; original raw results stay in place. Keep only
compact metrics/hashes/commands. Original frozen ELF/DSO hashes verified.

Preservation: all refs/reflog objects enumerated. Found 80 zero-byte loose
objects, none reachable; moved intact to results/accuracy_fde_phase2_20261009/
quarantine (zero payload bytes). Git connectivity fsck now exits 0; dangling
objects and old temporary pack files retained, no destructive clean/repack.
Auto-GC disabled locally. Baseline source and calibration FAIL status retained.

Plan: (1) preservation complete; (2) layer-separated normal/UWB/IMU positive
controls and actual production failure classification; (3) historical dirty
closure; (4) profile joint-order2 evidence/candidate and optimize certified
hotspot; (5) residual/robust-weight/conditioning explanation of STAR-Loc
regression before any model change; (6) freeze beneficial candidate and paired
integration/core safety checks. No top-K or scope/risk/publication relaxation.

History hypothesis: per-leaf recomputation repeats shared ancestors during one
request. Added dirty scheduling for upsert/erase/rotations and one left-right-
node refresh after the mutation batch. Exact group/priority/topology, raw UID/
provenance, merge order and full-row fallback unchanged. Exception clears the
partially mutated tree then rethrows. UWB_IMU_PL_EXHAUSTIVE_HISTORY_UPDATES=1
retains per-leaf reference in the same ELF/DSO. Mode response cache still opt-in.
4/4 targeted tests initially PASS: cold build, batch RHS change, exact hit,
insert/rotate/remove, one-bit fault edit, ordering/whitening/recovery rebuild,
nonfinite refusal plus existing 30-step independent raw-row oracle. All served
matrices and complete carrier rank/proof payload compare bit-identical; changed
node counts separately reported in XML. Exception-guard rebuild and rerun also
PASS (4/4); reference 1975 refreshed nodes vs batch 330, cold 374 vs 64.

Checkpoint measurements: one same-ELF/DSO pair, 35 attempts per side, mode cache
OFF. Normal core mean 293.268→180.120ms (38.58% reduction); joint-order2
3174.216→2689.025ms (15.29%, 1.180×). Single pairs do not establish a broad or
statistical speedup. Joint2 strict decision/coverage/risk/transaction/publication
comparison PASS. Normal strict comparison FAIL: input attempt 5 has a different
finish/publication deadline refusal reason. State, numeric history proof and
candidate comparisons PASS; independent publication-binding checks PASS on both
sides. Preserve the FAIL; no threshold change or comparator relaxation. This is
a research checkpoint, not final integration or proof of real-time readiness.
Compact commands, identities, hashes and mismatches are in
docs/benchmark/accuracy_fde_phase2_history_20261009.json. No new large archives.

Positive controls: three new deterministic numerical-layer tests PASS (3/3).
KEEP/UWB examples have finite PL and valid proof while risk availability and
formal gates remain closed; IMU test proves bridge model independence at a
common CV reference and finite box bounds only, not full protected recovery.
Existing production records distinguish finite PL blocked by risk/formal gates,
rank-certificate uncertainty (not proven mathematical infinity), and IMU step
gate failure with PL NOT_RUN. Compact review:
docs/benchmark/accuracy_fde_phase2_positive_controls_20261009.json.

Remaining work: investigate the normal deadline boundary difference, profile
and remove certified joint2 duplicate computation, explain Walk/STAR-Loc robust
generalization, then freeze and run core integration regressions. No protected
recovery yet; external IMU/formal qualification OPEN. Normal <100ms and joint2
~1s targets NOT_MET. This checkpoint does not complete the active goal.

Joint2 hypothesis: evidence first certifies raw Gram, then the existing trusted
combined protected-response constructor repeats the same Gram SVD/hash. Reuse
the constructor's complete `.gram` inside the same call; retain the original
non-raw/no-response paths and UWB_IMU_PL_EXHAUSTIVE_GRAM_CERTIFICATES=1 double
construction. Count actual raw Gram SVD construction points (including full-V
structural-kernel fallback), previously absent from fault_gram_svd counts.
8/8 targeted tests PASS: exact evidence/monitor/entry/complete proof bits for
dimensions 1–4 with fixed/generic paths and full/zero/hidden/near-rank cases,
existing frozen invalidation, dangerous/harmless nullspace and positive controls.
Reference proof is captured before optimized registry insertion to avoid a
circular lookup comparison. Same-ELF normal/joint2 strict comparisons both PASS.
Normal core 176.716→179.395ms (no gain observed); joint2 2668.971→2626.770ms
(1.58% reduction). Evidence 995.331→948.990ms, candidate 1369.997→1368.008ms.
Joint2 actual raw Gram SVDs 10823762→9552278; counts include proof verification.
Small saving does not meet the structural performance target; stop extending
this same-call optimization. Retain unchanged math and certified reuse; next
investigate repeated proof verification and raw-factor work. Root/proof tests
5/5, transaction/bridge/IMU/publication tests 12/12 and history oracle 4/4 PASS
against the new DSO (29/29 targeted total). Compact performance/identity report:
docs/benchmark/accuracy_fde_phase2_gram_reuse_20261009.json. One pair per scenario;
no broad significance or final integration claim.

Normal deadline diagnosis: comparator row 5 means input attempt 5 (previous
"sixth attempt" wording corrected). Finish gate is 40ms; the existing benchmark
packet gate is 50ms. Reference arrival-to-publish 83.842261ms breaches both;
optimized 40.926478ms breaches finish only. Both remain unprotected. This explains
the reason difference; retain strict FAIL and both gates, no comparison relaxation.

Nominal generalization diagnostic: added tools/diagnose_nominal_range_weights.py,
reusing frozen Gaussian/Huber Walk1 and STAR-Loc states plus exact sensor batch
timestamps, original filtered UWB batches, per-range sigma/lever and logged cost.
No GT read or parameter selection. Every output batch/count binds; reconstructed
Gaussian cost relative deviation max <1.4e-4 despite rounded state exports.
Weights are posterior-implied Huber(k=1.5), not historical IRLS weights. Geometry
information is range-only position information conditional on attitude, not full
navigation covariance. Compact reports accuracy_fde_phase2_weights_{walk1,starloc}
in docs/benchmark contain source hashes and reproduction commands.

Gaussian |z|>1.5 fraction: Walk1 5.39%, STAR-Loc 34.88%; |z| P95 1.575 vs 3.643.
Huber trajectories: 4.05% vs 38.70% downweighted, weight<0.5 0.32% vs 16.50%.
Mean minimum-eigen information ratio under applied weights: 0.9875 vs 0.8132.
Unweighted Gaussian geometry condition median 7.35 vs 20.11. STAR-Loc has broad
residual downweighting and poorer geometry, while Walk mainly clips a small tail;
this is consistent with the measured Walk gain/STAR-Loc regression, not a proof
of its unique cause. Anchor signed residual means persist (STAR-Loc anchor 10
+1.05 sigma, anchor 6 -0.77 Gaussian), so an IID outlier-only interpretation is
not established. Calibration/frame identities remain the frozen GT-assisted
Walk interface and STAR-Loc published rig/gyro rotation/range_calib/lever evidence.
Neither metadata nor posterior residuals qualify physical extrinsics. Keep the
Gaussian recommendation and separate Huber experiment; no unsupported new noise
model or dataset-dependent threshold. Actual per-factor IRLS history and physical
frame/calibration qualification remain needed to isolate causality.

Next compute experiment identified by independent High safety review: scoped
full PL validation repeats the same immutable arena payload at production bind
and candidate consumption, then again in winner/publication paths. One frozen
hypothesis check runs rebuilt Gram, independent Z SVD and another Gram
certificate; counted Gram SVDs exclude the separate Z classification SVD. Try
attempt-local reuse of successful full payload validation only, tied to actual
shared owner and payload address (arena numeric keys can be overwritten). Keep
every external candidate/detector/hypothesis/result binding, mutable public-proof
validation, exception retry and single-consume contract. This is a next-step
hypothesis, not an implemented optimization or permission to skip validation.

Scoped PL payload reuse implemented: private const wrapper owns const Proof;
existing result/identity arena indexes retain an alias to that same immutable
Proof. New runtime-only kind 8 holds its successful-validation memo. Require
same proof address, shared control block and arena generation. No global memo,
proof schema/hash change or caller-supplied validation receipt. Only full original
payload validation returning true publishes success; false and exceptions retry.
The per-payload lock is separate from the arena lock. Mutable public validators,
retain imports, legacy registries and final bundles still fully validate. Each
scoped consumer retains original external bindings/reason choices and proof hash.
UWB_IMU_PL_EXHAUSTIVE_PL_VALIDATION=1 restores original copy/full-validation path.
Actual full payload executions and reuse counters are emitted by the benchmark.

Scope limitation found in preexisting binding: result lookup key omits external
hypothesis_tail_used, axis_tail_used, fault_multiplier_used and noncentrality_used.
Full sidecar hash validates the stored proof, not every external result field.
This optimization preserves that existing behavior; do not describe it as added
all-field result authentication. That binding question remains OPEN separately.
The memo assumes the existing trusted detail typed-kind producer protocol; it
does not defend against arbitrary illegal C++ type writes or const_cast.

Scoped reuse tests PASS 20/20 (owner/address, false/exception retry, concurrent
first success, valid exhaustive comparison, external refusal reason equivalence,
mutable tamper, retained-owner replacement, cross-attempt creation, arena close,
existing proof ownership/concurrency and Gram/positive tests). Additional
transaction/IMU/bridge/publication tests PASS 12/12. One same-ELF pair per scenario,
35 attempts: both strict semantics and independent publication binding PASS.
Normal core 179.687→179.390ms (0.17%, negligible); joint2 2629.234→2572.669ms
(2.15%). Joint2 candidate 1378.704→1303.190ms; evidence 944.978→958.525ms.
Full payload validations 22→11 with 11 reuses in each scenario; counted joint2
Gram SVDs 9552278→9405759. Reuse coverage is small; stop extending this layer of
memo rather than claiming the structural target met. Compact commands/hashes/
metrics: docs/benchmark/accuracy_fde_phase2_pl_validation_20261009.json.

Profiler caution confirmed in source: diagnostic_candidates.pl_ms contains
flat-batch wall assigned to every root, so per-root sums duplicate time (not
worker CPU work). Use parent wall stages until batch-exclusive subdivisions are
exported. Candidate and evidence remain the next investigation, with model
generation separately measured. A narrow classifier reuse idea was reviewed,
but is not implemented or claimed as a gain; profile before another small change.
No new large backups or binary copies; previous numerical/campaign/calibration
FAIL records unchanged. Active goal and final integration remain OPEN.

Numerical hotspot audit instrumentation: process-start opt-in
UWB_IMU_PL_PROFILE_NUMERICAL_PHASES=1 records inclusive elapsed work/call counts
for Gram construction/SVD, classifier and frozen/PL payload validation. Nested
calls and worker times overlap; these are not exclusive wall or CPU timings.
Candidate kernel/post/shared PL/serial consumer now have once-per-batch wall
stages under candidate_evaluation. Public numerical behavior and reference
switches unchanged. Build PASS; focused integrity/ownership/concurrency/positive
controls and root math checks PASS. Profiling run pending; no performance gain
claimed for instrumentation. No ELF/DSO copies or large backup artifacts.
