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

Six-step research delivery: implemented, built and measured on
`feature/realtime-uwb-imu-pl`; initial HEAD `27a339c`, measured code `c84b13e`.
Previous WP-A–F and their scientific FAIL records above are preserved. No new
worktree, 40-sequence rerun, long campaign or nominal parameter search.
Final compact identities, commands, all frozen precision metrics, raw-file hashes,
latency distributions and 69 current-DSO tests:
[accuracy_fde_phase2_integrated_20261009.json](benchmark/accuracy_fde_phase2_integrated_20261009.json).

### Preservation and scope

All refs/reflog objects inspected: 11,243 reachable objects; 80 zero-byte loose
objects were unreachable and moved intact to ignored quarantine, connectivity
fsck exited 0. No destructive clean/reset/repack; old temporary packs retained,
local auto-GC disabled. Existing frozen ELF/DSO/source-diff hashes reverified at
final delivery. Redundant bundle/tar copies stopped/deleted under user instruction;
no new binary/dataset archives. `.gitignore` covers raw results, builds, datasets,
cache and root archives; compact source/config/JSON/CSV evidence is committed.
User-owned requirement file retained unchanged. Commits manage history.

### Positive controls and production refusal

Three deterministic numerical controls PASS: KEEP finite certified PL; UWB
exclusion removes the sole modeled faulty direction with finite PL; IMU bridge
shows factor independence at a common CV reference and finite box bounds.
The IMU result is BRIDGE_ONLY_NOT_FULL_PL. None bypasses risk/formal/publication
requirements. Production normal attempt10 finite candidate HPL=6.3289m,
VPL=1.31634m is blocked by risk/formal/deadline. Normal32/UWB S2 is a numerical
rank transition uncertainty, not proven mathematical infinity. IMU S3 fails the
step gate before PL (NOT_RUN), not a demonstrated recovered alarm. Compact
[positive controls](benchmark/accuracy_fde_phase2_positive_controls_20261009.json)
retain those distinct layers. No protected recovery was obtained.

### Implemented compute changes and stopping decisions

| Change | Controlled checkpoint normal core ms | Joint2 core ms | Disposition |
|---|---:|---:|---|
| Batch dirty history dependency closure | 293.268→180.120 | 3174.216→2689.025 | Default; largest structural gain |
| Same-call complete raw Gram certificate reuse | 176.716→179.395 | 2668.971→2626.770 | Default; narrow 1.58% joint2 gain, stop expansion |
| Immutable full PL payload successful validation reuse | 179.687→179.390 | 2629.234→2572.669 | Default; narrow 2.15% joint2 gain, stop expansion |
| Classifier-only unconsumed identity hash omission | 176.704→176.928 | 2555.401→2393.333 | Default; 6.34% joint2 gain, same SVD work |
| Frozen hypothesis memo, setup-inclusive pair | 176.832→178.692 | 2389.299→2426.540 | No net gain; default OFF, stop direction |

One same-build pair/scenario at each checkpoint, 35 attempts/side. These are
local experiments, not additive/multiplicative gains or statistical guarantees.
Compact checkpoint records remain in `docs/benchmark/accuracy_fde_phase2_*`.

History preserves treap topology/rotations, left-node-right merge arithmetic,
raw row provenance, RHS/rank/dof/fault coverage and exact full-row fallback.
Mutation exceptions invalidate partial tree and rethrow. Oracle includes original
per-leaf bit/proof parity and 30-step full raw-row reconstruction; the raw builder
is not an independent implementation of all mathematics. Initial history pair
normal strict comparison FAIL at input attempt5: reference 83.842261ms breaches
40ms finish and 50ms packet gates; optimized 40.926478ms breaches finish only.
State/proof/bindings matched. Preserve that FAIL and both gates; no comparator
relaxation. Joint2 checkpoint strict PASS.

Gram reuse consumes the already complete protected-response constructor's `.gram`
in that same call. Classifier omits only unused identity hashes via a private path;
public certifiers retain original hashes, both SVDs, rank/kernel/error/reasons.
Public proof identity tests pin original 902ff99 DSO IDs. Counted Gram SVDs exclude
the classifier's separate no-V Z-reference SVD. No approximate Gram/rank shortcut.

Full PL memo requires private const proof, exact address/shared owner and arena
generation. Only successful full validation caches; false/throw retry. Mutable,
imported, legacy and final proof bundles validate exhaustively; all external
root/window/dual/model/tail/risk/selection/publication bindings remain. Trusted
private typed-kind protocol is assumed, not arbitrary illegal C++ type writes.
Reference flags now also restore original plain producer setup (no private memo
index) for honest net cost. Frozen memo initial reuse-only pair suggested 2.09%
joint2 gain but charged wrapper setup to both sides; corrected net pair regressed
1.05% normal and 1.56% joint2. `UWB_IMU_PL_FROZEN_VALIDATION_REUSE=1` retains this
experiment only; default plain/full, exhaustive overrides opt-in. Old mode response
cache remains opt-in because strict tie/alert equivalence is unresolved.

Preserved engineering failures: generic frozen-proof fixture initially expected
valid but exhaustive independently rejected it; corrected test checks identical
reason/false retry, without weakening validator. Standalone classifier probe
initially crashed with mismatched Eigen compile flags; exact target
`-march=native -DNDEBUG` fixed it and temporary probe was removed. Neither is
relabelled as a production acceptance failure or silently discarded.

Profiler is process-start opt-in `UWB_IMU_PL_PROFILE_NUMERICAL_PHASES=1`.
One joint2 audit: Gram inclusive work 63.046s (SVD 15.476s), classifier 32.256s,
frozen validation 37.042s, PL payload 1.981s. These nested/worker wall intervals
are neither exclusive wall nor CPU; do not add. Frozen count 2,497,242 decomposes
into 1,271,484 seals, 1,152,504 KEEP reads, 73,254 PL checks; the 534 difference
from H is consistent with invalid/short-circuit seals, not exact callsite proof.
Batch wall kernel244.855/post3.820/sharedPL971.078/consume18.838ms belongs under
candidate1308.319ms. Per-root `pl_ms` duplicates flat-batch wall: never sum roots.
Final profiling is OFF. Remaining major work is full hypothesis certification and
flat candidate PL, not historical ancestor repetition.

### Nominal generalization

Gaussian remains the research baseline; Huber(k=1.5) remains an independent
UNPROTECTED experiment. All seven previous Gaussian/Huber execution pairs are
reused with their original source/ELF/DSO/config identities, not c84b13e replays.
No nominal estimation arithmetic, model/config or evaluation code changed here.
All APE/RPE/world increment/coverage/valid ratio/initialization/gap fields remain
in the frozen CSV and integrated JSON; no selective metric deletion.

| Sequence | Gaussian APE RMSE m | Frozen Huber APE RMSE m |
|---|---:|---:|
| Walk1 | .396252 | .244599 |
| Walk2 | .226705 | .213287 |
| Walk3 | .428497 | .227860 |
| Own no-obstacle | 1.249486 | 1.191427 |
| Own obstacle | .746763 | .746763 |
| Simulation | .049685 | .049715 |
| STAR-Loc loop-3d_s3 | .600472 | .874786 |

`tools/diagnose_nominal_range_weights.py` reads no GT and selects no parameters.
Exact sensor batches/counts and original sigma/lever bind, reconstructed Gaussian
cost max relative discrepancy <1.4e-4 from rounded exports. Posterior-implied
weights are not recorded historical IRLS. Geometry is range-only position
information conditional on attitude, not full navigation covariance.
Walk1/STAR Gaussian |z|>1.5: 5.39%/34.88%, P95 1.575/3.643. Huber downweight:
4.05%/38.70%, weight<.5 .32%/16.50%; minimum-eigen information ratio .9875/.8132;
Gaussian geometry condition median7.35/20.11. STAR anchor bias10 +1.05sigma and
6 −.77sigma persists. Broad STAR downweight with poorer geometry is consistent
with the regression; unique physical cause is unproven. Frozen Walk GT-assisted
interface calibration and STAR published rig/gyro rotation/range_calib/lever are
evidence, not formal physical extrinsic qualification. Actual IRLS histories and
independent calibration evidence remain needed. No unsupported new noise model
or sequence/dataset-specific tuning was introduced.

### Final combined acceptance and remaining gaps

Same current ELF/DSO, all five exhaustive flags versus defaults; four fresh native
35-attempt processes, one pair/scenario, OMP/BLAS/MKL=1. Mode cache and frozen memo
OFF on both sides. Final source c84b13e clean; profiler OFF. Actual combined
measurement, not products of checkpoint gains:

| Scenario | Core mean reference→default ms | Reduction | Arrival→publication mean reference→default ms | Strict / independent binding |
|---|---:|---:|---:|---|
| Normal | 280.398→176.517 | 37.05% | 280.411→176.530 | PASS / PASS both sides |
| Joint-order2 | 3170.796→2395.440 | 24.45% | 3170.814→2395.461 | PASS / PASS both sides |

All 13 compared CSVs PASS, zero numerical mismatches, exact hypothesis/action
counts retained (normal15,600/35; joint2 1,272,018/439). Only independently checked
finish-wall packet checksum differences are admissible under unchanged comparator.
Joint2 mean candidate1276.694ms includes flatPL938.877ms; evidence811.675ms,
model generation175.942ms, history/window52.690ms. Default core P95 normal274.985ms,
joint2 3030.985ms; maxima304.118ms/22802.5ms from rounded timing CSV. This is a fixed
native stream, not a live frequency or tail guarantee. Detailed means use higher
precision diagnostic stages; timing CSV rounding explains tiny mean differences.

Current-DSO core safety PASS **69/69**: integrity/proof ownership/concurrency/
transaction/bridge/IMU/publish-boundary35, classifier/root8, history4, publication
identity3, wiring6, P0-05 publication13. XML, binary, DSO hashes and exact filters
are bound in the integrated JSON; earlier 68-test evidence has its old identity.
CMake run-manifest Git stamp remains e3e0c80/dirty from configure time. Raw files
are unchanged; actual clean c84b13e execution identity, build log and source/ELF/
DSO hashes bind this measurement. No reproducible-build qualification is claimed.

Reproduce after Release catkin build and clean tracked source, choosing a NEW
output path (runner refuses overwrite):
`python3 tools/run_accuracy_fde_phase2_integration.py --binary /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark --output results/accuracy_fde_phase2_reproduction_NEW --epochs 35`.
Current safety test commands/LD_LIBRARY_PATH and frozen precision identities are
in the integrated JSON; no new binary copies are needed.

Research <100ms normal/~1s joint2 targets **NOT_MET**; 40ms core/50ms protected
publication and live/hardware qualification **NOT_MET/NOT_RUN**. Both final sides
have zero protected, risk-valid and formal-eligible outputs. Joint2 recovery is
RIGHT_CENSORED with null latency, not zero recovery time; normal NO_FAULT_INPUT.
Default normal35/35 and joint2 34/35 committed (one joint attempt refused), with
normal31 and joint2 33 deadline misses. Do not claim protected realtime readiness.
Preexisting result binding OPEN: lookup key omits hypothesis_tail_used,
axis_tail_used, fault_multiplier_used, noncentrality_used; stored proof hash does
not authenticate every external result field. This phase preserves that behavior,
without claiming to solve all-field result authentication.

The six requested research steps are complete as a research delivery: executable
changes, minimal measured experiments and explicit stopping decisions. Independent
High final review PASS, 218 read-only checks of execution/comparison/test/frozen
baseline identities and report conclusions; no further build or experiment needed.
This completion does not imply realtime or protected qualification. Further work
needs certification/flat-PL cost reduction and independent calibration/risk/formal
evidence; no blind long campaign or relaxation of gates is justified by results.
