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


## 2026-10-10 — FDE operability W1–W5 focused development (PARTIAL_BLOCKED)

Scope is `requirements/FDE_Operability_PL_Performance_Requirements.md`; historical
WP-A–F conclusions above remain historical. Initial actual HEAD was
750953e606a3d55e8777ffafb183981c1ba3edc8, clean tracked worktree; user-owned
untracked requirements retained. No reset, Huber experiment, budget/prior/p_md/AL,
fault scope, selection, rank tolerance or step gate change. Gaussian nominal,
Combined IMU, iSAM2 and fixed lag retained. B-correct correctness source is
958b700; the later performance prototype adds no model changes. Actual run HEAD,
ELF/DSO/config hashes are in execution records; the configure-time Git stamp is
stale and is not the source identity.

W1 (`b1206b0`): the four externally mutable `*_used` scalars were accepted at real
validate/retain/mint/publication consumers: 57 failed assertions before repair.
Exact semantic comparison now covers every external ProtectionLevelV2Result
member against proof.served_result; registry key remains only an index. Positive
fixtures, nextafter/NaN mutations, cross-candidate/arena and self-contained final
bundle cases pass. Existing immutable binding and bundle tests remain passing.

W2 (`b1206b0`): additive FdeDecisionContextV4 binds CompleteRiskInputs to attempt,
window/version/time domain, scope/manifest/model and risk-contract identity. It
validates each source/domain and keeps simulation separate from deployment.
Final selector consumes non-selection evidence and derives selection-extra from
its actual eligible guarantees. A nonempty calibration_id no longer manufactures
model qualification. Production binds exact envelope coverage, but missing
omitted/bridge/history/model evidence stays UNKNOWN. Unit positive context is a
conditional mathematical selector test, **not a raw-stream recovery**. Wrong
identity/source/domain and simulation qualification claims fail. Gate trace
separates numerical, risk, AL, qualification and deadline; unreachable gates are
NOT_RUN. Selector deadline is NOT_RUN and existing actual pipeline deadline
metrics remain authoritative. Exact ledger known charge determines overspend;
individually upward-rounded exported terms must not be summed as a new decision.
Full-precision normal10 and UWB7 ledger/trace are retained in the compact JSON.
Legacy final Inf does not establish mathematical unboundedness.

W3 correctness repairs: an individual PSD history channel need not have resolved
rank when it is never inverted; combined detector W still needs its original
certified rank/nullspace containment. Complementary PSD/true singular, negative
PSD and joint rank-transition negatives pass. Normal32 history rank transition
now yields a finite bound, without jitter or removing history. Other PSD interval
failures remain refused. Separately, a real transaction reproduced protected map
using nominal IMU attitude while frozen factors used CV pose, and mint centre
using nominal position. Map and mint now use the actual frozen proposed Pose3
(`45d5005`, `958b700`); deterministic before/after tests preserve this evidence.

Three fixed original-profile streams were attempted, with raw generated IMU and
ranges entering the production estimator/detector/actions/post/PL/risk/selector
and transaction core. GT is generation/evaluation only. Normal32: 32 best-effort
nominal commits, zero selected conditional FDE results. UWB18: single-epoch
2.25 m bias at epoch7 alarms and produces six finite exclusion candidates, but
zero exclusion commits. One covering candidate has HPL=0.55107088655002923 m
(<HAL=2), with exact known risk charge=4.6900000000000008e-05 >
4.0000000000000003e-05. IMU18: single-epoch 20 m/s2 accel fault at epoch7 does not
alarm; KEEP's dominant velocity correction=0.896043 exceeds unchanged0.25 gate;
PL is NOT_RUN, independent bridge plus full conditional PL recovery is absent.
17 UWB/18 IMU nominal commits are **not** exclusion recovery. Normal10 known
charge=5.6080000000000005e-05; rank-fixed normal32=8.2403846153846172e-05,
both exceed4e-05 even if missing nonnegative terms were proved zero. Recovery
latency is null/RIGHT_CENSORED. No new simulation-positive stream or Gaussian
FDE-OFF paired accuracy claim is made. A4 fails; software blocks are not attributed
to unavailable hardware qualification.

Concrete contract decision proposal (NOT APPLIED): current ledger charges each
hypothesis occurrence's prior times miss bound. Normal10 has nominal3e-05,
p_nm1e-07, allocation9.9e-06 plus miss excess1.608e-05. UWB7 already requires at
least4.69e-05 before unknown terms. No source binding or implementation speedup
can make this satisfy4e-05. If persistent occurrences describe the same physical
fault event, a separately reviewed event partition with authenticated mapping
could remove duplicate *event* charges; this changes the probability contract
and requires explicit approval plus new event-union counterexamples before any
implementation. If occurrences are distinct events, reject this proposal and
retain these profiles as unavailable. This proposal authorizes neither different
priors/p_md nor larger budget. IMU no-alarm/step attribution remains a separate
software/model problem; approving an event partition alone would not solve it.

W4: numerical production/validation/consumption was traced in normal/joint2 fixed
12-epoch snapshots (original35-epoch fault schedule). Evidence produces raw/Gram,
state response and root seals; PL reads the frozen leaf, forms dual certificates,
and immutable result/bundle consumers validate self-contained payloads. The first
root-response reuse direction had zero matching reuse (normal2328/joint272618
fallbacks), same SVD work, and was stopped; flag remains opt-in off. Direction2
(`1182dc5`) stores one continuous immutable proof vector, indexes alias owners in
one arena batch, runs original complete root seal once, then permits exact-owner
read reuse only after successful root validation and away from rank boundaries.
Imported/replaced/tampered/closed-arena and failed roots fall back to original
validation; external/final bundle validation remains full. Old mode cache and
old per-leaf frozen memo remain off. Micro probe: normal960.101→962.492 ms core
(regression), joint231988.332→31485.795 ms (1.57% local decrease); validation
237954→165336. Both unchanged strict comparisons pass. Small timing differences
are not a significant-gain claim, and this prototype remains opt-in off.
No second speculative memo direction is added. Final35 pairs and safety results
are appended below after completion; their failures remain part of the evidence.

Reproduction, after building the existing Release workspace:
```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target realtime_performance_benchmark test_integrity_v2 test_square_root_context test_p0_04_risk_oracle -j2
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
# Strict production research replay: unknown evidence never grants protection.
UWB_IMU_PL_SCENARIO=noiseless "$BIN/realtime_performance_benchmark" config/fde_joint_order1.yaml results/fde_NORMAL_NEW 32
UWB_IMU_PL_SCENARIO=uwb_recovery "$BIN/realtime_performance_benchmark" config/fde_uwb_order1.yaml results/fde_UWB_NEW 18
UWB_IMU_PL_SCENARIO=imu_recovery "$BIN/realtime_performance_benchmark" config/fde_imu_order1.yaml results/fde_IMU_NEW 18
# B-correct paired timing; clean tracked source, NEW destination, no old exhaustive flags.
python3 tools/run_accuracy_fde_phase2_integration.py --reference-model b-correct --binary "$BIN/realtime_performance_benchmark" --output results/fde_PAIR_NEW --epochs 35
# Conditional context boundary only; not a simulation-conditional recovery entry.
"$BIN/test_p0_04_risk_oracle" --gtest_filter=FdeOperabilityRisk.*
"$BIN/test_integrity_v2" --gtest_filter=FdeOperabilityBinding.*:FdeOperabilityRank.*:FdeOperabilityReference.*
# Default prototype OFF; opt-in reader/owner negative tests:
UWB_IMU_PL_BATCH_FROZEN_PROOFS=1 "$BIN/test_integrity_v2" --gtest_filter=FdeOperabilityBatchProofs.*
```
Frozen Gaussian nominal continues through the existing nominal replay commands
and original configs documented above; no nominal tuning is included here.
There is currently **no accepted simulation-conditional raw-flow command**.
Such a caller needs generator-derived omitted/envelope/bridge/history/model
bounds bound to the exact attempt/model/contract, with production formal and
publication gates false. A literal known/validated flag or calibration name is
not an admissible substitute. integrity.csv, diagnostic_attempts/stages.csv,
candidates.csv and timing.csv expose actual commits/refusals/deadlines. A finite
but late result remains unprotected. No recovered latency or hardware/formal
qualification is claimed.


Final frozen integration (`4aa795e`, numerics ELF/DSO from `1182dc5` build):
normal/reference35, normal/optimized35, joint2/reference35, joint2/optimized35;
exactly four benchmark processes,140 attempts, profiler off. Both original
strict comparisons PASS, zero numerical contract mismatches, same hypothesis/
action/commit counts. Per-side deadline misses remain visible (normal31/30,
joint233/33); no comparator relaxation. Normal mean core205.102505→202.941219 ms
(1.05%, no significant gain claim); joint2 mean2613.468418→2400.921788 ms
(8.13% measured net decrease). Evidence851.300048→744.691796 ms and flat PL
1092.354543→1001.214316 ms account for the structural benefit. Joint2 full frozen
validations2497242→1344738, with1152504 exact-owner reuses; generated hypotheses
1272018 and actions439 unchanged. Root seal, indexing and construction are
included in core. The returned arena lease keeps proof storage until the end of
an epoch: its final destruction is outside the core sample, so complete process
wall93.126784→84.813356 s (8.93% decrease, includes generation/logging/teardown)
is also retained. Do not label core alone as including all lease destruction.
Normal entire process7.244267→7.142598 s. Short safety tests overlapped the start
of the first normal run; that limits interpretation of its small gain. No
benchmark repetition was added to hide this limitation.

Worst optimized normal is attempt30 at358.174065 ms; worst joint2 is fault
attempt12 at25163.722096 ms (reference25675.154478 ms), retained along with its
refusal. Joint2 fault action fan-out and full immutable payload work dominate;
no failed attempt or history transition was dropped. P95 normal326.532533 ms,
joint22981.347125 ms;35 samples establish neither P99 nor rare-event probability.
Normal35/35 and joint234/35 nominal commits, zero protected/risk-valid/formal
outputs on both sides. <100 ms / ~1 s exploratory targets and40/50 ms deployment
thresholds are NOT_MET. Finite late candidates do not receive timely protection.

Final safety:102 distinct deterministic tests cover all original69 plus new
binding/risk/map/rank/owner tests and directly affected dual-channel/nullspace/
risk boundaries. Original69 test names were checked as a subset of this run.
One historical default-off invariant failed under the opt-in batch environment;
only that test was rerun with BATCH_FROZEN_PROOFS=0 and passed, without a code
change. Its failure/XML is retained. Remaining101 passed initially; the new
batch-owner test also passed separately in reference and optimized environments.
The expanded mathematical coverage is required by the public dual-channel rank
repair, not an old whole-project acceptance campaign. Test filters, ELF/DSO
identities, XML and single rerun are recorded in the compact result JSON.

Use `UWB_IMU_PL_BATCH_FROZEN_PROOFS=1` explicitly for the measured continuous
proof path; absent flag retains B-correct storage/validation. Root-response reuse,
mode cache and old frozen memo remain off. This provides an opt-in measured
joint2 cost reduction, not conditional recovery or deployment qualification.
A1/A2 binding boundaries and negatives are closed; A3 trace exists with noted
legacy Inf/deadline boundaries; A4 fails; A5 remains fail-closed; A6 paired timing
and numeric comparison completed; A7 code,commands,102-test evidence and compact
`benchmark/fde_operability_20261010.json` delivered. Overall **PARTIAL_BLOCKED**:
UWB over-budget selection and IMU no-alarm/step path still prevent actual
conditional exclusion plus three subsequent commits. Contract proposal above is
pending, not applied; no FUNCTIONAL_ACCEPTED or goal-complete claim.

The frozen-pose repair affected PL geometry, so only the affected UWB18/IMU18
short flows were rerun once on the final library (`final_strict_*` execution
records); no additional35-attempt pair. UWB epoch7 statistic440.7804520984601 >
threshold121.34881015252451, revised finite covering HPL0.55117685012311124 m,
known risk4.6900000000000008e-05 still exceeds4e-05. IMU epoch7 statistic
0.18701637172207716 <121.34881015252451, same velocity step0.896043 >0.25.
Actual18-epoch commit counts remain17/18, exclusion commits0/0. Generation is
50 ms epochs and10 IMU samples per epoch at5 ms; truth fault labels never enter
selection. Earlier geometry values remain as historical before-fix evidence.


Continuation — actual IMU reference/row coordinate defects (`88e9ffa`, `ee87e43`,
`a1ef121`), not a change to the probability contract:

A real prepared CV transaction reproduced IMU analytic sensitivity at the
measurement-only nominal point differing by4.156136% from the actual frozen
CombinedImuFactor Jacobian. Its finite-difference oracle used that same wrong
point, so mutual agreement alone had missed this. Both now use the six actual
frozen Values; missing/wrong-typed/nonfinite frozen values refuse. Historical
material uses its recorded actual states, and no-Values legacy semantics remain.
The new deterministic positive compares directly with the physical factor's
bias Jacobian, and changing unused nominal attitude cannot change the result.
A separate physical-Jacobian assertion reproduced1.98403% raw mismatch: H/z
were in GTSAM's upper information-root coordinate while Combined IMU raw rows,
current sensitivities and historical material used a lower Cholesky root.
Current and historical IMU material now use the actual Gaussian root; nominal
backend factors/weights and H/z are unchanged. Generator raw-column recovery
also still assumed a lower root: shared-material regression caught it, and
coordinate-independent unwhitening now restores the physical column. No
rank threshold, jitter, scope, gate or budget change accompanies these fixes.

Regressions were expanded only because these shared IMU physical-response paths
changed. Point repair5/5, row-coordinate direct tests21/21; historical
parameterization9/10 initially, then shared-material failure rerun identified
and closed the generator conversion:1/1 history and2/2 current tests passed.
The new-samples-only support negative plus existing interval-interior negative
pass2/2. Opt-in batch-owner negative passes1/1. Original failed XML and the
initial wrong executable-path launch log remain. No second69/102-item campaign
was run, and the old102-test source/library identity is not relabelled current.

S3 causal calculation on actual epoch7 (one680 kB lossless numerical snapshot;
pre-fix snapshot retained separately): corrected physical unit accel-x response
has whitened norm2.2694023873263407, parity norm0.017456591641680029,
Gamma=0.00030473259174437306, unit state-step norm0.047398551513348604.
Only5.9169226e-05 of its whitened energy remains in parity; nearly all is fitted
by state changes. For the full-interval declared template, amplitude20 gives
lambda0.12189303669774923 and step0.947971030266972. Original observed S3 still
has statistic0.18701637172207716 < threshold121.34881015252451 and dominant
velocity step0.896043. Spectral/independent pivoted-QR state differences are
<=2.04e-12, parity differences<=1.18e-10 across six axes. This is insufficient
immediate detector power, not evidence that increasing the0.25 gate is valid.
Under the conditional chi-square model, p_md=.001 needs lambda136.48083127273105;
linear extrapolation would imply accel669.2315 m/s2. **No new injection uses
that extrapolation**: it lies outside the observed linearization domain and
preintegration covariance also depends on the trajectory/input.

The current scalar step gate takes an identity-scaled concatenation of rad,
local m, world m/s, m/s2 bias and rad/s bias components. It is not a physically
normalized metre limit. This turn identifies its units without inventing new
block scales or relaxing the frozen rule. Current IMU18 still has18 nominal
best-effort commits, zero exclusion commits, zero conditional risk-valid or
protected outputs; latency remains null/RIGHT_CENSORED.

Raw-input support is also a concrete model gap. Existing runner corrupts the10
new samples, leaves the old boundary sample healthy and carries the last bad
sample into the next epoch's first trapezoid. Independent reintegration gives
raw dp/dv=.025 for the whole-interval template versus.02381578947368421 for
new-samples-only; best scalar fit leaves7.6028023944664% whitened residual.
The inherited boundary alone gives5% of full-interval dv. This is not the
manifest's currently implemented one-factor single-parameter response, which
assumes the same bias in every interval mean. The original S3 therefore cannot
qualify that full-interval PL merely by being named imu_recovery.

Additional contract proposal (NOT APPLIED): review whether IMU events mean a
preintegrated factor-bias template or a raw sensor-sample occurrence. For the
latter, declare exact sample support and the adjacent-factor boundary exposure,
with corresponding coverage/history/repair and probability-event identity.
The current manifest declares single_parameter / same_epoch / one_imu_interval
and excludes imu_imu. Do not silently widen this into a new two-factor fault
scope or pass actual truth labels to selection. The existing risk-event proposal
is also pending; known UWB charge4.69e-5 still cannot close4e-5. Neither proposal
is implemented by these physical coordinate fixes.

Numerical production/validation/consumption table (same structure in normal
and joint2; current joint2 targeted pair confirms the owner path):

| Numerical object | Production | Validation | Consumption / retained boundary |
|---|---|---|---|
| frozen H/z/C and spectral solve | actual frozen pose and Gaussian rows | existing frozen numerical contract and QR/rank fallback | FD, state increment, physical output map |
| hypothesis raw response / Gamma / G | Evidence physical maps and projection | continuous owner root runs full original seal | exact-owner cached PL leaf read; imported/closed/near-rank uses full validation |
| weighted dual W / nullspace / noncentrality | PL current/history channels | original dual numerical certificate | new W differs from pooled Gamma; the stopped root-response shortcut cannot substitute it |
| result and final payload | PL candidate and immutable carrier | full result semantics and self-contained payload validation | retain/mint/publication; no external mutable-copy waiver |

B-correct **958b700 and the140-attempt timings are now superseded** by the
necessary IMU corrections. Their8.13% joint2 result remains historical, not a
current-model speedup claim. Current mathematical B-correct is a1ef121 (subsequent
test/report commits do not change it). One targeted joint2 reference/optimized
pair of12 epochs with the original35-epoch fault schedule passes the unchanged
strict comparator, zero numerical mismatches:31,560.910778→31,372.741081 ms total
core,0.60% inconclusive difference. Complete frozen validations237945→165327,
with72618 owner reuses; hypotheses92616/actions416/commits11 equal. Evidence
mean129.417161→116.266462 ms, flat PL1583.542073→1587.124470 ms. No significant
current full-core gain is claimed, and batch prototype remains opt-in off.
No additional35-attempt processes were launched to evade the140-attempt limit.
Current-model final35 comparison is consequently unverified, and A6 is explicitly
incomplete for the latest model. The compact JSON distinguishes historical
102-test/140-attempt results from current targeted results.

Additional reproduction (existing programs; new output paths):
```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target test_integrity_v2 test_history_fault_parameterization candidate_replay realtime_performance_benchmark -j2
"$BIN/test_integrity_v2" --gtest_filter=FdeOperabilityImuReference.*:IntegrityV2ImuOracle.*
"$BIN/test_history_fault_parameterization" --gtest_filter=HistoryFaultParameterization.WindowAndHistoryColumnsAgreeOnSharedMaterial:FdeOperabilityImuSupport.*
UWB_IMU_PL_SCENARIO=imu_recovery UWB_IMU_PL_REPLAY_EXPORT_DIR=results/IMU_NEW/replay UWB_IMU_PL_REPLAY_ATTEMPTS=7 "$BIN/realtime_performance_benchmark" config/fde_imu_order1.yaml results/IMU_NEW 18
"$BIN/candidate_replay" results/IMU_NEW/replay/attempt-7.bin results/IMU_NEW/power.csv 1 1 imu-power
python3 tools/run_accuracy_fde_phase2_integration.py --reference-model b-correct --workload joint2 --fault-schedule-epochs 35 --epochs 12 --binary "$BIN/realtime_performance_benchmark" --output results/JOINT_SNAPSHOT_NEW
```
Overall remains PARTIAL_BLOCKED. These are real implementation repairs and
specific causal/coverage evidence, not an IMU or UWB conditional recovery claim.

Expected missing-key/type exceptions alone become frozen-point refusals; other
exceptions propagate. That filter rebuilt and its two directly related tests
passed. It does not change valid-input numerical products or authorize broader
fault events. No pending contract question is treated as approval. Remaining
work includes actual conditional P-N/P-U/P-I recovery, complete applicable
model/omitted/history/bridge evidence and current-model final integration; none
is replaced by these software fixes or the historical performance result.

Further terminal-status repair (same mathematical B-correct a1ef121): a real
raw IMU/UWB pipeline fixture with a valid measurement model returned an
unavailable final Inf PL labelled UNBOUNDED, despite having no unboundedness
proof for the final reference. The new deterministic test failed before the
change (`status_before.xml`, one failure). `IntegrityOutput` now defaults to
NOT_COMPUTED; pipeline finalization classifies no candidate PL execution as
NOT_COMPUTED and candidate work without a final result as INDETERMINATE.
Existing enum ordinals, numeric matrices, risk contract, selection and
publication refusal remain unchanged. UNBOUNDED is retained as an explicit
status, not inferred from a final placeholder. Candidate bounds remain in the
existing candidate audit rather than becoming final protected bounds.

Both test_integrity_v2 and realtime_performance_benchmark rebuilt successfully.
The eight directly related operability/profile/publication tests passed in
`status_after.xml`; no full safety campaign or new 35-attempt processes were
run. An initial launch loaded the old DSO through the existing LD_LIBRARY_PATH
and exited before any test; `status_after.log` retains that symbol error. The
successful launch explicitly prepended the current private build library:
```sh
LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib:$LD_LIBRARY_PATH "$BIN/test_integrity_v2" --gtest_filter='FdeOperability*:FdeProfiles.NominalPipelineAndFaultCensusFollowResolvedScope:P106SimulationAcceptance.PublishCallNotAnalysisControlsDeadline'
python3 tools/report_fde_operability.py --results results/fde_operability_20261010 --output docs/benchmark/fde_operability_20261010.json
```
The JSON now explicitly marks A3 partial: the five selector gates and separate
pipeline diagnostics are present, but a single dependency-bearing trace across
every section 4.4 gate is not complete. Correct terminal Inf classification
does not by itself establish that wider trace or any recovery.

The unchanged risk contract remains a concrete functional blocker across
three consecutive goal turns, independent of the newly repaired software
defects. Reading the existing strict_noiseless ledger, without another run,
gives known charge4.0000000000000003e-5 at epochs1–3 and
4.1680000000000008e-5 at epoch4, versus the binary64 budget
4.0000000000000003e-5. Thus even the earliest normal KEEP plus three subsequent
epochs hits a known over-budget ledger on that original profile. This is
probability-contract evidence from the retained flow, not a new-model numeric
qualification. The UWB alarm at epoch7 separately charges4.69e-5; missing
evidence cannot lower either charge. The IMU raw-sample support and weak parity
response blockers above also remain. Pending risk-event and IMU support
proposals have not been authorized or applied. No earlier-fault shortcut,
budget/AL/step change, history removal or hardware qualification claim resolves
these blockers. Overall remains PARTIAL_BLOCKED, recovery latency
null/RIGHT_CENSORED, simulation-conditional flow not implemented and current
full 35-attempt integration unverified.

### 2026-10-10：获授权的物理事件概率合同研究，供再次审查，未实施

用户已授权研究“持久故障按物理事件分区计费”，要求实施前审查具体映射；
未授权替换生产合同、风险配置、先验或发布逻辑。以下推导与离线对照属于
研究交付。原功能结论仍为 PARTIAL_BLOCKED，formal_eligible=false，
publication_protected=false；没有新的条件恢复或定位资格声明。

研究结果先行：**现有先验信息不足以证明 epoch7 存在可安全删除的概率收费。**
按质量守恒的事件合并给出相同的已知风险4.69e-5。原实现确有数值覆盖空间的
重叠，但没有“这些标签指向同一个物理概率事件”的来源证明。只重写
persistent-constant 项，即使理想化地免去其全部收费，仍有4.13e-5的下界；
所以它单独不能解决4e-5预算。下面给出具体反例、正确合并条件及接口提案。

研究只新增默认关闭的 benchmark 诊断导出
`UWB_IMU_PL_RESEARCH_HYPOTHESIS_AUDIT=1`，并使 hypotheses.csv 以17位有效数字
往返 binary64。它增加日志工作，不改变配置中的概率值、排序、发布或计算
证书；不能把此次诊断运行用于性能收益比较。构建及一次7-epoch前缀成功，
保留原18-epoch故障时间表，仍在epoch7注入原2.25m单epoch UWB故障。
数据路径为 `results/fde_operability_20261010/event_contract_snapshot`，取该
同一短流的健康epoch4和报警epoch7两个固定时刻。没有复制二进制或另建快照
归档，没有新增35-attempt流程。源代码9ebbf99及二进制、DSO、配置、manifest、
命令、环境、CSV哈希都保存在原JSON的 `event_contract_research` 项。

1. 独立账本复核与反例

epoch7实际导出168条假设，三种形状各56条：8个锚点×7个起点。
`HypothesisGenerator::generate` 对每条均设置 pi=1e-4、beta=1e-3，等额
allocation=5.8928571428571424e-8；不是给8个“任意时序的源事件”各一份先验。
参考 `src/uwb_imu_pl/integrity/hypothesis_generator.cpp` 的模式生成、
`risk_budget_audit.cpp::buildRiskLedger` 和
`fde_manager.cpp::decideImpl` 的真实计费及选择上下文。

| 项 | epoch7的C++全精度导出 | 来源/重叠处理 |
|---|---|---|
| nominal | 3.0000000000000004e-5 | 原三轴tail；逐项导出向上舍入 |
| p_nm | 9.9999999999999995e-8 | 原非名义、不可归属质量上界 |
| hypotheses allocation | 9.9000000000000001e-6 | 168项实际allocation之和向上舍入 |
| miss excess | 6.9000000000000025e-6 | sum pi*max(0,beta-alpha)，与allocation配合，不是另一份完整beta |
| selection extra | 8.4703294725430034e-22 | 选择器实际eligible guarantee groups；保留其保守舍入 |
| envelope | 0 | 此冻结registry完整exact覆盖的结构证明 |
| bridge/history/model/omitted | UNKNOWN | 未伪造零证明；完整总风险仍未知 |
| once-rounded known total | 4.6900000000000008e-5 | 未舍入项高精度相加后只向上舍入一次 |

`tools/research_physical_event_contract.py` 使用 Fraction 精确解释每个binary64
输入，独立计算 alpha=r/pi、allocation与miss，nominal从配置的原始单轴tail
乘3，而不是再相加已上舍入的导出值。两个时刻的 once-rounded 结果与C++
逐位相等。epoch7无tail截断，alpha≈0.0005892857142857142 < beta；因此
每条charge=pi*beta，168项fault部分=16.8e-6，再加nominal、p_nm及selection。
epoch4有96条，alpha≈0.00103125 > beta；fault部分9.9e-6、miss=0，已知
总量4e-5，但四项UNKNOWN仍不允许完成风险通过。

以下是**概率合同的确定性反例，不是新数据集或实际HMI频率估计**：
令168个“源/起点/实际时间规律”物理事件互斥，各有质量原pi=1e-4，剩余
0.9832为正常。每个原单项先验均成立，总概率恰为1。给每个事件一个条件
漏检集合，质量原beta=1e-3，则故障漏检联合质量16.8e-6。若把每个锚点的
21条标签直接合成一个上界1e-4的源事件，得到0.8e-6，低估21倍。
即使物理时间规律在最后一个观测上无法区分，它们仍可在未来轨迹上互斥。
现有接口没有禁止这个概率空间，不能仅凭同一个source字符串排除它。
这个反例证明“不从现有上界推出更小联合上界”；它不声称所有实际Gaussian
几何都能同时达到该最坏HMI上界。

这些“已知收费下界”指当前保守合同要求计入的上界金额，不是实际HMI概率
的下界。4.69e-5证明当前合同无法出具预算证书，不证明实际物理风险已经
超过4e-5。单个报警快照也不能估计故障发生率或校准联合先验。

相反，如果有来源证明21个标签确实都是**同一个物理事件E的数值覆盖**，且
原1e-4本来就约束P(E)，则保持同一先验，21*pi*epsilon可改为
pi*max(epsilon_1,...,epsilon_21)。脚本验证这一数学正例；它不能作为当前
epoch7已有该联合先验的证据，更不是生产条件恢复正例。

实际末端每个锚点的epoch-independent、persistent、ramp三条audit的Gram、
slopes与noncentrality相同；ramp的第二列时间系数在该最后观测为零。
这8组 numerical aliases 在JSON中明确列出。它们支持减少数值重复的讨论，
但不证明“单次故障现在结束”与“故障此时开始并将持续/变化”的发生事件
相同，不能把数值hash等价变成概率先验来源。

2. 互斥、完备分区的具体候选定义

候选概率空间是**无噪声物理故障历史**及名义噪声，不是拟合出来的标签。
保护时刻t固定，范围H_t包含该窗口及仍影响输出的history/carry-in；时间基准
仍是 per_protected_epoch。以下定义从物理历史到唯一cell的确定性映射，
仅用于合同推导和离线覆盖证明，不能用真值来帮助实际选择。

- E0：H_t上没有任何source处于物理故障episode。
- 单UWB cell：确切一个锚点、确切一个episode、起点桶s、终止/持续类别d、
  时间规律ell及未限定幅值参数集合。起点桶含before-H_t carry-in及窗口内
  各测量时间桶；桶内起点不能随意当成同一时刻，需相同原始响应或额外误差
  上界。d区分区间内停止、保护时刻停止、以及仍在持续的right-censored事件。
  规律按确定规则划为常量、非零斜率仿射、其他；zero-slope affine归
  constant。episode由物理故障状态定义，不按测得的幅值切割；误差暂时过零
  不结束episode。故障状态下零响应也保留在相应cell，不因拟合为零免去
  先验收费。单观测上相同响应的不同持续类别仍是不同cell，
  可以由同一个数值witness覆盖。
- 单IMU cell：确切一个器件轴、确切一次episode、起止及样本支持集合。
  当前可验证的数值模型仅是一个preintegrated interval的单参数响应。
  原始样本边界相邻factor暴露问题尚未解决；不能因事件分区把它判为完整覆盖。
- 组合cell：确切一个UWB episode和一个IMU episode组成的无序集合，两者
  起止及支持分别保留。“确切”同时要求其他物理source健康。不同起点、
  持续时长或参数组合不能被一次最优拟合替代；参数共享/结构独立性须保持。
- E_perp：以上受支持定义以外的所有历史，包含两个UWB、两个IMU、多轴
  common cause、同源多episode、未覆盖的停止形状/桶内变化、旧history
  carry-in缺失或任何缺少数值映射的cell。其质量是omitted/escape，仍UNKNOWN，
  不把“未模拟”当作概率为零。bridge与模型误差也保留原证据渠道。

每个物理历史有唯一的source集合、最大连通episode及上述确定标签；若它
不能满足某一受支持cell的全部限定条件，就进入E_perp。因此cells互斥，
与E0/E_perp联合完备。该完备性是**分类规则**的性质，不是声称当前generator
数值覆盖全部cells。完整幅值/持续时间的实现集合必须另作覆盖证明。

物理cell与数值hypothesis ID不是一一对应。取真实epoch7的anchor1：
onset1到末端的constant cell由leaf2(persistent)与leaf3(ramp)覆盖；非零
斜率affine cell由leaf3覆盖；只影响早期第1次观测、已结束的cell由leaf1
覆盖。只影响末端第7次观测的cell由leaf19/20/21共同覆盖，包含多种未观测
未来持续情况；它不是三个互斥cell。JSON中由CSV实际生成这组映射及其他
source的全部leaf IDs。连续起止时间可细分物理cell，但映射只在每个cell
所有实现具有声明的原始观测/历史支持时成立；缺证据的边界仍是UNKNOWN。
这些cover关系不赋予cell任何额外先验。上面的168互斥原子反例是另一种
原prior谓词允许的、按latent episode规律细分的概率空间，不是声称这个
数值cover列表已经构成了168个互斥物理事件。

当前single-epoch和onset-to-window-end的constant/ramp模型并不覆盖任意
中途停止。脚本的f=(0,1,1,0)不能落入任何一个当前单epoch或到末端仿射
子空间，必须留在E_perp。f=(1,1,1)也不能仅由latest-onset=(0,0,1)覆盖。
直接拼接所有起点B矩阵扩大成一个任意参数空间，同样改变fault scope：
detector=[1,1]、protected=[1,-1]时，两个单列各有有限slope=1，拼接后
(1,-1)却是detector零空间中的危险保护方向。正确对象是子空间的union，
不是它们的span；原rank/nullspace判定和fallback不能删除。

事件概率接口使用明确的谓词和约束，不只一个prior数值。令q_e=P(E_e)，
Q={q>=0, sum q<=1, Mq<=p}，M描述每份已有先验证据实际约束哪些cells。
这里q只列受监控fault cells；完整空间还包含q0、q_perp且
sum q+q0+q_perp=1。不能以这个投影的sum<=1省略q_perp风险。
若pi_h只约束互斥的onset/law cell，则每个q_h<=pi_h，联合上界只能用sum pi_h；
若证据约束source在H_t上任意一次故障的union，则加入相应sum q_e<=pi_source。
后者当前为UNKNOWN，不能把原1e-4重解释成这个上界来闭合预算。

相关性由joint cell质量体现，不自动相乘。例如P(U)<=1e-4、P(I)<=1e-5，
允许P(U and I)=1e-5；乘积1e-9低估可能联合质量一万倍。若两个已认证
**marginal**确实约束同一对primitive events，可用
q_U_only+q_UI<=p_U、q_I_only+q_UI<=p_I及q_UI<=min(p_U,p_I)。
若原输入只是exclusive-mode上界，则不能擅自加入这些marginal限制。
脚本给出完全相关和互斥joint分配；未使用独立性假设。

同一持续事件在**一次保护输出的不同时间假设**下可能重复表示，但跨不同
保护epoch的HMI不是同一个结果事件。序列风险仍须计算
P(union_t HMI_t|E)：三个各有beta条件质量的互斥漏检集合给出3*beta，不能
以“源故障一直是同一个”只收一次beta。新提案不把per-epoch预算变成任务期
预算，也不从发生率凭空换算carry-in占用概率。

3. 条件HMI、漏检与PL推导，以及选择边界

证明域是固定、有效的线性Gaussian模型；需要对物理故障/幅值条件化仍成立
的噪声过界、同一accepted output event、同一最终均值与保护量。随机生成
冻结线性化本身的部署资格不由此证明，非线性余项、history与bridge等证据
缺失仍使完整结果UNKNOWN。

对一个已有leaf h和输出group g，复用实际Gamma_{h,j}、G_h、detector阈值和
自由度；原beta_h不变。每个有效通道求原Lambda_j，使
F_ncx2(nu_j,Lambda_j)(tau_j)<=beta_h。W_h=sum_j w_j Gamma_{h,j}/Lambda_j，
w_j>0且sum w_j=1，仍要求原PSD/rank及ker(W_h)包含于ker(G_h)的证书。
保持名义协方差、rho和轴tail：

    L_{h,d} = sqrt(g_{h,d} W_h^dagger g_{h,d}^T)
              + rho_{h,d} + Phi^-1(1-alpha_h/6)*sigma_{g,d}

若真实参数f满足f^T W_h f<=1，Cauchy-Schwarz与nullspace证书给出上式的
确定性偏差项；三轴Gaussian tail的union不超过alpha_h。
若f^T W_h f>1，由权重和为1，至少一个通道满足
f^T Gamma_{h,j} f>Lambda_j。实际accepted event必须包含该通道通过条件，
故其漏检概率不超过原beta_h，无需假设current/history通道独立。
两个参数区域互斥，因而对每个固定f的HMI条件上界是
epsilon_h=max(alpha_h,beta_h)，不是删掉漏检项，也不是alpha+beta。

对每个物理cell e、每个有效输出group g，要求一个**不依赖观测噪声选标签**
的全覆盖证明：每个f in F_e至少属于一个原leaf响应空间，且raw factor、
history、边界、作用时间及输出均值都一致。令C(e,g)为这些witness：

    L_{e,g,d} = max_{h in C(e,g)} L_{h,g,d}
    epsilon_{e,g} = max_{h in C(e,g)} max(alpha_h,beta_h)

对任一固定实现f，取由其物理实现决定的一个cover witness，以上max包含
它的L和epsilon，因此对F_e上的任意幅值、起点、时长和未知分布都一致成立。
这不是按观测择优挑最小PL，没有遗漏其他实现。所有cells的最终axis bound
仍取max，HPL=sqrt(L_x^2+L_y^2)、VPL=L_z，原AL不变。witness集合或模型
证据缺失就UNKNOWN；危险零空间可证明无界；数值不确定不能伪装成有限。

只有一个eligible output group时，完整风险保守上界为

    R <= 3*alpha0 + p_nm + B_bridge + B_history + B_model
         + B_omitted + B_envelope + sup_{q in Q} sum_e q_e epsilon_{e,g}.

外部项仍按原证据计入；其中有重叠但没有集合证明时继续union相加，不擅自
相减。简单独立cell上界时r_e=qbar_e*alpha_e，miss excess为
qbar_e*max(0,beta_e-alpha_e)，两者和qbar_e*max(alpha_e,beta_e)。若以后改变
分配，必须在同一个Q及原beta/总预算中证明新的r_e，重新生成所有quantile
与完整结果证书；本研究没有这样调参或实施。当前对照复用原alpha、beta
及全套数值PL，故conditional数值PL完全不变，但新风险身份不能借用旧发布
证书宣称通过。

多个可选输出需要保留**实际数值guarantee groups**，不能因事件源相同就
合并输出。没有原common-reference/accepted-event/time/output/triangle
transfer证书时，仍为singleton。对噪声驱动选择，各group失败事件union：

    u_e = min(1, sum_g epsilon_{e,g})
    U_fault_selected = sup_{q in Q} sum_e q_e u_e.

事件cell互斥允许对e只收一份q_e，但不意味着对多个数据驱动输出只取最小
epsilon。nominal group tails同样需要联合证明；没有共享失败事件证明时用
min(1,sum_g 3*alpha0)，不能只收一次nominal tail。有效的共享group可使用
已有共同参考证明的一份epsilon；本研究未制造任何新共享group证书。

可诊断地拆出base= sup_Q sum_e q_e max_g epsilon_{e,g}以及
selection-extra=U_fault_selected-base>=0，并单独保留额外nominal selection
tail；选择器必须从真实eligible groups计算，caller不得提供这个charge。
向上舍入须在最终完整表达式上完成，不能引入负的selection-extra抵消miss。
这是一份新版本合同设计，尚未接入当前消费者。

特别是不能把新physical tail charge降低后，仍减去当前消费者的旧9.9e-6
allocation reserve，来声称旧selection-extra=0。原
ActionGuarantee.epsilon_budget=风险allocation总量；将其与新事件联合证明
衔接也需再次审查。若保持旧S选择reserve，一个保守兼容路径必须至少保留
max(new_tail_charge,S)+new_miss_excess，而不是仅new_tail+new_miss。
本次mass-preserving对照两边金额完全相同，没有绕过这个边界。

4. 固定快照旧/新对照及可用性

| 固定时刻 | 旧leaf数 | 按source合并的数值covers | 原先验导出的source联合上界 | 旧/新已知风险 | 旧/新可用性 |
|---|---:|---:|---|---|---|
| 健康epoch4 | 96 | 8，每组12个cover | 每组sum12*pi=0.0012 | 两侧4e-5，逐位相同 | 四项UNKNOWN，均不可用 |
| 报警epoch7 | 168 | 8，每组21个cover | 每组sum21*pi=0.0021 | 两侧4.6900000000000008e-5，逐位相同 | 已知超预算且四项UNKNOWN，均不可用 |

健康epoch4来自同一个uwb_order1故障前缀，不替代原joint_order1正常流验收。

这里的source合并仍保留所有原始leaf/time-shape集合与数值证书，只把先验
质量上界求和，再乘该组最大的uniform epsilon；不是把21个参数合成一个
新参数，也不把source union先验假设成1e-4。此例同组epsilon相同，合并恰好
无净风险收益；不同epsilon时sum(pi)*max(epsilon)甚至可能更保守，脚本另有
确定性测试。更精确时可保留小cell或Q的约束，而不能为收益删cell。

报警候选的原conditional HPL=0.55117685012311124、VPL约0.744743m
（全精度HPL在selector trace中，候选CSV保留原显示精度）低于原HAL=2/VAL=3。
两侧原alpha/beta、detector、所有覆盖leaf及PL表达式不变；没有生成一个
“新的、已验证可发布PL”。ledger超预算仍阻止选择，candidate_bound有限不
改变最终输出或deadline，UNKNOWN也没有被置零。

即使给persistent-constant最有利的零收费，其余112条epoch-independent/
ramp原beta floor为11.2e-6，加nominal及p_nm约30.1e-6后仍是41.3e-6。
仅persistent重写达不到预算。对一般source联合先验，在当前统一beta下，
若希望只从fault项闭合，需要已认证总source mass不超过约
(4e-5-3e-5-1e-7)/1e-3=0.0099（还未扣selection和UNKNOWN）。
现有可证明上界0.0168。0.0099是**证据需求的阈值**，不是建议将先验改成
这个数；本研究没有人为挑更小的prior、缩fault scope或提高AL来报成功。

5. 待审查的软件接口与决策

以下是设计提案，不是生产头文件或已实现的入口：

```cpp
struct PhysicalEventContractProposalV1 {
  ContractId contract_id;                  // new namespace; never old risk ID
  ScopeId scope; ManifestId manifest; ModelId model;
  TimestampNs information_cutoff;
  PhysicalHorizon horizon;                 // includes carry-in/history
  std::vector<ExclusiveCellPredicate> cells;
  std::vector<PriorConstraintEvidence> constraints; // explicit M,q,p,time basis
  CompletePartitionProof partition;        // E0 + covered + explicit E_perp
};
struct EventCoverWitnessProposalV1 {
  CellId cell; ActionGuaranteeGroupId group;
  std::vector<ImmutableLeafProofId> covers;
  AllRealizationsCoverageProof raw_history_sample_support;
  FinalReferenceIdentity reference;        // mean, cutoff, quantity, arena/root
  UniformAcceptedEventProof acceptance;    // same tests as actual consumer
};
struct SelectedEventRiskProofProposalV1 {
  PhysicalEventContractProposalV1 contract;
  std::vector<EventCoverWitnessProposalV1> witnesses;
  CompleteExternalRiskEvidence external;
  ActualEligibleGuaranteeGroups groups;    // selector-owned, not caller fee
  ExactRiskBound base, selection_extra, nominal_selection_extra, total;
  EvidenceDomain domain;                  // simulation cannot become formal
};
```

审查后若实施，使用加版本的sidecar/独立验证入口（例如FdeDecisionContextV5），
保留旧V1–V4 ABI及旧默认合同。验证需要检查source/model/manifest/window/
cutoff/time-basis绑定、cell无重叠与完备性、每个实现的raw/history/sample支持、
遗漏质量、uniform tail/miss证据、实际output groups、最终均值及完整结果语义。
拒绝伪造联合先验、校准ID冒充证据、跨arena/candidate/time引用和UNKNOWN。
新risk identity及最终证书必须绑定事件映射、Q、coverage witnesses和真实
groups；registry key或root数值相同不能代替这些语义。生产publication启用
资格另行完成，离线研究接口没有retain/mint/publish权限。

当前可提交的具体映射是“全部168条原covers保留，8组source union质量用
sum原pi”，这是无净风险收益但可证明守恒的基线。**尚不建议批准一个能
闭合epoch7的replacement**。下一决策是审查原prior究竟意指exclusive
onset/law occurrence，还是有独立依据的source任意故障/episode联合概率。
需要确切predicate、时间基准及概率来源才能增加联合约束；仅批准数值合并
或改字段名不够。若证据只能支持当前per-occurrence边界，就保留旧profile
CONFIGURATION_INFEASIBLE，并结束这一风险收益方向。

IMU独立诊断继续保留：20m/s²的整interval模板估算lambda约0.1219，实际
statistic约0.187，低于阈值121.348810；KEEP step约0.896>0.25。达到原
p_md的线性外推幅值约669m/s²不在已验证线性域，不作为新注入或解决方案。
raw new-samples-only模板与整interval模板的7.6028%残差及次epoch边界5%
暴露仍未由事件分区修复。下一软件/模型审查先定义精确支持及相邻factor
覆盖，再用原固定快照检查可信线性域的parity/state响应；需要改变IMU事件
scope时另审查。不能提高幅值强行跨阈值、扩大step gate或删除history来
制造报警/恢复。此次研究没有跑新IMU流或宣称其已恢复。

复现研究（已有一次capture；不要不加区别地重复运行capture）：

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target realtime_performance_benchmark -j2
# New output path; original config, prior/beta/AL/step remain unchanged.
LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 UWB_IMU_PL_SCENARIO=uwb_recovery UWB_IMU_PL_FAULT_SCHEDULE_EPOCHS=18 UWB_IMU_PL_RESEARCH_HYPOTHESIS_AUDIT=1 UWB_IMU_PL_BATCH_FROZEN_PROOFS=0 UWB_IMU_PL_ROOT_RESPONSE_REUSE=0 UWB_IMU_PL_MODE_RESPONSE_CACHE=0 UWB_IMU_PL_FROZEN_VALIDATION_REUSE=0 "$BIN/realtime_performance_benchmark" config/fde_uwb_order1.yaml results/EVENT_RESEARCH_NEW 7
# Reuse the existing captured CSV: seven deterministic checks and two times.
python3 tools/research_physical_event_contract.py --self-test --snapshot results/fde_operability_20261010/event_contract_snapshot --report docs/benchmark/fde_operability_20261010.json
```

八项确定性检查及两个固定时刻的精确对照通过，日志
`event_contract_selection_after.log`。首次数学测试本身六项通过，之后旧Python3.8
没有math.nextafter导致报告生成异常；原 `event_contract_math.log` 保留，
改用numpy.nextafter后只重跑报告，再因新增跨epoch反例执行七项小测试，
最后因新增自适应输出group反例执行八项；均为毫秒级确定性数学检查。
未修改公共数值内核，未重复安全大矩阵、35-attempt或多seed campaign。
原102项/140次历史结果的适用边界不因此改变，当前部署和性能结论不升级。

文献只用于核对事件概念：[Stanford GPS Lab ARAIM chapter，§4.2–4.3](https://web.stanford.edu/group/scpnt/gpslab/pubs/books/Chapter12ARAIM.pdf)
区分primitive events与互斥完备的组合fault modes，并在consolidation时保留
所合并模式的概率质量。该GNSS文献的独立性前提不适用于本工程，不予继承；
上面的时序分区、相关性反例、PL/选择边界与具体金额来自本地合同与独立推导。

### 2026-10-10：先验溯源与 IMU 速度吸收的独立开发

用户接受 `4f21648` 的研究及停止结论，不批准更换事件合同。本次没有重跑
persistent-constant 合并研究、原短流、35-attempt 或核心安全大矩阵。
旧 prior、预算、p_md、AL、检测门限、history、step gate 和发布逻辑均未改变。
结论仍为 **PARTIAL_BLOCKED**，没有新的剔除/bridge/后续受保护 commit。

**先验来源与当前实际语义。** `git log -S prior_probability_bound` 将当前
配置/类型默认值追溯至 `1c547f31981854c03d68f557c7fd5179fd35c417`
（2026-09-04，research Gates B–I）。UWB `1e-4`、加速度/陀螺各 `1e-5`
已作为常量出现；提交和当前配置没有硬件型号、观测暴露量、故障次数、置信度、
故障持续时间分布或供应商概率承诺。仓库中不能证明这些数字来自测量，当前应
作为**人为配置的研究参数，概率资格 UNQUALIFIED**。不是经硬件证明的保守界。
手册 §34.3 明列 prior 来源为待确认；§35.6 禁止无 calibration 的正式标签。
loader 的 probability 检查只证明数值合法，非空 calibration_id 也不提供概率证据。

路径：`integrity_config.hpp:79/90/91` → research/FDE YAML →
`IntegrityConfigLoader` → `integrity_monitor.cpp` generator_config →
`hypothesis_generator.cpp:2663/2731/2778/2816` → risk allocation/audit。
manifest 声明 per_protected_epoch，但没有把“每个保护时刻”转换为物理故障
起始率或任务风险的公式。生成器对每个 anchor/onset/shape 赋同一个 UWB prior；
IMU 对每个 occurrence 的 xyz 单轴分别赋 accel/gyro prior。它不是整个设备、
整个 anchor 集合、每秒、每个新样本或整条序列的概率。加速度三轴的上界之和
可以是 `3e-5`；不能未经事件证明把单轴 `1e-5` 当作设备全集的上界。
原始样本批次与 manifest one_imu_interval 的差异也使真实事件映射尚未合格。

联合事件使用 `min(member bounds)`，不是乘积。对 E、F：
`P(E∩F) ≤ min(P(E),P(F))` 不需独立性，嵌套事件可取等号；
两个数 `1e-4`、`1e-5` 不能推出联合 `1e-9`。该 min 是有效**形式上界**，
前提是边缘概率本身合格；当前边缘概率 UNQUALIFIED，因此联合也 UNQUALIFIED。
loader 明确拒绝 assume_independent_priors=true。未覆盖 two-UWB、IMU×IMU、
共同原因多轴事件不能因参数可分离而获零风险或独立性。

已有文献核对：
[Walter et al., Determination of Fault Probabilities for ARAIM, 2019, §III–IV](https://www.aoe.vt.edu/content/dam/aoe_vt_edu/people/faculty/joerger/publications/Determination_of_Fault_Probabilities_for_ARAIM_2019.pdf)
定义 GNSS SIS 故障状态、起始率和共同原因，并以供应商承诺/观测历史支持参数。
其时间与持续时长的区分可作为方法参考，不能将卫星概率借给 UWB 或 IMU。
[Bruvik et al., Protection Levels for Vision-Based Pose Estimation, §IV-C1](https://arxiv.org/html/2608.10023v1#S4.SS3.SSS1)
明确将 keypoint 故障概率实测标定留待后续，假设 keypoint 独立；它也不提供
本工程传感器 prior 的实证来源。这两篇文献各只用于此处的语义/前提核对。

**供审查的 prior 证据合同提案，未实施、无替代数值。** 保留当前逐叶 union
账本，不重新研究合并收益。新增只读 PriorEvidence 设计：绑定物理来源/型号、
轴/共同原因、可独立判定的故障谓词、适用环境、时间单位、窗口及 history
影响范围、initial occupancy、onset count 上界、有效期限、数据来源/置信度和
model/scope/manifest digest。它输出叶事件的合格上界或 UNKNOWN；没有证据时
不猜测数值、不默认 validated=true。数学上令窗口起点已有影响事件概率为
q0，窗口 W 内新 onset 的期望次数 ≤ λW，则 union + Markov 给出

`P(任何相关故障影响窗口) ≤ min(1, q0 + λW)`。

这不要求相邻 epoch 独立。若另外证明平稳 episode 过程、影响持续时长 D 和
history 保留影响 L，则 Campbell/期望占用界给 q0 ≤ λ E[D+L]；未证明这些
条件时 q0 必须单独提供证据，不能取零。因此不能把 per-hour 起始率简单乘
50 ms 当成故障状态概率。每个已证明属于此来源全集的叶事件可继承该上界，
原逐叶计费仍保守，**不因此合并叶或保证预算通过**。组合继续用 min；只有
明确覆盖共同原因、适用环境和事件条件的独立性证明才允许另审查乘积。
漏检、nominal tail、selection-extra 及 omitted/bridge/history/model 仍全部计费。

统计证据若为事先固定定义的独立 Bernoulli trial，零故障的单侧置信上界为
`1-δ^(1/n)`；若另外证明齐次 Poisson onset，暴露时间 T、k 次 onset 的率界为
`χ²_(2k+2,1-δ)/(2T)`，k=0 为 `-lnδ/T`。不能把连续相关流的 35 行当作
35 独立 trial，更不能将确定性注入的成功/失败比例当成自然硬件故障 prior。
置信失覆盖 δ 必须作为 qualification 的显式假设，或经批准计入系统风险，
不能藏在 p_nm 中。审批对象是此 evidence/时间映射接口及适用模型，绝非调小
`1e-4` 的请求；没有采集/硬件证据前仍 UNQUALIFIED。

**IMU 新假设及最小验证。** 假设“偏置吸收导致无报警”与“原始注入未到达
预积分残差”分别验证。新增 `candidate_replay ... imu-absorption`，对已有
`consistent_imu_recovery/replay/attempt-7.bin` 只运行一次。用独立列主元 QR，
在同一个 H/z 上作四种离线列约束，分解 Pose3/速度/偏置增量，并核对原始
statistic 与 normal-equation residual。受约束模型不是生产模型，各自 dof
不同，不能把它们与原阈值比较来声称生产报警或合法恢复。

| 离线约束 | observed 残差平方 | 全状态增量范数 | 速度范数 | accel bias 范数 |
|---|---:|---:|---:|---:|
| 原自由状态 | 0.18701637172207713 | 0.90280503676313051 | 0.90239885717078627 | 0.0019846221592347188 |
| 偏置列固定 | 0.18706763052887970 | 0.90291237655202083 | 0.90250774735406880 | 0 |
| 速度列固定 | 1542.9977892477254 | 0.78656877111335122 | 0 | 0.0962773154215886 |
| 速度与偏置固定 | 1545.1411474199867 | 0.7868721164844239 | 0 | 0 |

完整 interval accel-x 的单位故障 parity 能量为 `0.00030473259174437328`，
固定 bias 为 `0.00030489194855592289`，固定 velocity 为 `4.2789798528082121`。
所以主要是速度状态解释了输入，不是 bias 吸收。该快照满列秩 120，dof 56，
这是有限但很弱的故障可观测性，不能误称所有方向严格不可观测或算法漏算。
28 行 QR 正交性相对误差最大 `1.8915282072069805e-15`；与已冻结 statistic 匹配。

新增确定性有限幅值测试，对单个静止 epoch 的十个新 raw samples 注入
`20 m/s²`、保留健康左边界，再独立 reintegrate 并求实际 frozen-point
CombinedImuFactor error。raw dp=`0.022625 m`，dv=`0.95 m/s`，bias residual
变化为零，whitened norm=`30.16445995740861`，与同样 sample-support 的微分
预测相对差 `1.1892274488700195e-16`。因此注入确实影响真实因子残差。
此处 covariance 相对变化 `0.0058016760542469667` 是该静止小 fixture 的值，
不是运动 epoch7 的值，不能据此改 production whitening/noise 合同。
此前 sample-support 阴性结果直接复用：健康左端导致首梯形只有半幅，下一
epoch 继承坏边界；它不等价于整个 interval 常量故障，不再次运行旧测试。

因果链：真实流 all_in statistic≈0.187 < 原阈值121.348810，
`integrity_monitor.cpp:3411` 只生成 KEEP；没有硬件 barrier。
随后 KEEP 的原线性步长仍超 0.25，PL 未运行；不是 step gate 阻止了一个已
通过独立 detector/风险/PL 的剔除恢复。表中 QR base 是原 H/z 的增量，不能
冒充 pipeline 的 canonical KEEP 数值（已有流 KEEP step≈0.896）。
本次未发现可据此自主修补的 detector 实现错误：当前测试场景的短时间位移
很小、速度自由，导致弱 parity；当前故障模型与 raw sample-support 不一致则
是需要明确审查的建模问题。未降低门限、冻结生产速度、加 prior 或扩 step。

**下一数学决策，独立于概率事件合并。** 建议先批准把“十个新样本的单轴
故障 episode”精确卷积到两个相邻 preintegration 及 history 的共享参数图：
每个梯形的权重为 `(s_left+s_right)/2`，旧健康边界 s=0、新批次 s=1，
次 epoch 第一梯形保留 1/2 carry。实际姿态/旋转必须逐段求导，不能将整段
bias Jacobian 乘 0.95 替代。该支持模型变化需要 scope/manifest、故障映射、
桥接剔除集合、history 和完整条件 PL 的共同检查；并不会自动提供检测功效。
如要获得独立 IMU 检测功效，另需批准并证明独立运动/速度参考或物理 envelope
的适用条件和误差上界。上述冻结 velocity 仅用于定位问题，**不提议作为真实
传感器约束**。在没有独立参考时，不承诺这个 50 ms 单故障场景可按原合同恢复。

构建 candidate_replay 与 test_history_fault_parameterization 成功。新有限
注入 gtest 1 项通过，prior 形式检查 4 项通过；未改公共数学层，未扩大回归。
新增证据统一存入原 JSON 的 `prior_imu_followup`，报告生成器保留该段。

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target candidate_replay test_history_fault_parameterization -j2
LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/candidate_replay results/fde_operability_20261010/consistent_imu_recovery/replay/attempt-7.bin results/fde_operability_20261010/imu_absorption.csv 1 1 imu-absorption
LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_history_fault_parameterization --gtest_filter=FdeOperabilityImuSupport.FiniteInjectedAccelerationChangesActualFactorResidual
python3 tools/research_prior_and_imu.py --self-test --results results/fde_operability_20261010 --report docs/benchmark/fde_operability_20261010.json
```

### 2026-10-10：授权后的隔离 raw-episode / 条件恢复研究（A–E）

**结论仍为 PARTIAL_BLOCKED；研究正例为 CONDITIONAL。** 从 d9e575aa 的
工作区继续，完整读取当天进度；没有重做已接受的 prior 审计、persistent
合并反例或旧 campaign。新增 research/ 独立静态库与 EXCLUDE_FROM_ALL
入口，不链接回生产 DSO/node、不安装、不 mint/publish。原配置、概率预算、
检测/step/rank 门限、Gaussian nominal、Huber 和发布资格均未替换。

**A VERIFIED（声明的局部数学模型）。** 从真实 raw 样本重积分，使用真实
CombinedImuFactor 做 central/half-step FD 与独立原始样本腐化 oracle。
静止/旋转、六轴、非零 biasHat/端点 bias 漂移、相邻共享样本、协方差导数
和无支持负例通过。一个 episode 同时映射两个相邻因子后，真实 H/z 的历史
正交消元与独立 QR 最小化成本相等。原 batch 响应 dp/f=.00113125、
dv/f=.0475；下一因子 .00011875、.0025，不使用整 interval bias J 比例。
实际 FD 还检出共享样本 cross covariance 范数约 4.335038e-9；生产独立
PIM/history 噪声合同未悄悄改动。剔除后从 raw history 枚举 latent carry。

**B CONDITIONAL / 不可检测情形保留。** 仅四个定向窗口，没有 seeds 或
大矩阵。静止 epoch12、accel-x=20、持续 .05/.25/.5 s 的 lambda 为
2.95938/77.1511/174.6448，冻结独立 Gaussian p_md 为
.99999656/.579234/.000236743。500 ms 的 unit step .348918，乘20后
仍超过 .25，不能把功效通过叫恢复通过。静止 gyro-z 的 lambda 约1e-26，
缺少 yaw 独立信息。共享 raw 噪声时普通 nc-chi² 前提失效；新隔离
quadraticPower 推导/验证 generalized quadratic 的均值、方差与 Cantelli
漏检界，两个真实因子窗口得到 miss 下界约 .98244。没有降门限、冻结速度
或扩大 step；硬件 p_md 保持 UNQUALIFIED。

**C CONDITIONAL 实际闭环。** 新合同 simulation-only/raw-range-episode/v1：
350 ms 内完整 categorical law，nominal 或一个 epoch/anchor UWB +2.25m，
或一个 accel-x raw batch +20m/s²；原1e-4/1e-5作为声明生成参数，nominal
质量 .99433，不声称硬件先验。物理条件为 stationary、零 lever arm、健康
range 误差≤1e-12m、每 epoch 至多一坏 range。IMU 正例另有明确的独立
仿真 odometer（优化 sigma .02m/s，速度仍为变量）。GT只生成/评价输入。
所有模式 raw map 行数及独立观测行 lineage 均检查，未喂 GT 故障标签。

新增独立 range 可行集 union/共同参考三角界，拒绝不一致子集，不将 LS
残差无限放大成半径；源码/时间/模型/完整 PL 字段重算匹配，KEEP 和成功
IMU 路径也在实际 FdeManager 前验证，逻辑 proof ID 仅作索引。共同事件
只用于隔离仿真选择器，保留原 strict singleton 风险 FAIL，不改变生产函数。
健康/UWB/IMU 各7次 selected、风险闭合、实际一次 backend update/commit；
UWB及IMU在epoch3自动剔除，随后4次selected commit。IMU为覆盖时间歧义
保守移除2001和3001，独立generic bridge加完整当前位置条件PL，非
BRIDGE_ONLY。非线性commit后再绑定实际mean。全精度KEEP/报警账本保存：
nominal3e-5、p_nm1e-7、hypotheses9.9e-6，已知miss项保留，selection-extra
由选择器算，合计原4e-5；escape零来自独立range保证的声明条件。
该精度与数学合同是研究场景，不能称原joint-order2或硬件已可用。
formal_eligible/publication_protected始终false；故障处理超过40/50ms，
过时有限值不算及时保护，生产未恢复latency仍null/RIGHT_CENSORED。

**D UNQUALIFIED / fail-closed。** PriorEvidence只读原型明确事件谓词、
onset、duration、每声明horizon的时间单位、互斥categorical联合概率与有效
范围；过期、错模型、重复/非法质量、越域onset、deployment查询均拒绝。
不凭calibration字符串证明概率，不调小原先验，不推断独立乘积。旧依据
审计结果直接复用。生产原合同修改与真实odometer/noise证据仍需另行批准。

**E VERIFIED 限定成本收益；不是保护延迟。** 先冻结新正确 raw 支持模型，
normal/joint2（98模式，98/2450假设）逐数值字段对照一致，之后仅四进程
各35次，共140连续attempt。新增默认关闭的无raw支持精确零分支仍执行一次
实际重积分/噪声/错误校验；敏感/非空支持回到四次FD。216/294配对无支持，
integrations1176→528。包含对象/索引、seal、arena、最终验证与销毁；复用
原Evidence与exhaustive Flat，不启用mode cache/frozen memo。总中位数
normal49.20→35.58ms（27.68%），joint2 173.68→161.46ms（7.04%）。
收益在研究Evidence输入构造，Evidence/Flat自身无可宣称收益。两侧Gaussian
Flat全部保留hypothesis62不可监测拒绝；不以失败计时宣称可用或完成部署。
normal的40/50ms逐侧耗时判定有35/10次差异，strict时间比较FAIL单独保留，
固定快照数值比较PASS，旧production strict比较未改/未重跑。所有attempt、
失败和最慢值保留，35样本不宣称P99。此研究快照不是旧joint-order2验收。

验证：184个实际support/history/noise检查、23个range/prior/consumer正负例、
11个PriorEvidence检查通过；最后一次核心边界回归6项通过，复用前102项
结果，未改公共数值层而扩大回归。全部运行命令、数值生产/校验/消费表、
假设及生产建议见 research/README.md；单一原JSON追加research_continuation，
报告生成器保留该字段。research/report_results.py实际校验21次selected
commit、两个排除及其4次后续commit、140attempt与数值对照。新增结果文本
约0.5MB，未归档新snapshot/binary备份；旧失败与用户未提交requirements保留。

停止边界有具体证据：原UWB epoch7 4.69e-5超预算不变；原50ms IMU缺独立
信息/被velocity吸收不变；新正例依赖明确的额外sensor/有界range/事件合同，
不能未经批准进入生产受保护路径。只读证据接口、原始支持/lineage与拒绝
诊断可建议常规review；raw episode/joint covariance、odometer、range PL和
共同参考选择器的生产采用仍需合同批准与传感器证据，不自动切换。
