# Repository truth audit

## 1. Version Anchor

| Item | Observed value |
| --- | --- |
| Branch | `feature/uwb-imu-fusion-ie-postprocessing` |
| HEAD | `439282c2790ab1414ca9b276dd61a18b5e1643a8` |
| Audit date | 2026-09-12, Asia/Shanghai |
| Repository | `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie` |
| Initial tracked working-tree changes | None |
| Initial untracked items | `doc/paper_writing/`, `doc/v2/ie_0911/`, `doc/v2/paper_structure_v4.tex:Zone.Identifier` |
| Actual v4 blueprint | `doc/paper_writing/paper_structure_v4.tex` (untracked user material) |
| Blueprint SHA-256 | `a8e61b9c15cd3b8aee23f72f80a51229e83486e87e981b2e60c77bc068b65cb3` |
| Production paper executable | `uwb_imu_fgo_paper_runner`, built from `tools/run_ie_paper.cpp` (`CMakeLists.txt:187`) |
| Production support mode/provider | `nlos.mode=pl_bidirectional_cusum` / `PL_BIDIRECTIONAL_CUSUM_V1` |
| Primary final policy | Explicit `--method lcb_fixed_full`, designated primary by the current controlled E2E protocol |

“Production” here means the integrated paper execution path at this HEAD, not a universal application default or a scientifically validated release. `tools/run_offline.cpp` is a separate legacy ROS entry point. `Config::nlos_mode` still defaults to `disabled` (`include/uifgo/config.h:165`). The inline paper runner defaults to `FULL_GATE` when no method is supplied (`tools/run_ie_paper.cpp:4681`); it does **not** implicitly select full-fixed inference.

The concrete controlled configuration is `config/paper/ie0911/sfuise_walk1_pl_bidirectional_injected.yaml`, with its clean counterpart. `walk1_pl_bidirectional_controlled.yaml` schedules a separate Cauchy baseline, one shared Stage-2 producer, and four final consumers. `six_inputs_pl_bidirectional_cusum.yaml` applies the locked PL override to the six base configurations; those base YAMLs alone are not equivalent to the effective PL configurations. The producer cell is named `structured_debias` but has `execution_type=CACHE_PRODUCER`: this does not select the paper's primary final policy. See `tools/paper/run_experiments.py::apply_unit_nlos_override` and the executable's `CACHE_PRODUCER` branches.

Locked detector parameters are forward `kappa=0.5`, `h=7.0234689587858723`; backward `kappa=0.5`, `h=7.0234689587858714`; link gap `1.0 s`; provenance `PL_BIDIRECTIONAL_CUSUM_SUPPORT_20260912_LOCKED`. `src/config.cpp:466` requires these exact values, explicit scoring/final enablement, and prohibits oracle support. They are not recalibrated on each input. Current controlled configuration uses keyframe step 4, nominal base range sigma 0.15 m, `v_max=2 m/s`, fixed anchors/lever arm, zero time offset, no online calibration, and an **empty** `fixed_beta_by_link` table.

Evidence scope: this is a source/configuration/test-source/report audit. Build, GTest, CTest, estimator replay, new accuracy experiments, and independent verification of archived run artifacts are **NOT_RUN in this audit**. Earlier reported passes remain attributed to their report. Protected `doc/v2/ie_0911/` contents were not read or changed. No production source/configuration, blueprint, frozen roadmap, or historical result was modified.

Read-only command evidence: `git branch --show-current`, `git rev-parse HEAD`, `git status --short`, `git ls-files doc/paper_writing`, `sha256sum doc/paper_writing/paper_structure_v4.tex`, and targeted `rg`/`sed`/`cat` reads produced the anchors and source findings below (exit 0 for these anchor checks). An initial attempted read of `doc/v2/paper_structure_v4.tex` returned exit 1 because that file does not exist; discovery located the actual blueprint above. Commands and tool output are retained in the audit session; no estimator run directory or experiment log was created.

Source priority: current HEAD and its reachable paper execution path; supplied v4 blueprint; current tests/reports/docs; older designs. `doc/ie_sprint/STATUS.md` and its current PL E2E protocol were read first. Their linked older `doc/v2/paper_structure.tex` and `doc/v2/v2_roadmap.md` provide historical context, not authority to replace current code semantics.

## 2. End-to-End Implemented Pipeline

### Inputs and common preparation

1. `tools/run_ie_paper.cpp::main` loads configuration and IMU/UWB inputs through `LoadData`, then constructs `PaperInputPlan`. Inputs are timestamped accelerometer/gyroscope samples, raw tag-to-anchor ranges and their source identities, configured anchor positions, lever arm, noise parameters and optional fixed per-link static offsets. GT and oracle support are not inputs to the PL provider. Prefix/cache interval selection belongs to loading, before initialization; this audit did not execute a new prefix experiment.
2. `src/paper_input.cpp::BuildPaperInputPlan` creates a ledger before strategy selection. It preserves raw range, source time/order, tag/anchor, RSSI, validity, `suspected_nlos`, stable `obs_id`, keyframe assignment and frozen nominal sigma. Nonfinite/protocol-invalid/out-of-range/unknown-anchor observations are invalid; keyframe decimation determines `planned`. Suspected NLOS is a label, not an exclusion. Thus “all measurements” below means **all valid planned measurements**, not every raw sensor row. Multiple tags in one paper run are rejected.
3. `StableObservationId` hashes recording/source ordinals, checks collisions and verifies cached identity. Changing a measurement's range does not change identity when source ordinals are preserved. `AllPlannedObservationMask` and `MaterializePaperKeyframes` retain suspected NLOS. Nominal sigma includes the frozen inter-observation gap term described in Section 4; it is not simply 0.15 m for every observation.
4. `src/initializer.cpp::Initializer::Run` estimates gravity alignment and gyro bias from an initial IMU static window when available, otherwise uses configured IMU orientation or identity; it trilaterates using up to five early UWB frames, with origin fallback. Optional yaw alignment is configurable (off in the controlled configuration). `GraphBuilder::Build` constructs pose/velocity/IMU-bias states, initial priors, `CombinedImuFactor`s and raw UWB factors. `ReplacePosePriorsForPaperPath` replaces the pose prior with the paper factor implementation.
5. The runner calls `PrepareRawGaussianReference` before the PL provider (`tools/run_ie_paper.cpp:3261`). This is an all-planned-range, tightly coupled Gaussian batch LM reference. Its converged Values seed Stage 2. It is **not** the state source used to generate PL conditional innovations. The ordinary common initializer has access to the loaded interval; the signal producer receives a separately initialized graph as described next.

### Pre-commit conditional signal producer

6. The PL branch (`tools/run_ie_paper.cpp:4075`) reruns initialization on keyframes 0–4 and IMU cropped through keyframe 4. It builds a separate physical graph with this bootstrap initialization and passes it to `PlBidirectionalCusumSupportProvider::Run`. This graph is an input to a sequential GTSAM fixed-lag producer; the all-range batch reference Values are not passed into it.
7. `src/pl_bidirectional_provider.cpp` uses `ProductionFixedLagBackend`, lag **200 epochs**, `relinearizeThreshold=0.1`, `relinearizeSkip=200`. Epoch timestamps, not sensor seconds, are supplied to the smoother. Non-UWB factors are assigned to epochs; new states are seeded by IMU prediction from the previous committed estimate (`SeedFromPrediction`). The prediction/non-UWB update is performed before the current UWB commit.
8. Keyframes 0–4 are bootstrap history: their raw UWB is committed, but no conditional signal rows are emitted. Detection starts at keyframe 5. At each later keyframe, planned ranges are grouped in stable source-message/range/observation/ID order; duplicate anchors in a group fail. `CurrentMarginal` solves through the current ISAM2 clique closure for the 15-dimensional pose/velocity/IMU-bias covariance. The producer evaluates each actual raw factor's unwhitened residual and a numerical pose Jacobian, checks factor sigma against the frozen ledger, forms the innovation covariance, and calls `EvaluatePlConditionalRows`.
9. The signal has access to bootstrap measurements, prior committed raw UWB, IMU through the current epoch, current-group raw UWB, physical configuration and the current linearized covariance. Conditionalization uses the **other ranges in the current group**. Pre-commit means the current group's raw factors have not yet been assimilated into the state; it does not mean those ranges are unavailable to the signal calculation. The producer receives the full loaded graph/plan/IMU arrays and their identity hashes, but the sequential state updates do not use future UWB factors or the batch-reference solution. After diagnosis, it **always commits all current-group raw UWB**, including suspect measurements. There is no detector-driven rejection feedback.
10. IMU gaps exceeding `0.02+1e-12 s` are recorded; when a gap event falls in the current epoch interval, the producer restarts the fixed-lag backend with the previous pose/velocity/bias and `ReinitializationPriors`. These priors have pose sigmas `[0.1,0.1,0.1,0.5,0.5,0.5]`, velocity sigma 0.5, bias sigmas `[0.1,0.1,0.1,0.01,0.01,0.01]`. This is distinct from the CUSUM link gap rule. No special CUSUM “IMU restart” marker is sent; bootstrap exclusion is keyed to absolute keyframe index, not restarted after each backend reset.

### Persistent support and freeze

11. After signal collection, the provider runs `EvaluatePlPersistentCusum(signal_rows, ...)` and `EvaluatePlBackwardCusum(signal_rows, ...)` on the same stored sequence. State is separate for every `(tag_id, anchor_id)`. Forward sorting is ascending `(timestamp,keyframe_id,source_order,obs_id)`; backward sorting reverses that tuple. A malformed reverse-time forward input fails closed before sorting. The production forward pass is executed offline after collection even though its recurrence depends only on preceding rows.
12. Each pass accumulates positive excess evidence, subtracting 0.5 per row, clamping at zero. The first **inclusive** threshold crossing in a positive excursion latches one alarm. Earlier positive-excursion rows are backfilled; subsequent rows remain candidates while the statistic stays positive, even below the alarm threshold. Returning to zero closes the excursion and excludes the zero-return row. Invalid/nonfinite signal resets without a candidate; a strictly greater than 1 s link gap resets before processing the next valid row. Negative increments can reduce an excursion without immediately ending its support.
13. Backward CUSUM uses only stored signal rows and their identity/time/validity fields. It does not propagate IMU states, rerun the estimator backward, recompute innovations, or perform a backward Kalman smoother. Reverse excursion backfill supplies a noncausal closure of the forward tail.
14. `src/pl_bidirectional_support.cpp::IntersectPlCusumSupport` verifies identical link/ID universes and row timestamps/keyframes/source orders, then takes exact **forward AND backward** membership. It splits runs on a nonmember row of the same link or a gap greater than 1 s. No padding, union rescue or forward-only fallback is applied. Other links' rows do not interrupt a run. Segment endpoints are first/last member row times; membership is an explicit ID list, not a later time-window query.
15. This PL construction applies **no separate minimum-count or minimum-duration filter**. The old `BuildPlConditionalSupport` temporal filter is not called. `SupportSegment::short_support_debug` defaults to false and PL does not set it. “Persistent” therefore describes alarmed excursions and their intersection, not a guaranteed duration or minimum observation count.
16. The provider returns `SUCCESS_SUPPORT_FROZEN` with a source-neutral `SupportPartition` carrying segment IDs/ordinals, link IDs, endpoints, count/duration, explicit `obs_ids`, and provider/input/config/calibration/solver/support identities. Support is frozen **before** Stage-2 amplitude recovery; downstream stages do not rerun CUSUM or adjust boundaries. Despite historical `preflight` strings in reusable core output types, the runner uses the production wrapper, which sets provider `PL_BIDIRECTIONAL_CUSUM_V1` and rehashes the partition.

### Stage 2, scoring, admission and final inference

17. With nonempty support, `SegmentRefitter::Run` starts from the converged batch raw-reference navigation Values. It preserves physical non-UWB factors and replaces each candidate raw UWB factor by one raw-with-live-segment-C factor. All noncandidate planned UWB remain raw. Each frozen segment has one nonnegative scalar amplitude shared by all its IDs. The refit alternates fixed-C navigation LM and exact projected weighted-mean C updates. Its objective has no L1/TV or C prior. Success requires joint objective/step/C-KKT/navigation-stationarity checks, not merely an LM return or exhausted iteration budget.
18. With empty PL support, `ReuseEmptyPlReference` returns verified `SUCCESS_EMPTY`, reusing the converged raw graph/Values without another Stage-2 optimization. The final engine checks graph/Values identity and reuses that result for all four candidate-aware policies. Empty support is a legitimate no-candidate outcome, not evidence of successful NLOS recovery.
19. `ScoreRefitRecoverability` constructs transitive closed-time-interval overlap groups of segments, including overlapping segments on different links. For each group it linearizes at the **same converged Stage-2 Values**: all original non-UWB factors and all noncandidate ranges, plus this group's candidate factors exactly once. Candidate factors from every other group are omitted. This equals a common candidate-excluded reference augmented by one scored group; it is not a separately optimized held-out reference. Nuisance/bias columns and factor-row/ID mapping are exported. `ComputeSparseRecoverability` projects group amplitude columns away from nuisance columns.
20. `FreezeFixedCompensations` obtains the full group's inverse-information diagonal via `LocalAmplitudeSigmas`, validates segment/key/column mapping, eligibility and Stage-2 amplitudes, then admits a segment exactly when the clipped local LCB is positive. `lcb_fixed_full` freezes its complete Stage-2 amplitude; `lcb_partial` freezes the LCB amplitude. They have the same pre-final admission set given the same Stage-2 graph/Values and scores. Neither applies `tau_eta`, `tau_s_m` or `tau_gamma` as admission thresholds, although the final API still validates the gate-configuration envelope and numerical score validity is required.
21. `FinalInferenceEngine::Run` calls `RunFixedOffsets` for fixed policies. It keeps all noncandidate raw factors, replaces admitted candidates with one fixed-offset factor each, and omits every nonadmitted candidate. Fixed-C variables are removed from graph and Values. The raw range and sigma are preserved; compensation is inserted as a model constant. Fixed modes skip live-C final rescoring. The actual final solve is the existing refitter's no-live-amplitude path and retains its navigation convergence checks.
22. `AuditFinalFactors` checks observation multiplicity and actual graph keys. Fixed factors also undergo independent residual/linearization comparison against reconstruction from the ledger and frozen compensation. Original and corrected copies of one observation are not both permitted in a valid final result. `ComputeFinalGraphCovariance` uses GTSAM `Marginals` on the same final graph/Values used by trajectory, bias and residual export. Fixed compensation records are Stage-2-derived constants, not estimated final bias states.
23. A final recovery solve, factor audit, or applicable live-C final-score audit failure triggers **at most one** all-candidates-suppressed solve from Stage-2 Values. Successful fallback is `FALLBACK_OK` and fixed policy `actual_fixed_method=suppress_all`. Failed fallback returns `ESTIMATION_FAILED`, with no valid final graph/trajectory. A numerical scoring failure that produces an unavailable score can instead cause ordinary admission suppression. Invalid provider/input/cache or failed Stage 2 may terminate **before** final inference; they do not guarantee a rejection trajectory. Covariance failure is separately `UNAVAILABLE`, not automatically a recovery fallback.
24. Batch execution stores an immutable provider-aware Stage-2 cache. Final consumers verify/rebuild graph/Values identities; full/partial recompute scoring at those Values to obtain the full matrix and column map, with no Stage-2 optimizer calls (`tools/run_ie_paper.cpp:3089`). Legacy/FDE/PL cache namespaces cannot be silently interchanged. A producer failure leaves dependent final cells unavailable, including the shared-support `suppress_all` comparison.

## 3. Paper-to-Code Mapping

Status labels describe reachability in the audited primary configuration. A retained alternative can be compiled and tested without being the production method.

| v4 block / related alternative | Repository file and important symbol | Actual semantics | Status |
| --- | --- | --- | --- |
| III: input/state/physical range model | `src/paper_input.cpp::BuildPaperInputPlan`; `src/graph_builder.cpp::GraphBuilder::Build`; `src/uwb_factor.cpp` | Single tag, fixed anchors/lever/time; X/V/IMU-bias states; stable valid/planned ledger; Gaussian range factors | CURRENT |
| III: independently calibrated static beta | `src/paper_input.cpp::ValidatePaperFixedBeta`; `FixedBetaForLink` | Fixed beta supported; controlled table empty, zero default, development missing-calibration status; provenance not independently verified by table completeness | EXPERIMENTAL (calibrated scientific premise not established) |
| IV-A: pre-commit conditional innovation | `src/pl_bidirectional_provider.cpp::PlBidirectionalCusumSupportProvider::Run`, `CurrentMarginal`; `src/pl_conditional_raim.cpp::EvaluatePlConditionalRows` | Separate bootstrap/fixed-lag producer; current-group Gaussian conditioning; always commit | CURRENT |
| IV-B: forward support | `src/pl_persistent_cusum.cpp::EvaluatePlPersistentCusum` | Per-link positive excursion, inclusive crossing, last-zero backfill/latch/reset | CURRENT |
| IV-B: backward support | `src/pl_bidirectional_support.cpp::EvaluatePlBackwardCusum` | Descending stored conditional-z sequence; no state smoother | CURRENT |
| IV-B/C: intersection/freeze | `IntersectPlCusumSupport`; `include/uifgo/nlos_support.h::SupportPartition`; production provider | Exact ID intersection; run segmentation, no min-count/duration postfilter; hashed support before refit | CURRENT |
| V-A: nonnegative segment refit | `src/nlos_refit.cpp::SegmentRefitter::RunFrozenCandidatePolicyImpl`, `ComputeNonnegativeSegmentAmplitude` | Physical Gaussian objective; alternating navigation and constrained scalar blocks; joint stopping conditions | CURRENT |
| V-B: local information | `src/nlos_scoring.cpp::ScoreRefitRecoverability`, `BuildClosedIntervalOverlapGroups`; `src/nlos_recoverability.cpp::ComputeSparseRecoverability`, `LocalAmplitudeSigmas` | Group reference-plus-candidate rows at Stage-2 Values, nuisance projection, checked inverse diagonal | CURRENT |
| V-C: LCB/full correction | `src/nlos_inference.cpp::FreezeFixedCompensations`; runner `LCB_FIXED_FULL` dispatch | Positive LCB admits; full Stage-2 amplitude is fixed | CURRENT |
| V-C: partial correction | Same function, `LCB_PARTIAL` dispatch | Same admission set; smaller LCB amplitude | OPTIONAL BASELINE |
| V-D: fixed final graph/fallback | `FinalInferenceEngine::Run`, `AuditFinalFactors`; `SegmentRefitter::RunFixedOffsets`; `MakeFixedOffsetUwbFactor` | Exactly-once retained factors, no live C, one final-stage suppress fallback | CURRENT |
| V-D: live-C comparison | `FreezeGroupDecisions`, policy `STRUCTURED_DEBIAS`; `ScoreFinalRefitRecoverability` | All candidate groups USE, including ineligible groups; no eta/s/gamma or LCB gate; live nonnegative C refit and final audit | OPTIONAL BASELINE |
| VI: same-support rejection | `SUPPRESS_ALL`; `RunFrozenCandidatePolicy` | Removes every frozen candidate; differs from residual-based `fixed_rejection` baseline | OPTIONAL BASELINE |
| VI: robust baselines | `src/paper_methods.cpp`; runner baseline dispatch | Separate raw-graph Cauchy/Huber and other registered baselines; not weighting inside primary Stage 2 | OPTIONAL BASELINE |
| Old eta/s/gamma full gate | `FreezeGroupDecisions`, `FULL_GATE`, `FIT_ONLY`, `S_FIT`, `ETA_ONLY` | Live-C alternatives, some requiring diagnostic provenance; not a serial gate in full-fixed inference | OPTIONAL BASELINE |
| Old L1/TV support narrative | `src/nlos_discovery.cpp`, runner `automatic_discovery` branch | Retained automatic/development provider; not called by PL mode | DEPRECATED (as primary-method description) |
| Earlier FDE/group/window detectors | `src/nlos_fde.cpp`; `imu_aided_fde` branch | Retained opt-in providers, not PL production support | EXPERIMENTAL |
| Group chi-square / leave-one-anchor-out support | `src/pl_conditional_raim.cpp::{EvaluatePlConditionalGroup,BuildPlConditionalSupport}` | Separate conditional-RAIM preflight machinery; production PL calls only row diagnostics | EXPERIMENTAL |
| Backward IMU estimator | No call on the PL production path | Backward CUSUM is not such an estimator | NOT IMPLEMENTED (in this pipeline) |
| VI: evaluation/artifact accounting | `tools/paper/{evaluate_runs.py,evaluate_range_recovery.py,run_experiments.py}`; `src/nlos_inference_io.cpp` | Separate evaluator and identity-bound outputs; existing controlled development evidence | CURRENT |
| Proposed E1–E9/C3 campaign and uncertainty calibration | v4 evaluation/writing plan, existing limited reports | The complete proposed independent-recording campaign and calibrated coverage guarantees are not established by HEAD | NOT IMPLEMENTED (complete promised evidence package) |
| Full support/bias-selection posterior propagation | Fixed-offset factor has no uncertainty input for delta | No selection-aware marginalization or probability protection level | NOT IMPLEMENTED |

## 4. Mathematical Semantics

### State and observation

Navigation variables are `X(k)=Pose3`, `V(k)` and `B(k)=ConstantBias` (accelerometer and gyro bias), collectively theta. During Stage 2 and live-C inference, add `C(j)=Symbol('c',j)`, one scalar in metres per segment. Static `beta_(tag,anchor)` is a configuration constant, not IMU `B` or dynamic `C`. The primary runner forbids online anchor/lever/range-bias/time calibration (`ValidateSupportedConfig`).

For observation i at assigned keyframe k and link ell, let

`d_i(theta) = ||R_k * lever + p_k - anchor_ell||`.

The model is `z_i = d_i(theta) + beta_ell + c_j + noise_i` inside segment j; outside all support, `c=0`. The factor residual sign is **prediction minus measurement**:

`r_i = d_i(theta) + beta_ell + c_j - z_i`.

The frozen range noise is `sigma_i = sqrt(sigma_range^2 + (v_max * dt_link / 3)^2)` when positive elapsed time is available, otherwise the base sigma. `dt_link` is determined from planned observations before policy selection (`BuildPaperInputPlan`). No robust residual-dependent weight or sigma inflation by local bias uncertainty is introduced by full-fixed inference.

The inspected controlled YAML does not opt into a different IMU covariance model, so `Config::paper_imu_covariance_model` retains `LEGACY_GTSAM_COMBINED_DEFAULT_V1`. `src/imu_preint.cpp::ImuPreintegrator` sets configured accelerometer/gyro and bias random-walk covariances and retains `integrationCovariance=1e-9 I`. The alternative `PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1` explicitly zeroes `biasAccOmegaInt` only when selected; its existence must not be mistaken for selection in this configuration. Graph/model identities are checked on the paper path.

### Conditional innovation

At a pre-commit epoch, let P be the 15x15 marginal after the prediction update, H the group range Jacobian (numerically differentiated pose columns; other columns zero), and Sigma the diagonal factor noise covariance. Then

`nu = z - d(theta_minus) - beta = -raw_factor_residual`,

`S = H P H^T + Sigma`.

For row m, with the other rows denoted q:

`nu_cond_m = nu_m - S_mq solve(S_qq, nu_q)`,

`var_cond_m = S_mm - S_mq solve(S_qq, S_qm)`,

`u_m = conditional_z = nu_cond_m / sqrt(var_cond_m)`.

For a singleton, the conditional quantities equal marginal ones. These are physical-unit innovations and a dimensionless statistic, not the post-fit Stage-2 residual. The implementation checks finite/positive covariance solves. A group-wide inability to create diagnostics can fail the provider; row diagnostics marked invalid are reset inputs to CUSUM. The group statistic computed by the diagnostic routine does **not** gate candidates.

### CUSUM and support

For each link in ascending order, `G_n = max(0, G_(n-1) + u_n - 0.5)`. Backward, in descending order, `B_n = max(0, B_previous_reverse + u_n - 0.5)`. They start at zero. First crossing is `G>=h_forward` or `B>=h_backward`, once per positive excursion. The final support is the intersection of **alarmed excursion memberships**, not `{n: G_n>=h}` intersected with `{n: B_n>=h}`. Excursion onset is the first positive row after the last zero/reset; the zero row itself is not backfilled. Directional support ends before a zero/invalid row or gap reset, or at sequence end. Final intersection boundaries can be interior to either directional excursion.

`S_j` is an explicit ordered set of observation IDs within one link. All S_j are disjoint by observation identity. Timestamps and durations describe those memberships; they are not optimized change-point parameters. No continuous-time boundary interpolation is implemented. Bootstrap rows have no signal and hence cannot become PL candidates, but remain raw constraints.

### Stage-2 objective and amplitude update

The optimized GTSAM objective is

`J(theta,c) = J_IMU_and_initial_priors(theta) + (1/2) sum_(i in O) [(d_i(theta)+beta_i+1_(i in S_j)c_j-z_i)/sigma_i]^2`, with `c_j>=0`.

`J_IMU_and_initial_priors` itself uses the GTSAM half-squared whitened residual convention. There is no discovery L1/TV penalty, segment-amplitude prior, robust range kernel or extra information from LM damping in this objective. Nonlinearity and alternating minimization mean this is a local numerical solution, not an established global argmin.

Given navigation, `ComputeNonnegativeSegmentAmplitude` uses

`c_j <- max(0, sum_(i in S_j) w_i*(z_i-d_i-beta_i) / sum_(i in S_j) w_i)`, `w_i=1/sigma_i^2`.

At an interior amplitude, KKT violation is absolute objective derivative; at a boundary (`c<=1e-9 m` in the controlled config), it is `max(0,-derivative)`. Relative objective change <=1e-8, scaled step <=1e-6, C-KKT <=1e-8, and scaled navigation stationarity <=1e-6 with the implemented roundoff allowances must all pass; Stage-2 outer cap is 50, conditional LM cap 100. The stationarity repair reuses fixed-checkpoint conditional navigation recovery when the other outer checks pass but navigation does not; it does not alter the model or grant convergence at the cap (`src/nlos_refit.cpp:971`, `:1334`; `src/nlos_solver_utils.cpp`).

### Local information and uncertainty

For overlap group g, assemble the whitened Jacobian `[F G]` of common noncandidate/IMU/prior factors plus g's candidate factors, at the converged Stage-2 Values. F contains all actual nuisance coordinates (X/V/B on the production configuration); G contains only this group's scalar C columns. The measurement RHS is exported separately and never becomes an information column. On candidate rows, G has `1/sigma_i` in the corresponding segment column.

`N = G^T G`; `E = G - F argmin_Y ||F Y-G||_F`; `R_c = E^T E`, represented as `GroupRecoverabilityScore::numerical.R`. This is numerically equivalent to `G^T (I-P_F) G` on the supported rank domain. It is not the full Stage-2 inverse Hessian, the raw range noise covariance, or a final trajectory covariance.

`ComputeSparseRecoverability` normalizes nonzero nuisance columns, removes exact zero/duplicate/sign-duplicate representations without changing their column span, and uses sparse QR with rank/conditioning/projection/orthogonality checks. It does not blindly accept QR pivot rank as the frozen rank definition. Uncertain rank or lost numerical resolution fails closed; a mathematically definable pseudoinverse in an unsupported rank case is not silently used as usable information. The core checks N positive definite, R positive semidefinite, N-R consistency, and R rank/PD with an additional projection-roundoff floor. Base absolute/relative rank and PD tolerances are 1e-12/1e-10 (`include/uifgo/nlos_recoverability.h`).

Diagnostics are `eta=lambda_min(N^(-1/2) R N^(-1/2))`, `s_m=1/sqrt(lambda_min(R))` when PD (otherwise infinite), and `gamma_j=mean_(i in S_j)(r_i/sigma_i)^2`. Gamma is post-fit mean squared normalized residual, not a degrees-of-freedom-corrected independent validation statistic.

`LocalAmplitudeSigmas` additionally requires status OK, complete amplitude rank/dimensions, finite symmetric R and checked positive eigenvalues. It solves `R x=e_j` by Cholesky and returns `sigma_c,j=sqrt(x_j)` in metres. It uses the diagonal of the **inverse matrix**, not `1/sqrt(R_jj)`. It adds no jitter, damping, artificial prior or pseudoinverse. Existing physical initial priors are part of F and do contribute; claiming completely prior-free information would be false.

### Admission, amplitude and final measurement set

Let E_j require successful Stage 2, exactly one matching score group, **group eligibility** and valid exported score, nonboundary/non-short segment, finite positive amplitude exactly matching its Values entry, a complete unique scalar amplitude-column mapping, and valid local sigma. Group eligibility requires all segments in that overlap group to be nonboundary and non-short; one boundary segment can therefore exclude its peers from fixed admission.

`L_j=max(0,c_hat_j-2*sigma_c,j)`; `a_j=1` iff E_j and `L_j>0`.

`delta_full_j=a_j*c_hat_j`; `delta_partial_j=a_j*L_j`.

For the full-fixed primary, `O_final=(O \ union_j S_j) union (union_(j:a_j=1) S_j)` with residual `d_i+beta_i+delta_j-z_i` on an admitted candidate, and `d_i+beta_i-z_i` otherwise. Writing `z_corrected=z_raw-delta_j` is algebraically correct; the actual factor keeps the raw measurement and adds delta to the prediction constant. Nonadmission means absence, **not** inclusion with zero correction. On a successful fallback, `O_final=O \ union_j S_j`.

`structured_debias` instead marks all groups USE without LCB or eta/s/gamma admission, retains and reoptimizes their live C variables under nonnegativity, and follows the live-C final-score/audit path. Its final C may differ from Stage-2 C. It is neither the primary fixed policy nor a guaranteed matched-mask full-fixed comparator. `suppress_all` omits all candidates and has no C variables.

Final covariance is a local Gaussian marginal over variables actually retained in the final physical graph at final Values. For fixed policies, it includes IMU/navigation coupling and retained measurement noise, conditional on chosen support, admission and delta. It does **not** propagate support-selection uncertainty, admission uncertainty, delta-estimation variance/correlation, anchor/lever/static-beta calibration uncertainty, or a mixture over rejected/support hypotheses. Live-C can retain local C/navigation coupling within its chosen graph, but also does not solve support-selection uncertainty. Exported per-key marginal blocks are not a claim that all cross-covariance blocks were exported.

## 5. Differences from paper_structure_v4.tex

The v4 blueprint largely matches the intended integrated path, but these discrepancies/qualifications must remain visible rather than silently harmonized:

1. **Strict versus inclusive alarm.** v4 equation `eq:cusum` specifies `G>h`; both implementations use `>=`. Equality is explicitly asserted by `PlPersistentCusum.PersistentModestSignalAndInclusiveThreshold` and `PlBackwardCusum.ReverseRecurrenceAndInclusiveCrossing` tests.
2. **Independent static calibration is not an observed configuration fact.** v4 III describes beta as independently calibrated. Controlled PL configs have an empty beta table; `ValidatePaperFixedBeta` reports `MISSING_CALIBRATION_DEVELOPMENT_ONLY`. An explicit complete table is still only `EXPLICIT_COMPLETE_NOT_PROVENANCE_VERIFIED`. This scientific prerequisite needs evidence.
3. **Innovation beta notation is ambiguous.** v4 writes `nu=z-h(theta)` after defining a range model with beta. Code negates a factor residual that includes beta. If h means geometric distance alone, the v4 equation is missing beta; if h means the complete nominal prediction, state that explicitly.
4. **Primary designation is not a default.** v4's P=`lcb_fixed_full` agrees with the current protocol and explicit manifest consumer. It does not describe `Config` defaults or an inline call with omitted `--method`, which uses the old full gate.
5. **The two reference estimates must be distinguished.** v4's short overview omits the batch raw-reference solve that occurs before signal generation and seeds Stage 2, and the separate bootstrap initialization/fixed-lag state used for innovations. Calling the producer a batch post-fit residual detector, or claiming it uses Stage-2 states, would be false.
6. **Persistent/short support qualification is underspecified.** PL membership has no minimum-count/duration postfilter and sets no short-support flag. The E_j short-support check exists downstream but is inactive for normally constructed PL segments. Do not borrow the earlier FDE's `count>=2`, `duration>=0.01` rule. An isolated sufficiently large excursion can satisfy the CUSUM mechanism; persistence is not a separate guaranteed duration constraint.
7. **LCB eligibility is partly group-wide.** v4's per-segment E_j shorthand must include group eligibility; a boundary peer can invalidate a group's exported score. Admission is then per segment. It is not simply independent `c_hat>2*sigma` checks.
8. **Refit equation is conceptual, not exact scaling or a global-solution guarantee.** v4 `eq:refit` omits the GTSAM 1/2 multiplier on range cost; the same scaling must be used for the IMU/prior term. Actual optimization is alternating with explicit stationarity checks, not a proved global argmin. Support outside candidates has zero dynamic amplitude.
9. **Fallback is not total over all pipeline failures.** v4 V-D's statement is valid for failures inside the final recovery attempt. Failed support production, raw reference, Stage 2 or cache validation can leave no final result. Even rejection can fail to converge. The shared-support rejection consumer depends on a successful producer cache.
10. **Fixed graphs do not carry uncertainty weights from sigma_c.** v4's covariance caveat is correct; add that sigma_c is used only for local admission/amplitude selection, while final factor sigma remains the frozen nominal value. A group inverse-information diagonal is not a calibrated confidence bound or final C posterior.
11. **Forward processing is mathematically causal after bootstrap, operationally offline here.** Current implementation first stores the signal, then evaluates both passes. No production streaming CUSUM feedback loop or end-to-end real-time service is established by the forward alarm timestamp. v4 correctly calls the whole system offline but must retain this execution detail when discussing latency.
12. **Planned validation is not delivered evidence.** v4 explicitly presents future E1–E9 experiments and C3 text as a plan. Current results are a controlled development success plus negative/incomplete six-input diagnostics, not that full campaign. Local source audit does not validate the blueprint's external citations or future experimental claims.

Separate historical inconsistency: the older frozen structure/roadmap prescribe L1/TV discovery, live accepted C, and a different contribution narrative. Older sections of `STATUS.md` and `paper/CLAIM_EVIDENCE.md` still describe FDE or failed forward-only admission. These are historical snapshots. They do not negate the current PL E2E path or authorize importing old gate/live-C semantics into the fixed primary.

### Explicit verification of the requested claims

| Claim | Verdict at audited HEAD | Reason / source |
| --- | --- | --- |
| L1/TV is not production support discovery | TRUE for audited PL path | Runner PL branch invokes the production CUSUM provider, not `automatic_discovery` |
| Forward/backward CUSUM operate per link | TRUE | Both state maps use `PlCusumLinkKey{tag_id,anchor_id}` |
| Backward CUSUM is not backward IMU estimation | TRUE | `EvaluatePlBackwardCusum` consumes only stored `PlCusumInputRow`s/options |
| Persistent discovery precedes bias recovery | TRUE | Provider completes before `SegmentRefitter::Run` |
| Support frozen before Stage 2 | TRUE | `SUCCESS_SUPPORT_FROZEN`, `SupportPartition`, provider-aware cache |
| `lcb_fixed_full` is current production policy | TRUE as explicitly designated primary; FALSE as repository-wide default | Current E2E protocol/configured consumer versus `Config::nlos_mode=disabled` and inline `FULL_GATE` default |
| Positive LCB is admission condition | TRUE for fixed policies, not all policies | `FreezeFixedCompensations`; additional structural/numerical checks also required |
| Admitted full-fixed subtracts c_hat, not LCB | TRUE | `full_variant ? segment.amplitude_m : partial` |
| Fixed inference does not marginalize full bias/support uncertainty | TRUE | Delta is a factor constant; no C keys or selection variables in fixed graph |
| Rejection remains fallback | TRUE with stage boundary | One `RunFrozenCandidatePolicy(...,{})` after invalid final recovery; upstream failures need not reach it |
| No global identifiability, certified integrity, universal superiority or guaranteed improvement justified | TRUE | Local numerical information/admission only; existing report includes worse and unavailable cases |

## 6. Allowed Scientific Statements

Implementation directly supports the following, with the configuration and scope qualifications above:

- An offline C++/GTSAM pipeline constructs link-wise candidate support from pre-commit conditional innovations and forward/backward one-sided CUSUM, freezes it by observation identity, and then estimates nonnegative constant segment biases.
- The signal producer commits all planned raw ranges and may retain historical NLOS contamination; the offline backward pass operates on stored innovations.
- Stage 2 minimizes a physical Gaussian objective with no L1/TV amplitude regularization, using explicit joint convergence checks. Its local information diagnostic accounts for nuisance directions on a declared reference-plus-group linearization.
- Positive local LCB and structural/numerical conditions determine fixed-policy admission. Full-fixed uses the full Stage-2 amplitude; partial uses the clipped LCB on the same initial admission set.
- A valid final result contains each retained observation exactly once. Rejected candidates are absent. Fixed graph uncertainty is conditional on the frozen decisions and offsets. A final-stage invalid recovery has one suppression fallback, whose failure is recorded.

Existing evidence, **reported previously and not rerun here**, is in `doc/ie_0911/PL_CUSUM_E2E_INTEGRATION_ACCURACY_RESULT.md`, evidence root `/home/mint/ws_fusion_uwb/res/pl_cusum_e2e_integration_accuracy_20260912T041820Z`. It reports one controlled Walk1 segment with 30 IDs, Stage 2 converged at outer 42, full compensation `0.47134578518036691 m`, and aligned ATE RMSE `0.16314948912901142 m` versus same-support suppression `0.18163686608072543 m` on 229 common GT samples. The paired range error is relative to the clean counterpart's ranges, not geometric range truth. The same report records six-input diagnostics: **0 better, 1 worse, 1 tied, 4 unavailable**. Any quotation of the positive case must preserve this development-only scope and contrary/incomplete evidence.

Current test sources cover inclusive thresholds, per-link separation, invalid/gap resets, last-zero backfill, exact intersection, absence of backward rescue, analytic conditional rows, constrained amplitude update, inverse-diagonal versus inverse-diagonal-entry distinction, observation multiplicity, identity checks, forced fallback and explicit covariance unavailability. Relevant files are `test/test_pl_{persistent_cusum,bidirectional_support,conditional_raim}.cpp` and `test/test_nlos_{refit,recoverability,inference}.cpp`. Test definitions are implementation evidence; they were **NOT_RUN here**. The prior E2E report attributes 100/100 affected GTests, 34/34 CTest and a package build pass to its own run, not this audit.

## 7. Evidence-Dependent or Forbidden Statements

- Do not claim global identifiability/observability, globally optimal refits, certified integrity, calibrated protection levels, or a guaranteed 95% one-sided confidence bound from the constant 2. Numerical rank certification is an implementation-domain check, not system integrity certification.
- Do not claim universal recovery superiority, guaranteed trajectory improvement, guaranteed range improvement, low-redundancy benefit, nominal noninferiority, or cross-dataset generalization from code or the controlled case. Positive delta only guarantees algebraically decreasing a range; it does not guarantee approaching truth.
- Do not call the primary posterior a full bias/support-marginalized posterior, use sigma_c as final bias posterior uncertainty, or present the final marginals as calibrated system coverage. Do not claim independent reuse merely because factors are not duplicated: offsets and support are estimated from overlapping data.
- Do not claim independent LOS beta/noise calibration merely because configuration fields exist; current controlled beta is missing. Do not equate unlabelled nominal data with verified LOS.
- Do not claim the producer prior is free of faults, multiple concurrent faults cannot contaminate conditionalization, or bidirectional support always improves recall/precision. The code supplies mechanisms, not such guarantees.
- Do not claim every candidate is admitted, every upstream failure returns a rejection trajectory, covariance is always available, or no-candidate/fallback cases are successful recovery. Preserve all unavailable/failed cells and distinguish requested versus actual policy.
- Do not present earlier oracle-support results as detector E2E, clean-paired range differences as geometric range error, post-fit residual as independent bias truth, or SE(3)-aligned ATE as verified raw-world-frame accuracy.
- Do not claim a complete E1–E9 study, calibrated false-alarm budget on unseen records, real obstruction generalization, future-information benefit or formal held-out C1–C3 validation without new appropriately locked evidence. This audit does not upgrade T10=C2-C or T11=C.

## 8. Methodology Writing Contract

1. Name the audited executable, HEAD, explicit PL mode and explicit full-fixed primary. Separate runtime defaults, legacy node and optional comparators.
2. Define X/V/IMU B, fixed static beta, and dynamic segment C separately; include lever-arm geometry, fixed-calibration scope, valid/planned input selection and frozen gap-dependent range sigma.
3. Distinguish batch raw-reference initialization for Stage 2 from the bootstrap/fixed-lag state used for conditional innovations. State bootstrap 0–4, 200-epoch lag and always-commit information budget.
4. Define innovation as negative nominal raw-factor residual, including beta, conditioned on other current-group ranges. Do not substitute a post-fit residual or group chi-square detector.
5. Use per-link `max(0,previous+u-0.5)` in both directions, inclusive threshold crossing, excursion backfill/latching, zero/invalid/strict-gap reset and exact ID intersection. Backward processing is noncausal sequence processing, not backward IMU estimation.
6. Freeze explicit supports before bias recovery; do not invent minimum-duration/count filtering, boundary refinement, detector feedback or readmission.
7. Give the actual half-squared Gaussian Stage-2 objective, nonnegative scalar update and local convergence/failure conditions. No L1/TV, C prior, global optimum or arbitrary iteration-cap success.
8. Define overlap groups, common candidate-excluded reference plus scored-group factors, shared Stage-2 linearization, nuisance projection, supported rank domain and checked inverse-information diagonal. Existing physical priors count; solver damping does not.
9. Define structural/group eligibility and positive `max(0,c_hat-2 sigma_c)` admission. Full-fixed freezes c_hat; partial freezes the LCB. No eta/s/gamma threshold chain in the primary.
10. Distinguish live-C `structured_debias` (all groups USE) and same-support `suppress_all`; do not imply their masks/gates/posterior semantics equal full-fixed.
11. Final fixed residual is `d+beta+delta-z` with unchanged nominal sigma and one factor per retained ID, no live C. A rejected candidate contributes no factor. Trajectory/IMU bias/residuals/covariance use the same final graph/Values.
12. State conditional uncertainty limits and the bounded final-stage fallback. Keep upstream failures, failed rejection, covariance-unavailable, zero-candidate and fallback outcomes visible. No GT/oracle/clean-counterpart input to the estimator; quantitative scientific conclusions require separate evidence.
