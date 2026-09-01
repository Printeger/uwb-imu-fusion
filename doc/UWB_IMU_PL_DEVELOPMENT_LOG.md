# UWB–IMU–PL development log

Overall delivery status: `IMPLEMENTED_UNVERIFIED`.

## 2026-09-01 — M0 research contract and infrastructure

- Branch baseline: `main@6f8aaa1`; feature branch: `feature/realtime-uwb-imu-pl`.
- Added a ROS-independent `uwb_imu_pl` namespace while preserving the existing `uifgo` package, APIs, launch files and batch LM/GNC path.
- Added nanosecond timestamps and strongly typed anchor, measurement, factor, state, batch and hypothesis identifiers.
- Added strict YAML parsing: safety-relevant fields and seed are mandatory, unknown keys are rejected, probability/range checks fail fast, and Method B/fixed-lag capability choices are explicit.
- Added resolved-config, manifest, states, residuals, integrity, timing, event and summary output contracts. The config hash uses deterministic FNV-1a-64 as an experiment identity checksum, not as a cryptographic integrity mechanism.
- Fixed conventions: Z-up world, body-to-world pose, body-frame specific force, TWR metre ranges, `z-h(x)` innovations and common Cholesky whitening.
- Defined HMI, availability and formal-scope semantics. The formal scope is current-group, single-physical-anchor faults only.
- Configuration `config/realtime_uwb_imu_pl_research.yaml` is explicitly research-only.
- Verification status: source implemented; build/tests deferred because the available host is Ubuntu 24.04 / ROS2 only and lacks the required ROS1 Noetic + GTSAM 4.2.x environment.
- Commit SHA: `ed75347`.

## Known limitations carried from M0

- TDoA, multiple simultaneous faults, FDE, nonlinear remainder bounds and certified noise overbounds are not implemented.
- Manifest config hashing is reproducibility metadata, not tamper evidence.
- All formal labels remain unvalidated until the deferred test ledger is executed in the required environment.

## 2026-09-01 — M1–M4 snapshot UWB RAIM

- Implemented 3D TWR Gauss–Newton WLS/NLS with heteroscedastic diagonal or full covariance, positive-definite validation and one common Cholesky whitener for residuals, Jacobians and fault incidence.
- Added convergence, numerical rank, condition number, covariance and linearization-step diagnostics. Rank-deficient or over-condition-limit geometry is explicitly unavailable.
- Added post-fit chi-square detection with `DOF = n - rank(H)` and threshold from configured `P_FA`.
- Generated hypotheses by physical `AnchorId`, not row index. Added detector Gram, protected-axis failure slope, explicit unmonitorable/infinite-slope handling and a bracketed noncentral chi-square missed-detection boundary.
- Added per-axis nominal/fault components, maximizing anchor, `PL_xyz`, box-horizontal PL, vertical PL and alert-limit availability decisions.
- Added a deterministic sweep executable spanning straight/circle/figure-eight trajectories, geometry scale, noise, fault anchor/magnitude and explicit risk/seed dimensions. It is source-only in this delivery and has not been run.
- Added deferred GoogleTest coverage for whitening/full covariance, analytic Jacobian, chi-square DOF, physical-anchor mapping, noncentral boundary and rank-deficient infinite PL.
- Commit SHA: `1a28547`.

## 2026-09-01 — M5–M7 incremental estimation

- Added a UWB-only iSAM2 bridge with deterministic position/velocity keys, a constant-velocity factor and group UWB factor. Smoothness rows are exported as `RowRole::Regularizer`; reported detector measurement-row count excludes them.
- Added immutable `EstimationSnapshot` capability negotiation, factor/measurement/anchor row provenance, linearization versions, current marginal and information-solve surface without exposing a mutable graph or mutable values.
- Added a full-history tightly coupled iSAM2 estimator with `(Pose3, velocity, IMU bias)`, Combined IMU preintegration and covariance-preserving UWB group factors including the body-frame lever arm.
- Implemented the Method A lifecycle `predictTo -> preMeasurementSnapshot -> commitUwbBatch/rejectUwbBatch`. The snapshot contract asserts that the prior excludes the current UWB batch.
- Current `(Pose3,v,bias)` covariance is extracted as a 15×15 joint marginal through public GTSAM factor/marginal APIs. World-frame protected position uses the Pose3 tangent Jacobian `[0, R_WB]`, with a finite-difference regression test, rather than direct covariance indices.
- Joint marginals are assembled explicitly from keyed blocks in contract order;
  `JointMarginal::fullMatrix()` ordering is not assumed.
- Added source-level capability placeholders for fixed lag and historical fault provenance; both explicitly report unsupported. The implementation retains full history.
- Strengthened the legacy `uifgo::ImuPreintegrator` with finite-sample, positive-dt, valid-interval and null-output checks while preserving its API.
- Added deferred tests for regularizer row semantics, Pose3 protected-position Jacobian and Method A exclusion of current UWB.
- Commit SHA: `9385e3a`.

## 2026-09-01 — M8 conditional detector and fusion PL

- Implemented Method A conditional monitoring from an immutable pre-current-UWB prior. The monitor rejects missing capabilities, current-batch double counting, blind cached linearization and any version mismatch.
- Computes whitened innovation covariance `S_w = H_w P^- H_w^T + I`, chi-square statistic with DOF equal to current UWB group size, standardized per-anchor diagnostics, posterior covariance and current-only single-anchor slopes.
- Uses the same full-covariance Cholesky whitener for the innovation, factor Jacobian and physical-anchor fault incidence. The protected world-position map is `[0,R_WB]` in the 15-state tangent ordering.
- Added explicit comparison outputs for global graph squared residual, UWB post-fit residual and conditional innovation. Only the conditional detector feeds the main fusion PL.
- Added a pipeline enforcing detect-before-commit. Passing groups are committed in a second iSAM2 update; failed groups are rejected wholesale and publish the IMU/history prior with infinite PL and `ALERT` (or `UNAVAILABLE` for invalid assumptions).
- Implemented the Method B information-downdate candidate with dimension, SPD and condition-number gates. It is disabled by default and is not used for formal output pending Method A/B equivalence and timing validation.
- All formal outputs are scoped and labelled `FORMAL_LOCAL_CURRENT_FAULT_ONLY`; fixed-lag, persistent/history fault PL and provenance-preserving marginalization remain unsupported.
- Added deferred tests for conditional DOF/statistics and Method B algebraic recovery.
- Commit SHA: `83f29f7`.

## 2026-09-01 — Simulation, ROS1 adapter and delivery documentation

- Added a ROS1 realtime node with callback-only enqueueing and one ordered worker. LinkTrack frames map to UWB groups; configured anchors are resolved by physical ID; odometry, integrity status and diagnostics are published.
- Added strict anchor-map, initial-state, prior-sigma, lever-arm and range-noise configuration. Anchor frames must match the protected world frame.
- Added deterministic straight, circle and figure-eight trajectory generation and explicit-seed UWB simulation modes for no fault, one-anchor step/ramp bias, magnitude sweep and outage.
- Added run output wiring for resolved config, manifest, states, integrity comparisons, timing, events and summary. Generated results are gitignored.
- Added the authoritative deferred-test ledger and README status/scope warning.
- Static verification passed: `git diff main --check`; conflict-marker scan;
  Node YAML parsing of 27 configs; Python AST parsing of 5 scripts; XML parsing
  of 7 package/launch files; changed-Markdown newline/NUL checks; CMake/message
  structural checks; and changed-path artifact/sensitive-pattern scans. The
  existing tracked `doc/experimental_results.pdf` is unchanged baseline content.
- Verification status remains `IMPLEMENTED_UNVERIFIED`; no compile, GoogleTest,
  Monte Carlo, ROS, simulation or runtime/performance command was run.
- Commit SHA: see the final delivery report; a commit cannot embed its own
  immutable SHA without changing that SHA.
