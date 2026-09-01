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
- Commit SHA: to be recorded after the milestone commit.

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
- Commit SHA: to be recorded after the milestone commit.
