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
