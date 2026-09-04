# Gate A baseline evidence

- Captured: 2026-09-04, Asia/Shanghai
- Branch: `feature/realtime-uwb-imu-pl`
- Commit: `ae54fb8ca55dfbfaf64fe45615b6bcd106548a93`
- Configuration: `config/realtime_uwb_imu_pl_research.yaml`
- Existing acceptance inventory: 138 tests
- Local CTest result: 16/16 suites passed, 0 failed
- Build profile used by the frozen baseline: Release
- Known warning: the existing simulator-generated-message symlink warning is
  retained as a baseline warning; it is not an integrity algorithm failure.

The resolved V4 configuration and its runtime hash are written by `RunLogger`
for every new run.  No claim in this evidence file elevates the research
implementation to Gate J formal status.
