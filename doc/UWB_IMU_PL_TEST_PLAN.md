# UWB–IMU–PL test ledger

Deterministic acceptance status: `PASS` on 2026-09-01. This is the authoritative
execution ledger for the P0 baseline recovery and P1 M0–M8 correctness work. It
does not make a certification claim.

## Verified environment

| Item | Actual value |
|---|---|
| Host | Ubuntu 20.04.6 LTS |
| ROS | ROS 1 Noetic |
| GTSAM | 4.2a5 (`GTSAM_VERSION_NUMERIC=40200`) |
| GCC | 9.4.0 |
| CMake | 3.16.3 |
| catkin tools | 0.9.2, Python 3.8.10 |
| Package | `uwb_imu_pl`; legacy C++ namespace/API `uifgo` retained |
| Research configuration | `config/realtime_uwb_imu_pl_research.yaml` |
| Verification date | 2026-09-01 (Asia/Shanghai) |

The package was cleaned independently; build products belonging to the sibling
`uwb_imu_fgo` package were not cleaned. The final retained build is Release.
There are 51 distinct GoogleTest cases. `catkin_test_results` reports 102 because
the generated GoogleTest XML contains both root and suite aggregate counts; the
result remains 0 errors, 0 failures and 0 skipped.

## Build and regression

| ID | Command | Status | Actual result |
|---|---|---|---|
| BUILD-DBG | `source /opt/ros/noetic/setup.bash && catkin clean uwb_imu_pl -y && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug && catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl && catkin_test_results --verbose` | `PASS` | Clean Debug build; all 51 GoogleTest cases passed. |
| BUILD-REL | `source /opt/ros/noetic/setup.bash && catkin clean uwb_imu_pl -y && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl && catkin_test_results --verbose` | `PASS` | Clean Release build; 0 errors, 0 failures, 0 skipped. Release retained. |
| REG-001 | Same package test command | `PASS` | All legacy config, outlier, trilateration, IMU, UWB-factor, graph-builder and optimizer tests passed. |
| LAUNCH-001 | `roslaunch --files uwb_imu_pl realtime_integrity_sim.launch` | `PASS` | Exactly one top-level forwarding launch and one uniquely named internal `realtime_integrity_stack.launch` resolved. |

## Deterministic M0–M8 acceptance

All commands below run from `/home/mint/ws_fusion_uwb` after sourcing
`devel/setup.bash`.

| ID | Coverage / command | Acceptance | Status |
|---|---|---|---|
| M0-001 | `test_integrity_config` | Valid research YAML; every required section field missing; unknown root/section/anchor keys; invalid total risk; unsupported Method B/fixed lag | `PASS` (4 cases) |
| M0-002 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.RepeatedExecutionIsDeterministic'` | Deterministic repeated estimator output and audit state | `PASS` |
| M0-003 | `test_run_logger` | `write_residuals` and `write_timing` independently control CSV creation | `PASS` |
| M1-001 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.FullCovarianceAndAnalyticJacobianAreConsistent'` | Analytic range Jacobian and finite differences agree | `PASS` |
| M1-002 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.DiagonalAndExplicitCovarianceWhiteningAreEquivalent'` | Diagonal/full-covariance whitening equivalence | `PASS` |
| M1-003 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.RankDeficientGeometryIsUnavailable'` | Invalid rank fails closed with infinite PL | `PASS` |
| M2-001 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.ChiSquareDofAndPhysicalAnchorHypotheses'` | Projector symmetric/idempotent, correct DOF and physical mapping | `PASS` |
| M3-001 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.NoncentralityBoundaryMeetsMissedDetectionAllocation'` | Noncentral CDF inverse error `<1e-10` | `PASS` |
| M3-002 | snapshot slope assertions in `test_snapshot_integrity` | Closed form agrees with dense oracle; unmonitorable modes fail closed | `PASS` |
| M4-001 | `test_snapshot_integrity --gtest_filter='SnapshotIntegrity.FormalGateRiskAlarmAndAlertLimitBranchesFailClose'` | Radial HPL, total-risk allocation and formal gate/alarm/AL branches | `PASS` |
| M5-001 | `test_realtime_incremental --gtest_filter='UwbIncremental.TwentyEpochsMatchBatchPositionAndMarginal'` | 20-epoch iSAM2 position `<1e-6 m`; marginal relative error `<1e-6` | `PASS` |
| M5-002 | `test_realtime_incremental --gtest_filter='UwbIncremental.RegularizerRowsAreExcludedByRole'` | Regularizer rows excluded from formal statistic | `PASS` |
| M5-003 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.BatchSkewFailsBeforeGraphMutation'` | Epoch span/member skew rejected before graph mutation | `PASS` |
| M6-001 | `test_imu_preint` | Static specific force, constant world acceleration and angular rate; position/velocity `<1e-6`, rotation `<1e-7 rad` | `PASS` (8 cases total) |
| M6-002 | `test_imu_preint --gtest_filter='ImuSync.*'` | Exact interval, boundary interpolation, adjacent means, gap/order fail-close | `PASS` |
| M6-003 | realtime gap/future-sample cases | Gap/duplicate leaves audit unchanged; future sample cannot affect current prediction | `PASS` |
| M7-001 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.FiveSecondIsamMatchesBatchPoseAndMarginal'` | 5 s Pose3 local error `<1e-5`; marginal relative error `<1e-5` | `PASS` |
| M7-002 | `test_realtime_incremental --gtest_filter='RealtimeFactors.*'` | Nonzero-lever-arm Pose3 Jacobian central-difference error `<1e-7` | `PASS` (2 cases) |
| M8-001 | Method A prior/provenance/formal-gate cases in `test_realtime_incremental` | Prior excludes current UWB; factor count/version unchanged; 15x15 SPD prior and innovation/provenance gates | `PASS` |
| M8-002 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.MethodBDowndateCandidateRecoversPrior:UwbImuIncremental.MethodAAndLinearMethodBCandidateGiveEquivalentOutput'` | Offline Method B candidate recovers/equates prior, statistic and PL; online Method B remains rejected | `PASS` |
| M8-003 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.FaultAlarmRejectsOnlyCurrentUwbAndNextBatchRecovers'` | 20 m fault alarms; no UWB commit/version change; exact IMU-only state; next changed-anchor batch recovers | `PASS` |
| M8-004 | `test_realtime_incremental --gtest_filter='UwbImuIncremental.RiskOrAlertLimitUnavailableStillCommitsValidUwb'` | Risk/AL unavailability does not suppress a detector-passing UWB commit | `PASS` |

## Explicitly out of this acceptance scope

These items were not executed and are not represented as passing.

| ID | Item | Status | Reason / next artifact |
|---|---|---|---|
| MC-001 | Snapshot covariance, H0 false-alarm, noncentral-fault and PL-coverage Monte Carlo | `NOT_IMPLEMENTED` | Long-run harness and confidence-interval report are deferred. |
| MC-002 | IMU bias-Jacobian/covariance Monte Carlo and conditional-detector calibration | `NOT_IMPLEMENTED` | Statistical harness is deferred. |
| SIM-001 | Full straight/circle/figure-eight ROS topic simulations and fault sweeps | `NOT_RUN_SCOPE` | Launch parsing was verified; long-running topic-level experiments are outside this batch. |
| ROS-001 | Dedicated `rostest` topic/queue/recovery fixtures | `NOT_IMPLEMENTED` | Deterministic estimator-level equivalents pass; ROS integration fixtures remain future work. |
| PERF-001 | 600 s 20 Hz latency and PL-overhead study | `NOT_IMPLEMENTED` | Benchmark harness/report are deferred. |

## Static checks

| Check | Status | Actual result |
|---|---|---|
| `git diff --check` | `PASS` | No whitespace errors. |
| Conflict-marker scan | `PASS` | No unresolved markers in source/config/launch/tests. |
| YAML, XML and Python syntax parsing | `PASS` | Repository configuration, launch/package XML and Python tooling parsed successfully. |
| Legacy-name scan | `PASS` | No ROS package/node/topic/launch compatibility alias for `uwb_imu_fgo`; only the intended legacy `uifgo` C++ API remains. |
