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

## P2/P3 closure update — 2026-09-01

P0/P1 acceptance above remains historical. The current P2/P3 overall status is
`IMPLEMENTED_UNVERIFIED`, not PASS. There are now 60 distinct GoogleTest cases;
the latest Release run reports the catkin aggregate `120 tests, 0 errors,
0 failures, 0 skipped`. Detailed Chinese/HTML evidence is in
`doc/P2_P3_VALIDATION_REPORT.md` and `.html`.

| ID | Actual execution | Status | Samples / artifact |
|---|---|---|---|
| P2-ORACLE | Independent dense SVD covariance/slope and unmonitorable fail-close | `PASS` | 1000 random full-rank cases + failure fixtures; `test_dense_oracle` |
| P2-MC-SMOKE | H0/noncentral runner functional smoke | `PASS_FUNCTIONAL_ONLY` | H0 16×1000; noncentral 160×1000; `/tmp/uwb_pl_quick.*/mc/` |
| P2-SWEEP-SMOKE | resumable sweep + seed block-bootstrap functional smoke | `PASS_FUNCTIONAL_ONLY` | 4104 raw rows / 2052 summary rows; temporary artifact |
| P2-MC-H0-FULL | 16 jobs × 200k formal H0 sampling | `PASS` | 16/16 target-in-exact-CI and KS p>0.01; `results/p2_monte_carlo/` |
| P2-MC-NC-FULL | 160 jobs × 100k formal noncentral sampling | `FAIL` | 154/160 theory-in-exact-CI; six failures retained; same artifact |
| P2-SWEEP-FULL | planned 100 seeds × 200 epochs sweep | `NOT_RUN` | runner implemented; target `results/p2_sweep.csv` |
| P3-DETERMINISM | 10 s event ordering, stale UWB, drop and anchor-set fixtures | `PASS` | deterministic tests; 1 s drop; `8→6→4→8` |
| P3-ROS-SMOKE | Early 12 s nominal figure-eight plus v2 schema validation | `FAIL_RETAINED` | 217 epochs; 122 commits; 94 ALERT; 1 fail-closed numerical reinit; `/tmp/uwb_pl_ros_smoke6.O57oge/` |
| P3-ROS-43S-F8 | Dogleg/Cholesky, relinearize skip 10, nominal figure-eight | `PASS` | 836/836 commits; all AVAILABLE; PE mean/P95/max 0.081/0.145/0.229 m; schema PASS; temporary artifact |
| P3-ROS-43S-NOMINAL | straight/circle/figure-eight nominal | `PASS` | all three 43 s runs: 838/838, 836/836, 836/836 commits; no reject/reinit |
| P3-ROS-43S-FAULT | step/ramp/magnitude-sweep/outage | `PARTIAL` | all detected/aligned as expected; persistent step caused one explicit controlled reinit because FDE is not implemented |
| P3-METHOD-AB | 600 real multi-epoch comparisons | `NOT_RUN` | linear mean/covariance downdate fixtures PASS; online switch still rejected |
| P3-PERF | 3×600 s timing gate | `NOT_RUN` | 43 s warm core mean/P99 19.18/81.48 ms PASS only for current duration; not the 600 s conclusion |

## Build and regression

| ID | Command | Status | Actual result |
|---|---|---|---|
| BUILD-DBG | `source /opt/ros/noetic/setup.bash && catkin clean uwb_imu_pl -y && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug && catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl && catkin_test_results --verbose` | `PASS` | Clean Debug build; all 51 GoogleTest cases passed. |
| BUILD-REL | `source /opt/ros/noetic/setup.bash && catkin clean uwb_imu_pl -y && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build uwb_imu_pl && catkin run_tests uwb_imu_pl && catkin_test_results --verbose` | `PASS` | Clean Release build; 0 errors, 0 failures, 0 skipped. Release retained. |
| REG-001 | Same package test command | `PASS` | All legacy config, outlier, trilateration, IMU, UWB-factor, graph-builder and optimizer tests passed. |
| LAUNCH-001 | `roslaunch --files uwb_imu_pl realtime_integrity_sim.launch` | `PASS` | Exactly one top-level forwarding launch and one uniquely named internal `realtime_integrity_stack.launch` resolved. |
| VIZ-BUILD-001 | `catkin build uwb_imu_pl --no-status && catkin run_tests uwb_imu_pl --no-status` | `PASS` | Release incremental build; 122 aggregate tests, 0 errors/failures/skips, including the new visualization rostest. |
| VIZ-ROS-001 | `rostest uwb_imu_pl realtime_integrity_viz.test` | `PASS` | Bounded paths, exact timestamp pairing, PL geometry/colors and infinite-PL stale-marker deletion passed. |
| VIZ-SMOKE-001 | `realtime_integrity_sim.launch trajectory:=figure_eight rviz:=true` | `PASS` | 76 s live run; GT/fused paths and four PL markers updated in RViz, 1528 commits, 2 explicit stale-UWB rejects, 0 processing errors; v2 schema PASS at `/tmp/uwb_imu_pl_rviz_final.jLfLgH`. |

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
| MC-001 | Snapshot covariance, H0 false-alarm, noncentral-fault and PL-coverage Monte Carlo | `PARTIAL_FAIL` | Formal H0 PASS and noncentral FAIL (6/160); full PE–PL/coverage sweep remains NOT RUN. |
| MC-002 | IMU bias-Jacobian/covariance Monte Carlo and conditional-detector calibration | `PARTIAL_NOT_FULL_RUN` | Detector calibration/ROC tool exists; full independent calibration remains NOT RUN. |
| SIM-001 | Full straight/circle/figure-eight ROS topic simulations and fault sweeps | `PARTIAL` | all three nominal runs PASS; four fault runs executed, with step recovery limitation retained. |
| ROS-001 | Dedicated `rostest` topic/queue/recovery fixtures | `NOT_IMPLEMENTED` | Deterministic estimator-level equivalents pass; ROS integration fixtures remain future work. |
| PERF-001 | 600 s 20 Hz latency and PL-overhead study | `IMPLEMENTED_NOT_RUN` | v2 stage timing/report plumbing exists; 3×600 s execution remains NOT RUN. |

## Static checks

| Check | Status | Actual result |
|---|---|---|
| `git diff --check` | `PASS` | No whitespace errors. |
| Conflict-marker scan | `PASS` | No unresolved markers in source/config/launch/tests. |
| YAML, XML and Python syntax parsing | `PASS` | Repository configuration, launch/package XML and Python tooling parsed successfully. |
| Legacy-name scan | `PASS` | No ROS package/node/topic/launch compatibility alias for `uwb_imu_fgo`; only the intended legacy `uifgo` C++ API remains. |
