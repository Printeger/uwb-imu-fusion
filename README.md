# UWB–IMU realtime integrity and protection levels

This branch develops the realtime UWB–IMU estimator, integrity monitor, fault
detection/exclusion research pipeline, and horizontal/vertical protection-level
outputs. It is a ROS 1 Noetic catkin package named `uwb_imu_pl`.

The repository was forked from an offline UWB–IMU factor-graph project. Files
that belonged only to that legacy `uifgo`/offline implementation are preserved
under [`archive/legacy-ie`](archive/legacy-ie/README.md); they are not compiled,
installed, launched, or tested by the active package.

## Active system

- Realtime ROS adapter: `tools/run_realtime_integrity.cpp`
- PL/integrity implementation: `include/uwb_imu_pl`, `src/uwb_imu_pl`
- Research configuration: `config/realtime_uwb_imu_pl_research.yaml`
- Hardware/data launch: `launch/realtime.launch`
- Closed-loop simulation launch: `launch/realtime_integrity_sim.launch`
- Current tests: `test/test_snapshot_integrity.cpp`,
  `test/test_realtime_incremental.cpp`, `test/test_integrity_v2.cpp`, and the
  remaining PL/tool/visualization tests in `test/`

This remains research software rather than a certified integrity implementation.
The latest detailed evidence and known limitations are recorded in
`doc/UWB_IMU_PL_TEST_PLAN.md` and `doc/P2_P3_VALIDATION_REPORT.md`.

## Build and test

From the catkin workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin build uwb_imu_pl
source devel/setup.bash
catkin run_tests uwb_imu_pl
catkin_test_results --verbose
```

Run against live topics:

```bash
roslaunch uwb_imu_pl realtime.launch
```

Run the deterministic closed-loop simulator:

```bash
roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  trajectory:=figure_eight rviz:=false
```
