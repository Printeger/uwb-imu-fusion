# UWB–IMU realtime integrity and protection levels

This ROS 1 Noetic package provides a tightly coupled UWB–IMU factor-graph
estimator and research FGO/FDE/PL pipeline. Legacy offline code is kept under
[`archive/legacy-ie`](archive/legacy-ie/README.md) and is not part of the active
build, launch or test path.

This remains research software, not a certified integrity implementation.
Active profiles remain `formal_eligible=false`; the measured attitude,
active-profile realtime, PL availability and recovery-action failures are
reported without relaxation in
[`doc/UWB_IMU_FGO_FDE_IMPLEMENTATION_REPORT.md`](doc/UWB_IMU_FGO_FDE_IMPLEMENTATION_REPORT.md).

## One build, five runtime profiles

The complete system is compiled once. All five profiles run the same binary:

```text
devel/lib/uwb_imu_pl/uwb_imu_pl_realtime_node
```

The supported runtime profiles are:

- `off`
- `uwb_order1`
- `imu_order1`
- `joint_order1`
- `joint_order2`

They share one estimator, IMU preintegration, UWB measurement model,
transaction layer and fixed-lag backend. A profile changes only the FDE fault
scope, hypotheses and permitted recovery actions. `off` still fuses UWB and
IMU and publishes the optimized state; its protection level is explicitly
`NOT_COMPUTED`.

Switching profile only requires stopping and restarting the node with another
version-6 `config/fde_<profile>.yaml`. It does not require recompilation.
In-process hot switching is not supported. Do not apply an `fde_profile`
override to the default legacy v5 research config: pair the override with the
matching v6 profile config as shown below.

## Build and test

From the catkin workspace root:

```bash
cd /home/mint/ws_fusion_uwb
source /opt/ros/noetic/setup.bash
catkin build uwb_imu_pl --cmake-args -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
catkin run_tests uwb_imu_pl --no-status
catkin_test_results build/uwb_imu_pl --all
```

No rebuild is needed for any of the following profile changes.

## Daily simulation and RViz (no files by default)

Ordinary ROS and RViz runs default to `enable_run_logging:=false`. They publish
topics and diagnostics normally, but do not construct a `RunLogger`, create an
`online_<timestamp>` directory, or write a manifest, resolved config or CSV.

```bash
PROFILE=joint_order1
roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  trajectory:=figure_eight \
  rviz:=true \
  config_path:="$(rospack find uwb_imu_pl)/config/fde_${PROFILE}.yaml" \
  fde_profile:="${PROFILE}"
```

Set `rviz:=false` for a headless simulation. The same command is valid for
each of the five profile values listed above.

## Live IMU/UWB topics (no files by default)

`realtime.launch` consumes the topics configured in the selected YAML; the
standard config uses `/imu/data` and `/nlink_linktrack_nodeframe3`.

```bash
PROFILE=uwb_order1
roslaunch uwb_imu_pl realtime.launch \
  config_path:="$(rospack find uwb_imu_pl)/config/fde_${PROFILE}.yaml" \
  fde_profile:="${PROFILE}" \
  enable_run_logging:=false
```

The authoritative atomic result is `/uwb_imu_pl/solution`, with message type
`uwb_imu_pl/NavigationIntegrity`. It binds state, timestamps, profile, scope,
PL, commit decision and publication certificate. Odometry and
`IntegrityStatus` are compatibility mirrors emitted after the atomic result;
new consumers should not reconstruct state/PL identity from those mirrors.

## Formal evidence logging (explicit opt-in)

Enable logging only for a run that needs artifacts. An explicit
`run_directory` must not already exist; the node fails closed instead of
overwriting it.

```bash
PROFILE=joint_order1
TEMP_ROOT="$(mktemp -d /tmp/uwb_imu_pl_evidence.XXXXXX)"
RUN_DIRECTORY="${TEMP_ROOT}/run"
roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  trajectory:=figure_eight \
  fault_mode:=none \
  random_seed:=20260901 \
  packet_loss_prob:=0.0 \
  nlos_probability:=0.0 \
  rviz:=false \
  config_path:="$(rospack find uwb_imu_pl)/config/fde_${PROFILE}.yaml" \
  fde_profile:="${PROFILE}" \
  enable_run_logging:=true \
  run_directory:="${RUN_DIRECTORY}" \
  write_residuals:=false \
  write_timing:=true \
  write_global_diagnostics:=false
```

The manifest records the expanded directory, not the shell variable name. It
also records the resolved `config_path`, `fde_profile`, `fixed_lag_epochs`,
seed, logging switches and simulator fault/trajectory parameters. Together
with the resolved config, config hash and scope digest, this identifies the
run.

If logging is enabled without `run_directory`, the node creates a unique
`<output.root>/online_<wall-time-ns>` directory. Relative `output.root` values
are resolved from the ROS process working directory (normally `~/.ros` under
`roslaunch`), so formal runs should provide an absolute directory explicitly.
Fine-grained `write_residuals`, `write_timing` and
`write_global_diagnostics` overrides continue to apply only after run logging
has been enabled.

## Offline validation tools

Offline campaign tools explicitly create their own evidence and are unaffected
by the interactive ROS default:

```bash
cd /home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl
python3 tools/generate_fde_profile_configs.py
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate fgo \
  --output results/fde_profiles/fgo
python3 tools/run_fde_profile_validation.py \
  --protocol config/fde_profiles_validation.yaml --gate profiles \
  --profiles all --output results/fde_profiles/profiles
python3 tools/analyze_fde_profile_runs.py \
  --input results/fde_profiles --report results/fde_profiles/summary.md
```

These commands can be expensive. The preserved implementation results and
known failures are in the implementation report; a short interface smoke must
not be represented as a replacement for those campaigns.
