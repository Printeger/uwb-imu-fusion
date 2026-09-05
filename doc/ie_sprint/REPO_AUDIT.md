# 仓库审计：T00 本地工程、数据与旧基线

> 审计时间：`2026-09-05T10:27:45Z` 起（UTC）
> 状态：**T00 `BLOCKED`**。当前提交的构建已实际执行并保留失败证据，但测试未达到可执行前置；预存旧二进制也在载入阶段因 GTSAM ABI 不兼容退出，没有生成轨迹或评估结果。因此不满足 roadmap 的 T00 `DONE` 标准。

本文件严格区分三类结论：**源码声明**只描述静态实现；**当前环境观察**只描述本次 shell 和本机文件；**实际运行通过**必须有命令、退出码和产物。全部原始证据位于 [`evidence/t00_20260905T102745Z/`](evidence/t00_20260905T102745Z/)。

## 1. 仓库快照与冻结材料

| 项目 | 分类 | 结果 | 证据 |
|---|---|---|---|
| 实际仓库 | 当前环境观察 | `/home/dev/ws_uwb_ie/src/uwb-imu-fusion` | [`00_start_snapshot.log`](evidence/t00_20260905T102745Z/00_start_snapshot.log) |
| 分支 | 当前环境观察 | `feature/uwb-imu-fusion-ie-postprocessing` | 同上 |
| 完整提交 | 当前环境观察 | `28d8e9fff8fe08f2210553f4fc1d78e44191f014` | 同上 |
| T00 开始时 dirty 状态 | 当前环境观察 | dirty；已有未跟踪 `AGENTS.md`、`doc/ie_sprint/STATUS.md`、`doc/ie_sprint/REPO_AUDIT.md`。没有 tracked 或 staged 改动；这些已有文件均保留 | 同上 |
| 冻结论文结构 | 当前环境观察 | SHA-256 `8ac373919823d755d9c1b4ceb807527432d16b69d368042e19aa56d93dc67144` | 同上 |
| 冻结 roadmap | 当前环境观察 | SHA-256 `b9bb63b65ebdb8a505bf99b26181318c6c64da1a72817b08cfa61315e2333bdb` | 同上 |
| 相邻源码差异 | 当前环境观察 | `/home/dev/ws_uwb/src/uwb-imu-fusion` 为 `main@cfe6d29cc2a0077d9a0b958461d1615aa5e9079e`；`cfe6d29..28d8e9f` 只移动/增加 `doc/` 文件，无源码差异 | [`02_source_identity.log`](evidence/t00_20260905T102745Z/02_source_identity.log)、[`03_old_build_resolution.log`](evidence/t00_20260905T102745Z/03_old_build_resolution.log) |

## 2. 工作空间、overlay 与二进制解析

T00 开始时 `/home/dev/ws_uwb_ie` 只有 `src/`，没有自己的 `build/`、`devel/` 或 catkin 配置。构建尝试在该目录初始化了隔离的 catkin workspace；它现在明确使用：

- source：`/home/dev/ws_uwb_ie/src`
- build：`/home/dev/ws_uwb_ie/build`
- devel：`/home/dev/ws_uwb_ie/devel`
- logs：`/home/dev/ws_uwb_ie/logs`
- underlay：仅 `/opt/ros/noetic`
- 当前包的 `CMAKE_HOME_DIRECTORY`：`/home/dev/ws_uwb_ie/src/uwb-imu-fusion`

以上由实际 `catkin build` 输出、`catkin config` 和当前 CMake cache 共同确认；不是从旧 workspace 推断。[`11_build_current_retry.log`](evidence/t00_20260905T102745Z/11_build_current_retry.log)、[`20_build_resolution_and_test_status.log`](evidence/t00_20260905T102745Z/20_build_resolution_and_test_status.log)

登录 shell 原有 overlay 是 `/home/dev/ws_rl/devel:/home/dev/ws_LIU/devel:/opt/ros/noetic`，不含新旧 UWB workspace。在该环境以及仅 source `/opt/ros/noetic` 时，`rospack find uwb_imu_fgo`、`uwb_driver`、`isas_msgs` 均 exit 1。source `/home/dev/ws_uwb/devel/setup.bash` 后只有 `uwb_imu_fgo` 解析到旧源码；`uwb_driver` 和 `isas_msgs` 仍为 exit 1。[`05_ros_overlay_resolution.log`](evidence/t00_20260905T102745Z/05_ros_overlay_resolution.log)

相邻 `/home/dev/ws_uwb` 的 cache 明确绑定旧源码 `/home/dev/ws_uwb/src/uwb-imu-fusion`，并有预存二进制：

`/home/dev/ws_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_node`

其 SHA-256 为 `96e7415d1f5391f0cf43fdd78d08728166a90add94167b78d2110d62b36b0ad8`，mtime 为 `2026-06-11 08:11:08 UTC`。二进制没有嵌入可验证的 Git revision，故不能声称它对应旧 checkout 当前的 `cfe6d29`，更不能当作当前提交构建成功。[`03_old_build_resolution.log`](evidence/t00_20260905T102745Z/03_old_build_resolution.log)、[`16_baseline_wrapper.log`](evidence/t00_20260905T102745Z/16_baseline_wrapper.log)

## 3. 环境与依赖

| 组件 | 分类 | 本次核实结果 |
|---|---|---|
| ROS | 当前环境观察 | Noetic；`ros-noetic-ros-base 1.5.0-1focal.20250521.010531` |
| catkin | 当前环境观察 | catkin `0.8.12`；catkin_tools `0.9.4`，使用 Python `3.8.10` |
| CMake / 编译器 | 当前环境观察 | CMake `3.16.3`；GNU C++ `9.4.0`；binutils `2.34` |
| Eigen / Boost | 当前环境观察 | Eigen `3.3.7-2`；Boost `1.71.0.0ubuntu2` |
| yaml-cpp / Armadillo / GTest | 当前环境观察 | `0.6.2-4ubuntu1` / `9.800.4` / `1.10.0` |
| GTSAM 工程要求 | 源码声明 | [`CMakeLists.txt:72`](../../CMakeLists.txt#L72) 要求 `find_package(GTSAM 4.2 REQUIRED)` |
| GTSAM 本机配置与库 | 当前环境观察 | `/usr/local/lib/cmake/GTSAM/GTSAMConfigVersion.cmake`、headers 和 `libgtsam.so.4.0.3` 均为 `4.0.3`；本地 `/home/dev/3rdparty/gtsam` 也是 `4.0.3-dirty` |
| `uwb_driver` | 当前环境观察 | 本机 `/home/dev`、`/opt`、`/usr/local` 未找到 package.xml 或 CMake package config；当前 cache 为 `uwb_driver_DIR-NOTFOUND` |
| `isas_msgs` | 当前环境观察 | 源码存在于 `/home/dev/ws_uwb/src/SFUISE/isas_msgs`，但它不是当前 workspace 或 underlay 中可解析的 package；`rospack` exit 1 |

版本和搜索证据见 [`24_dependency_version_summary.log`](evidence/t00_20260905T102745Z/24_dependency_version_summary.log)、[`07_dependency_versions.log`](evidence/t00_20260905T102745Z/07_dependency_versions.log)、[`08_dependency_search.log`](evidence/t00_20260905T102745Z/08_dependency_search.log)。独立最小 CMake 探针执行 `cmake -S .../gtsam_probe -B .../gtsam_probe/build`，exit 1，并明确只找到不满足 `4.2` 请求的 `/usr/local` GTSAM `4.0.3`。[`19_gtsam_cmake_probe.log`](evidence/t00_20260905T102745Z/19_gtsam_cmake_probe.log)

## 4. 当前提交构建与测试

工作目录 `/home/dev/ws_uwb_ie`；关键环境被收敛为 `/opt/ros/noetic`，避免继承无关 overlay。实际命令：

```bash
source /opt/ros/noetic/setup.bash
time catkin build uwb_imu_fgo --no-status
```

结果：**exit 1**，wall `2.159 s`。catkin prebuild 成功，`uwb_imu_fgo` 在 CMake 的 [`CMakeLists.txt:10`](../../CMakeLists.txt#L10) 处查找组件时因缺少 `uwb_driverConfig.cmake` 失败，尚未走到 GTSAM 检查。完整包装日志为 [`11_build_current_retry.log`](evidence/t00_20260905T102745Z/11_build_current_retry.log)，catkin 原生日志为 `/home/dev/ws_uwb_ie/logs/uwb_imu_fgo/build.cmake.000.log`。

首次包装命令因本机没有 `/usr/bin/time` 而 exit 127，catkin 当次未启动；该日志仅保留命令准备失败，不能计为构建失败或构建通过。[`10_build_current.log`](evidence/t00_20260905T102745Z/10_build_current.log)

[`CMakeLists.txt:175`](../../CMakeLists.txt#L175) 至 [`CMakeLists.txt:194`](../../CMakeLists.txt#L194) 静态声明七个测试：`test_config`、`test_outlier`、`test_trilaterate`、`test_imu_preint`、`test_uwb_factor`、`test_graph_builder`、`test_optimizer`。由于当前包配置失败，测试 targets 未生成：

| 项目 | 状态 | 原因 |
|---|---|---|
| `catkin test uwb_imu_fgo` | `NOT_RUN` | 当前提交 CMake 前置失败；按 T00 规则停止后续执行 |
| 测试结果/失败测试复查 | `NOT_RUN` | 没有测试进程或测试产物 |

证据：[`20_build_resolution_and_test_status.log`](evidence/t00_20260905T102745Z/20_build_resolution_and_test_status.log)。

## 5. 本机数据清点

### 5.1 自有 UGV / Vicon

下列三个非零 bag 均由 `rosbag info --yaml` 实际读取成功；传感器包含 Livox IMU、Livox LiDAR、NLink UWB 和五个 Vicon rigid-body pose/accel/twist topic。`/vrpn_client_node/tas_uwb_0/pose` 是当前配置选择的移动 tag GT。健康检查见 [`15_bag_health.log`](evidence/t00_20260905T102745Z/15_bag_health.log) 和 [`21_additional_data_health.log`](evidence/t00_20260905T102745Z/21_additional_data_health.log)。

| 记录 | 时长 / SHA-256 | GT、anchor、杆臂、时钟 | 独立性 / 权限 / 证据角色 |
|---|---|---|---|
| `/home/dev/Dataset/vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle.bag` | `73.549494 s`; `4b973dd5a6faa8a4b523017feaccb59f4f085e50592a114a963dbd9a816de8be` | GT topic 为 `tas_uwb_0` 的 `PoseStamped`；本地重标定 JSON 定义 moving tag origin 为 `imu_origin`，但报告同时说明物理 Vicon rigid-body origins 是 UWB antenna centers，刚体到 IMU/天线关系仍需硬件记录确认。当前 `vicon_test.yaml` 用硬编码 anchor、零杆臂、零时差；这些值的独立来源 `UNKNOWN` | 该 bag 被现有 anchor/time/range-bias 重标定产物直接使用，所以不能同时充当独立 LOS beta 标定和评测。发表权限 `UNKNOWN`。可作自有 LOS 候选，正式证据角色待合同和独立标定确认 |
| `/home/dev/Dataset/vicon/2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle.bag` | `67.783005 s`; `fc3ccb3079184d9f2aea550ea69d9adce3bca19f585e44bff4b5687c2acefe61` | 传感器/GT 点定义同上；现有重标定报告给出此记录时差 `0.0061 s`，当前配置仍为 `0.0` | 同样被重标定产物直接使用，非独立标定。发表权限 `UNKNOWN`。可作自有 NLOS 候选，正式证据角色待确认 |
| `/home/dev/Dataset/vicon/2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle.bag` | `123.314095 s`; `8e901962a4b381fbc83adc884ac2b1ac39359a61cc39a8776bc21bd1510bb5dc` | 传感器/GT topic 同上；该记录的 anchor/杆臂/时钟来源 `UNKNOWN` | 标定独立性与发表权限 `UNKNOWN`。可作额外自有 NLOS 候选。注意 `/home/dev/Dataset/vicon/NLOS/48/` 内同名副本为 **0 bytes**，不能使用 |

当前配置 [`config/vicon_test.yaml`](../../config/vicon_test.yaml) 的 SHA-256 为 `f53c802819838ff70f25620f5dc9b631ff310d9b828b3c12d43fa1ed816d7499`；其硬编码 anchors 与较新的 `/home/dev/experiments/vicon_anchor_calibration/1a810aa/` 结果不同。该重标定报告 SHA-256 为 `6b93c6a902375a0b68921c743326223a688fed5cd5c3ba01044b86194c5cb34c`，并明确用上述 15:31 与 15:46 bag，因此只能说明已有产物及其口径，不能证明独立 LOS beta。[`25_dataset_semantics.log`](evidence/t00_20260905T102745Z/25_dataset_semantics.log)

### 5.2 独立 LOS 标定

`/home/dev/Dataset/calib/2025-10-31-16-05-17.bag` 实际可读，时长 `119.072078 s`，SHA-256 `4df6f86ddb15e3ade1e476bbdfd297070bd756eef5a0fb503ed4e189fbd964fa`；含 Livox IMU/LiDAR、NLink UWB 和 GNSS topic，但没有 Vicon/独立距离真值。没有找到能把它认定为 LOS、给出独立 anchor/杆臂/时钟及 bias truth 的本地 manifest，发表权限也是 `UNKNOWN`。所以它只是**标定候选包**，不能承担独立固定 `beta` 的证据角色。T00 结论：本机独立 LOS 静态偏置标定集 **`UNKNOWN` / 未建立**。

### 5.3 仿真

仓库配置引用的 `sim_circle_*.bag` 在本机未找到。源码 [`simulator/src/uwb_twr_sim.cpp:29`](../../simulator/src/uwb_twr_sim.cpp#L29) 至 [`simulator/src/uwb_twr_sim.cpp:197`](../../simulator/src/uwb_twr_sim.cpp#L197) 声明 anchor、Gaussian range noise、clock/NLOS 注入和 ROS 当前时间戳；seed 为 `0` 时使用 `random_device`。配置存在不代表数据可用，本机没有实际仿真 bag、truth sidecar 或已锁定 seed。时长、实际传感器流、GT/bias truth、发表权限和可承担证据角色均为 `UNKNOWN`；仿真运行 **`NOT_RUN`**。

### 5.4 外部数据

| 记录 | 时长 / SHA-256 / 传感器 | GT、anchor、杆臂、时钟 | 独立性 / 权限 / 证据角色 |
|---|---|---|---|
| `/home/dev/ws_uwb/src/SFUISE/dataset/ISAS-Walk1.bag` | `62.701007 s`; `ca577e3666ad5876aa8347289a98793b8da0e68b5db459f2d7d69a6adcacd944`; RTLS UWB、Waveshare IMU、Vive GT、anchor list | GT 是 `/vive/transform/tracker_1_ref` 的 tracker transform，loader 不做刚体点变换；anchor 取同一 bag 前 20 条 `/anchor_list` 平均；配置杆臂 `[0.1,-0.025,0]` 且在线标定；时差 `0`。这些来源对本项目是否独立均为 `UNKNOWN` | SFUISE README 称其为 own dataset，仓库只明确 source code 为 GPLv3，数据发表条款 `UNKNOWN`。可作 legacy/portability 候选，不可作 bias truth 或自有主结果 |
| `ISAS-Walk2.bag` / `ISAS-Walk3.bag`（同目录） | `78.532506 s` / `84.154618 s`; SHA-256 `dea07fd11d5776226be6b390574c4b3789c48cfdce6a1731996981f3f91c453c` / `45df7ffe69b9d8c8be7784414ffcdc6e0f4674d0d7b8db3541e567e757bae0a0`; topic 结构同 Walk1 | 定义与来源同 Walk1 | 独立性、数据发表权限同为 `UNKNOWN`；仅 portability 候选 |
| `/home/dev/ws_uwb/src/awesome-uwb-localization/bag/data_example.bag` | `45.124783 s`; `25900cb237e9c01f8d1a82e8098add940aa9a8ec8cba3a4d21bdba02938abbc9`; IMU、`uwb_driver/UwbRange`、自定义 Vicon topic | GT 刚体/点、anchor/杆臂/时钟来源 `UNKNOWN`；当前 loader 的 Vicon 类型也不匹配 | 权限 `UNKNOWN`；缺 `uwb_driver`，当前不可执行，只是发现项 |
| `/home/dev/Dataset/ntu_day01/ntu_day_01_mid70.bag` + `ntu_day_01_ltpb.bag` | 约 `602.3 s`；前者只有 LiDAR，后者只有双 tag UWB | 本次未找到当前 MCD 配置要求的 IMU/GT 组合；其余均 `UNKNOWN` | 权限与证据角色 `UNKNOWN`；当前 loader 输入不完整，不能宣称 MCD 可用 |

当前配置引用的 MILUV CSV、VIUNet、MCD `gt_pose_inW.csv`、`tnp_03_fusion.bag` 均未在本机找到。文件名搜索与健康检查见 [`12_config_data_paths.log`](evidence/t00_20260905T102745Z/12_config_data_paths.log)、[`13_local_data_search.log`](evidence/t00_20260905T102745Z/13_local_data_search.log)、[`14_candidate_data_files.log`](evidence/t00_20260905T102745Z/14_candidate_data_files.log)、[`18_dataset_metadata.log`](evidence/t00_20260905T102745Z/18_dataset_metadata.log)。

## 6. 实际 loader 审计

自有 Vicon 路径的 loader 在 [`src/data_loader.cpp:27`](../../src/data_loader.cpp#L27) 至 [`src/data_loader.cpp:165`](../../src/data_loader.cpp#L165) 中：UWB frame 时间取 message header，tag ID 取 frame `id`，anchor ID/距离取 node `id`/`dis`，RSSI 原样保留；IMU 时间取 header，轴分量不重排，gyro 按 rad/s 使用，acc 是否乘重力由 `imu_acc_in_g` 控制。GT loader 在 [`src/data_loader.cpp:1041`](../../src/data_loader.cpp#L1041) 至 [`src/data_loader.cpp:1081`](../../src/data_loader.cpp#L1081) 中直接读取 `PoseStamped` 的 translation/quaternion 和 header stamp，不做杆臂或 rigid-body 点转换。

实际选择的 SFUISE loader 在 [`src/data_loader.cpp:797`](../../src/data_loader.cpp#L797) 至 [`src/data_loader.cpp:1038`](../../src/data_loader.cpp#L1038) 中：路径固定为 `data_dir/ISAS-WalkN.bag`；anchors 由前 20 条 `/anchor_list` 按 ID 平均；裁剪边界用 bag time，样本时间用 header stamp；IMU acceleration 保持 m/s²（配置 `imu_acc_in_g=false`），gyro 保持 rad/s，轴不重排；UWB tag ID 为 `RTLSStick.id`、anchor ID 为 range `id`、距离为 `range`，`ra==0` 被删除，模块自带的毫秒字段未使用；GT 直接读取 Vive `TransformStamped`。loader 源码片段已归档：[`23_loader_snippets.log`](evidence/t00_20260905T102745Z/23_loader_snippets.log)。由于二进制在载入阶段失败，这些单位和 ID 语义没有得到本次运行核实。

## 7. 旧基线复现尝试

为避免覆盖旧结果，选择最小且实际可读的 SFUISE Walk1，并在隔离目录 [`evidence/t00_20260905T102745Z/baseline_sfuise_walk1/`](evidence/t00_20260905T102745Z/baseline_sfuise_walk1/) 运行。有效配置只把原配置 `data_dir` 改为实际绝对路径；原配置 SHA-256 `0d61f4b85a9df1062d8e8bba988fbb559fa2767d4c1d21d6e3e3bb6d514c2df0`，有效副本 SHA-256 `968e621ad413c7c16ac2ccee3c703d74b42040507b57637bae7cdda725091125`。

工作目录和命令：

```text
PWD=.../evidence/t00_20260905T102745Z/baseline_sfuise_walk1
/home/dev/ws_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_node \
  _config_path:=.../baseline_sfuise_walk1/config/config_effective.yaml
```

隔离 ROS master 为 `http://127.0.0.1:11322`，`ROS_HOME` 也指向 run 目录。节点 **自然退出，exit 127**，stderr 为：

```text
libuwb_imu_fgo.so: undefined symbol: gtsam::NonlinearFactor::error(gtsam::HybridValues const&) const
```

本机搜到的 GTSAM provider 都是 4.0.3，均不提供该符号。[`17_gtsam_abi_search.log`](evidence/t00_20260905T102745Z/17_gtsam_abi_search.log) 预存二进制未进入数据加载、优化或可视化循环；无需人工 SIGINT。完整命令、环境、输入/配置/二进制哈希、退出码和 stderr 见 [`16_baseline_wrapper.log`](evidence/t00_20260905T102745Z/16_baseline_wrapper.log)。日志中的 run label 含 `old_cfe6d29`，但如第 2 节所述，这只是运行脚本记录的旧 checkout HEAD，**不能证明预存二进制的编译提交**。

结束检查没有发现遗留的节点或 roscore 进程，也没有找到 trajectory、groundtruth、residuals 或 covariance 结果文件。[`27_process_cleanup_and_outputs.log`](evidence/t00_20260905T102745Z/27_process_cleanup_and_outputs.log)

| 字段 | T00 结果 |
|---|---|
| 输入健康检查 | 实际通过；`rosbag info` exit 0 |
| 旧节点启动 | 失败；exit 127，GTSAM ABI 缺符号 |
| 源码版本 | 当前审计源码 `28d8e9f...`；旧 checkout `cfe6d29...`；预存二进制实际源码版本 `UNKNOWN` |
| 实际二进制 | 上述 `/home/dev/ws_uwb/devel/.private/.../uwb_imu_fgo_node`，SHA-256 已记录 |
| 轨迹 / GT 输出 / 残差 / 协方差 | `NOT_RUN`；没有生成 |
| ATE / P95 / RPE | `NOT_RUN`；没有估计轨迹，不编造数值 |
| 运行规模 | `NOT_RUN`；loader 未启动，不能用 bag topic count 冒充 estimator 规模 |
| estimator wall time / 分阶段耗时 | `NOT_RUN`；wrapper 的 `0.105 s` 只是动态载入失败耗时 |
| estimator 峰值内存 | `NOT_RUN`；wrapper 记录的 `8 KiB` 只是失败进程采样，不是 estimator 峰值 |
| 一次重跑差异 | `NOT_RUN`；首次运行未进入 estimator |

源码定义的 ATE 在 [`src/trajectory_io.cpp:159`](../../src/trajectory_io.cpp#L159) 至 [`src/trajectory_io.cpp:234`](../../src/trajectory_io.cpp#L234)：按时间 `lower_bound` 配对，然后以固定 scale `1` 做单个 SE(3) 刚体对齐，报告位置欧氏误差 RMSE。这是 aligned ATE，不是 raw/map-frame ATE；配对没有显式最大时间差，早于首个 GT 的估计点会匹配首 GT，晚于末 GT 的点跳过。由于本次无轨迹，GT 虽存在于 bag，ATE 仍为 `NOT_RUN`。

## 8. 重点代码审计

下列均为**源码声明**；当前提交未构建，旧二进制未载入成功，所以运行核实全部为 `NOT_RUN`。相关片段集中归档于 [`22_code_audit_snippets.log`](evidence/t00_20260905T102745Z/22_code_audit_snippets.log)。

| 审计项 | 源码结论 | 路径与行号 |
|---|---|---|
| RSSI 候选前删除 | `PreFilter` 在 `rx_rssi - fp_rssi` 超阈值时直接 `continue`；offline 主流程在任何候选阶段前执行该过滤并只保留 retained ranges。`UwbRange/UwbFrame` 没有稳定 `obs_id` 或选择 mask。因此当前 legacy 路径会在候选前删除 suspected NLOS | [`src/outlier_filter.cpp:8`](../../src/outlier_filter.cpp#L8)-[`17`](../../src/outlier_filter.cpp#L17)；[`tools/run_offline.cpp:314`](../../tools/run_offline.cpp#L314)-[`339`](../../tools/run_offline.cpp#L339)；[`include/uifgo/types.h:25`](../../include/uifgo/types.h#L25)-[`35`](../../include/uifgo/types.h#L35) |
| `calib_bias=false` 的固定 `beta` | factor 预测只有在在线 `calib_range_bias` 为真时才加 `Z(m)`；为假时只用几何距离。配置没有独立固定 beta 值入口。因此 `false` 时 beta **漏用**，不是恰好使用一次；未见重复加入 | [`src/uwb_factor.cpp:33`](../../src/uwb_factor.cpp#L33)-[`48`](../../src/uwb_factor.cpp#L48)；[`src/graph_builder.cpp:61`](../../src/graph_builder.cpp#L61)-[`68`](../../src/graph_builder.cpp#L68)、[`116`](../../src/graph_builder.cpp#L116)-[`120`](../../src/graph_builder.cpp#L120)；[`src/config.cpp:62`](../../src/config.cpp#L62)-[`71`](../../src/config.cpp#L71) |
| key / 状态语义 | `X(k)` Pose3、`V(k)` velocity、`B(k)` IMU `ConstantBias`；`A(m)` anchor correction、`L(0)` lever、`Z(m)` 每 anchor 标量静态 range bias。仓库内没有动态分段 bias `c_s` 的 key、状态或 measurement→segment 映射；当前不存在碰撞实例，但计划语义尚未实现 | [`src/graph_builder.cpp:16`](../../src/graph_builder.cpp#L16)-[`28`](../../src/graph_builder.cpp#L28)、[`87`](../../src/graph_builder.cpp#L87)-[`120`](../../src/graph_builder.cpp#L120)、[`142`](../../src/graph_builder.cpp#L142)-[`150`](../../src/graph_builder.cpp#L150) |
| `residuals.csv` | 数值并非固定占位，但它按 keyframe/anchor 计算未白化的 RMSE，只使用 pose translation、配置中的 nominal anchor 和原始 distance；忽略 lever、anchor correction、静态 `Z(m)`、factor sigma/robust weight 及 rejection mask。它不是最终 graph 上逐因子的 post-fit residual | [`src/logger.cpp:133`](../../src/logger.cpp#L133)-[`165`](../../src/logger.cpp#L165)；stdout summary 同样见 [`tools/run_offline.cpp:664`](../../tools/run_offline.cpp#L664)-[`687`](../../tools/run_offline.cpp#L687) |
| `covariance_diag.csv` | 明确固定写 `-1.0,-1.0,-1.0`，是占位。可视化 covariance 也使用固定 `0.01 I` fallback；没有把实际 `Marginals` 写入 CSV | [`src/logger.cpp:170`](../../src/logger.cpp#L170)-[`186`](../../src/logger.cpp#L186)；[`src/visualizer.cpp:323`](../../src/visualizer.cpp#L323)-[`350`](../../src/visualizer.cpp#L350) |
| 同一最终 graph / Values | multi-pass 先选 `best_values`，随后用 `base_cfg` 重建 `final_graph` 并再次 `Optimize`，却忽略第二次返回的 `OptimizerResult.values`。轨迹由原 `best_values` 写出；covariance 从 `final_opt` 的第二次结果取；CSV covariance 又传 `res1` 且写占位；visualizer 同时接收 `best_values`、重建的 `final_graph` 和 `res1`。四类输出不是同一最终结果对象 | [`tools/run_offline.cpp:363`](../../tools/run_offline.cpp#L363)-[`465`](../../tools/run_offline.cpp#L465)、[`469`](../../tools/run_offline.cpp#L469)-[`539`](../../tools/run_offline.cpp#L539)、[`876`](../../tools/run_offline.cpp#L876)-[`877`](../../tools/run_offline.cpp#L877) |
| 被拒因子 / range 重复 | optimizer 的 clean graph 会删除 GNC/chi-square rejected range，单次 `GraphBuilder::Build` 对每条输入 range 只添加一次。风险在于主流程重建并第二次优化，可形成另一套 rejection；输出仍用前一 pass 的 `best_inliers`，残差和可视化遍历全部 filtered ranges，没有传播 rejected mask。没有发现同一 `Build` 内重复计入，但最终报告集合不一致 | [`src/optimizer.cpp:20`](../../src/optimizer.cpp#L20)-[`29`](../../src/optimizer.cpp#L29)、[`130`](../../src/optimizer.cpp#L130)-[`171`](../../src/optimizer.cpp#L171)；[`src/graph_builder.cpp:61`](../../src/graph_builder.cpp#L61)-[`68`](../../src/graph_builder.cpp#L68)；[`tools/run_offline.cpp:450`](../../tools/run_offline.cpp#L450)-[`465`](../../tools/run_offline.cpp#L465)、[`657`](../../tools/run_offline.cpp#L657)-[`680`](../../tools/run_offline.cpp#L680) |
| 输出覆盖 | 主输出目录由配置目录固定推导为 `../data`，并覆写固定文件名 `trajectory.txt`、`calibration.txt`、`groundtruth.txt`。logger 目录只精确到秒，同一 bag 同秒可能碰撞，流会 truncate；`logs/latest` 每次被 unlink/relink。legacy 自身没有可靠 run isolation | [`tools/run_offline.cpp:138`](../../tools/run_offline.cpp#L138)-[`142`](../../tools/run_offline.cpp#L142)、[`545`](../../tools/run_offline.cpp#L545)-[`596`](../../tools/run_offline.cpp#L596)；[`src/logger.cpp:23`](../../src/logger.cpp#L23)-[`45`](../../src/logger.cpp#L45)、[`271`](../../src/logger.cpp#L271)-[`295`](../../src/logger.cpp#L295) |
| 持续可视化 | 成功路径发布后进入 `while (ros::ok())`，需要 Ctrl-C/SIGINT 结束。本次 ABI 失败发生在进入 main 逻辑前，故为自然退出 | [`tools/run_offline.cpp:873`](../../tools/run_offline.cpp#L873)-[`885`](../../tools/run_offline.cpp#L885) |

另有 covariance 语义风险：stdout 将 Pose3 covariance 的 `P(0..2)` 标为 `sigma_xyz`（[`tools/run_offline.cpp:469`](../../tools/run_offline.cpp#L469)-[`499`](../../tools/run_offline.cpp#L499)），但 GTSAM Pose3 tangent 分量通常先旋转后平移。当前 GTSAM 与程序均未成功运行，故只登记为待运行核实的源码风险，不给出实测结论。

## 9. T00 判定、阻塞与下一步

| T00 完成标准 | 判定 |
|---|---|
| 当前提交实际 build/test 命令、退出码、日志 | build 已执行并失败；test `NOT_RUN` |
| 至少一条真实记录产出轨迹与明确定义评估 | **未满足**；节点 ABI 载入失败，trajectory/ATE `NOT_RUN` |
| baseline 输入、配置、源码/二进制、耗时、内存可追溯 | 输入/配置/二进制和失败耗时可追溯；二进制源码 revision、estimator 耗时/内存仍 `UNKNOWN`/`NOT_RUN` |
| 四类数据清点 | 已完成本机清点；独立 LOS、仿真记录和多数发表权限明确保留为 `UNKNOWN` |
| 重点代码审计 | 静态审计完成；运行核实 `NOT_RUN` |

因此 T00 保持 **`BLOCKED`**。解除阻塞所需的最小外部修复是：提供/恢复当前 workspace 可解析的 `uwb_driver` 和 `isas_msgs`，以及与工程 `>=4.2` 要求和预存二进制 ABI 相容的 GTSAM；随后在不串用旧构建的前提下重建当前提交、运行七项测试，并重做隔离 baseline。依赖安装、升级、workspace 迁移和源码修复均超出本次授权，本次未实施。

剩余证据风险：没有独立 LOS beta 标定集；自有/外部数据发表权限未知；自有 GT 刚体到 IMU/UWB 测量点定义仍有本地材料间口径差异；legacy residual/covariance 和最终对象不一致；固定 beta 漏用；旧输出存在覆盖风险。`pdflatex` 可用性仅登记为 **T01 待检查项**，本次为 `NOT_RUN`，没有安装或探测。
