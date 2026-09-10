# 仓库审计：T00 新环境工程、数据与旧基线

> 新环境复核时间：`2026-09-05T15:43:44Z` 起（UTC）
> 状态：**T00 `DONE`**。当前提交已在 `/home/mint` 工作空间重新构建；现有测试已实际执行并保留失败项；SFUISE Walk1 旧基线在两个隔离目录中完成，生成轨迹和 aligned ATE。全部新证据位于 [`evidence/t00_newenv_20260905T154344Z/`](evidence/t00_newenv_20260905T154344Z/)。

本报告严格区分“源码声明”“当前环境观察”和“实际运行通过”。测试失败、未知的数据来源和未执行的数据适配均保持可见。上一台 `/home/dev` 机器的 T00 失败证据仍保留在 [`evidence/t00_20260905T102745Z/`](evidence/t00_20260905T102745Z/)，但不再代表当前环境。

## 1. 仓库快照与冻结材料

| 项目 | 分类 | 当前结果 | 证据 |
|---|---|---|---|
| 仓库 | 当前环境观察 | `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie` | [`00_preflight.log`](evidence/t00_newenv_20260905T154344Z/00_preflight.log) |
| 分支 / T00 起始提交 | 当前环境观察 | `feature/uwb-imu-fusion-ie-postprocessing` / `aa6f76a285ca42ae00825b8d8ec3969060f61dd6` | 同上 |
| T00 开始时工作区 | 当前环境观察 | clean；执行后只增加本次证据并修改本报告与 `STATUS.md` | 起始 shell 快照及最终核验 |
| 冻结论文结构 | 当前环境观察 | SHA-256 `8ac373919823d755d9c1b4ceb807527432d16b69d368042e19aa56d93dc67144` | 同上 |
| 冻结 roadmap | 当前环境观察 | SHA-256 `b9bb63b65ebdb8a505bf99b26181318c6c64da1a72817b08cfa61315e2333bdb` | 同上 |
| 源码相对 roadmap 参考提交 | 当前环境观察 | `cfe6d29..aa6f76a` 只涉及 `doc/` 与 `AGENTS.md`；方法源码未改变 | [`07_code_audit.log`](evidence/t00_newenv_20260905T154344Z/07_code_audit.log) |

本次没有修改 [`../v2/paper_structure.tex`](../v2/paper_structure.tex) 或 [`../v2/v2_roadmap.md`](../v2/v2_roadmap.md)。

## 2. 当前工作空间与依赖

| 组件 | 分类 | 当前结果 |
|---|---|---|
| 系统 | 当前环境观察 | Ubuntu 20.04.6 LTS，WSL2，kernel `6.18.33.2-microsoft-standard-WSL2` |
| ROS / catkin | 当前环境观察 | ROS Noetic；catkin_tools `0.9.2`，Python `3.8.10` |
| CMake / 编译器 | 当前环境观察 | CMake `3.16.3`；GCC/G++ `9.4.0` |
| GTSAM | 当前环境观察 | `/usr/local/lib/cmake/GTSAM` 报告 `4.2a5`，headers 为 `4.2.0`，运行库解析到 `/usr/local/lib/libgtsam.so.4.2.0`；另有未被本次构建选中的系统 GTSAM `4.0.3` |
| ROS 消息依赖 | 当前环境观察 | `uwb_driver` 与 `isas_msgs` 都在当前 catkin workspace 内，可被 catkin 解析和构建 |
| 当前包源码 | 当前环境观察 | `catkin locate -s uwb_imu_fgo` 指向本仓库；重建后的 `CMAKE_HOME_DIRECTORY` 也指向本仓库 |

环境、配置文件位置、二进制哈希和动态链接结果见 [`00_preflight.log`](evidence/t00_newenv_20260905T154344Z/00_preflight.log) 与 [`04_build_and_test_after_cleanup.log`](evidence/t00_newenv_20260905T154344Z/04_build_and_test_after_cleanup.log)。登录环境仍带有 `/home/mint/ws_calib` overlay；后续 runner 应记录完整 overlay，避免依赖解析随 shell 改变。

## 3. 当前提交构建与测试

第一次执行：

```bash
cd /home/mint/ws_fusion_uwb
catkin build uwb_imu_fgo --force-cmake --no-status --summarize
```

结果为 **exit 1**。原因是迁移前的可再生 build cache 仍绑定不存在的 `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion`，并非源码或依赖编译失败。日志见 [`02_build_and_test.log`](evidence/t00_newenv_20260905T154344Z/02_build_and_test.log)。

随后只清理该包的 build/devel 产物：

```bash
catkin clean --yes uwb_imu_fgo
```

清理 **exit 0**，目标仅为 `/home/mint/ws_fusion_uwb/build/uwb_imu_fgo` 和 `/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo`。证据见 [`03_targeted_cache_cleanup.log`](evidence/t00_newenv_20260905T154344Z/03_targeted_cache_cleanup.log)。再次执行相同 build 命令为 **exit 0**，wall `80.079 s`。当前节点 SHA-256 为 `7d7d58d4ed6d0fc7819c08d87fdf0146f3a5d54c760ebe45b39ce65d889db128`，共享库 SHA-256 为 `9789820d733bada7f6f410787efe57fd1cdfe6d271cc78937f46e234cea947c6`。

linked devel 阶段报告 `libso3_control_nodelet.so`、`libquadrotor_dynamics.so`、`libSO3Control.so` 三个同名仿真库已存在且哈希不同。主节点和本次基线没有依赖解析失败，但该 workspace 级名称碰撞是后续复现风险。

实际测试命令：

```bash
catkin test uwb_imu_fgo --no-status --summarize
catkin_test_results build/uwb_imu_fgo/test_results --verbose
```

两条命令均为 **exit 1**。汇总为 `38 tests, 0 errors, 2 failures, 0 skipped`；七个测试可执行文件全部启动，失败集中在 `ImuPreintegrator.StaticZeroVelocity`：期望零重力时 `z≈0`、`|v|≈0`，实际为 `z=-4.905 m`、`|v|=9.81 m/s`。测试传入零 `gravity_world`，而 [`src/imu_preint.cpp:8`](../../src/imu_preint.cpp#L8) 至 [`15`](../../src/imu_preint.cpp#L15) 忽略该参数并始终使用 `cfg.gravity`。这是已确认的 API/测试语义冲突，本次 T00 不修改生产源码；T01 必须登记，进入依赖 IMU 数值合同的实现前处理。

其余测试均运行通过。完整编译、gtest stdout、XML failure 和汇总见 [`04_build_and_test_after_cleanup.log`](evidence/t00_newenv_20260905T154344Z/04_build_and_test_after_cleanup.log)。

## 4. 当前机器数据清点与角色

下表各 bag 都由 `rosbag info --yaml` 实际读取成功，退出码为 0。文件哈希、topic 类型和消息数见 [`01_data_health.log`](evidence/t00_newenv_20260905T154344Z/01_data_health.log)，目录和配置引用见 [`06_data_inventory.log`](evidence/t00_newenv_20260905T154344Z/06_data_inventory.log)。“文件健康”不等于公平 benchmark 已建立。

| 数据 | 路径、时长与关键流 | GT / 标定来源 | T00 角色与限制 |
|---|---|---|---|
| 自有 UGV / Vicon LOS 候选 | `data/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle.bag`；`73.549494 s`；Livox IMU/LiDAR、NLink UWB、五个 Vicon rigid body | 当前配置选 `/vrpn_client_node/tas_uwb_0/pose`，硬编码四 anchors、零杆臂、零时差；本机没有配套来源 manifest | 文件可读，可作自有 LOS 候选；GT 点定义、标定独立性和发表权限 `UNKNOWN`，estimator `NOT_RUN` |
| 独立 LOS 固定 `beta` 标定 | 未找到具备独立距离参考、anchor/杆臂/时钟来源和权限说明的专用 manifest | 现有 no-obstacle 记录及旧 UWB/Vicon 文件不能仅凭文件名认定为独立标定 | **未建立**；T01 必须保留为未决项，不能编造固定 `beta` 数值 |
| 仿真 | `data/sim_circle_2026-06-15-16-04-35.bag`；`82.038332 s`；`/sim/imu`、UWB、`/sim/odom` | 配置给出八 anchors 和零杆臂/时差；没有本次生成 commit、锁定 seed 或逐观测 bias truth sidecar | 文件可读，可作 legacy/smoke 候选；正式 truth/provenance 未建立，estimator `NOT_RUN` |
| SFUISE Walk1/2/3 | `data/SFUISE/ISAS-Walk{1,2,3}.bag`；`62.701007 / 78.532506 / 84.154618 s`；RTLS UWB、Waveshare IMU、Vive GT、anchor list | anchors 取同一 bag 前 20 条 `/anchor_list` 平均；GT 是 tracker transform；配置杆臂 `[0.1,-0.025,0]` 并在线估计，时差 0 | Walk1 已跑通旧基线，适合作 legacy/portability；许可、GT 点和标定独立性 `UNKNOWN`，不可作 bias truth |
| MILUV random/circular | `data/MILUV/default_1_{random3,circular3D}_0`；bag 分别 `205.158262 / 135.248404 s`，adapter 所需 PX4 IMU、UWB range、mocap CSV 均存在 | 配置硬编码六 anchors 和双 tag levers；本次未核实一手标定与许可文本 | 文件健康、接口材料齐；adapter/trajectory `NOT_RUN`，暂作 external portability 候选 |
| NTU VIRAL `tnp_03` | `data/tnp_03/tnp_03_fusion.bag`；`240.333991 s`；IMU、P440 UWB、Leica pose | anchors 从 UWB responder location 自动提取，配置零杆臂/时差；仓库有传感器 YAML，本次未核实独立标定与许可 | 文件健康，adapter/trajectory `NOT_RUN`，暂作 external portability 候选 |
| awesome-uwb-localization 示例 | `/home/mint/ws_LIU/src/awesome-uwb-localization/bag/data_example.bag`；`45.124783 s` | 自定义 Vicon message 与当前普通 loader 不匹配，其余来源及许可 `UNKNOWN` | 仅发现项，estimator `NOT_RUN` |

当前机器只有一条与 2025 自有实验直接对应的 no-obstacle 记录；上一台机器记录的两条 obstacle bag 与专用 calibration bag 在本机未找到，不能继续作为当前机器可用输入。

## 5. 实际 loader 语义

源码证据归档于 [`08_loader_audit.log`](evidence/t00_newenv_20260905T154344Z/08_loader_audit.log)。

- original/Vicon 路径以 message header 为 IMU、UWB 和 GT 时间；IMU 轴不重排，gyro 按 rad/s，acc 是否乘 `gravity` 由 `imu_acc_in_g` 决定；GT 直接读取 `PoseStamped` translation/quaternion，不做 rigid body 到天线/IMU 点的变换。
- SFUISE loader 固定读取 `data_dir/ISAS-WalkN.bag`；anchors 取前 20 条 `/anchor_list` 按 ID 平均；裁剪边界使用 bag time，样本时间使用 header stamp；IMU acceleration 保持 m/s²，gyro 保持 rad/s；UWB tag ID 取 `RTLSStick.id`，anchor ID/距离取 range 的 `id/range`，`ra==0` 被删除；GT 直接读取 Vive `TransformStamped`。
- 本次 Walk1 实际加载 `4839 IMU samples`、`913 UWB frames`（来自 `970 RTLSStick messages`）、`5 anchors` 和 `3168 GT poses`，说明这些类型与当前编译环境兼容；这不能证明未运行 adapter 的语义正确。

## 6. 旧基线复现与重跑

选择本机最短且现有配置完整的 SFUISE Walk1。两次运行分别位于 [`baseline_sfuise_walk1/`](evidence/t00_newenv_20260905T154344Z/baseline_sfuise_walk1/) 和 [`baseline_sfuise_walk1_rerun/`](evidence/t00_newenv_20260905T154344Z/baseline_sfuise_walk1_rerun/)。每次均使用独立 `config/`、`data/`、`logs/`、`ROS_HOME` 和 ROS master。

`config_original.yaml` 与仓库 [`config/sfuise_walk1.yaml`](../../config/sfuise_walk1.yaml) 字节一致；effective config 只将 `sfuise.data_dir` 改为当前机器绝对路径并启用 debug log。旧入口在发布可视化后按其交互语义接收 SIGINT，节点均 **exit 0**，没有遗留 node 或 roscore。

```text
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_node \
  _config_path:=<隔离目录>/config/config_effective.yaml
```

| 字段 | 首次 | 重跑 |
|---|---:|---:|
| keyframes / factors / UWB inliers | `229 / 1145 / 913 of 913` | 相同 |
| estimator 日志 elapsed | `6.21217 s` | `6.52498 s` |
| wrapper wall / peak RSS | `6.953 s / 66,608 KiB` | `7.133 s / 68,680 KiB` |
| GT 配对 | `229 of 3168` | 相同 |
| aligned ATE RMSE | `0.169642 m` | `0.169642 m` |
| P95 / P99 / max | `0.289732 / 0.384659 / 0.41622 m` | 相同 |

轨迹、GT 和 calibration 三类输出在两次运行中逐字节一致；比较与哈希见 [`10_rerun_comparison.log`](evidence/t00_newenv_20260905T154344Z/10_rerun_comparison.log)。首次完整命令、输入/配置/二进制哈希、退出码、资源量和 stdout 见 [`05_baseline_wrapper.log`](evidence/t00_newenv_20260905T154344Z/05_baseline_wrapper.log)，重跑见 [`09_baseline_rerun_wrapper.log`](evidence/t00_newenv_20260905T154344Z/09_baseline_rerun_wrapper.log)。

这里的 ATE 是 [`src/trajectory_io.cpp:159`](../../src/trajectory_io.cpp#L159) 实现的 scale=1 SE(3) 对齐后位置 RMSE，不是冻结结构要求的独立坐标变换 raw-frame ATE，只作为 legacy baseline 记录。现有 `covariance_diag.csv` 仍全写 `-1.0`；文件存在不代表 covariance 已输出。

## 7. 重点代码审计

当前方法源码与上一台机器审计时一致。下列是源码声明；Walk1 只核实 legacy 路径能运行。完整片段见 [`07_code_audit.log`](evidence/t00_newenv_20260905T154344Z/07_code_audit.log)。

| 审计项 | 当前结论 |
|---|---|
| RSSI 前置删除 | [`src/outlier_filter.cpp:8`](../../src/outlier_filter.cpp#L8) 会在候选阶段前删除 RSSI 差超阈值的 range；观测结构无稳定 `obs_id` 和完整 mask。T02 必须为 paper path 保留 suspected NLOS |
| 固定静态 `beta` | [`src/uwb_factor.cpp:33`](../../src/uwb_factor.cpp#L33) 只在在线 `calib_range_bias` 时加入 `Z(m)`；关闭时没有固定非零 `beta` 入口，当前语义是漏用固定 `beta` |
| 状态 key | `X/V/B` 分别为 pose/velocity/IMU bias，`A/L/Z` 为 anchor/lever/static range bias；尚无动态 segment `c_s` 状态或 measurement-to-segment 映射 |
| residual 输出 | [`src/logger.cpp:133`](../../src/logger.cpp#L133) 的 CSV 是未白化几何 RMSE，忽略 lever、anchor correction、静态 bias、权重与 rejection mask，并非最终 graph 逐因子 post-fit residual |
| covariance 输出 | [`src/logger.cpp:170`](../../src/logger.cpp#L170) 固定写 `-1.0,-1.0,-1.0`，本次实际产物也如此 |
| final graph / Values | offline 入口重新构建并优化 covariance graph，却忽略第二次返回的 Values；轨迹、残差、covariance 和可视化并非统一来自同一最终 graph/Values |
| 输出隔离 | legacy 固定写配置目录上级的 `data/trajectory.txt` 等文件，并维护共享 `latest`；本次以每 run 独立配置目录规避，T09 仍需正式 runner |
| 持续可视化 | 成功路径进入 `while (ros::ok())`；两次基线均在检测到 “Visualization published” 后正常 SIGINT 结束 |

## 8. T00 判定与下一步开发边界

| T00 完成标准 | 当前判定 |
|---|---|
| 当前提交实际 build/test 命令、退出码、日志 | 满足；build exit 0，tests 已执行且 2 failures 完整保留 |
| 至少一条真实记录产出轨迹与明确定义评估 | 满足；SFUISE Walk1 两次生成轨迹、GT 与 aligned ATE |
| baseline 输入、配置、源码/二进制、耗时、内存可追溯 | 满足；两个隔离 run 及哈希齐全 |
| 四类数据清点 | 满足；自有 UGV、独立 LOS、仿真、外部数据逐项记录，未知项未补写 |
| 重点代码审计 | 满足；源码片段与实际 legacy 产物分开记录 |

因此当前机器的 T00 标为 **`DONE`**。测试仍有失败并不被隐藏；它是后续 correctness 工作项，而不是继续把 T00 标为未执行的理由。

按冻结 roadmap，下一任务是 T01：建立唯一方法/实验合同与论文工程。T01 必须登记以下未决项：独立 LOS 固定 `beta` 数据未建立、自有与外部数据发表权限未知、自有 GT 刚体点/杆臂/时钟来源未闭合、IMU preintegration 构造参数与测试语义冲突、linked-devel 同名仿真库碰撞。随后 T02 先补原始观测保留、稳定 ID/mask、固定 `beta` 与最终输出一致性；T03 做小矩阵数值参考；再按 T04/T05/T08 形成 C++/GTSAM oracle 短闭环。这里仅复述冻结 roadmap 的开发顺序，没有提前创建合同或实现 NLOS 方法。
