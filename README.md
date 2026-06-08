# uwb-imu-fusion — UWB-IMU Tightly-Coupled FGO SLAM

基于 GTSAM 的批量因子图优化（batch FGO）UWB-IMU 紧耦合定位系统。追求离线后处理的最高精度，非实时在线 SLAM。

## 1. 算法概述

| 组件 | 方案 |
|------|------|
| 优化框架 | GTSAM 4.x **batch Levenberg-Marquardt**（非 iSAM2） |
| 状态变量 | 15D keyframe: `[Pose3, Vector3(vel), ConstantBias]` |
| IMU 因子 | `CombinedImuFactor` — 中点预积分 + 偏置一阶修正 + 协方差传播 |
| UWB 因子 | 自定义 `ExpressionFactor<double>`: $r = \|A - (R \cdot t_{UI} + p)\| + \beta - z$ |
| 鲁棒策略 | IRLS 迭代加权 + $\chi^2$ 硬剔除回环 |
| 在线标定 | 杆臂 $t_{UI}$、锚点偏移 $\delta A_m$、每锚点测距偏置 $\beta_m$（多趟渐进打开） |
| 噪声模型 | 时间自适应 $\sigma^2 = \sigma_r^2 + (v_{\max} \cdot \Delta t / 3)^2$ |
| NLOS 过滤 | RSSI 差值法: `rx_rssi - fp_rssi > 6 dB` → 丢弃 |

## 2. 项目结构

```
src/uwb-imu-fusion/
├── CMakeLists.txt                     # catkin + GTSAM + yaml-cpp + GTest
├── package.xml                        # ROS Noetic
├── README.md
├── msg/                               # UWB 消息定义（兼容 LinkTrack 协议）
│   ├── LinktrackNode2.msg
│   ├── LinktrackNodeframe3.msg
│   └── LinktrackTagframe0.msg
├── config/
│   └── vicon_test.yaml               # VICON 数据集测试配置
├── launch/
│   └── offline.launch                 # roslaunch 入口
├── include/uifgo/                     # 公开头文件
│   ├── types.h                        # ImuSample, UwbRange, UwbFrame, NavState
│   ├── config.h                       # Config 结构体 + ConfigLoader
│   ├── data_loader.h                  # RosbagDataLoader (IMU + UWB + VICON GT)
│   ├── outlier_filter.h               # NLOS / 量程 / 距离一致性过滤
│   ├── initializer.h                  # 静止检测 + 三角定位 + 重力对齐
│   ├── imu_preint.h                   # ImuPreintegrator + IMU 同步插值
│   ├── uwb_factor.h                   # MakeUwbFactor (Expression 自动微分)
│   ├── graph_builder.h                # 关键帧化 + 因子图组装
│   ├── optimizer.h                    # LM + χ² 剔除 + Marginals
│   └── trajectory_io.h               # TUM 格式输出 + ATE/RPE 评估
├── src/                               # 11 个源文件
│   ├── config.cpp, data_loader.cpp, outlier_filter.cpp
│   ├── initializer.cpp, imu_preint.cpp, imu_sync.cpp
│   ├── uwb_factor.cpp, graph_builder.cpp, optimizer.cpp
│   ├── chi2.cpp, trajectory_io.cpp
├── tools/
│   └── run_offline.cpp                # 离线批处理主入口
├── test/                              # GoogleTest 单元测试 (36 tests)
└── doc/                               # 调研文档 + 设计文档
    ├── 00-overview.md                 # 三算法对比总览
    ├── 01-awesome-uwb-localization.md
    ├── 02-SFUISE.md
    ├── 03-uwb-imu-positioning.md
    └── dev.tex                        # LaTeX 设计文档
```

## 3. 依赖

| 依赖 | 版本 | 用途 |
|------|------|------|
| ROS | Noetic | catkin 构建 + rosbag 读取 |
| GTSAM | ≥ 4.0 | 因子图优化 (LM, Expression, CombinedImuFactor) |
| Eigen3 | ≥ 3.3 | 线性代数 |
| yaml-cpp | ≥ 0.6 | 配置文件解析 |
| Boost | ≥ 1.65 | filesystem 等 |
| GoogleTest | ≥ 1.11 | 单元测试 |

## 4. 编译

```bash
cd /home/mint/dev/ws_uwb
catkin build uwb_imu_fgo
source devel/setup.bash
```

## 5. 运行单元测试

```bash
catkin test uwb_imu_fgo
# 预期: 36 tests, 0 errors, 0 failures
```

## 6. 配置文件说明

完整配置示例见 `config/vicon_test.yaml`。关键字段：

### 6.1 锚点配置 (anchors)

```yaml
anchors:
  - {id: 1, pos: [x, y, z], prior_sigma: 0.10}
  - {id: 2, pos: [x, y, z], prior_sigma: 0.10}
  # ... 至少 4 个非共面锚点以保证 3D 可观
```

### 6.2 ROS Topic 配置

```yaml
topics:
  imu: "/livox/imu"                              # sensor_msgs/Imu
  uwb: "/nlink_linktrack_nodeframe3"              # LinktrackNodeframe3
  vicon: "/vrpn_client_node/tas_uwb_0/pose"      # geometry_msgs/PoseStamped (可选)
```

### 6.3 Rosbag 配置

```yaml
bag:
  path: "data/xxx.bag"   # 绝对路径 或 相对 config 文件的路径
  start: 0.0             # 起始偏移 (s)
  durr: -1.0             # 持续时间 (-1=全部)
```

### 6.4 关键帧控制

```yaml
keyframe:
  step: 10               # 每 N 个 UWB 帧取 1 个关键帧 (1=全部, 10=下采样 10 倍)
```

大数据量时增大 `step` 可显著加速（3669 帧 × step=10 → 367 关键帧，<1min）。

### 6.5 标定开关

```yaml
extrinsics:
  lever_arm_init: [0.0, 0.0, 0.0]
  calib_lever: false                    # true = 在线标定 UWB-IMU 杆臂

calibration:
  calib_anchor: true                    # true = 在线标定锚点位置偏移
  calib_range_bias: true                # true = 在线标定每锚点距离偏置
  calib_td: false                       # true = 在线标定 IMU-UWB 时间偏移
```

- 全部关闭 (`false`) → 仅做轨迹优化（最快、最稳定）
- 逐项打开 → 自动多趟处理（每趟开一项，卡方监控防退化）

### 6.6 噪声参数

```yaml
imu:
  sigma_a: 0.1        # 加速度计噪声 (m/s²)
  sigma_g: 0.01       # 陀螺仪噪声 (rad/s)
  sigma_wa: 0.001     # 加速度计随机游走
  sigma_wg: 2.0e-5    # 陀螺仪随机游走
  gravity: 9.81

uwb:
  sigma_range: 0.15   # 基础测距标准差 (m)
  v_max: 3.0           # 载体最大速度 (用于自适应协方差)
  nlos_rssi_diff: 6.0  # NLOS 检测阈值 (dB)
  min_range: 0.3       # 最小有效距离
  max_range: 100.0     # 最大有效距离
```

### 6.7 求解器

```yaml
solver:
  lm_max_iter: 200
  rel_error_tol: 1.0e-6
  abs_error_tol: 1.0e-8
  cauchy_k: 1.0
  chi2_reject_prob: 0.99      # χ² 剔除置信度
  max_rejection_rounds: 3     # 剔除回环最大轮次
```

## 7. 运行方式

Rosbag 通过**直接读取文件**的方式导入（`rosbag::Bag::open()` + `rosbag::View` 迭代，非 `rosbag play` 回放），无需单独终端运行 `roscore` + `rosbag play`。

### 一键启动（使用默认 config/slam.yaml）

```bash
cd /home/mint/dev/ws_uwb
catkin build uwb_imu_fgo && source devel/setup.bash
roslaunch uwb_imu_fgo offline.launch
```

### 使用自定义配置

```bash
roslaunch uwb_imu_fgo offline.launch config_path:=/path/to/your_config.yaml
```

## 8. 输出文件

运行后在**配置文件所在目录**生成：

| 文件 | 内容 |
|------|------|
| `trajectory.txt` | TUM 格式轨迹: `t x y z qx qy qz qw` |
| `calibration.txt` | 标定结果: 杆臂、锚点位置修正、距离偏置 |

控制台输出包含：
- 每个 Pass 的因子数、优化误差、卡方值、内点比例
- 最终 Summary: 总帧数、UWB 内点比例、每锚点 RMSE、运行耗时
- **VICON GT 对比**（如配置了 vicon topic）: ATE RMSE / P50 / P95 / P99 / Max

## 9. VICON 数据集快速测试

`config/slam.yaml` 已预配好 VICON 数据集全部参数，一键启动：

```bash
cd /home/mint/dev/ws_uwb
catkin build uwb_imu_fgo && source devel/setup.bash
roslaunch uwb_imu_fgo offline.launch
```

默认配置: **kf_step=10** (367 关键帧, <1min)、**标定全关** (基线精度)。
调参: 改 `config/slam.yaml` 中的 `keyframe.step`（5 更精，1 最精）或开启 `calibration.*`。

## 10. 开发指南

### 10.1 里程碑

| M | 内容 | 测试 |
|---|------|------|
| M0 | 脚手架 | cmake 通过 |
| M1 | 数据读写 + 过滤 | UT-1~UT-3 |
| M2 | 三角定位初始化 | UT-4 |
| M3 | IMU 预积分封装 | UT-5 |
| M4 | UWB Expression 因子 | UT-6 |
| M5 | 因子图构建 + LM | UT-7 |
| M6 | IRLS + χ² 剔除 | UT-8 |
| M7 | 在线自标定 | UT-9 |
| M8 | 诊断 + 轨迹输出 | IT-1~IT-4 |

### 10.2 添加新模块

1. 在 `include/uifgo/` 中声明接口
2. 在 `src/` 中实现
3. 在 `test/` 中写 GoogleTest（合成数据 + 已知真值）
4. 在 `CMakeLists.txt` 中添加源文件

### 10.3 添加新的 UWB 消息类型

当前支持 `LinktrackNodeframe3`。要添加新类型，在 `src/data_loader.cpp` 的 `LoadFromBag()` 中添加对应的 `instantiate<>()` 分支。

## 11. 设计参考

| 参考来源 | 借鉴技术 |
|----------|---------|
| awesome-uwb-localization | UWB 两步外点剔除、时间自适应协方差 |
| SFUISE | 在线外参/重力标定、四元数流形插值 |
| uwb-imu-positioning | IMU 预积分 + 偏置一阶修正、Ceres 流形参数化 |
| C-LIUO | UWB 消息格式 (LinktrackNodeframe3)、NLOS RSSI 检测 |

## 12. License

BSD
