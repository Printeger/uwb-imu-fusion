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
│   ├── offline.launch                 # 离线批处理 roslaunch 入口
│   └── offline_with_viz.launch        # 离线批处理 + RViz 同步可视化
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
│   ├── trajectory_io.h               # TUM 格式输出 + ATE/RPE 评估
│   └── visualizer.h                  # RViz 可视化发布器 (9 类 marker)
├── src/                               # 12 个源文件
│   ├── config.cpp, data_loader.cpp, outlier_filter.cpp
│   ├── initializer.cpp, imu_preint.cpp, imu_sync.cpp
│   ├── uwb_factor.cpp, graph_builder.cpp, optimizer.cpp
│   ├── chi2.cpp, trajectory_io.cpp, visualizer.cpp
├── tools/
│   ├── run_offline.cpp                # 离线批处理主入口
│   └── analyze_log.py                 # 日志解析 + 可视化脚本
├── config/
│   ├── slam.yaml                      # 默认配置 (VICON 数据集)
│   └── uwb_fgo_viz.rviz              # RViz 预配置文件
├── logs/
│   ├── latest → <timestamp>_<bag>/    # 符号链接 → 最新日志
│   └── <timestamp>_<bag>/            # 每次运行的完整日志
│       ├── config.yaml                #   配置快照
│       ├── summary.json               #   结构化汇总
│       ├── trajectory.txt             #   融合轨迹 (TUM)
│       ├── groundtruth.txt            #   真值轨迹 (TUM)
│       ├── calibration.txt            #   标定结果
│       ├── optimization.csv           #   每趟优化指标
│       ├── state_trace.csv            #   每关键帧状态
│       ├── residuals.csv              #   每锚点残差
│       ├── covariance_diag.csv        #   协方差对角
│       ├── gt_comparison.csv          #   GT 对比
│       ├── *.png                      #   可视化图表
│       └── report.md                  #   Markdown 综合报告
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

### 6.8 Debug 日志

```yaml
debug:
  log: true                   # true = 启用详细日志 (logs/<timestamp>_<bag>/)
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

### 带 RViz 可视化（推荐）

```bash
roslaunch uwb_imu_fgo offline_with_viz.launch
```

处理完成后，RViz 自动加载全部可视化内容。保持终端运行，在 RViz 窗口中可旋转/缩放查看。详见 [12. RViz 可视化](#12-rviz-可视化)。

## 8. 输出文件

### 8.1 基础输出（始终生成）

运行后在 `data/` 目录生成：

| 文件 | 内容 |
|------|------|
| `data/trajectory.txt` | TUM 格式轨迹: `t x y z qx qy qz qw` |
| `data/groundtruth.txt` | TUM 格式真值轨迹（如有 VICON） |
| `data/calibration.txt` | 标定结果: 杆臂、锚点位置修正、距离偏置 |

### 8.2 Debug 日志（`debug.log: true` 时）

启用后，所有输出自动同步保存到 `logs/<timestamp>_<bag_name>/`：

| 文件 | 格式 | 内容 |
|------|------|------|
| `config.yaml` | YAML | 本次运行的配置快照 |
| `summary.json` | JSON | 结构化汇总（ATE/P95/内点比...） |
| `trajectory.txt` | TUM | 融合轨迹 |
| `groundtruth.txt` | TUM | 真值轨迹 |
| `calibration.txt` | 文本 | 标定结果 |
| `optimization.csv` | CSV | 每趟优化: 误差、χ²、内点数、耗时 |
| `state_trace.csv` | CSV | 每关键帧: t, x, y, z, qw, qx, qy, qz, vx, vy, vz |
| `residuals.csv` | CSV | 每帧每锚点: 残差 RMSE、测距数量 |
| `covariance_diag.csv` | CSV | 每关键帧: σ_x, σ_y, σ_z |
| `gt_comparison.csv` | CSV | ATE / P50 / P95 / P99 / Max 误差 |

### 8.3 日志分析（`tools/analyze_log.py`）

```bash
# 分析最新日志
python3 tools/analyze_log.py

# 分析指定日志
python3 tools/analyze_log.py logs/2026-06-08_15-30-45_no_obstacle/
```

脚本自动读取 log 目录中的所有 CSV/JSON 文件，生成：

| 输出 | 说明 |
|------|------|
| `trajectory_xy.png` | 俯视轨迹（标记起点/终点） |
| `trajectory_3d.png` | 3D 轨迹图 |
| `position_time.png` | X/Y/Z 位置随时间变化 |
| `velocity_time.png` | 速度随时间变化 |
| `convergence.png` | 每趟优化误差和 χ² 柱状图 |
| `residual_heatmap.png` | 每锚点残差热力图 |
| `covariance_diag.png` | 位置不确定度随时间变化 |
| `report.md` | **Markdown 综合报告**（含所有表格+图表） |

在 VS Code 中打开 `report.md` 后按 `Ctrl+Shift+V` 即可预览完整报告。

`logs/latest/` 始终指向最近一次运行，每次运行自动更新该符号链接。

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

## 12. RViz 可视化

### 13.1 概述

优化完成后，系统自动将所有结果发布为 ROS 可视化消息（latched topics），RViz 可同步渲染 9 类内容。

```
┌──────────────────────────────────────────────────────┐
│  offline_with_viz.launch                             │
│                                                      │
│  ┌──────────────────────┐   ┌─────────────────────┐  │
│  │ uwb_imu_fgo_node     │   │ rviz                │  │
│  │  (run_offline.cpp)   │   │  uwb_fgo_viz.rviz   │  │
│  │                      │──►│                     │  │
│  │  处理完成后发布 ──────│   │  订阅并渲染 9 类    │  │
│  │  visualization_msgs  │   │  Marker / Path      │  │
│  └──────────────────────┘   └─────────────────────┘  │
│                                                      │
│  TF: 独立线程 10Hz 广播 world→map (帧始终有效)       │
└──────────────────────────────────────────────────────┘
```

### 13.2 可视化内容 (9 类)

| # | RViz 显示名称 | 数据类型 | ROS Topic | 颜色/样式 |
|---|-------------|---------|-----------|-----------|
| 1 | Anchor Initial (Green) | `MarkerArray` | `/uwb_imu_fgo/anchor_initial` | 🟢 绿色球体 + 白色文字标签 |
| 2 | Anchor Optimized (Orange+Arrows) | `MarkerArray` | `/uwb_imu_fgo/anchor_optimized` | 🟠 橙色球体 + 绿→红位移箭头 |
| 3 | GT Trajectory (Blue) | `Path` | `/uwb_imu_fgo/gt_trajectory` | 🔵 蓝色实线 (VICON 真值) |
| 4 | Fused Trajectory (Red) | `Path` | `/uwb_imu_fgo/fused_trajectory` | 🔴 红色实线 (融合结果) |
| 5 | Keyframe Poses (RGB axes) | `MarkerArray` | `/uwb_imu_fgo/keyframe_poses` | RGB 坐标轴 (红=X,绿=Y,蓝=Z) |
| 6 | Covariance Ellipsoids (cyan) | `MarkerArray` | `/uwb_imu_fgo/covariance_ellipsoids` | 🔷 半透明青色 2σ 椭球 |
| 7 | UWB Range Edges (green→red) | `MarkerArray` | `/uwb_imu_fgo/uwb_edges` | 🟢→🔴 残差颜色编码连线 |
| 8 | Error Vectors (est → GT) | `MarkerArray` | `/uwb_imu_fgo/error_vectors` | 🟢→🔴 估计→真值误差箭头 |
| 9 | Metrics Text | `Marker` | `/uwb_imu_fgo/metrics_text` | ⬜ 屏幕文字 (ATE/P95/耗时) |

### 13.3 各显示项说明

**Anchor 初始位置 & 优化位置** — 锚点初始位置用绿色球体标出，如果开启了 `calib_anchor`，橙色球体显示优化后位置，并用箭头连接初始→优化位置（箭头颜色从绿到红编码位移大小）。

**真值轨迹 & 融合轨迹** — VICON 真值用蓝色 Path 显示，FGO 融合轨迹用红色 Path 显示，两条轨迹叠加可直观评估定位精度。

**关键帧位姿** — 对轨迹按下采样步长绘制 RGB 三色坐标轴（红=X, 绿=Y, 蓝=Z），可观察载体朝向变化。

**协方差椭球** — 从 GTSAM Marginals 提取位置协方差矩阵，Eigen 特征值分解后渲染为 2σ 置信椭球。椭球越大 → 该处定位不确定性越高。

**UWB 测距边** — 每个采样关键帧，绘制从融合位置到各锚点的线段。线段颜色按测距残差编码：绿色 = 小残差（一致性高），红色 = 大残差（可能 NLOS）。

**误差向量** — 箭头从估计位置指向对应时刻的真值位置，颜色按误差大小编码（绿→红）。

**评测指标** — 屏幕上方显示 ATE RMSE、P95 误差、关键帧数、UWB 内点比例、运行耗时。

### 13.4 运行方式

```bash
# 默认配置 + RViz
roslaunch uwb_imu_fgo offline_with_viz.launch

# 自定义配置 + RViz
roslaunch uwb_imu_fgo offline_with_viz.launch config_path:=/path/to/slam.yaml
```

> **注意**: 节点启动后会立即广播 `world→map` TF（10Hz 独立线程），确保 RViz 从始至终有合法的 Fixed Frame，Grid 和所有 Marker 都能正常渲染。

> **WSL 用户**: 由于 WSL 的 D3D12 Mesa GL 后端与 RViz 的 OGRE 引擎不兼容，launch 文件已自动设置 `LIBGL_ALWAYS_SOFTWARE=1` 强制使用 llvmpipe 软件渲染。

### 13.5 RViz 交互提示

| 操作 | 快捷键 |
|------|--------|
| 旋转视角 | 鼠标左键拖拽 |
| 平移视角 | 鼠标中键拖拽 |
| 缩放 | 鼠标滚轮 |
| 切换 Display 开关 | 左侧面板勾选/取消 |
| 聚焦某物体 | 选中后按 `F` |

建议在左侧 Displays 面板中按需开关各类显示（如关闭 UWB Edges 可大幅提升渲染帧率）。

### 13.6 设计说明

- **Latched topics**: 所有 marker 以 latched 模式发布，RViz 即使后启动也能接收完整数据
- **下采样**: 轨迹/关键帧/UWB 边/误差向量按 `traj.size() / 300` 自动下采样，防止 RViz 渲染过载
- **协方差**: 使用 Optimizer 的 Marginals 接口提取逐帧位置协方差，仅当优化器完成了 covariance 计算时可用
- **TF**: 固定发布 `world → map` 恒等变换，因为本系统无漂移坐标系需求

---

## 13. License

BSD
