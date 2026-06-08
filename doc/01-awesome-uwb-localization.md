# awesome-uwb-localization: g2o 滑动窗口图优化

## 1. 算法概览

基于 **g2o（General Graph Optimization）** 的批量图优化定位系统，支持多机器人/多传感器融合。核心思想：将每个传感器的异步测量转化为因子图中的**边（Edge）**，连接 SE(3) **顶点（Vertex）**，通过 Levenberg-Marquardt 算法进行批量非线性优化。

```
异步传感器测量 → 因子图构建 → g2o LM 批量优化 → TF/odometry 发布
```

---

## 2. 状态变量

### 2.1 每个机器人的轨迹表示

每个机器人维护一个**滑动窗口**，包含 $N \in [10, 500]$ 个 SE(3) 顶点：

$$\mathbf{T}_i = \begin{bmatrix} \mathbf{R}_i & \mathbf{t}_i \\ \mathbf{0} & 1 \end{bmatrix} \in SE(3)$$

- **Position**: $\mathbf{t}_i = (x, y, z)^T \in \mathbb{R}^3$
- **Orientation**: 四元数 $(w, x, y, z)$ 通过 `g2o::SE3Quat` 参数化
- **数据结构**: 循环缓冲区 `vertices[index]`，按传感器类型索引

### 2.2 多天线支持

每个 agent 可配备最多 3 个天线，天线偏移存储为：

$$\text{offsets} = \{\mathbf{T}_{offset,0}, \mathbf{T}_{offset,1}, \mathbf{T}_{offset,2}\}, \quad \mathbf{T}_{offset,k} \in SE(3)$$

在 range 误差计算时应用偏移变换。

---

## 3. Factor 类型与残差公式

### 3.1 UWB Range Factor（主因子）

**类型**: `EdgeSE3Range`（自定义二元边，g2o）

**测量**: 两 agent 间的距离 $d$（米）

**残差函数**（含天线偏移）:

$$e_{\text{range}} = d - \left\| \text{pos}\left(\mathbf{T}_{\text{req}} \cdot \mathbf{T}_{\text{offset},i}\right) - \text{pos}\left(\mathbf{T}_{\text{resp}} \cdot \mathbf{T}_{\text{offset},j}\right) \right\|$$

其中 $\text{pos}(\cdot)$ 取 SE(3) 变换的平移部分。

**信息矩阵**（协方差逆）:

$$\Lambda_{\text{range}} = \frac{1}{\sigma_d^2 + \sigma_{\text{velocity}}^2}$$

其中速度引起的不确定度（3-sigma 原则）:

$$\sigma_{\text{velocity}} = \frac{v_{\max} \cdot \Delta t}{3}$$

**鲁棒核**: `RobustKernelCauchy` — $\rho(x) = \log(1 + x)$

### 3.2 Pose Factor（视觉 SLAM / VICON 可选）

**类型**: `EdgeSE3`

**测量**: 相对 SE(3) 变换 $\mathbf{T}_{\text{rel}} = \mathbf{T}_i^{-1} \mathbf{T}_j$

$$e_{\text{pose}} = \text{se3\_log}\left(\mathbf{T}_{\text{rel}}^{-1} \cdot \mathbf{T}_i^{-1} \mathbf{T}_j\right) \in \mathbb{R}^6$$

**信息矩阵**: 传感器提供的 $6 \times 6$ 协方差逆

### 3.3 Twist / Optical Flow Factor（可选）

将 twist 测量 $\boldsymbol{\xi} = [\mathbf{v}, \boldsymbol{\omega}]^T$ 转换为 SE(3):

$$\mathbf{T}_{\text{meas}} = \exp(\boldsymbol{\xi} \, \Delta t)$$

其中：
- 线速度: $\mathbf{v} \cdot \Delta t$
- 角速度 $\boldsymbol{\omega}$ 转换为 RPY: $[\omega_x \Delta t, \omega_y \Delta t, \omega_z \Delta t]$
- 协方差按 $\Delta t^2$ 缩放

### 3.4 IMU Orientation Prior（可选）

**类型**: `EdgeSE3Prior`（一元约束）

仅约束旋转不约束位置：

$$e_{\text{imu}} = \begin{bmatrix} \mathbf{0}_{3\times 1} \\ \text{quat\_diff}(\mathbf{q}_{\text{imu}}, \mathbf{q}) \end{bmatrix}$$

$$\Lambda_{\text{imu}} = \text{diag}\left(0, 0, 0, \frac{1}{\sigma_{roll}^2}, \frac{1}{\sigma_{pitch}^2}, \frac{1}{\sigma_{yaw}^2}\right)$$

### 3.5 LiDAR Height Prior（可选）

仅约束 Z 坐标:

$$\Lambda_{\text{lidar}} = \text{diag}\left(0, 0, \frac{1}{0.05}, 0, 0, 0\right)$$

### 3.6 轨迹平滑因子

连接同一 agent 的相邻顶点，`distance = 0`:

$$e_{\text{smooth}} = \left\| \mathbf{t}_i - \mathbf{t}_{i+1} \right\|$$

$$\sigma_{\text{smooth}} = \frac{v_{\max} \cdot \Delta t}{3}$$

---

## 4. 求解器配置

### 4.1 优化器

```cpp
typedef g2o::BlockSolver_6_3 SE3BlockSolver;   // 6-DOF pose, 3-DOF edges
typedef g2o::LinearSolverCholmod<...> Solver;    // Cholmod 稀疏求解器
g2o::OptimizationAlgorithmLevenberg *solver;     // LM 算法
```

- **线性求解器**: Cholmod（首选）/ CSparse（备选）
- **优化算法**: Levenberg-Marquardt
- **参数化**: SE(3) via `g2o::SE3Quat`
- **每次优化**: `optimizer.optimize(10~20)` 次迭代

### 4.2 非 iSAM2

⚠️ 该系统使用**批量重优化**（batch optimization），**不是增量平滑与建图（iSAM2）**。每次新测量到达时，整个图被重新优化。

---

## 5. 关键 Trick 清单

### 5.1 滑动窗口（伪边缘化）

- 每个机器人仅保留最近 $N$ 个顶点
- 缓冲区满时**直接删除**旧顶点（非真正的边缘化，信息丢失）

```cpp
index = (index + 1) % trajectory_length;
optimizer.removeVertex(vertices[index], false);  // false = 不更新缓存
vertices[index] = new_vertex;
```

**影响**: 内存 $O(N)$，但丢失旧位姿信息。

### 5.2 UWB 离群值两步剔除

**Step 1: 距离一致性检查**（所有阶段）

```cpp
distance_estimation = ||p_requester - p_responder||;
if (|distance_estimation - uwb->distance| > distance_outlier_threshold)
    reject;  // 典型阈值: 1~5m
```

**Step 2: 延迟启用**（warm-up 阶段）

仅在收到 `trajectory_length` 次测量后才启用 outlier rejection，避免初始化阶段的误剔除。

### 5.3 时间不确定性自适应协方差

根据距上次测量的时间差 $\Delta t$ 调整协方差：

$$\sigma_{\text{age}} = \frac{v_{\max} \cdot \Delta t}{3}$$

对请求方和应答方分别计算，合并为总体距离协方差：

$$\sigma_{\text{total}}^2 = \sigma_{\text{range}}^2 + \sigma_{\text{req}}^2 + \sigma_{\text{resp}}^2$$

### 5.4 多天线偏移

支持 3 个天线分别带偏移，range 误差计算时应用：

```cpp
dt = (T1 * offset[antenna_0]).translation() - (T2 * offset[antenna_1]).translation();
error = measurement - ||dt||;
```

### 5.5 全局 Cauchy 鲁棒核

所有边统一使用 `RobustKernelCauchy`，降低大误差边的影响，防止单个坏测量破坏整体解。

### 5.6 帧管理策略

- 同一 sensor frame: 添加两个 range edge（请求方+应答方）+ 轨迹平滑 edge
- 不同 sensor frame: 只添加单个 range edge + 合并协方差
- 静态 anchor: 只添加 range edge，不添加轨迹 edge

---

## 6. 坐标系

| Frame | 角色 | 类型 |
|-------|------|------|
| `world` / `local_origin` | 全局参考系 | 固定 |
| `uwb` / `uwb_imu` | 定位输出 | 移动 |
| `vicon` / `camera_link` | 真值参考 | 可选 |
| `imu_frame` / `dvs` | 传感器帧 | 附着于 agent |

发布的 TF 变换链: `world` → `uwb_imu`（多机器人时: `world` → `rl_<robot_id>`）

---

## 7. 配置变体

| 配置文件 | 窗口大小 | 传感器 | 适用场景 |
|---------|---------|--------|---------|
| `uwb_only.yaml` | 10 | UWB range only | 纯 UWB 定位 |
| `uwb_imu.yaml` | 12 | UWB + IMU 姿态 | UWB-IMU 融合 |
| `uwb_imu_lidar.yaml` | 20 | UWB + IMU + LiDAR | 全传感器融合 |
| `uwb_pose.yaml` | 500 | UWB + 视觉里程计 | 高精度参考轨迹 |
| `RL_uwb.yaml` | — | 多机器人 UWB | 相对定位模式 |

---

## 8. 对新算法的参考价值

| 技术点 | 价值 |
|--------|------|
| UWB 离群值两步剔除 | ⭐⭐⭐ 实用性强，可直接复用 |
| 时间不确定性自适应协方差 | ⭐⭐⭐ 处理异步传感器关键 |
| 多天线偏移建模 | ⭐⭐ 多天线场景必备 |
| 滑动窗口策略 | ⭐⭐ 平衡精度与计算量 |
| Cauchy 鲁棒核 | ⭐ 通用鲁棒估计技巧 |
| g2o 图优化框架 | ⭐ 学习参考（新算法建议用 iSAM2） |
