# uwb-imu-positioning: Ceres 紧耦合 FGO

## 1. 算法概览

基于 **Ceres Solver** 的**紧耦合因子图优化（Tightly-Coupled FGO）** 框架，面向无人机室内定位。采用 IMU 预积分 + UWB 多锚点距离测量联合优化的方式，支持紧耦合与松耦合两种模式。

```
初始化（三角定位）→ IMU 预积分累积 → UWB 到达时 FGO 更新（紧/松耦合）→ odometry 发布
```

⚠️ **不使用 iSAM2**，而是固定大小的两帧窗口 + Ceres 批量求解。

---

## 2. 状态变量（15 维）

每个 keyframe 维护完整 15 维状态：

$$\mathbf{x} = \begin{bmatrix} \mathbf{P} \\ \mathbf{Q} \\ \mathbf{V} \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^3 \times \mathbb{S}^3 \times \mathbb{R}^3 \times \mathbb{R}^3 \times \mathbb{R}^3$$

### 2.1 Ceres 参数块设计

| 参数块 | 维度 | 内容 |
|--------|------|------|
| `para_pose[k]` | 7D | $[x, y, z, q_x, q_y, q_z, q_w]$ — 位置 + 四元数姿态 |
| `para_speed_bias[k]` | 9D | $[v_x, v_y, v_z, b_{a,x}, b_{a,y}, b_{a,z}, b_{g,x}, b_{g,y}, b_{g,z}]$ |

### 2.2 流形优化 — 位姿局部参数化

**全局参数化**: 7D $[\mathbf{P}(3), \mathbf{Q}(4)]$（过参数化，四元数有单位范数约束）

**切空间更新**（6D → 7D）:

$$\mathbf{P}_{\text{new}} = \mathbf{P}_{\text{old}} + \delta\mathbf{P}, \qquad \mathbf{Q}_{\text{new}} = \mathbf{Q}_{\text{old}} \otimes \exp\left(\frac{\delta\boldsymbol{\theta}}{2}\right)$$

通过自定义 `PoseLocalParameterization` 类实现，Ceres 自动在切空间进行优化，保证四元数始终是合法旋转。

---

## 3. Factor 类型与残差公式

### 3.1 IMU 预积分因子（15D）

**积分方法**: 中点积分（Midpoint Integration）

$$\Delta\mathbf{q}_{k+1} = \Delta\mathbf{q}_k \otimes \exp\left(\frac{1}{2}\bar{\boldsymbol{\omega}}_k \Delta t_k\right)$$

$$\Delta\mathbf{p}_{k+1} = \Delta\mathbf{p}_k + \Delta\mathbf{v}_k \Delta t_k + \frac{1}{8}\left(\Delta\mathbf{a}_k + \Delta\mathbf{a}_{k+1}\right) \Delta t_k^2$$

**15D 残差**（含偏置一阶修正）:

$$\mathbf{r}_{\text{IMU}} = \begin{bmatrix} \mathbf{Q}_i^{-T}\left(\frac{1}{2}\mathbf{g}\Delta t^2 + \mathbf{P}_j - \mathbf{P}_i - \mathbf{V}_i \Delta t\right) - \Delta\mathbf{p}_{\text{corr}} \\ 2\left(\Delta\mathbf{q}^{-1}_{\text{corr}} \otimes \mathbf{Q}_i^{-1}\mathbf{Q}_j\right)_{\text{vec}} \\ \mathbf{Q}_i^{-T}\left(\mathbf{g}\Delta t + \mathbf{V}_j - \mathbf{V}_i\right) - \Delta\mathbf{v}_{\text{corr}} \\ \mathbf{b}^a_j - \mathbf{b}^a_i \\ \mathbf{b}^g_j - \mathbf{b}^g_i \end{bmatrix} \in \mathbb{R}^{15}$$

**偏置一阶修正**（避免每次偏置更新后重新积分）:

$$\Delta\mathbf{p}_{\text{corr}} = \Delta\mathbf{p} + \mathbf{J}^P_{b^a}\Delta\mathbf{b}^a_i + \mathbf{J}^P_{b^g}\Delta\mathbf{b}^g_i$$

$$\Delta\mathbf{v}_{\text{corr}} = \Delta\mathbf{v} + \mathbf{J}^V_{b^a}\Delta\mathbf{b}^a_i + \mathbf{J}^V_{b^g}\Delta\mathbf{b}^g_i$$

$$\Delta\mathbf{q}_{\text{corr}} = \Delta\mathbf{q} \otimes \exp\left(\frac{1}{2}\mathbf{J}^Q_{b^g}\Delta\mathbf{b}^g_i\right)$$

**噪声参数**:
- 加速度计: $\sigma_a = 0.1\ \text{m/s}^2$, random walk $\sigma_{wa} = 0.01\ \text{m/s}^3$
- 陀螺仪: $\sigma_g = 0.01\ \text{rad/s}$, random walk $\sigma_{wg} = 2\times 10^{-5}\ \text{rad/s}^2$

### 3.2 UWB Range 因子（1D each，紧耦合模式）

对每个可见锚点建立独立的 range 残差:

$$r_{\text{range},k} = \left\|\mathbf{A}_k - \mathbf{P}_j\right\| - z_k$$

加权: $\frac{1}{\sqrt{\sigma_k}}$，其中 $\sigma_k = \text{precisionRangeErrEst} / 1000 \times 5$

- $\mathbf{A}_k$: 第 $k$ 个锚点的已知位置
- $\mathbf{P}_j$: 当前 keyframe 位置（优化变量）
- $z_k$: 测量距离 `precisionRangeMm / 1000`

**典型配置**: 4~6 个锚点同时可见

### 3.3 UWB Position 因子（1D，松耦合模式）

松耦合模式下，先进行独立三角定位得到位置 $\mathbf{P}_{\text{tri}}$，再作为软约束加入:

$$r_{\text{pos}} = \left\|\mathbf{P}_{\text{tri}} - \mathbf{P}_j\right\|$$

### 3.4 初始化因子（仅首次 UWB 测量时使用）

**三角定位**（Ceres 求解）:

$$\min_{\mathbf{P}} \sum_k \left(\left\|\mathbf{A}_k - \mathbf{P}\right\| - r_k\right)^2$$

附加约束:
- 坐标硬边界约束
- 点到线距离几何约束

---

## 4. Ceres Solver 配置

### 4.1 求解器设置

```cpp
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;     // 小规模问题用稠密 QR
options.max_num_iterations = 10000;                // 充分迭代
options.minimizer_progress_to_stdout = false;
```

### 4.2 鲁棒核

所有 UWB range 因子使用 Cauchy 损失:

$$\rho(r) = \log(1 + r^2), \quad \rho'(r) = \frac{2r}{1 + r^2}$$

IMU 因子**不使用**鲁棒核（已通过马氏距离加权）。

---

## 5. 关键 Trick 清单

### 5.1 流形上的位姿局部参数化

> ⭐⭐⭐ **Ceres 中最推荐的做法**

自定义 `PoseLocalParameterization` 实现 SO(3) 流形优化：
- 全局 7D（过参数化），局部 6D（最小参数化）
- Ceres 自动在切空间做 $\oplus$ 和 $\boxminus$ 运算
- 保证四元数始终是单位范数的合法旋转

### 5.2 IMU 预积分雅可比传播

维护 $15 \times 15$ 雅可比矩阵和 $18 \times 18$ 协方差矩阵，在预积分过程中递推传播:

**状态转移矩阵**:

$$\mathbf{J}_{k+1} = \mathbf{F}_k \mathbf{J}_k$$

**协方差传播**:

$$\mathbf{P}_{k+1} = \mathbf{F}_k \mathbf{P}_k \mathbf{F}_k^T + \mathbf{V}_k \mathbf{N}_k \mathbf{V}_k^T$$

其中 $\mathbf{N}_k$ 是 $18 \times 18$ 噪声协方差矩阵。

### 5.3 偏置一阶修正

IMU 预积分因子使用雅可比 $\mathbf{J}^P_{b^a}, \mathbf{J}^P_{b^g}, \mathbf{J}^V_{b^a}, \mathbf{J}^V_{b^g}, \mathbf{J}^Q_{b^g}$ 对偏置变化进行一阶近似修正，避免每次优化迭代都重新积分。这是 VINS-Mono 风格的经典技巧。

### 5.4 IMU-UWB 时间对齐

```yaml
td: 0.03  # IMU 与 UWB 间的固定时延（秒）
```

- 所有 IMU 测量在两 UWB 更新之间累积
- 在 UWB 到达时刻，IMU 线性插值到精确时间戳
- 确保紧耦合同步

### 5.5 偏置连续性约束

相邻帧间的偏置被约束为缓慢变化，防止不合理的偏置跳变:

$$\mathbf{b}_j - \mathbf{b}_i \approx \mathbf{0} \quad \text{（小权重软约束）}$$

### 5.6 预测状态维护

在两次 UWB 更新之间，利用 IMU 传播预测状态:
- `m_pred_position`, `m_pred_velocity`, `m_pred_rotation`
- 即使 UWB 短暂不可用，仍能输出连续估计
- 作为下次优化的初始猜测

### 5.7 紧/松耦合双模式

| 模式 | 方法 | 精度 | 计算量 |
|------|------|------|--------|
| **紧耦合** | 每个锚点距离作为独立因子 | 更高 | 较大 |
| **松耦合** | 先三角定位再作为位置因子 | 稍低 | 较小 |

通过 `tightlyFGOUpdate()` 和 `looselyFGOUpdate()` 两个函数实现切换。

### 5.8 UWB 测量验证

```cpp
if (range < 0.3) reject;  // 最小距离阈值，过滤无效/零测量
```

---

## 6. 坐标系

- **World frame**: NED 或任意全局原点（默认首个锚点 = 原点）
- **Body frame**: IMU 坐标系（安装在无人机上）
- **测量变换**:
  - UWB range: 已在世界系（欧氏距离）
  - IMU 加速度: 体坐标系 → 世界系 $\mathbf{a}_{\text{world}} = \mathbf{R}^T \mathbf{a}_{\text{body}}$
  - 重力补偿: 在世界系加速度中减去

---

## 7. 配置文件结构（slam.yaml）

```yaml
slam_fps: 50                    # 优化频率 (Hz)
td: 0.03                        # IMU-UWB 时间同步偏差 (s)
mobile_id: 301                  # 移动端 ID
anchor_list: [101, 102, ...]    # 已知锚点 ID 列表
anchor_101: [x, y, z]           # 锚点 101 坐标 (m)
anchor_102: [x, y, z]           # 锚点 102 坐标 (m)
G: 9.8                          # 重力加速度 (m/s²)
```

---

## 8. 对新算法的参考价值

| 技术点 | 价值 |
|--------|------|
| Ceres 流形局部参数化 | ⭐⭐⭐ 在 iSAM2 中对应 GTSAM 的内建 Lie 群类型 |
| IMU 预积分 + 偏置一阶修正 | ⭐⭐⭐ VINS-Mono 经典方案，可直接用于 iSAM2 |
| IMU-UWB 时间对齐 | ⭐⭐ 异步传感器融合必备 |
| 紧/松耦合双模式 | ⭐⭐ 灵活性设计 |
| 预测状态维护 | ⭐⭐ 保证输出连续性 |
| Cauchy 鲁棒核用于 UWB | ⭐ 通用鲁棒估计 |

> 💡 **关键启示**: 该算法的 IMU 预积分因子设计（中点积分 + 偏置雅可比修正）是 VINS-Mono 的简化版，可直接迁移到 GTSAM iSAM2 的 `CombinedImuFactor` 或 `PreintegratedImuMeasurements` 中。
