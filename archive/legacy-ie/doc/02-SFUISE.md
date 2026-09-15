# SFUISE: 自定义 LM + B 样条连续时间融合

## 1. 算法概览

SFUISE（**S**pline **FUS**ion for **I**MU and **U**WB **S**tate **E**stimation）是一种基于 **B 样条连续时间轨迹表示** 的 IMU-UWB 紧耦合融合系统。核心创新在于使用**三次 B 样条**对连续轨迹进行参数化，避免了传统离散时间方法的插值误差，IMU 残差通过样条解析导数直接计算。

```
传感器数据降采样 → 初始化（累积控制点 + 重力估计 + 外参标定）
                        ↓
              滑动窗口 LM 优化（IMU + TDoA/ToA + 偏置平滑）
                        ↓
                    发布 B 样条轨迹 + 标定参数
```

⚠️ **不使用 iSAM2 或 g2o/Ceres**，而是完全自定义的 Levenberg-Marquardt 优化器。

---

## 2. 状态变量

### 2.1 每个 B 样条控制点

$$\mathbf{x}_i = [\mathbf{p}_i, \mathbf{q}_i, \mathbf{b}_{a,i}, \mathbf{b}_{g,i}] \in \mathbb{R}^3 \times \mathbb{S}^3 \times \mathbb{R}^3 \times \mathbb{R}^3$$

| 分量 | 符号 | 维度 | 描述 |
|------|------|------|------|
| 位置 | $\mathbf{p}_i$ | $\mathbb{R}^3$ | 世界坐标系下 IMU 位置 |
| 姿态 | $\mathbf{q}_i$ | $\mathbb{S}^3$ | 世界→IMU 的单位四元数 |
| 加速度计偏置 | $\mathbf{b}_{a,i}$ | $\mathbb{R}^3$ | IMU 加速度计偏置 |
| 陀螺仪偏置 | $\mathbf{b}_{g,i}$ | $\mathbb{R}^3$ | IMU 陀螺仪偏置 |

### 2.2 全局参数（窗口内为常数）

| 参数 | 符号 | 维度 | 描述 |
|------|------|------|------|
| 重力向量 | $\mathbf{g}$ | $\mathbb{R}^2$ | 切空间参数化，模长固定 $9.81\,\text{m/s}^2$ |
| UWB→IMU 外参平移 | $\mathbf{t}_{UW}$ | $\mathbb{R}^3$ | UWB 天线在 IMU 系下的位置 |
| UWB→IMU 外参旋转 | $\mathbf{q}_{UW}$ | $\mathbb{S}^3$ | UWB 系到 IMU 系的旋转 |
| UWB 标签偏移 | $\mathbf{r}_{tag}$ | $\mathbb{R}^3$ | 标签在刚体上的位置 |

---

## 3. B 样条连续状态表示

### 3.1 三次 B 样条插值

位置和偏置是欧氏函数，使用 4 个相邻控制点插值：

$$\mathbf{p}(t) = \sum_{i=0}^{3} N_i(u) \, \mathbf{p}_{k+i}, \quad u = \frac{t - t_k}{\Delta t} \in [0, 1)$$

其中 $N_i(u)$ 是三次 B 样条基函数。

**解析导数**（闭式）:

$$\dot{\mathbf{p}}(t) = \sum_{i=0}^{3} \dot{N}_i(u) \, \mathbf{p}_{k+i}, \qquad \ddot{\mathbf{p}}(t) = \sum_{i=0}^{3} \ddot{N}_i(u) \, \mathbf{p}_{k+i}$$

### 3.2 四元数流形插值

利用 SO(3) 的对数/指数映射：

1. 计算相邻控制点的相对四元数: $\mathbf{q}_{\delta,i} = \mathbf{q}_i^{-1} \mathbf{q}_{i+1}$
2. 对数映射到切空间: $\mathbf{v}_i = \log(\mathbf{q}_{\delta,i}) \in \mathbb{R}^3$
3. B 样条线性插值: $\mathbf{v}_{\text{itp}}(u) = \sum_i N_i(u) \, \mathbf{v}_i$
4. 指数映射回 SO(3): $\mathbf{q}_{\text{itp}} = \exp(\mathbf{v}_{\text{itp}})$

**角速度**: $\boldsymbol{\omega}(t) = 2 \, \text{vec}\left(\dot{\mathbf{q}}_{\text{itp}}(t) \, \mathbf{q}_{\text{itp}}^*(t)\right)$

---

## 4. Factor 类型与残差公式

### 4.1 IMU 残差（6D）

$$\mathbf{r}_{\text{imu}}(t) = \begin{bmatrix} \boldsymbol{\omega}_{\text{imu}}(t) - \boldsymbol{\omega}_{\text{interp}}(t) + \mathbf{b}_g(t) \\ \mathbf{R}_{\text{interp}}^T(t)\left[\ddot{\mathbf{p}}_{\text{interp}}(t) - \mathbf{g}\right] - \mathbf{a}_{\text{imu}}(t) + \mathbf{b}_a(t) \end{bmatrix} \in \mathbb{R}^6$$

- $\boldsymbol{\omega}_{\text{interp}}, \ddot{\mathbf{p}}_{\text{interp}}$: 样条 1 阶和 2 阶导数
- $\mathbf{R}_{\text{interp}} = \mathbf{R}(\mathbf{q}_{\text{interp}})$: 插值姿态的旋转矩阵
- $\mathbf{g}$: 被估计的重力向量

**权重**: $\text{diag}\left(\boldsymbol{\sigma}_{a,\text{inv}}^2 w_a,\; \boldsymbol{\sigma}_{g,\text{inv}}^2 w_g\right)$

> ⚡ **关键创新**: IMU 残差不依赖传统预积分，而是直接对比样条解析导数与 IMU 原始测量。

### 4.2 TDoA 残差（1D，Time-Difference-of-Arrival）

$$\mathbf{r}_{\text{tdoa}}(t) = \left\|\mathbf{p}_{\text{tag}}^U(t) - \mathbf{a}_j\right\| - \left\|\mathbf{p}_{\text{tag}}^U(t) - \mathbf{a}_i\right\| - \Delta_{ij}$$

其中 UWB 标签在 UWB 系下的位置:

$$\mathbf{p}_{\text{tag}}^U(t) = \mathbf{R}_{UW} \left[\mathbf{R}_{\text{interp}}(t) \, \mathbf{r}_{\text{tag}} + \mathbf{p}_{\text{interp}}(t)\right] + \mathbf{t}_{UW}$$

- $\mathbf{a}_i, \mathbf{a}_j$: 锚点在 UWB 系下的已知位置
- $\Delta_{ij}$: 到达时间差（乘以光速 $c$ 转换为距离）
- 权重按测量数量归一化: $w_{\text{uwb}}^2 / \sqrt{N_{\text{tdoa}}}$

### 4.3 ToA 残差（1D，Time-of-Arrival）

$$r_{\text{toa}}(t) = \left\|\mathbf{p}_{\text{tag}}^U(t) - \mathbf{a}_i\right\| - r_i - \Delta d_i$$

- $r_i$: 测得距离
- $\Delta d_i$: **与状态量联合优化的每锚点偏置**（ToA 特有）
- 权重按测量数量归一化: $w_{\text{uwb}}^2 / \sqrt{N_{\text{toa}}}$

### 4.4 偏置平滑正则化（6D）

$$r_{\text{bias}}(t_0, t_1) = \begin{bmatrix} \mathbf{b}_a(t_1) - \mathbf{b}_a(t_0) \\ \mathbf{b}_g(t_1) - \mathbf{b}_g(t_0) \end{bmatrix}$$

权重: $\text{diag}\left(\boldsymbol{\sigma}_{ba,\text{inv}}^2,\; \boldsymbol{\sigma}_{bg,\text{inv}}^2\right)$

---

## 5. 优化器：自定义 Levenberg-Marquardt

### 5.1 目标函数

$$\min_{\mathbf{x}} \; \left\| \mathbf{r}(\mathbf{x}) \right\|_{\mathbf{W}}^2 = \mathbf{r}(\mathbf{x})^T \mathbf{W} \, \mathbf{r}(\mathbf{x})$$

### 5.2 LM 迭代更新

$$(\mathbf{J}^T \mathbf{W} \mathbf{J} + \lambda \, \text{diag}(\mathbf{J}^T \mathbf{W} \mathbf{J})) \, \Delta\mathbf{x} = -\mathbf{J}^T \mathbf{W} \, \mathbf{r}(\mathbf{x})$$

### 5.3 自适应阻尼策略

| 条件 | 操作 |
|------|------|
| 初始值 | $\lambda = 10^{-6},\; \lambda_{\text{vee}} = 2$ |
| 步质量 $q < 0$（变差） | $\lambda \leftarrow \min(100,\; \lambda_{\text{vee}} \cdot \lambda),\; \lambda_{\text{vee}} \leftarrow 2\lambda_{\text{vee}}$ |
| 步质量 $q \geq 0$（改善） | $\lambda \leftarrow \max(10^{-18},\; \lambda \cdot \max(\frac{1}{3},\; 1 - (2q-1)^3))$ |

### 5.4 收敛判据

- 最大迭代次数: 5（通常几次就收敛）
- $|\max(\nabla f)| < 10^{-8}$
- 相对误差改进 $< 10^{-6}$

### 5.5 稀疏线性求解

- **累加器**: Hash 四元组 `(row, col, rows, cols) → MatrixXd` — 仅存储非零块
- **求解器**: Cholmod (SuiteSparse) — 稀疏 Cholesky 分解，$O(n_{\text{nnz}})$

---

## 6. 关键 Trick 清单

### 6.1 样条解析导数代替 IMU 预积分

> ⭐⭐⭐ **最具参考价值的创新**

传统方法对 IMU 进行数值预积分，SFUISE 直接对 B 样条轨迹求解析导数（速度 $\dot{\mathbf{p}}$、加速度 $\ddot{\mathbf{p}}$、角速度 $\boldsymbol{\omega}$），与 IMU 原始测量比较。**避免了预积分的离散误差和线性化近似**。

### 6.2 四元数流形插值

- 利用 SO(3) 的 $\log / \exp$ 映射在切空间进行 B 样条插值
- 保证插值结果始终是合法四元数（单位范数）
- 雅可比通过 `Qleft`、`Qright` 矩阵和 `dexp`、`dlog` 解析计算

### 6.3 Hash 稀疏块累加器

```cpp
// 仅存储非零的 Hessian 块
Key: (start_row, start_col, rows, cols) → MatrixXd
```

相比稠密 Hessian 大幅节省内存，适合滑动窗口中状态维数变化的情况。

### 6.4 滑动窗口 + 边缘化

- **初始化阶段**: 固定首个位姿，累积控制点直到 $N \geq \text{window\_size}$
- **满窗阶段**: 固定窗口大小，每次移除最老状态（边缘化或直接丢弃）
- **标定窗口**: 重力和外参仅在前 5~50 次迭代中优化

### 6.5 UWB 离群值拒绝

可选的离群值拒绝:

$$\text{if } |r_{\text{toa/tdoa}}| > \text{reject\_thresh} \implies \text{跳过该因子}$$

- 典型阈值: `reject_uwb_thresh = 0.3` m
- 仅在窗口宽度 $> 0.5 \times \text{window\_size}$ 后激活（避免初始化阶段误剔除）

### 6.6 重力切空间参数化

重力 $\mathbf{g} \in \mathbb{R}^2$（二维切空间），模长固定为 $9.81\ \text{m/s}^2$。避免优化过程中重力模长漂移。

### 6.7 在线外参标定

- UWB↔IMU 外参 $(\mathbf{t}_{UW}, \mathbf{q}_{UW})$ 在优化中联合估计
- 标签偏移 $\mathbf{r}_{tag}$ 也作为优化变量
- 前 $n_{\text{calib}}$ 个窗口用于外参收敛

### 6.8 多模式 UWB 测量支持

- **TDoA 模式**: 高频率（~447 Hz），高权重 $w_{\text{uwb}} = 300$
- **ToA 模式**: 包含每锚点偏置估计，权重 $w_{\text{uwb}} = 7$
- 权重按 $\frac{1}{\sqrt{N}}$ 归一化，平衡不同频率的传感器

### 6.9 自适应 LM 阻尼

不固定 $\lambda$，而是根据步质量动态调整，确保收敛速度与稳定性平衡。

---

## 7. 坐标系与变换

```
  World (W)                  UWB (U)
     ↑                          ↑
     │ T_IW (optimized)         │ T_UW (calibrated)
     │                          │
  IMU/Body (I) ←──────── tag offset (r_tag)
```

$$\mathbf{p}_{\text{tag}}^U = \mathbf{R}_{UW} \left[\mathbf{R}_{I/W} \, \mathbf{r}_{\text{tag}} + \mathbf{p}_{I/W}\right] + \mathbf{t}_{UW}$$

---

## 8. 消息格式

### 输入

- **TDoA**: `int32 idA, idB; float32 data`（到达时间差）
- **ToA**: `RTLSStick` 容器，含 `RTLSRange[]`（每锚点距离+时间戳）

### 输出

- **Spline**: 所有控制点状态（knots）+ idle knots + 时间信息
- **Knot**: `[p(3), q(4), b_a(3), b_g(3)]`
- **Estimate**: 完整样条 + 窗口状态 + 运行时间
- **Calib**: 重力、外参变换、标签偏移

---

## 9. 对新算法的参考价值

| 技术点 | 价值 |
|--------|------|
| B 样条连续时间轨迹 | ⭐⭐⭐ 避免预积分离散误差的核心创新 |
| 样条解析导数计算 IMU 残差 | ⭐⭐⭐ 全新的 IMU 融合范式 |
| 四元数流形插值 | ⭐⭐⭐ SO(3) 上的正确插值方法 |
| 在线外参标定 | ⭐⭐ 减少离线标定依赖 |
| Hash 稀疏块 | ⭐⭐ 自定义求解器的内存优化 |
| 自适应 LM 阻尼 | ⭐⭐ 快速收敛策略 |
| 重力切空间参数化 | ⭐ 保证重力模长不变 |
| 多模式 UWB | ⭐ 兼容不同硬件 |

> ⚠️ **注意事项**: SFUISE 自定义了整个优化框架（LM + 稀疏求解），代码复杂度高。新算法若基于 GTSAM iSAM2，**可借鉴其 B 样条轨迹表示和解析导数思路**，但无需复现求解器。
