# UWB-IMU FGO 融合算法调研总览

## 1. 调研范围

| 算法 | 目录 | 优化库 | 论文/参考 |
|------|------|--------|-----------|
| awesome-uwb-localization | `src/awesome-uwb-localization/` | g2o (LM) | 多传感器图优化 |
| SFUISE | `src/SFUISE/` | 自定义 LM | B 样条连续时间融合 |
| uwb-imu-positioning | `src/uwb-imu-positioning/` | Ceres (LM/GN) | 紧耦合因子图 |
| C-LIUO（参考） | `src/C-LIUO/` | Ceres (LM) | LiDAR-IMU-UWB NURBS |

---

## 2. 三算法横向对比

| 维度 | awesome-uwb-localization | SFUISE | uwb-imu-positioning |
|------|--------------------------|--------|---------------------|
| **优化库** | g2o (Levenberg-Marquardt) | 自定义 LM + Cholmod | Ceres Solver (LM/GN) |
| **状态表示** | SE(3) 离散顶点 | B 样条控制点（连续时间） | 15D 离散 keyframe |
| **状态维度（每帧）** | 6-DOF + 可选 IMU 姿态 | 13D: `[p,q,b_a,b_g]` | 15D: `[P,Q,V,b_a,b_g]` |
| **IMU 因子** | ⚠️ 仅姿态先验（无预积分） | ✅ **样条解析导数**（无预积分） | ✅ 中点预积分 + 偏置修正 |
| **UWB 因子** | Range (1D, 含天线偏移) | TDoA / ToA (1D, 含偏置) | Range (1D) / Position (1D) |
| **额外因子** | Pose, Twist, LiDAR Height, 平滑 | 偏置平滑正则化 | 偏置连续性约束 |
| **窗口策略** | 滑动窗口 $N=10\sim500$（直接删除） | 滑动窗口 + 初始化累积 | 固定两帧 |
| **边缘化** | ❌ 直接丢弃旧顶点 | 部分边缘化 | ❌ 两帧替换 |
| **鲁棒核** | Cauchy（所有边） | 离群值硬阈值（0.3m） | Cauchy（仅 UWB 边） |
| **求解器** | Cholmod / CSparse（稀疏） | Cholmod + Hash 累加器（稀疏） | DENSE_QR（稠密） |
| **iSAM2** | ❌ | ❌ | ❌ |
| **外参标定** | 手动配置天线偏移 | ✅ **在线标定** $(\mathbf{t}_{UW}, \mathbf{q}_{UW})$ | 手动配置 |
| **在线重力估计** | ❌ | ✅ 切空间 2D 参数化 | ❌ |
| **多模式 UWB** | 仅 ToA | TDoA + ToA | 紧耦合 + 松耦合 |
| **时间对齐** | 无特殊处理 | 样条统一时间轴 | `td=0.03s` 参数 + 线性插值 |

---

## 3. 各算法核心 Trick 汇总

### 3.1 awesome-uwb-localization

| # | Trick | 描述 |
|---|-------|------|
| 1 | **UWB 两步离群值剔除** | 距离一致性检查 + warm-up 延迟启用 |
| 2 | **时间不确定性自适应协方差** | $\sigma = v_{\max} \cdot \Delta t / 3$，建模异步传感器不确定性 |
| 3 | **多天线偏移建模** | SE(3) 偏移变换纳入 range 残差 |
| 4 | **Cauchy 全局鲁棒核** | 所有边统一使用，防止单点破坏解 |
| 5 | **帧管理策略** | 根据 sensor frame 一致/不一致采取不同建图策略 |

### 3.2 SFUISE

| # | Trick | 描述 |
|---|-------|------|
| 1 | **样条解析导数代替预积分** | 对 B 样条轨迹直接求 $\ddot{\mathbf{p}}, \boldsymbol{\omega}$，与 IMU 原始值对比 |
| 2 | **四元数流形插值** | $\log/\exp$ 映射 + 切空间 B 样条，保证四元数合法性 |
| 3 | **Hash 稀疏块累加器** | 仅存非零 Hessian 块，$O(n_{\text{nnz}})$ 内存 |
| 4 | **自适应 LM 阻尼** | 根据步质量 $q$ 动态调整 $\lambda$ |
| 5 | **在线外参 + 重力标定** | 与状态量联合优化的 $\mathbf{t}_{UW}, \mathbf{q}_{UW}, \mathbf{g}$ |
| 6 | **重力切空间参数化** | 2D 切空间 + 固定模长，避免漂移 |
| 7 | **多模式 UWB 权重归一化** | $w / \sqrt{N}$ 平衡不同频率传感器 |

### 3.3 uwb-imu-positioning

| # | Trick | 描述 |
|---|-------|------|
| 1 | **流形局部参数化** | 7D→6D 切空间更新，Ceres 自动处理 SO(3) 约束 |
| 2 | **IMU 预积分雅可比传播** | $15 \times 15$ 雅可比 + $18 \times 18$ 协方差递推 |
| 3 | **偏置一阶修正** | 利用雅可比近似修正，避免偏置更新后重积分 |
| 4 | **IMU-UWB 时间对齐** | `td` 参数 + 线性插值 |
| 5 | **预测状态维护** | IMU 传播保证 UWB 不可用时的连续输出 |
| 6 | **紧/松耦合双模式** | 灵活切换，平衡精度与计算 |

---

## 4. C-LIUO 参考：UWB 消息格式与读取方法

> 用户编写了 C-LIUO（LiDAR-IMU-UWB NURBS 融合），其 UWB 消息格式和读取方法可作为新算法的参考。

### 4.1 UWB 消息类型

**主要格式**: `LinktrackNodeframe3`

```
std_msgs/Header header
uint8 role           # 节点角色
uint8 id             # 节点 ID
uint64 local_time    # 本地时间 (ns)
uint32 system_time   # 系统时间
float32 voltage      # 电压
LinktrackNode2[] nodes   # 测距数组
```

每个 `LinktrackNode2`:

```
uint8 id              # 锚点 ID
float32 dis           # 距离 (m)
float32 fp_rssi       # 首径 RSSI
float32 rx_rssi       # 总接收 RSSI
```

### 4.2 UWB 数据解析

```cpp
struct UwbData {
    int64_t timestamp;
    int64_t anchor_num;
    std::unordered_map<int, Eigen::Vector3d> anchor_positions;  // ID → 3D 位置
    std::unordered_map<int, double> anchor_distances;            // ID → 测量距离
    float fp_rssi, rx_rssi;  // 信号质量
};
```

### 4.3 NLOS 检测

```cpp
// RSSI 差值法：首径 RSSI 与总 RSSI 差异过大概率是多径
if (node_.rx_rssi - node_.fp_rssi > 6) {
    continue;  // 丢弃该测量
}
```

### 4.4 UWB Factor（C-LIUO NURBS 版本）

```cpp
// 残差公式
p_U_in_M = S_GtoM * (S_ItoG * p_U + p_IinG) + p_GinM;  // NURBS 样条变换
residual = (measured_distance - ||p_U_in_M - anchor_pos||) * weight;
```

### 4.5 关键配置

```yaml
uwb_range_topic: "LinktrackNodeframe3"
AnchorId: [0, 1, 2, ...]
AnchorPos: [x0, y0, z0, x1, y1, z1, ...]
Extrinsics:
    Trans: [tx, ty, tz]    # UWB→IMU 平移
    Rot: [[r11, r12, r13], # UWB→IMU 旋转矩阵
          [r21, r22, r23],
          [r31, r32, r33]]
max_range: 100.0            # 最大有效距离 (m)
```

---

## 5. 对设计新 UWB-IMU iSAM2 FGO 算法的推荐

### 5.1 核心技术选型

| 组件 | 推荐方案 | 参考来源 |
|------|---------|---------|
| **优化框架** | GTSAM iSAM2 | — |
| **状态变量** | 15D: `[p, q, v, b_a, b_g]` 每 keyframe | uwb-imu-positioning |
| **IMU 因子** | `CombinedImuFactor` / `PreintegratedImuMeasurements` | uwb-imu-positioning + GTSAM |
| **UWB 因子** | 自定义 1D Range Factor (含 NLOS 权重) | awesome + C-LIUO |
| **鲁棒核** | Cauchy / Huber | awesome + uwb-imu-positioning |
| **UWB 消息** | `LinktrackNodeframe3`（复用 C-LIUO 格式） | C-LIUO |

### 5.2 推荐 Trick 组合

1. **IMU 预积分 + 偏置一阶修正**（来自 uwb-imu-positioning）— 成熟方案，GTSAM 原生支持
2. **UWB 两步离群值剔除**（来自 awesome-uwb-localization）— 距离一致性 + warm-up
3. **NLOS 检测**（来自 C-LIUO）— RSSI 差值法，`rx_rssi - fp_rssi > 6`
4. **时间不确定性自适应协方差**（来自 awesome-uwb-localization）— $\sigma = v_{\max} \cdot \Delta t / 3$
5. **Cauchy 鲁棒核用于 UWB**（来自 uwb-imu-positioning）— 降低异常测量影响
6. **在线 UWB↔IMU 外参标定**（来自 SFUISE）— 可选进阶特性
7. **B 样条连续轨迹**（来自 SFUISE）— 可选进阶表示，需大幅修改框架

### 5.3 不建议直接复用的

- ❌ g2o 框架（awesome）— 不如 GTSAM iSAM2 适合增量 SLAM
- ❌ 自定义 LM 求解器（SFUISE）— 实现复杂度太高
- ❌ 两帧固定窗口（uwb-imu-positioning）— 信息丢失严重，iSAM2 可以做更好的边缘化

---

## 6. 文件索引

| 文件 | 内容 |
|------|------|
| `00-overview.md` | 本文件：总览与对比 |
| `01-awesome-uwb-localization.md` | awesome-uwb-localization 详细分析 |
| `02-SFUISE.md` | SFUISE 详细分析 |
| `03-uwb-imu-positioning.md` | uwb-imu-positioning 详细分析 |
