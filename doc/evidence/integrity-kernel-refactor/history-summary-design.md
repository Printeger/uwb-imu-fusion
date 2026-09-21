# C1 设计冻结：故障保持的历史消元摘要（§7.1–7.7）与 C2 双通道接口（§7.3–7.5、§8.2）

状态：**A0 设计冻结已交付**；A1–A6 的实现与 B0–B5 **本轮未执行**（原因见 §8）。
基线：`d58a45d`（P5）。本文件是下一轮实现的执行依据，符号与义务先在此锁定。

## 1. 表示（§7.1）

历史压缩对象为**平方根/消元后**的算子，禁止长期累加易相消的正规方程：

| 量 | 形状 | 含义 |
|---|---|---|
| `R_b` | `n_b × n_b`（上三角，可带排列/缩放） | 边界（分隔变量）信息的平方根 |
| `T_b` | `n_b × q_hist` | 历史故障参数在边界上的响应 |
| `d_b` | `n_b` | 边界均值右端项 |
| `F_b` | `ν_⊥ × q_hist` | 与状态无关的残差对历史故障的响应 |
| `d_perp` | `ν_⊥` | 状态无关残差（含历史故障的检测代价） |

**审计视图（导出，不参与求解）**：`Ω_b = F_bᵀ F_b`、`ξ_b = F_bᵀ d_perp`、`κ_b = ‖d_perp‖²`、
`ν_⊥ = dim d_perp`。数值上只保留 `F_b, d_perp`，`Ω/ξ/κ` 按需重算并作为证书字段导出。

**名义估计取 `f = 0`；故障列 `T_b`/`F_b` 保留**，不并入名义信息。

## 2. 符号约定（锁定，代码与测试必须写明）

对边界参数 `x_b` 与历史故障 `f`，块系统写成

```
[ R_b   T_b ] [ x_b ]   [ d_b    ]
[ 0     F_b ] [ f   ] = [ d_perp ]
```

* `R_b⁻¹ T_b`：**正向注入的边界均值响应**——把历史故障 `f` 当作已实现量，其对边界解的
  平移（用于"故障在窗口内被看见时边界均值如何移动"）。
* `−R_b⁻¹ T_b`：**把 `f` 当作条件变量**（即以 `f` 为条件的边界后验均值相对边际的差）
  的响应——用于条件协方差/条件均值口径的检查与 oracle 对照。
* 两者相差一个符号与语境；代码中函数名必须显式区分（`boundaryMeanShiftForFault(f)` 与
  `conditionalBoundaryMeanDelta(f)`），测试对同一 `(R_b,T_b,d_b,f)` 断言二者互为相反数。

## 3. 组合系统（§7.2）

窗口组装：

```
H_c = [[R_b, 0], [H_active]]        A_c = [[T_b], [A_active]]        z_c = [[d_b], [z_active]]
```

交给**同一个**平方根上下文（一次 QR）；`DetectorOnlyRows` 承载 `(F_b, d_perp)`，
**不得删除自由度或检测信息**（§5.4）。上下文指纹必须纳入历史摘要版本（缓存键失效），
扩展 `StatisticalBoundKey` 的 `contract_version`/`envelope_fingerprint` 之外新增
`history_summary_version`。

**行级归属**：每条摘要行可追溯到原始测量/IMU 样本 → 因子 → 摘要（HIS-03 断言）。

**不重复计入**：与主估计器已边缘化的先验不得二次进入（同一 `group_id`/`noise_model_id` 的唯一性检查）。

## 4. 生命周期（§7.6，逐行实现+测试）

| 行 | 规则 | 本轮状态 |
|---|---|---|
| 1 | 未来 onset 参数：仅当物理 onset **确实晚于已压缩数据**才可零列 | NOT_RUN |
| 2 | 模式进入历史：输入响应与检测代价**一起**消元；保留 event ID/时间支撑/概率依据 | NOT_RUN |
| 3 | 源离窗不自动删除 | NOT_RUN |
| 4 | 故障结束 ≠ 清除：需有效清除证据（含 `F_b`/`d_perp` 归零的证明） | NOT_RUN |
| 5 | 淘汰历史模式：需"对未来保护也成立"的无影响证明 + 包络 + 风险处理，或真正清洁重置 | NOT_RUN |
| 6 | 运行中 order/family 热切换：**默认拒绝**，需重建 | NOT_RUN |
| 7 | 容量超限的合法动作：经证明的更粗包络 / 有效重置或重建 / 停止受保护发布；**绝不静默丢最旧故障** | NOT_RUN |
| 8 | `auto_shrink` 保持**关闭**（§7.7：证明与测试完成前不启用） | 保持关闭 ✓ |

配置新增 `resources` 容量键（严格 loader + 迁移注记）：
`history.max_summary_rows`、`history.max_fault_columns`、`history.max_perp_rows`、
`history.capacity_action`（`REFUSE` / `RESET` / `STOP_PROTECTED`，默认 `REFUSE`）。

## 5. 重线性化与原始数据（§7.7）

* 摘要绑定：线性化点/局部坐标、白化模型、物理故障映射；可证明的**线性**坐标变换可一致转换
  （`(R,T,d,F,d_perp)` 左乘/右乘相应变换）。
* Jacobian 的实际非线性变化必须**从原始数据重建**或有明确余项包络；**禁止只改
  `linearization_id`**。
* 保留重积分所需 IMU 样本/时间边界/版本；PIM 名义 bias 变化 → 故障映射缓存按依赖更新（HIS-04）。

## 6. 冷启动（A4）

无原始历史且无有效既有摘要 → 显式不可认证状态：`HISTORY_SUMMARY_INVALID`
（§4.5 枚举，加入 reason 字典）。**禁止**从 nominal marginal 反推缺失的检测信息。

## 7. Oracle 与场景（A5/A6）

* HIS-01：关闭 fixed-lag（无界历史）作为 oracle，对**同一冻结问题**逐项比较
  `Σp / G / Γ / 检测量 / df / κ / 代价`（不只轨迹）；**多个消元顺序**各跑一遍；
  容差按 ADR 0002 分级（1e-9/1e-7/1e-6）。226 历元若内存不可行 → oracle 用短序列，
  成熟运行只报行为、不宣称等价。
* HIS-02：`config/r0_r1_development_scenarios.yaml` 新增"故障早注入、跨边缘化边界持续"场景，
  保留 raw-stream 证据。

## 8. C2 目标布局（B0–B5，接口先冻结）

* 通道 1 = 当前系统残差（含历史边界因子，**命名不得说成"仅新数据"**）；
  通道 2 = 历史消元后的残差（状态已消元）。联合接受 `A = {T_c ≤ τ_c} ∩ {T_b ≤ τ_b}`。
  池化检测仅保留为短序列参考实现（验证信息保持与检出能力），不另发产品版本。
* 双通道 PL：`λ_{h,j} = f_hᵀ Γ_{h,j} f_h`；每通道解 `Λ_{h,j}`（复用 B3 的保守侧求解）；
  `w_j>0, Σw_j=1`；`W_h = Σ_j w_j Γ_{h,j}/Λ_{h,j}`；要求 `ker W_h ⊆ ker G_h`；
  `b_{h,d} = sqrt(g_{h,d} W_h† g_{h,d}ᵀ)`；`L_{h,d} = b + k_{h,d} σ_d`。
  实现：各 `Z_{h,j}` 按 `sqrt(w_j/Λ_{h,j})` 缩放后堆叠，走 B1 小型分解路径，**不显式求 W 逆**；
  无自由度/无故障贡献的通道不得凭空生成 `Λ`；单有效通道退化为 `s√Λ`；
  证书含 `detector_id/τ_j/ν_j/β_h/Λ_{h,j}/w_j/W 证明 id`；证明不依赖通道独立性。
* fault-span 投影（§7.5）：`U` 覆盖**声明内全部**保留历史故障方向（不得按观测故障/top-K 挑选），
  要求 `Z_bᵀ Z_b = F_allᵀ F_all` 精确成立；声明/线性化/模式变化 → 重建；改检测器必须重设阈值并重跑 §7.4。
* §8.2 模型误差通道：`ρ_{p,d}`（位置）与 `ρ_{r,j}`（残差），`τ_{j,risk} = (√τ_{j,actual} + ρ_{r,j})²`，
  逐轴加 `ρ_p`；名义虚警用独立门限；`ρ` 默认 0 且**状态入证书**（不得编造界）；`ρ=0` 退化为标准 χ²。

## 9. 实现触点（已定位，供下一轮直接开工）

| 步骤 | 文件 | 关键函数/位置 |
|---|---|---|
| A1 边界构造替换 | `src/uwb_imu_pl/estimation/incremental_estimator.cpp` | `buildIntegrityWindow()` 的 BoundaryPrior 段（`partial-QR + 固有值分解`，约 1178–1245 行） |
| A1 摘要先于删除 | 同上 + `commitEpoch`/边缘化路径 | 与 fixed-lag 边缘化同一历元内完成，顺序在测试中锁定 |
| A2 生命周期 | 新 `include/uwb_imu_pl/integrity/history_fault_summary.hpp` + `src/.../history_fault_summary.cpp` | 容量动作、冷启动枚举、事件清理证明 |
| B1 双通道上下文 | `src/uwb_imu_pl/estimation/square_root_context.cpp` | `faultResponse()`（缩放堆叠 + 单次分解）；`DetectorOnlyRows` 已可用（NUM-02） |
| B1 PL | `src/uwb_imu_pl/integrity/protection_level_v2.cpp` | `faultAxisBounds()` 扩展为多通道 `b_{h,d}` |
| B5 诊断 | `include/uwb_imu_pl/common/types.hpp`、`src/uwb_imu_pl/io/run_logger.cpp` | v15：`channel_id/statistic/τ/ν/Λ/w/W_id/ρ`、池化参考列 |

## 10. 本轮未执行的原因（诚实记录）

C1 的 A1 需要替换估计器核心的边界先验构造（含与 fixed-lag 边缘化的时序重排、行级归属、
重线性化绑定），C2 需要新的双通道检测与 PL 公式并重跑全部场景与 oracle；两者都必须配套
HIS-01..06、DET-02..04 的完整验证（多消元顺序 oracle、新跨边界场景、定量对比），否则不能声明闭合。
在本轮可用的执行窗口内无法完成"实现 + 完整验证"的闭环，因此按纪律**不做半成品改动**：
A1–A6、B0–B5 全部记 NOT_RUN，本文件作为冻结设计与实施清单交付。

## 11. P7 实现勘察（A1 触点的实际代码事实，供下一轮直接开工）

对 `src/uwb_imu_pl/estimation/incremental_estimator.cpp`（`buildIntegrityWindow`，边界段
约 1187–1270 行）逐行核对后确认：

1. **当前算法**：`boundary_graph.linearize(*tx.frozen_values)` →
   `eliminatePartialMultifrontal(outside_variables, EliminateQR)`（稀疏因子图上消元**外部**
   变量）→ `reduced->hessian(inside_ordering)` 得到**稠密正规矩阵** `(Λ, η)`（窗口内变量，
   列数 = `inside_columns`）→ `SelfAdjointEigenSolver(Λ)` 取特征值 > `rank_tolerance·max`
   的方向 → 紧凑块 `compact_i = √λ_i · v_iᵀ`（行数 = 保留方向数，列 = 全部窗口列，
   右端用 COD 伪逆求解）。块写为 `FactorKind::BoundaryPrior` /
   `RowRole::TrustedPrior` / `whitening_model_id = "frozen_graph_partial_qr_schur"`，
   并声明 `window.capabilities.includes_boundary_prior`。
2. **故障列不存在**：`boundary_graph` 只含名义因子，消元只对状态变量做，`(Λ, η)` 里没有
   任何故障方向 ⇒ 现路径**结构上不可能**产出 `T_b`/`F_b`/`d_perp`。A1 需要在
   `linearize()` 之前把**可监测故障模式的旧历元列**加入线性化（与
   `HypothesisGenerator` 的 mode 构造共用映射），并保证消元**只消状态、不消故障列**。
3. **平方根形式**：现在的做法是"稠密正规矩阵 + 特征分解"（与 §7.1 的要求"平方根/消元后算子"
   不一致）。A1 应改为对 `(Λ, η)` 做对称 Cholesky（窗口维度小，成本可控）得到 `R_b`，
   由交叉块得到 `T_b`，并由**被消元掉的残差行**构造 `F_b, d_perp`（`EliminateQR` 的
   multifrontal 因子图保留了这些行：需取 `reduced` 中与故障列相关的部分，而不是只取
   `hessian(inside_ordering)`）。
4. **检测自由度**：`DetectorOnlyRows` 基础设施（B1）与 `detector_only_rows` 计数已存在；
   `(F_b, d_perp)` 必须以"零状态列"的残差行进入窗口，`ν_⊥ = dim d_perp` 计入
   `ν_pooled = ν_c + ν_⊥`（§7.2 池化口径），否则会丢检测信息。
5. **版本/指纹绑定**：`tx.frozen_values`、`tx.base_version`、`tx.frozen_slots`、
   `boundary_graph` 是现成的绑定材料；摘要版本需并入 `FrozenWindowNumerics`
   指纹与 `StatisticalBoundKey`（否则重线性化/模式变化会复用旧摘要）。
6. **行级归属**：`window.slot_accounting` / `frozen_slots` 已有槽位身份校验
   （`every_active_factor_accounted_once`），摘要行归属可复用同一套 slot 身份；
   HIS-03 的核心断言（原始测量/IMU 样本 → 因子 → 摘要行）可直接建在其上。
7. **风险点**：(a) 故障列加入后 `hessian()` 的列数膨胀与稀疏性损失（需要按
   `inside ∪ fault` 的块结构取子块，避免全稠密）；(b) `EliminateQR` 的 multifrontal
   顺序决定 `F_b` 的行含义，多消元顺序 oracle 必须覆盖；(c) 消元与 fixed-lag 边缘化的
   **同一历元内顺序**（摘要先更新、再删除原始信息）需要在 `commitEpoch` 路径上加断言；
   (d) 池化检测 `T_pooled = ‖r_c‖² + κ_b` 进入检测即改变语义（历史信息首次入检测），
   必须与 P5/P6 的场景差异表逐条对比，不得用"无差异"掩盖。
