# C2（M1）：分离残差布局、双通道 PL、fault-span 与模型误差通道

范围：roadmap §7.3–7.5 + §8.2 + `history-summary-design.md §8`。合同：现有阈值（0.25 gate、
128 上限、p_fa/p_md、alert limits、风险预算）**不动**；`formal_eligible=false`；不 push。

本轮里程碑：**M1 = C2**。M2（C3）/M3（C4）见 §7 状态与续跑起点。

---

## 1. §7.3 分离残差布局（已实现 + 已接线）

| 项 | 实现 | 测试 |
|---|---|---|
| 通道 1（当前系统残差，**含**历史边界因子） | `evaluateDualChannel`：`T_c = T_pooled − T_b`、`ν_c = ν_pooled − ν_⊥`（命名 `joint_window_state_supported`，不说「仅新数据」） | `DualChannelDetector.RealWindowSplitIsExactAndTablesTheDifference` |
| 通道 2（历史消元后残差，状态已消元） | `T_b = κ_b + constant_offset`、`ν_b = nu_perp`（命名 `history_eliminated_detector_only`） | 同上 |
| 联合接受 A={T_c≤τ_c}∩{T_b≤τ_b} | `joint_accepted`，各通道阈值用**未改动**的 `p_fa_per_test` 各自算 | 同上 + DET-02 分区界 |
| 池化聚合 | 保留为短序列参考实现（C1 的窗口统计量），不另发产品版本 | 同上（表内对照列） |
| 生产接线 | `DetectorResultV2` 新增 11 个字段（statistic/threshold/dof/accepted×2 + joint + split_valid + reason），由 `JointWindowDetector::evaluate` 填充；**不接受判定替代**原 pooled 判定 | 真实窗口测试断言检测器字段与模块一致 |

**精确性（真实窗口实测）**：`T_c + T_b = T_pooled`（rel ≤1e-9）、`ν_c + ν_b = ν_pooled`（整数恒等）。
**诚实记账**：两通道各用原预算 ⇒ 平台级上界 `operation_p_fa_upper_bound = min(1, 2·p_fa·horizon)`
（并集界），已随检测结果导出，不隐藏在通道里。

## 2. §7.4 双通道 PL（独立公式评审项）

公式（实现即定义，`dual_channel_detector.cpp`）：

$$
\lambda_{h,j}=f_h^\top\Gamma_{h,j}f_h,\qquad
\Lambda_{h,j}: F_{\chi^2_{\nu_j}(\Lambda)}(\tau_j)\le\beta_h,\qquad
W_h=\sum_j w_j\,\Gamma_{h,j}/\Lambda_{h,j}
$$

$$
\ker W_h\subseteq\ker G_h,\qquad
b_{h,d}=\sqrt{g_{h,d}\,W_h^{\dagger}\,g_{h,d}^\top},\qquad
L_{h,d}=b_{h,d}+k_{h,d}\sigma_d
$$

| 要求 | 实现事实 | 证据 |
|---|---|---|
| 每通道 Λ 复用 B3 保守侧求解 | `StatisticalBoundsCache::noncentralityBoundaryVerified(ν_j, τ_j, β_h)`；返回的 `residual` 即 `F(tau)−β`，测试逐通道断言 ≤0 | DET-02 `channel_lambda_gap ≤ 0` |
| 不显式求 W 逆 | 只累加 `W_h`，用其**细 SVD**（小矩阵）算 `b_d=‖Σ^{-1}V^\top g_d^\top‖`；秩截断=上面已认证的核 | DET-02 与显式 `W^{-1}` 形式逐位一致（1e-9） |
| `w_j>0, Σw_j=1` | 非正权重直接拒绝；权重在可用通道上归一化，`Σ=1`（断言 1e-12） | DET-02 |
| 无自由度/无故障贡献的通道不得凭空生成 Λ | 零 Gram 或 `ν_j≤0` 的通道被**丢弃**（不进入 Λ 列表）；若全部不可用 ⇒ 拒绝并给原因 | DET-02（丢弃与全拒两路） |
| 单有效通道退化为 `s√Λ` | 单通道时 `W=w Γ/Λ`，`b=√(gΓ^{†}g^\top)·√Λ`，与 B3 形式一致 | DET-02 oracle 比对 |
| `ker W ⊆ ker G` | 用 `W` 的 QR 取核基，断言 `‖G·ker‖ ≤ 1e-9·‖G‖`；违反即拒绝 | DET-02（两处拒绝路径） |
| 证书字段 | `certificate_id = "dual_channel_bound_w_" + FNV(detector_id/τ_j/ν_j/β_h/Λ_{h,j}/w_j/dim)` | DET-02 非空 + 字段一致 |
| 证明不依赖通道独立性 | 实现只做二次型界；DET-04 的联合概率用**显式标注的独立性模型**仅作量化，不作为证明 | DET-04 注释与输出 |

**公式评审结论（写入 proof-obligations §22）**：`b_{h,d}` 是 `W_h` 的二次型上确界型界，
在 `ker W_h ⊆ ker G_h` 下与单通道 B3 形式相容；权重归一化与通道丢弃不影响 `Σw_j=1`。

## 3. §7.5 fault-span 投影（已验证，管线采用「未投影残差」）

* `buildFaultSpanProjection`：取**声明内全部**方向的正交基（SVD 的 U，无 top-K、无按观测故障挑选）；
  声明秩亏**记录**（`declared_span_gap`）而非静默裁剪。
* `evaluateFaultSpanIdentity`：在全参数空间验证 `U(Z_b^\top Z_b)U^\top = F_{all}^\top F_{all}`；
  截断（top-K）基**必然失败**（DET-03 断言残差 > tol）。
* 管线现状：仍使用**未投影残差**（合同允许：「无法验证则用未投影残差并记录正确自由度」），
  自由度即 `ν_⊥ = nu_perp`（§1 精确恒等式）。声明/线性化/模式变化 → 需要重建（由 C1 的摘要
  指纹与版本分量守护）。

## 4. §8.2 模型误差通道

| 要求 | 实现 | 证据 |
|---|---|---|
| `τ_{j,risk}=(√τ_{j,actual}+ρ_{r,j})²` | `riskAdjustedThreshold`；`ρ=0` → **恒等** | DET-02（精确相等断言） |
| 逐轴 `ρ_{p,d}` | `DualChannelBoundRequest.position_rho_m` 随证书导出 | DET-02（`position_rho_m`） |
| 名义虚警用独立门限 | 通道阈值仍由原 `p_fa` 计算，与本项无关（ρ 只影响风险侧 τ） | 结构 |
| ρ 来源与状态入证书 | `rho_validated`/`rho_source`；未验证假设 ⇒ `model_error_validated=false` 且来源文本随证书 | DET-02（假设路径断言阈值已膨胀且被标记未验证） |
| ρ 不得编造 | 负值/非有限直接拒绝（返回 NaN / 拒绝） | DET-02 |

## 5. DET-04 定量对比（不报告「随时间单调」之类无条件结论）

合成双通道（ν_c=40、ν_b=12、p_fa=1e-4、单位灵敏度）实测表（`test_dual_channel_detector` 输出）：

| m | λ=m² | pooled 漏检 | joint 漏检 | 差 |
|---|---|---|---|---|
| 0 | 0 | 0.999900 | 1.000000 | +0.000100 |
| 2 | 4 | 0.997880 | 0.999998 | +0.002118 |
| 3 | 9 | 0.976724 | 0.999911 | +0.023187 |
| 4 | 16 | 0.834524 | 0.997096 | +0.162572 |
| 5 | 25 | 0.444415 | 0.958005 | +0.513590 |
| 6 | 36 | 0.094355 | 0.759377 | +0.665022 |
| 8 | 64 | 0.000077 | 0.115368 | +0.115290 |

**结论（定量、有界）**：在该配置下池化参考实现的漏检更低（即检出力更高），分离布局用检出
能力换取**逐通道与联合的可证明性**；差值随幅度非单调（6 → 8 时缩小）。
**不做**「分离总是更好/更差」或「随时间单调」的声明。

场景/布局差异表（检测布局变化 → 逐帧影响，真实窗口 epoch 18–22）：

| epoch | T_pooled | τ_pooled | ν_pooled | T_c/τ_c/ν_c | T_b/τ_b/ν_b | joint |
|---|---|---|---|---|---|---|
| 18 | 0 | 437.105 | 305 | 0 / 165.993 / 88 | 0 / 330.783 / 217 | 1 |
| 19 | 0 | 473.914 | 336 | 0 / 165.993 / 88 | 0 / 368.599 / 248 | 1 |
| 20 | 0 | 510.461 | 367 | 0 / 165.993 / 88 | 0 / 406.003 / 279 | 1 |
| 21 | 0 | 546.779 | 398 | 0 / 165.993 / 88 | 0 / 443.061 / 310 | 1 |
| 22 | 0 | 556.118 | 406 | 0 / 165.993 / 88 | 0 / 452.575 / 318 | 1 |

每条差异的保守方向：`ν_⊥` 增长只**增加**历史通道的接受阈值（更不易误报），
`ν_c` 固定 ⇒ 当前通道门限不变；两通道联合接受是**更严**的判据（不会让原本被拒的帧通过）。
本场景为无噪标称驱动 ⇒ 统计量为 0（无检测事件）；带故障的帧见 C1 的 F3 legacy 场景
（`κ_b=2.8e-29`、`ν_⊥=35` 等）。

## 6. 计数

| 量 | 值 |
|---|---|
| 新增模块 | `dual_channel_detector.{hpp,cpp}`（≈330 行） |
| 新增测试 | `test_dual_channel_detector.cpp`：DET-02/03/04 + 真实窗口切分 = **4 用例** |
| 检测器新增字段 | 11（`DetectorResultV2`，additive） |
| 全量测试 | **386 / 0 errors / 0 failures**（基线 378 + 新增 4 用例×2）；日志 `raw/run_tests_c2c3c4_baseline.log` |
| validation | `UWB_IMU_PL_VALIDATION_SHA=66b8eb6 --all` → **44 PASS / 0 FAIL / 9 NOT_RUN**（DET-01..04/CH 全 PASS）；日志 `raw/validation_c2c3c4.log` |
| 提交 | M1 checkpoint `wip(C-round): M1` = `66b8eb6`；证据提交见 `hashes-C1C2.txt` 头部 |

## 7. 里程碑状态与续跑起点

| 里程碑 | 状态 | 说明 |
|---|---|---|
| M1 = C2 | **完成**（§7.3 布局 + 接线、§7.4 公式 + 证书、§7.5 投影模块、§8.2 ρ 通道、DET-02..04） | 未采用的管线项已记录：fault-span 投影（合同允许用未投影残差）、ρ 进入风险账本的执行（模块 + 证书状态已就绪） |
| M2 = C3 | **未开始** | 续跑起点：`profile likelihood J_h^profile`、`GuaranteeGroup`、桥接决策序、IMU 条件修复；触点：`hypothesis_evidence.cpp`（Γ/score 组装）、`fde_manager.*`、`reinitialization.*`；验收 FDE-03..05 |
| M3 = C4 | **未开始** | 续跑起点：提交绑定表 + 状态机 + 时间/坐标系复用规则；触点：`integrity_monitor.cpp`（发布路径）、`run_logger.cpp`、`types.hpp`（`AttemptDiagnostics`）；验收 OUT-01..03 |

未开始原因：本执行窗口在完成 M1（实现 + 4 用例 + 全量回归 + 证据）后不足以再完成一个
「实现 + 完整验证」的里程碑；按纪律不提交半成品。
