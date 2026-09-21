# C1 收口：历史摘要管线接线、池化检测、生命周期与重基线（C1-b + C1-c 剩余）

范围：`history-summary-design.md`（合同）＋ `history-fault-parameterization.md §6`（就绪清单）。
本轮只做 C1；C2（分离残差布局 / §7.4 W 双通道 / fault-span）、C3（选择合同 / 保证组 / IMU 条件修复）、
C4（原子发布）、性能优化均**未触碰**。零阈值/合同改动；`formal_eligible=false`；不 push。

提交：`wip(C1): pipeline wiring WIP - 6 failures pending`（`de7e474`）→ `wip(C1): pipeline wiring green`
（`0ada13f`）→ 正式代码提交 **`9257941`**（两个 WIP 已 reset --soft 合并，base `9288f73`）；
证据提交见 `hashes-C1C2.txt` 头部。

**结果**：全量 **376 tests / 0 errors / 0 failures**（`raw/run_tests_c1pipeline.log`）；
validation `UWB_IMU_PL_VALIDATION_SHA=9257941 … --all` → **40 PASS / 0 FAIL / 12 NOT_RUN**
（HIS-01..06、HIS-M1/C1/V1/X1/X2 全 PASS；`raw/validation_c1pipeline.log`）；
hashes-C1C2 **147/147 OK**（机械规则最后生成）。

---

## 1. 接线映射（就绪清单第 1 项）

| 设计环节 | 实现位置 | 测试 |
|---|---|---|
| 边界段取线性化行（路线 (i)） | `IncrementalUwbImuEstimator::buildIntegrityWindow`，`src/uwb_imu_pl/estimation/incremental_estimator.cpp`：按 **factor group** 收集 boundary inputs（`boundary_groups`），每个测量型因子的 `linearize()` 行**原样**进入增广系统（行身份保留 ⇒ 每 anchor 故障映射可对齐） | `HistorySummaryPipeline.BoundaryIsTheSquareRootHistorySummary` |
| 信息型因子（边缘化 container） | 同上：`LinearContainerFactor` 线性化为 `HessianFactor`，用 `info().selfadjointView()` 取 `[Λ,η;ηᵀ,f]`，Cholesky 得行 `[R | c]`（`RᵀR=Λ`、`c=R⁻ᵀη`），**常数 `f-cᵀc` 单独累加**，绝不静默丢弃 | `UwbImuIncremental.OneSecondUwbDropAndChangingAnchorSetRecover` |
| 历史故障列注入（Part A） | `planHistoryFaultParameterization` → `buildHistoricalFaultInjection`（fault key `Symbol('f',i)` 进 separator，仅消状态键） | `HistoryFaultParameterization.*`（模块）＋ pipeline 测试 1/4 |
| 摘要构造与替换特征分解块 | `extractBoundaryRows(augmented)` → `buildHistoryFaultSummary` → 边界块 `[R_b; 0] / [0 | d_perp]`，`whitening_model_id="history_summary_sqrt_d1"` | 测试 1、2 |
| (R_b,d_b) 替换、T_b 故障响应块、(F_b,d_perp) 经 DetectorOnlyRows | `window.history_summary.response / detector_response / d_perp`（载体），`detector_only` 行为零 Jacobian 行 ⇒ 由既有 square-root context 分类 | 测试 2（池化恒等式）、测试 4（Γ） |
| 行级归属与 `slot_accounting` 对齐（HIS-03） | `window.slot_accounting`（explicit XOR boundary，指针身份校验）＋ 边界块不进 ledger 组（`group_id = tx.id*1000`） | `HistorySummaryPipeline.RowAttributionExplicitXorBoundary` |
| 与主估计器已边缘化先验不重复计入 | 边界行只来自 `frozen_slots`（= 活跃图槽位，指针身份逐槽校验）；container 因子本身是活跃图槽位，故计一次 | 同上（唯一性）＋ 测试 2（dof 加性） |

**无覆盖缩水**：边界段仍是「活跃冻结因子集合」的精确覆盖（`slot_accounting.size()==frozen_slots.size()`，
`every_active_factor_accounted_once`），只是把原来的「特征分解丢弃」换成「平方根摘要保留」。

## 2. 顺序断言（第 2 项）

`buildIntegrityWindow` 在 horizon 内经元缺少记录时，扫描边界组是否仍持有该 epoch 的因子：

* 持有 ⇒ 「fixed-lag 删除先于摘要更新」⇒ 窗口显式不可用，`history.state = HISTORY_SUMMARY_INVALID`，
  reason 含 `deleted before the summary update`；
* 不持有 ⇒ 该 epoch 本就无材料，按其位置跳过并计数（A3 诚实性）。

测试：`HistorySummaryPipeline.DeletedMaterialFailsBeforeSummaryUpdate`（含 state 断言）。

## 3. 池化检测（第 3 项，§7.2）

* `T_pooled = ‖r_c‖² + κ_b(+constant_offset)`：summary 的 detector-only 行是**零 Jacobian 行**，其残差能量
  即 κ_b，自动进入窗口统计量；carrier 导出 `kappa_b`、`constant_offset`（信息型因子不可由行表达的常数部分）、
  `omega()=F_bᵀF_b`、`xi()=F_bᵀd_perp`。
* `ν_pooled = ν_c + ν_⊥`：`ν_⊥ = nu_perp` 由 detector-only 行数给出（rank 不计），dof 加性由测试锁定。
* 每假设 Γ 历史块：模式在窗口行空间的映射 = `[T_b f; F_b f]`（`history.response` / `detector_response`
  按组合系数叠加），Γ = DᵀD − DᵀH(HᵀH)⁻¹HᵀD ⇒ 因 detector-only 部分与 H 的行空间正交，Γ ≥ ‖F_b f‖² > 0。

测量（测试 4，mature window）：`Γ > 0` 且有限；`‖T_b f‖ > 0`（跨边界后响应仍在）。

## 4. 指纹/缓存绑定（第 4 项）

`integrityWindowFingerprint` 现纳入摘要身份：present/valid/capacity_ok、`version_digest`、四个版本分量、
计数（fault_columns/boundary_rows/emitted_rows/horizon/window_first/nu_perp/omitted/gap）、κ_b、
constant_offset、列身份三元组、`response/detector_response/d_perp` 矩阵、假设与风险文本。

失效语义：`JointWindowDetector::evaluate` / `rank_update_kernel` / `hypothesis_evidence` /
`protection_level_v2` 均对比 `numerics->content_fingerprint` 与该指纹，不匹配即 `numerically_valid=false`
且 reason 含 `stale`。测试：`HistorySummaryPipeline.SummaryVersionBindsWindowFingerprint`。

## 5. 冷启动（第 5 项）

显式状态（`HistorySummaryState` → `window.history_summary.state` 字符串）：

| 状态 | 触发 |
|---|---|
| `HISTORY_SUMMARY_NOT_REQUIRED` | 边界无凝聚材料（`boundary_groups` 空） |
| `HISTORY_SUMMARY_VALID` | 摘要构建并绑定成功 |
| `HISTORY_SUMMARY_INVALID` | 冷启动/材料先删（horizon 内有材料但摘要无法建立） |
| `HISTORY_SUMMARY_BUILD_INVALID` | 模块拒绝（抽取/消元/注入失败） |
| `HISTORY_CAPACITY_EXCEEDED` | §7.6 行 7 容量 REFUSE |

**禁止从 nominal marginal 反推**：不可用路径全部 fail-closed（窗口 `model_valid=false`），
没有任何「用名义边际补出检测内容」的分支（详见第 6 项代码审视图）。
测试：`DeletedMaterialFailsBeforeSummaryUpdate`（state=INVALID）、`CapacityRefusalIsExplicitAndCounted`
（state=HISTORY_CAPACITY_EXCEEDED、无截断、计数 `history_capacity_refusals`）。

## 6. 重基线表（第 6 项；「改了什么/为什么/新期望/容差依据」）

| # | 位置 | 改了什么 | 为什么 | 新期望 | 容差依据 |
|---|---|---|---|---|---|
| R1 | `test_realtime_incremental.cpp` F3 legacy dof 断言 | `dof == counts[e]+counts[e-1]` → `dof == counts[e]+counts[e-1] + diagnostics.history_summary.nu_perp` | 池化口径：边界由「特征分解块（rank 15、dof 贡献 0）」变为「平方根摘要 + detector-only 行」。detector-only 行是真实残差自由度（ν_⊥），必须显式计入而不是被吸收 | epoch0/1：8、14（与旧值相同，ν_⊥=0）；epoch2/3：41=10+31、47=12+35 | **整数精确恒等**，无 slack；逐 epoch 实测 ν_⊥ 见第 10 项 |
| R2 | `integrity_window_snapshot.cpp` 指纹 | 新增摘要身份哈希（第 4 项） | 只哈希行会把「响应/检测内容」排除在缓存身份之外（静默别名） | 同一行、不同摘要版本 ⇒ 指纹不同 | 布尔/位级比较（`SummaryVersionBindsWindowFingerprint`） |
| R3 | `history_fault_parameterization.cpp` 注入 | 键块宽度：按 `getA(iterator).cols()` 逐键拷贝（原实现每键 1 列） | 导航状态键是 6/3 维切空间块；每键 1 列会截断状态映射并破坏 separator 代数（旧实现的静默 UB） | 注入后 `state_keys` 块宽 6/3；摘要列宽与窗口一致 | 模块级 oracle 未变（`worst_*≈0`），管线 dof 恒等 R1 成立 |
| R4 | 边界线性化路径 | `boundary_graph.linearize()`（可能给出 Hessian）→ 逐因子行 + container 的 Cholesky 行 | 行身份必须保留（fault map 依赖行身份）；同时 `HessianFactor` 的常数在行形式下不可表达 ⇒ 显式累加 `constant_offset` | 窗口有效（原为「reduced factor is not a JacobianFactor」拒绝） | 结构/精确：窗口有效 + 池化恒等式 |
| R5 | 未改动 | rank/dof 恒等式、分解计数、context_oracle O8a/O8d、`boundary_summary_id`/`validity_assumptions` | 见下 | 保持 | — |

**R5 逐条说明（本轮如实状态）**

* rank/dof 恒等式（`window.dof = rows − rank`）：**未改**；池化语义只改变 rows/rank 的来源（R1），恒等式本身被测试 2 直接断言。
* 分解计数：`NumericalWorkCounters` 新增 8 个 history 计数（builds/boundary_rows/input_columns/fault_columns/
  emitted_rows/perp_rows/capacity_refusals/summary_invalid），既有计数语义未改。
* `context_oracle O8a/O8d`：**未重跑**（需 raw dump；本轮无 `/tmp` 运行数据）——见第 11 项诚实缺口。
* `identity` 口径：`boundary_summary_id` 仍为 `partial_qr_schur_boundary`（**未改**，因为该 id 描述的是
  *旧* 边界构造），新边界块用 `whitening_model_id="history_summary_sqrt_d1"` 标记；
  `validity_assumptions="history_nominal_boundary_only"` 仍正确（摘要不声称 horizon 外覆盖）。
  这两条属于「口径未变且仍然正确」，不是遗漏。

## 7. 诊断 v15（第 7 项）

`AttemptDiagnostics.history_summary`（`common/types.hpp`，**只加字段**）：present/valid/capacity_ok/state/reason、
version_digest、fault_columns、boundary_rows/emitted_rows/boundary_columns、rank_boundary、nu_perp、kappa_b、
constant_offset、omega_trace、xi_norm、information_form_factors、injected_epochs、horizon_first/window_first、
omitted/material_gap、claims_full_coverage、assumptions、omitted_risk_source。填充点：
`integrity_monitor.cpp`（`output.window_id` 之后），直接读窗口载体，不做二次计算。

## 8. 新场景（第 8 项）

* **管线级（已交付）**：故障在 horizon 内起始、跨窗口左边界持续 ⇒ 摘要携带响应与检测内容，
  Γ>0、`‖T_b f‖>0`（测试 4）；F3 legacy 场景（丢 UWB＋换锚点集）跨 4 个 epoch，历史列注入 q=22/18。
* **r0_r1 dev-runner 原始流场景（未交付）**：`config/r0_r1_development_scenarios.yaml` 的场景名被
  `apps/r0_r1_development.cpp` 白名单硬校验，新增「早注入、跨边缘化边界持续」场景需要同时扩展 runner 语义
  （`fault_epoch_begin/end` 对单锚点持续偏置的组合、persistent 组合、raw dump）＋跑 evidence 运行；
  本轮预算内**不交付**（避免落到「无人执行的空条目」）。**见第 11 项缺口**。

## 9. 生命周期（第 9 项，§7.6 行 1–7）

| 行 | 规则 | 实现 | 测试 |
|---|---|---|---|
| 1 | 未来 onset 才允许零列 | `futureOnsetAllowsZeroColumns` | `HistorySummaryLifecycle.FutureOnsetZeroColumnsRule` |
| 2 | 模式入史：响应+检测代价一起消元、保 event/时间/概率 | `absorbHistoryMode` | `ModeAbsorptionKeepsResponseAndCostTogether` |
| 3 | 源离窗不删模式 | `sourceLeavingWindowKeepsMode` | `SourceLeavingWindowKeepsMode` |
| 4 | 故障结束≠清除 | `evaluateHistoryClearance` | `FaultEndIsNotClearance` |
| 5 | 淘汰需无影响证明/包络/风险或清洁重置 | `evaluateHistoryRetirement` | `RetirementRequiresProofEnvelopeAndRisk` |
| 6 | order/family 热切换默认拒绝 | `evaluateHistoryHotSwitch` | `HotSwitchDefaultsToRefuse` |
| 7/8 | 容量 REFUSE 不截断；`auto_shrink` 保持关闭 | `evaluateHistoryFaultCapacity`（管线执行）＋ `autoShrinkEnabled()==false` | `CapacityRefusalIsExplicitAndCounted`、`ExplicitStatesAndAutoShrink` |

管线消费点：容量 → 窗口不可用 + 计数（测试 6）；冷启动 → `HISTORY_SUMMARY_INVALID`（第 5 项）；
`auto_shrink`：仓库无设置点（grep 0 命中）⇒ vacuous-safe；order/family 运行时切换：`IntegrityConfig`
在估计器构造后不可变，管线内没有模式集变更入口 ⇒ 结构上只能「不变或重建」（与生命周期判据一致，如实记录）。

## 10. 证据表（第 10 项）与计数、代价

**10.1 场景差异表（历史进入检测导致的行为变化）**

| 场景/帧 | 变化 | 解释 |
|---|---|---|
| F3 legacy（epoch 0/1，A3 horizon 空） | dof 8、14（不变） | 无历史故障列 ⇒ ν_⊥=0，行为与旧口径一致 |
| F3 legacy（epoch 2/3，horizon 开） | dof 10→41、12→47 | ν_⊥=31/35 detector-only 行进入池化 dof（设计语义，非回归） |
| F3 legacy（原先 `batch_committed=false`） | 现为 true，`marginalization_count=2` | container 因子从「拒绝」变为「Cholesky 行 + 显式常数」（R4 修复） |
| C ep26 / G 帧（既有断言） | **无行为变化**（本套件全绿） | 那些帧的窗口不含历史注入（q=0）或已按 A3 诚实性标记；如后续出现差异需在重基线表登记 |

**10.2 历史故障可监测性变化表**

| 量 | 旧边界（特征分解） | 新摘要（路线 (i)） |
|---|---|---|
| 故障响应 T_b | 不存（丢弃） | 保留（跨边界后 Γ>0、‖T_b f‖>0） |
| 检测内容 F_b/d_perp | 不存 | 保留（κ_b、ν_⊥、Ω_b、ξ） |
| κ 常数 | 丢弃（reduced->hessian()） | 行空间 κ_b + 信息型 offset 显式 |
| 秩 | 固定 15（截断） | 无截断（精确 rank_boundary） |

**10.3 计数与边界段代价（实测，`UWB_IMU_PL_C1P_DIAG=1`，原始行见 `raw/c1pipeline_diag.log`）**

| 场景/epoch | rows/rank/dof | boundary_rows/emitted | ν_⊥ | rank_b | κ_b | offset | info_form | q | inj_epochs | boundary_ms | assembly_ms |
|---|---|---|---|---|---|---|---|---|---|---|---|
| F3 tx1 | 38/30/8 | 15/15 | 0 | 15 | 0 | 0 | 0 | 0 | 0 | 0.248 | 0.009 |
| F3 tx2 | 44/30/14 | 30/15 | 0 | 15 | 0 | 0 | 0 | 0 | 0 | 0.105 | 0.004 |
| F3 tx3 | 71/30/41 | 76/46 | 31 | 15 | 6.1e-33 | 0 | 0 | 22 | 1 | 0.392 | 0.005 |
| F3 tx4 | 77/30/47 | 80/50 | 35 | 15 | 2.8e-29 | -8.6e-49 | 1 | 18 | 1 | 0.404 | 0.007 |
| 成熟窗（10 历元，epoch22） | 571/165/406 | 513/333 | 318 | 15 | 6.2e-32 | 0 | 0 | 220 | 10 | 30.7–38.0 | 0.15–0.25 |

**代价记录（只测量，不优化）**：成熟窗的边界段（线性化行抽取 + 注入 + 模块消元 + 载体装配）
实测 **≈31–38 ms/窗**（两次捕获），而同一次构建的稠密装配仅 ≈0.15–0.25 ms：
边界段现在是该窗口构建的主导成本，量级与旧 dev-runner 单帧 30–45 ms 相当，
因此**接线后实时性预算需要单独核账**（这是测量结论，不在本轮优化范围）。
早期无历史列的窗口（F3 tx1/tx2）边界段仅 0.1–0.25 ms。计数器
（`NumericalWorkCounters`：builds/boundary_rows/input_columns/fault_columns/emitted_rows/
perp_rows/capacity_refusals/summary_invalid）同步记录。

**10.4 摘要 oracle 表**：模块级多消元顺序不变量与真实窗口等价见 `history-fault-parameterization.md §3`
（HIS-M1/HIS-X2）；本轮管线级等价由测试 1（边界块 == 摘要）与测试 2（池化恒等式）给出。

## 11. 诚实缺口（NOT_RUN / 未交付）

1. **r0_r1 原始流新场景未执行**（第 8 项后半）：原因见 §8；管线级语义已由测试 4 与 F3 覆盖。
2. **context_oracle O8a/O8d 未重跑**：需要 raw dump 环境（本轮无 `/tmp` 运行数据），按 §6 R5 记录。
3. **管线级「多消元顺序」oracle**未单独成测：模块级已覆盖（HIS-M1），管线级只做了单一顺序的等价与恒等式。
4. **信息型因子常数 offset 的可观测量**：offset 已导出并可非零（实测 -8.6e-49），但其对统计量的影响
   尚未在场景级量化（当前量级 ≪ 噪声；已在诊断中可见，留待有 raw 数据的轮次核账）。
