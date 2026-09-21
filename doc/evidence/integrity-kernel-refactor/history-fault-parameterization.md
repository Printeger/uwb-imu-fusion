# C1-b 解阻证据：Part A 历史故障参数化子系统 + Part B 表示抽取原型与定路

状态：**A、B 全绿并可复现**（代码提交 `1f05f11`；测试 328 → **346/0**，日志
`raw/run_tests_c1b_ab.log`）。Part C（管线接线）**未执行**（§6 就绪清单）；
D-1/D-2/D-3 逐条映射见 §5。生产行为零变化（无调用点；配置默认值更新见 §3）。

## 1. Part A：历史故障参数化（D-1 粒度 + A2 一致性 + A3 视野）

模块：`include/uwb_imu_pl/integrity/history_fault_parameterization.hpp` +
`src/.../history_fault_parameterization.cpp`。材料单一来源 =
`EpochTransaction::recoverable_history`（`HistoricalEpochContext`：前后状态、
preintegration、raw IMU slice、UWB batch、selected groups）；不改既有图、不触碰
`UnrecoverableHistory` 组。

**粒度（D-1 冻结，实现即此）**

| 源 | 列 | 行口径 | 值 |
|---|---|---|---|
| UWB | `UwbAnchorConstant`，每 (anchor, 历史历元) | 组 `source_measurements` 序 | 1（anchor 行） |
| UWB | `UwbAnchorTimeLinear`，同粒度伴生列 | 同上 | `t_meas − t_epoch_begin` |
| IMU | `ImuAxisConstant`，每 (轴 0..5, 历史历元) | 15 维 IMU 组 | `buildAnalytic` 解析灵敏度（与窗口生成器同路径） |
| 双故障 | **不存列**（单源列的组合） | — | — |

白化一次：`whitened = W·raw`，`W = L⁻¹`（`covariance = L Lᵀ`，
`whitenerFromCovariance`），与 `linearizePendingGroup` 约定逐字一致。

**注入机制**：`buildHistoricalFaultInjection()` 在冻结线性化点对历史组的因子
`linearize()`、按行堆叠、追加故障列（`gtsam::Symbol('f', i)`，白化单位），产出
重建的 `JacobianFactor`（states ∪ fault keys）；消元只消状态键——测试
`InjectionKeepsFaultKeysInSeparator` 断言：故障键在 separator 中存活、状态键被消、
原材料未变，且二阶差分误差恢复 `2‖a_i‖²`（列范数保持，`worst_norm_rel=0.0`）。

**A2 语义一致性（实测）**

* 窗口历元（生成器路径）vs 历史历元（子系统路径），同一材料：
  * UWB anchor 常数列：**逐位一致**（`norm(diff) = 0.0`，`uwb_exact=1`）；
  * IMU 六轴：同一解析路径，`worst_imu_rel = 0.0`；生成器的 mode 元数据
    （parameter_dimension=1、onset_epoch=发起历元）在测试中锁定。
* 独立 oracle：全部 160 条 UWB 列 vs 测试内独立重算的选择模式
  `worst = 0.0`；60 条 IMU 列的白化/原始往返 `worst_rel = 8.78e-17`。
* ramp 组合：`t − t_onset = (t − t_begin_e) + (t_begin_e − t_onset)`，以
  {时间线性列系数 1，常数列系数 `begin_e − onset`} 构造；逐行恒等式断言
  `≤ 1e-12`（二进制重排级）。persistent = 常数列系数 1 的精确组合。
* 测试映射：`HistoryFaultParameterization.{PlanCoversRecoverableHorizonWithoutOverclaiming,
  ColumnsMatchIndependentSelectionOracle, WindowAndHistoryColumnsAgreeOnSharedMaterial,
  RampCombinationIsExactOverTheStepBasis, InjectionKeepsFaultKeysInSeparator,
  CapacityRefusesWithoutTruncation}`。

**A3 视野**：覆盖范围 = `[oldest_recoverable_epoch + 1, window_first_epoch)`
（边界历元本身无记录，严格排除；`source` 记录派生/显式配置）。视野外 → 省略，
`omitted_epoch_count`、`material_gap_epoch_count` 如实计数；
`validityAssumptions()` 产出规范文本（含覆盖区间、omitted、material gap、
"no detection or protection claim"），`omittedRiskSource()` 产出风险账本
omitted 来源 id（`history_fault_omitted:[a,b) count=n`）。测试断言：显式放宽视野
不得声称覆盖缺失材料（gap=3 场景）、`claimsFullCoverage()=false`。
生产侧写入 identity/账本属 Part C。

## 2. Part A4：q_hist 实测与容量默认（D-2）

实测（默认研究配置，26 历元、fixed-lag=200 未触发边缘化，可恢复跨度 10/20）：
`[HFP-TABLE] q_hist=220 epochs_observed=10 omitted=6 gaps=0 skipped=0
first=6 window_first=16`。理论上界：跨度 ≤ `epochs + recovery_margin_epochs` = 20
历元 × 22 列/历元（8 anchors × 2 基 + 6 轴）= **440** ⇒ 默认容量 **512**（余量 ~16%），
`capacity_action=REFUSE`。已写入 `integrity_config.hpp` 默认值与
`config/realtime_uwb_imu_pl_research.yaml`，测试
`IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded` 锁定（缺节=512/512/512/REFUSE）。
超限语义（构造器层，未接线）：`evaluateHistoryFaultCapacity()` → `fits=false`,
`unusable=true`, `reason="HISTORY_CAPACITY_EXCEEDED"`（与
`FailureReason::HistoryCapacityExceeded` 字符串一致）；RESET/STOP_PROTECTED 仅显式
动作且同样先置不可用；**不存在截断 API**（绝不静默丢最旧故障）。

## 3. Part B：表示抽取原型与定路（D-3）

模块：`include/uwb_imu_pl/integrity/history_summary_extraction.hpp` + cpp。

**候选对比（全部实测，测试 `HistorySummaryExtraction.*`）**

| 路线 | 内容 | 数值等价 | κ/检测内容 | 秩亏 | 键身份 |
|---|---|---|---|---|---|
| **(i) 选定**：线性化行抽取 + 模块正交消元 | `boundary_graph.linearize()` 的行 [H\|z] → `buildHistoryFaultSummary`（先消 x_o） | 合成：`rel=0.0`（κ、d_b、R_bᵀR_b 全等）；真实窗口：`rel_to_reduced_information=2.9e-16`，κ vs 稠密最小二乘 oracle `rel=6.6e-47` | **保留**（κ=8.39e-1 精确复现） | 自然支持（rank_boundary/ν 正确） | 有（keys + 每键 `column_begin/key_dim`） |
| (i-b) 归约因子直取 | 对 `eliminatePartialMultifrontal(..., EliminateQR)` 的输出抽行 | `rel=6.0e-16`（正规内容） | **丢失**：部分被消因子的常数被 GTSAM 丢弃（κ=0 vs 8.39e-1） | 支持 | 有 |
| (ii) §11.3 Cholesky | `(Λ, η)` → `R=chol(Λ)`，`y=R⁻ᵀη` | `rel=6.0e-16` | **丢失**（同常数问题） | **拒绝**（LLT 失败→invalid，不伪造因子） | 无（无键身份） |

**定路（B4）**：选定 **(i) 线性化行 + 模块消元**。理由：唯一同时满足"无特征分解
截断、κ/ν_⊥ 检测内容完整、键身份完整、秩亏可裁决"的路线；与 C1-a 模块的
"只正交消元、平方根形式"合同直接对接。残留风险（记录在案）：
① 需要对完整稠密边界行做事后消元——成熟窗口代价需实测并按稀疏/多前端口径
再设计（本轮为原型：590×390 Householder 在测试内 ~0.2s 量级）；
② 键到列的**切空间维度展开**（x/b:6, v:3）是原型中发现并修正的要点（否则
150 维信息矩阵 vs 30 列摘要错配）——已写入 `ExtractedBoundaryRows.column_begin/key_dim`；
③ `(i-b)` 常数丢失是**实测结论**（非猜测），设计 §11.3 关于"归约保留被消元残差行"
的说法只在"因子被整体消元"时成立，Part C 不得以归约因子作为 κ 的唯一来源。

**B2 硬性**：无任何特征分解截断（真实窗口 rows=590, columns=390, **rank=390 全秩保留**）；
`ν_⊥` 正确（真实窗口 200；秩亏合成例 rank_boundary=1 < n_b，ν=1）；do/f/rank 进审计
（`ExtractedBoundaryRows.rank/constant_energy`）。

## 4. 关键实测数字（原文见 `raw/run_tests_c1b_ab.log`）

```
[HFP] mature26 q_hist=220 epochs=10 omitted=6 gaps=0 skipped=0
[HFP] oracle checked_uwb=160 worst=0.0 ; checked_imu=60 worst_rel=8.78e-17
[HFP] window_vs_history uwb_exact=1 imu_axes=6 worst_imu_rel=0.0
[HFP] injection epoch=15 columns=16 fault_keys=16 worst_norm_rel=0.0
[HSE] synthetic rel_selected=0.0 kappa_direct=8.390771e-01 kappa_reduced=0.0 kappa_route_ii=0.0
[HSE] rank_deficient rank_boundary=1 nu_perp=1 route_ii=invalid
[HSE] real_window rows=590 columns=390 rank=390 rel_information=2.895e-16
      kappa=9.392229e-32 kappa_oracle=9.392229e-32 kappa_rel=6.569e-47 nu_perp=200
```

## 5. D-1 / D-2 / D-3 逐条映射

| 决策 | 实现（文件+函数） | 测试 |
|---|---|---|
| D-1 UWB 每 (anchor, 历元) 步列；IMU 每 (轴, 历元) 区间常值列 | `buildHistoricalUwbColumnsForEpoch` / `buildHistoricalImuColumnsForEpoch` | `ColumnsMatchIndependentSelectionOracle` |
| D-1 persistent/ramp = 步列精确线性组合（构造性 T；零残差） | `persistentUwbCombination` / `rampUwbCombination`（时间线性伴生基） | `RampCombinationIsExactOverTheStepBasis` |
| D-1 双故障不存列 | 列 id 单源（类型级） | 同上（组合即双故障路径） |
| D-1 注入：故障键不被消元、材料重建、不改既有图/不触 Unrecoverable | `buildHistoricalFaultInjection` | `InjectionKeepsFaultKeysInSeparator` |
| D-2 实测 q_hist → 默认容量；REFUSE；不截断 | `HistoryCapacityConfig` 默认 512；`evaluateHistoryFaultCapacity` | `CapacityRefusesWithoutTruncation` + config 测试 |
| D-3 抽取路线定路 | `extractBoundaryRows`（选定）/`extractBoundaryRowsFromInformation`（对照） | `HistorySummaryExtraction.*`（3 例） |
| A3 视野/不声称 | `HistoryFaultHorizon` / `Plan::{validityAssumptions,omittedRiskSource,claimsFullCoverage}` | `PlanCoversRecoverableHorizonWithoutOverclaiming` |
| A2 窗口-历史一致性 | 同材料双路径 | `WindowAndHistoryColumnsAgreeOnSharedMaterial` |

## 6. Part C 就绪清单（未执行；下一轮执行序）

1. **接线点**：`buildIntegrityWindow` 边界段（`incremental_estimator.cpp:1187-1271`）：
   用 `extractBoundaryRows(linearized boundary_graph)` → 计划列（A）注入 →
   `buildHistoryFaultSummary` → `(R_b,d_b)` 替换特征分解块、`T_b` 为故障响应块、
   `(F_b,d_perp)` 经 `DetectorOnlyRows` 进窗口（`hpp` 头注 C1-b 映射）。
2. **顺序断言（B2）**：摘要先于 fixed-lag 删除；用 `commitEpoch` 的 frozen 材料
   加断言/钩子 + "更新前被删则失败"测试。
3. **池化检测（B4）**：`joint_window_detector.cpp:84,113-115,147-149`、
   `rank_update_kernel.cpp:569,720-731`、`hypothesis_evidence.cpp:505-518,540`
   （插桩点已定位）；κ_b/ν_⊥/Γ 历史块导出。
4. **指纹/缓存（B5）**：`FrozenWindowNumerics` 指纹 + `StatisticalBoundKey.history_summary_version`
   （字段已就绪）；`digestHistorySummaryVersion` 填入。
5. **冷启动（B6）**：产出 `HISTORY_SUMMARY_INVALID`（枚举已存在，生产者随接线）。
6. **重基线（§5 清单）**：`test_realtime_incremental.cpp:966,1042-1046`、
   `test_integrity_v2.cpp` rank/dof 恒等式、`tools/context_oracle.py:13,102-111`
   O8a/O8d、`integrity_monitor.cpp:1376-1395` identity 口径、诊断 v15。
7. **诊断 v15**：`run_logger.cpp:349,362,709`、`tools/validate_run_schema.py:388-393`、
   `tools/gate_d_diagnostics.py:32,69,130,149`、`test_fault_manifest.cpp:316,335`。
8. **新场景（B8/HIS-02）**：`config/r0_r1_development_scenarios.yaml` 增
   `fault_epoch_begin/end` 跨边界场景 + 可监测性断言。
9. **生命周期（C1c-C1）**：§7.6 行 1–7 以本模块计划/容量为基座逐行实现+测试。

## 7. 存储/命令/证据

* 测试（全量落盘）：`catkin run_tests uwb_imu_pl` → **346 tests / 0 errors / 0 failures**
  （= 328 + 2×9 新用例）；日志 `raw/run_tests_c1b_ab.log`（基线 + 两次全量 + 结果行）。
* 新证据：本文件；`validation-manifest.json` 新增 `HIS-X1`/`HIS-X2`；
  `proof-obligations.md §19`；`runbook.md §14`；`code-map.md` 附录更新；
  `config-migration.md §6`（默认值更新为 D-2 实测）。
* 不 push；无格式化；hashes 机械规则最后生成（base `e4d1f10` → 代码 `1f05f11` ∪ 证据树）。
