# square-root-context（P3 / B1，2026-09-21）

工作包 B 第一批：**一个冻结窗口一个平方根数值上下文**，detector / hypothesis evidence /
PL / 候选 KEEP 全部从同一上下文取值；被替代的重复在线分解降级为「证书不满足时的参考回退」。
本轮**不做**：分组包络与覆盖图（B2）、风险并集界与惰性 FDE（B3/B4）、历史摘要（C）、
真实性能收口（D）。**零阈值/算法语义改动**（0.25 gate、128 动作上限、p_fa/p_md、
alert limits、风险预算、rank/cond/残差门限全部不变）；`formal_eligible` 保持 `false`。

## 1. 上下文设计（`FrozenSquareRootContext`）

源码：`include/uwb_imu_pl/estimation/square_root_context.hpp` +
`src/uwb_imu_pl/estimation/square_root_context.cpp`；由 `finalizeIntegrityWindow`
在同一冻结快照上构建一次，存于 `LinearizedIntegrityWindow::square_root`。

约定（与 proof-obligations 第 3/4/5/7 项一致）：

```
A  = (H · S) · Pmat                     (m x n，列尺度 S + 列置换 Pmat)
A  = Q · R                              (Householder QR，只保留 R 与隐式 Q)
C~ = C · S · Pmat                       (受保护映射同步同一 S/Pmat)
dx = S · Pmat · x~,   R x~ = Q1^T z
(H^T H)^-1 = S P R^-1 R^-T P^T S
Y = Q1^T D,  Z = Q2^T D,  G = C~ R^-1 Y,  Gamma = Z^T Z (审计量)
Sigma_p = U U^T,  U = C~ R^-1
```

* **不形成**完整 `Q`、`Q2`、`P` 或 `I − HPHᵀ`：`Q^T` 通过 Householder 序列隐式作用于
  右端（`applyQt` 按字典块批量应用）。
* **尺度 S 与置换 Π 有物理依据且被记录**：默认策略 `unit + natural`（冻结列已完成
  白化与物理归一，列序沿用状态布局顺序，便于与 P2 基线逐位对照）；`column_norm`
  与 `column_pivot` 策略已实现并有专门测试（NUM-01），census 帧的 fill-in 观测
  （满窗 0、G 缺口帧 11）表明当前口径下无需稀疏重排。**S 与 Π 必须同步作用于 C**，
  并由测试锁定（只置换 H 忘掉 C 的错误会被 NUM-01 直接判 FAIL）。
* **零状态行（DetectorOnlyRows）**：H 行全零而 z/故障响应非零时，该行保留在残差、
  自由度与 Z 响应中（不做行删减）。`detector_only_rows` 作为符号分析结果缓存并导出。
* **可估计性**：秩亏时返回显式 `STATE_OUTPUT_UNOBSERVABLE`，`usable()=false`，
  `informationSolve` 返回空矩阵——**不用小正则化假装满秩**（NUM-03）。
* **SymbolicQR adapter**：本轮**未启用** Eigen SparseQR（H 为稠密组装、满窗 fill-in
  已是 0，无收益；不升级 GTSAM）。符号分析以 pattern 指纹缓存（`SquareRootSymbolicPlan`：
  detector-only 行 + 按 nnz 的列序），pattern 变化必然重分析、数值变化不得复用旧 R
  （NUM-04 用命中/未命中计数与 R 差异断言锁定）。

## 2. 证书与回退语义

每窗口低成本证书（`SquareRootCertificate`，全部导出）：`full_column_rank`、
`‖RᵀR − AᵀA‖/‖AᵀA‖`、parity 与 SVD 参考的相对差、解与 SVD 参考的相对差、
`condition_estimate × ‖∇(½‖Ax−z‖²)‖` 前向误差代理。**不使用「每 N 帧全量 SVD」替代中间帧检查**：
rank/cond/残差仍是每帧 SVD（原语义），QR 证书独立复核。

* 证书通过 → 消费者使用上下文（计数 `square_root_certificate_holds`）。
* 证书不通过或上下文缺失 → 走 `solveFrozenInformation(numerics, …)` 的 LLT/谱参考回退，
  **计数**（`square_root_fallbacks`），绝不静默混用。
* 判据不放宽：`model_valid` 仍要求 `rank==cols && dof>0 && cond≤1e10 && 残差≤1e-7`。

## 3. 消费者切换与删除/降级清单

| 消费者 | 之前 | 现在 | 说明 |
|---|---|---|---|
| 名义解 + parity/statistic（detector 入口） | BDCSVD 解 + `z−Hx̂` | 上下文 `R⁻¹Q₁ᵀz`、`Q₂ᵀz`（SVD 值保留为证书参考与回退） | `numerics->spectral_state_increment` 仅作参考 |
| hypothesis evidence 的 Σp/G/Γ 求解（`evaluateContiguous`、`modes.empty()` 分支） | `(HᵀH)⁻¹` 经 LLT（+谱回退） | 上下文 `informationSolve`（R 三角回代） | 数值等价由场景重放 + oracle 验证 |
| `evaluateMapped`（共享上下文关闭时的参考路径） | LLT | **保持** LLT（显式标记为参考路径） | 与 `enable_shared_context=false` 的对照测试继续有效 |
| 候选 KEEP：块缓存与内层交叉 | `solveFrozenInformation(*numerics,…)` | 上下文 solve | `base.state_increment` 现由上下文提供 |
| 每假设可监测性 | 仅 Gram 特征/SVD + LDLT | **新增** Z_h=Q₂ᵀD_h 的小型 SVD 分类（full/harmless/dangerous/indistinguishable）作为审计与 B2 输入；**本轮不改变任何决策** | Gram 与斜率仍取共享正规方程形式，保证与 P2 逐位一致 |

删除/降级：原有的「LLT 正规方程 + 谱回退」**不再位于主路径**（计数从 429 次/92,262 列
降到 15 次/1,695 列），仅保留为证书不满足时的参考回退。

## 4. 计数表（P2 → B1，409 帧场景流：A/C/D/E/F/G/H）

| 计数器 | P2 | B1 | 说明 |
|---|---|---|---|
| `covariance_rhs_solves` / `_columns` | 429 / 92,262 | **15 / 1,695** | 原 LLT 求解路径 |
| `spectral_rhs_solves` / `_columns` | 429 / 92,262 | **15 / 1,695** | 原谱回退路径 |
| `square_root_factorizations` | 0 | 421 | 每窗口一次 QR（含重算窗口） |
| `square_root_information_solves` / `_columns` | 0 | 414 / 90,567 | 由 R 服务的 (HᵀH)⁻¹ 求解 |
| `square_root_qt_applications` / `_columns` | 0 | 406 / 89,274 | 隐式 Qᵀ 应用（D 矩阵与 z） |
| `square_root_symbolic_hits` / `_misses` | 0 | 339 / 82 | pattern 级符号复用 |
| `square_root_certificate_holds` | 0 | 406 | 窗口证书通过数 |
| `square_root_fallbacks` | 0 | 15 | 证书不通过 → 参考回退 |

即 **98.2% 的信息求解与隐式 Qᵀ 应用已由同一平方根上下文承担**；剩余 3.5% 为 G 场景
拒绝帧（上下文不可用，显式回退并计数）。

## 5. 等价证据

### 5.1 场景重放（P2 基线 vs B1，逐帧逐列）

| 场景 | 文件数 | 离散差异 | 路线差异 | 元数据差异 | 超容差数值项 |
|---|---|---|---|---|---|
| A_nominal | 19 | **0** | 0 | 1500（schema/bytes） | 0 |
| C_uwb_fde | 19 | **0** | 1（`fallback_reason`） | 2717 | 0 |
| D_imu_bridge | 19 | **0** | 0 | 1701 | 0 |
| E_union | 19 | **0** | 0 | 1701 | 0 |
| F_ramp_unmonitorable | 19 | **0** | 0 | 1500 | 0 |
| G_continuous_rejection | 19 | **0** | 0 | 1686 | 0 |
| H_mature_union | 19 | **0** | 0 | 11866 | 0 |

* 容差口径（ADR 0002 + 日志 6 位有效数字）：相对 1e-5，近零量abs 1e-9 地板。
* 最大数值差：`hypotheses.csv conditioned_statistic` 相对 2.8e-4（**绝对 1.14e-11**，
  量级 1e-8 的中间量），未跨任何阈值；其余所有列相对差 ≤1e-5。
* 唯一路线差异：C 场景 1 个候选的 `fallback_reason` 由 `unstable normal solve;residual
  energy cancellation` 变为 `residual energy cancellation`（求解改由 R 承担后不再报
  LLT 不稳定），**决策列（valid/selected/statistic/PL）逐位一致**。
* 元数据差异仅两类：诊断 schema v11→v12（本轮新增列/新表）与
  `context_bytes`（审计字段增加导致共享结构体变大 3464→3992 等）。
* 逐帧明细：`/tmp/uwb_imu_pl_b1_20260921/equivalence_final.json`（不随仓库保存）。
* 说明：P2 基线 CSV 在 prune 时移出仓库（见 `prune-log.md`），复现命令：先用
  `runbook.md §5` 的 P2 命令重建基线（需切回旧构建），或直接以本文件的分级容差作为
  回归判据。

### 5.2 上下文 oracle（独立 numpy，`tools/context_oracle.py`）

5 个保留 fixture（A30 / C25 / C26 / G10 / H201）× O8a–O8f：**30 PASS / 0 FAIL / 0 NOT_RUN**
（rank/dof、R 对角与条件估计、parity 统计量、detector-only 行、‖Q₂ᵀz‖²==‖z−Hx̂‖²、
Σp=C(HᵀH)⁻¹Cᵀ 的有限性与 PSD）。结果：`square-root-oracle.json`。
每假设的 G/Z/Γ/斜率与 Λ 求解**不能**从 replay 架构重建（D_h 是生成器状态，不在 bin 内）：
该项在 oracle 中显式标注 NOT_RUN，由 `oracle_compare.py` 的 O5/O6（生产斜率、
λ* 闭包）与 C++ 测试 NUM-01..04 / COV-02 覆盖。

### 5.3 上下文证书统计（v12 导出，`diagnostic_square_root.csv`）

| 场景 | 行数 | usable | max cond | max 前向界 | max parity 相对差 | max 解相对差 |
|---|---|---|---|---|---|---|
| A_nominal | 30 | 30 | 3.7e4 | 3.8e-10 | 3.5e-21 | 2.2e-14 |
| C_uwb_fde | 30 | 30 | 3.7e4 | 5.9e-08 | 3.5e-21 | 3.1e-12 |
| D_imu_bridge | 30 | 30 | 2.4e5 | 2.5e-08 | 1.2e-15 | 1.4e-11 |
| E_union | 30 | 30 | 2.4e5 | 2.1e-08 | 1.2e-15 | 1.4e-11 |
| F_ramp_unmonitorable | 30 | 30 | 3.7e4 | 3.8e-10 | 3.5e-21 | 2.2e-14 |
| G_continuous_rejection | 45 | **30** | 1.8e5 | 2.1e-07 | 4.5e-16 | 2.7e-12 |
| H_mature_union | 226 | 226 | 2.0e5 | 3.0e-08 | 4.2e-15 | 2.6e-11 |

G 场景 15 个拒绝帧上下文不可用 → 回退（与计数表 15 次 fallback 一致）。

### 5.4 每假设 Z 响应分类（v12 `hypotheses.csv` 新列）

35,128 行分类为 full-rank；1,650 行未评估（G 拒绝窗口无上下文）。
harmless / dangerous / numerically-indistinguishable 三类在 7 个场景流中**未出现**
（流形条件良好）；三态判据本身由 `SquareRootContext.ClassifiesDetectionResponseTriState`
单元测试锁定（含危险零空间与容差带内两种情形）。

### 5.5 测试

`catkin run_tests uwb_imu_pl` → **254 tests / 0 errors / 0 failures**
（P2 为 240；+14 = 2×(NUM-01、NUM-02、NUM-03、NUM-03b、NUM-04、COV-02、三态分类)）。
新增 `test_square_root_context.cpp`（7 用例）。日志：`raw/run_tests_p2.log`（P2）、
本轮 `run_tests_b1.log`。

### 5.6 计时（诚实观测，不做 40ms 声明）

A 场景中位数：`hypothesis_evidence` 5.35→5.69 ms（+6.4%，每假设 Z 小 SVD 的开销）；
`window_fingerprint` 1.29→3.02 ms（该桶现在包含每窗口一次 QR 构建，约 1.7 ms）；
`window_svd` 8.23→7.92 ms。总体每帧增量约 +2 ms。

## 6. 本轮明确未做 / NOT_RUN

| 项 | 状态 | 理由 |
|---|---|---|
| 平方根内核（Cholesky 因子 L⁻¹、单一数值表示） | NOT_RUN | B1 只统一「求解与 Qᵀ 应用」；SVD 仍保留为 rank/cond 权威（与 roadmap B1 的“上下文”口径一致，真正的内核替换属 B1 后续/性能收口） |
| 分组包络与覆盖图（G5/B2） | NOT_RUN | 按本轮范围排除 |
| 风险并集界 §5.9（B3）、惰性 FDE（B4） | NOT_RUN | 按本轮范围排除 |
| 历史摘要 T_b/F_b（C） | NOT_RUN | 仅在上下文接口预留（`history_lineage_id` 等占位） |
| Eigen SparseQR adapter | NOT_RUN（决策性） | 无稀疏收益，不升级 GTSAM；见 §1 |
| 真实性能收口（D） | NOT_RUN | 只给观测计数与中位数 |

## 7. 存储纪律（本轮）

`raw/` 98MB → **7.5MB**；`doc/evidence/integrity-kernel-refactor/` 总 8.3MB
（≤20MB；单文件 ≤1MB）。被删 692 个文件的 sha256 与再生命令见 `prune-log.md`；
`results/`（1.2GB）未删除，评估见 `storage-inventory.md`。本轮所有大产物写
`/tmp/uwb_imu_pl_b1_20260921/`，仅摘要入库。
