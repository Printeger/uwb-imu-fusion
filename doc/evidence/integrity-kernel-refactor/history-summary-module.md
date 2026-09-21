# C1-a 证据：历史消元摘要的模块级构造与 oracle 等价

状态：**C1-a DONE（模块级）**；C1-b（管线接入/生命周期/容量/冷启动/重线性化绑定）与
C1-c **NOT_RUN**；C2 NOT_RUN。
基线：`c6544d5`（P7）；代码提交：`bc5722c`（本轮代码提交，run_sha 口径）；证据提交 = 本轮
evidence commit。
冻结合同：`history-summary-design.md` §1–§2（表示、恒等式、符号约定）。
生产行为声明：本模块**没有任何生产调用点**；本轮未改检测器/PL/风险/阈值/配置/场景。

## 1. 交付范围

| 文件 | 角色 |
| --- | --- |
| `include/uwb_imu_pl/integrity/history_fault_summary.hpp` | 接口：输入块系统、(R_b,T_b,d_b,F_b,d_perp)、审计视图、符号约定函数、C1-b 映射注释 |
| `src/uwb_imu_pl/integrity/history_fault_summary.cpp` | 构造：两段 Householder 消元 + 退化处理 + 三角回代 |
| `test/test_history_fault_summary.cpp` | 独立 dense oracle（JacobiSVD 投影/伪逆/联合解）与 9 个用例 |
| `CMakeLists.txt` | 库源 + `catkin_add_gtest(test_history_fault_summary ...)` 注册 |

关键函数：`buildHistoryFaultSummary()`、`splitBlockSystem()`、
`HistoryFaultSummary::{omegaBoundary,xiBoundary,kappaBoundary,nuPerp,
boundaryShiftUsable,boundaryMeanShiftForFault,conditionalBoundaryMeanDelta}`。

## 2. 构造算法（只正交消元；无正规方程、无求逆）

输入：已白化的线性块系统 `‖H_o x_o + H_b x_b + A f − z‖²`（`H_o: m×n_o`、
`H_b: m×n_b`、`A: m×q`、`z: m`）。

1. **Stage A**：对 `H_o` 做 Householder QR（不动 `x_o` 之外的列），反射子左乘到
   `[H_o | H_b | A | z]`；`rank(H_o)=n_o` 时前 `n_o` 行即消元输出，剩余
   `m2 = m − n_o` 行构成残差系统 `[H_b2 | A2 | z2]`。
2. **Stage B**：对 `H_b2`（m2×n_b）做 Householder QR；变换后前
   `k = min(m2, n_b)` 行给出 `(R_b, T_b, d_b)`（`R_b` 上三角，原始 `x_b` 列序；
   `k < n_b` 时尾部零行补齐到 `n_b×n_b`），其下 `ν_⊥ = m2 − k` 行的 `x_b` 部分为
   结构零，直接给出 `(F_b, d_perp)`。
3. 行账本：**每一残差行恰好落入一个块**（边界三角或检测块），无丢弃；反射子从不
   作用于其左侧列（结构零区）。`z` 列直接进入 `d_b/d_perp`（与 §2 的块方程同号）。
4. 全程只做正交变换与矩阵乘；唯一的求解是符号约定函数的 `R_b` 上三角回代（非求逆）。
   秩裁决用无置换 Householder 的枢轴幅值
   （`pivot > rank_tolerance·max_pivot`，默认 1e-12；枢轴比 `min/max` 作为
   `old_state_pivot_ratio` 审计导出）。

## 3. 恒等式与符号约定

对一切 `(x_b, f)`：

```
min_{x_o} ‖H_o x_o + H_b x_b + A f − z‖²
    == ‖R_b x_b + T_b f − d_b‖² + ‖F_b f − d_perp‖²
```

**符号约定（§2 冻结，已测试）**：

* `boundaryMeanShiftForFault(f) = R_b⁻¹ T_b f`（正向注入响应）；
* `conditionalBoundaryMeanDelta(f) = −R_b⁻¹ T_b f`（条件边界均值相对边际之差）；
* 二者对同一 `(R_b,T_b,f)` 互为相反数；稠密对照：
  `conditionalBoundaryMeanDelta(f) == x_b*(f) − x_b*(0)`（联合最小二乘的 `x_b` 解之差）。

审计视图（按需重算，不参与求解）：`Ω_b = F_bᵀF_b`、`ξ_b = F_bᵀ d_perp`、
`κ_b = ‖d_perp‖²`、`ν_⊥ = dim d_perp`。

## 4. Oracle 与容差

Oracle 全部在测试内独立实现（薄 SVD 投影、伪逆、稠密联合解），不调用被测构造；
生产求解器零接触。容差按 ADR 0002：良态 1e-9、中度 1e-7；κ(H_o)=1e10 一档实测
（见下表，`shift` 一项 1.41e-7）**略超** 1e-7 中度档，故按 ADR 0002 的固定 1e-6
比较档**保守对照并 tagged**（枢轴比 9.44e-10，距 1e-12 秩门限约 3 个数量级——
不声称 near-gate 位置，仅标注比较档；也不声称中度档达标）。

oracle 表（`HistoryFaultSummary.HISM1OracleTable`，确定性种子，原文见
`raw/run_tests_c1a.log`）：

| case | m/n_o/n_b/q | κ(H_o) | 枢轴比 | 档 | tol | cost | schur | omega | kappa | shift | delta | ν 失配 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| small_well | 12/4/3/2 | 1e0 | 1.000 | well | 1e-9 | 4.14e-16 | 4.08e-16 | 2.23e-15 | 3.93e-16 | 1.13e-15 | 1.13e-15 | 0 |
| medium_well | 60/10/6/4 | 1e0 | 1.000 | well | 1e-9 | 2.08e-16 | 3.74e-16 | 7.76e-16 | 1.41e-16 | 4.67e-15 | 4.67e-15 | 0 |
| moderate_k1e4 | 40/8/5/3 | 1e4 | 2.05e-3 | moderate | 1e-7 | 1.02e-14 | 1.49e-14 | 6.17e-14 | 1.70e-14 | 5.33e-13 | 5.33e-13 | 0 |
| ill_k1e8 | 36/6/4/3 | 1e8 | 3.36e-8 | ill | 1e-7 | 5.79e-10 | 5.59e-10 | 4.95e-10 | 8.40e-11 | 5.65e-09 | 5.65e-09 | 0 |
| harsh_k1e10 | 30/5/3/2 | 1e10 | 9.44e-10 | harsh(tagged,1e-6) | 1e-6 | 9.10e-08 | 2.32e-08 | 9.91e-08 | 5.17e-08 | 1.41e-07 | 1.41e-07 | 0 |

表中每列为该档所有探针/恒等式的**最大相对误差**（`nu` 为整数失配）。
洞见：良态档误差在 binary64 舍入地板（~1e-16）；`shift/delta` 是误差上界项；
κ=1e8 档仍在 1e-7 档内留 ≥1.5 个数量级余量。

## 5. 拒绝用例与退化处理（不静默、不产出数值）

| 类别 | 输入 | 结果 |
| --- | --- | --- |
| 非有限 | `z` 含 NaN、`H_o` 含 inf | `invalid_reason = non_finite_input: ...`；输出矩阵全空；符号函数全 NaN |
| 形状不一致 | `H_b` 行数 ≠ m | `shape_mismatch: ...`（附各矩阵行数） |
| 空输入 | m=0（全空） | `empty_input: no rows (m=0)` |
| H_o 结构性秩亏 | 零列 / 重复列 / n_o>m | `h_o_rank_deficient: rank r < n_o ... pivot ratio ... `；保留裁决秩（如 2/3）供审计 |

已核实的**非拒绝**边界行为（冻结合同只要求 `H_o` 秩裁决）：

* **q=0（名义边界）**：`T_b` 为 `n_b×0`、`F_b` 零列、`Ω_b` 空、`ξ_b` 空；
  `κ_b == 最小联合残差能量`（与 oracle 一致到 1e-9 档）。
* **纯检测行**：小例（n_o=0，`H_b=[I₂;0]`，`A=[0,0,3,0]ᵀ`，`z=[1,2,4,9]ᵀ`）中
  两检测行的 f/z 内容精确进入 `F_b=[3;0]`、`d_perp=[4;9]`，`κ_b=97`，`ν_⊥=2`；
  恒等式通过。
* **边界秩亏（R_b 奇异）**：不拒绝——恒等式仍精确成立（f 响应部分落在边界块
  `T_b/d_b`，由代价恒等式承载）；导出 `rank_boundary`，`boundaryShiftUsable()` 为
  false 时符号函数返回 NaN。小例：`H_b=0`、`A=[1,0]ᵀ`、`z=[2,5]ᵀ` →
  `R_b=[0]`、`T_b=[[1]]`、`d_b=[2]`、`F_b=[[0]]`、`d_perp=[5]`、`κ_b=25`。
* `k=m2<n_b`（边界行不足）：`R_b/T_b/d_b` 尾部零行补齐到 `n_b×n_b`，恒等式保持。

决策记录：`Ω_b == Aᵀ(I−P_{[H_o H_b]})A`、`κ_b == 最小联合残差`、`ν_⊥ == m − rank`
这些**不变量仅在边界满秩时**以 F_b 单独核验；边界未满秩时由代价恒等式承载
（测试内同样按此口径 guard，避免把不成立的等式写成断言）。

## 6. 测试映射与计数

| 需求 | 测试 |
| --- | --- |
| 1) 代价恒等式（随机 (x_b,f)，1e-9 良好档） | `HISM1CostIdentity` |
| 2) Σp 等价（R_bᵀR_b == H_bᵀ(I−P_{H_o})H_b 与协方差） | `HISM1SchurEquivalence` |
| 3) G 响应 == 稠密条件均值响应；符号对偶互为相反数 | `HISM1BoundaryResponseSignDuality` |
| 4) Ω/κ/ν（独立投影/最小残差/行数） | `HISM1FaultGramKappaNu` |
| 5) ≥2 种消元顺序（行/列置换，含 x_o、x_b、f 列置换） | `HISM1EliminationOrderInvariance` |
| 6) 拒绝用例（非有限/秩亏/空/形状） | `HISM1DegenerateRejection` |
| 小例（纯检测行、q=0、边界秩亏） | `HISM1DetectorOnlyAndNominal` |
| 7) 中等规模合成 + 病态档 oracle 表 | `HISM1OracleTable` |
| 辅助恒等式（块拆分一致性） | `HISM1SplitHelperConsistency` |

命令与结果（运行日志 `raw/run_tests_c1a.log`、`raw/catkin_test_results_c1a.txt`）：

* `make -C build/uwb_imu_pl test_history_fault_summary` → 编译通过；
* `devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_history_fault_summary` →
  **9 tests / 9 PASSED / 0 FAILED**（含 oracle 表打印）；
* `catkin run_tests uwb_imu_pl` + `catkin_test_results` →
  **322 tests / 0 errors / 0 failures / 0 skipped**（= 304 + 2×9，catkin 对每个用例
  计 suite+case 两条）。

## 7. C1-b 输入清单（映射适配所需的真实块/账本字段）

把 `buildIntegrityWindow`（`src/uwb_imu_pl/estimation/incremental_estimator.cpp`，
边界段约 1187–1270 行）的块/账本映射到 `H_o/H_b/A/z` 所需的既有字段：

| 目标 | 来源（真实字段） | 说明 |
| --- | --- | --- |
| 行（已白化） | `LinearizedFactorBlock::{jacobian_whitened, residual_whitened}`；白化标识 `whitening_model_id`、`whitener` | 取 multifrontal `eliminatePartialMultifrontal(..., EliminateQR)` 归约后的**历史/被消元行**（不是 `hessian()` 正规矩阵） |
| 列拆分 | `LinearizedIntegrityWindow::state_layout`（`StateLayoutEntry::{keys,column_offset,dimension}`）+ `blocks[].window_column_indices` | `H_o` = 将压缩的旧状态列；`H_b` = 保留显式的分隔/边界列 |
| 故障列 | `blocks[].fault_units`；模式构造与 `HypothesisGenerator` 共用 | `A` 必须在 `linearize()` **之前**注入；只消状态、不消故障列 |
| `z` | `residual_whitened`（同批行） | 白化右端 |
| 行级归属（HIS-03） | `slot_accounting`（`FactorSlotAccounting::{slot,group_id,explicit_window_block,boundary_input}`）、`frozen_slots`、`every_active_factor_accounted_once` | 摘要行 → 因子 → 原始样本 |
| 版本绑定 | `FrozenWindowNumerics::{content_fingerprint,numerical_contract_fingerprint}`、`FrozenNumericalContract::policy_version`；`StatisticalBoundKey` 新增 `history_summary_version` | 缓存键失效（重线性化/模式改变不得复用旧摘要） |
| 检测自由度 | `square_root_detector_only_rows`（窗口计数） | `ν_⊥` 并入 `ν_pooled = ν_c + ν_⊥`（§7.2 池化口径） |
| 边界标识 | `WindowCapabilities::includes_boundary_prior`、`FactorKind::BoundaryPrior`、`RowRole::TrustedPrior` | 现路径（部分 QR + 特征分解）**结构上**产不出 `T_b/F_b/d_perp`，C1-b 需替换该段 |

诚实说明：当前实现仍是"稠密正规矩阵 + `SelfAdjointEigenSolver`"路径；C1-b 的替换涉及
与 fixed-lag 边缘化的时序（摘要先更新、再删原始信息）与行级归属断言，属下一轮范围。

## 8. 状态与诚实边界

* **C1-a：DONE（模块级）**——本文件 §3–§6 的恒等式/等价/多顺序/退化/拒绝全部 LOCKED_BY_TEST。
* **C1-b / C1-c：NOT_RUN**——管线接入、生命周期 §7.6 行 1–7、容量键
  （`history.*`，默认 `REFUSE`）、冷启动 `HISTORY_SUMMARY_INVALID`、重线性化绑定、
  跨边界新场景（HIS-02）、管线级 oracle（HIS-01，多消元顺序的**管线**对照）。
* **可选项未做**：C1-b 输入适配器骨架与开发运行器离线影子导出 **NOT_RUN**（本轮
  窗口内优先保证模块级闭环质量，避免半成品）。
* 模块级恒等式**不**构成任何管线等价/性能声明；`auto_shrink` 保持关闭不变。

## 9. 证据与存储纪律

* 新增：本文件、`proof-obligations.md §17`、`validation-manifest.json`（`HIS-M1`
  MAPPED；`HIS-01..06` 维持 NOT_STARTED 并更新 note）、`raw/run_tests_c1a.log`、
  `raw/catkin_test_results_c1a.txt`、`runbook.md §12`。
* `validation-report.json` 以代码提交 SHA 生成（`UWB_IMU_PL_VALIDATION_SHA=bc5722c`）。
* `hashes-C1C2.txt` 按机械规则最后生成（`git diff --name-only c6544d5..<code> ∪
  证据树`，排除自身）；本文件完成前不生成。不 push；不做任何格式化。
