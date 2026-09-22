# 证明义务清单（A4 冻结版）

本文件把路线图里“数学正确性”相关的未闭合项固定成可追踪的证明义务。每条给出：
**命题 → 本轮状态 → 证据 → 阻塞与归属工作包**。状态词表：

* `LOCKED_BY_TEST`：已有独立实现/对照测试锁定，改动需先改测试与本文档。
* `EVIDENCE_ONLY`：本轮只有运行证据或口径事实，尚无独立证明。
* `DEFERRED`：明确推迟到 B/C/D 阶段，本轮不实现。

---

## 1. §7.4 双通道 PL 公式（检测通道 vs 保护通道）

* **命题**：保护级必须由“检测接受事件的分区界”构造，不能对两个检验量复用同一个
  χ² 公式；保护通道的多重性由 max-over-hypotheses 给出。
* **本轮状态**：`EVIDENCE_ONLY`。oracle O6 逐行复核了失败斜率与非中心度边界
  `P(χ²'_ν(λ) ≤ τ) = p_md·allocation` 的口径（`oracle-results.json`），
  `integrity.csv` 的 `pl_*` 数值未在本轮重建（其组合口径需 B 阶段配合 §7.4 冻结文档）。
* **证据**：`oracle-results.json`（O6：64 行、worst rel vs CSV 1.5e-6、闭包
  |P_miss(λ*)−p_md| ≤ 2.8e-8）；`raw/runs_p2/*/integrity.csv`。
* **阻塞/归属**：B（需要在接口层固定“哪个灵敏度集合、哪个 λ* 口径、哪个轴尾概率”）。

## 2. §5.9 正常帧风险的联合箱式并集界

* **命题**：“正常帧”风险必须与故障假设风险以联合接受事件分区（union bound over
  boxes）闭合，而不是各通道独立相加后再做单通道近似。
* **本轮状态**：`DEFERRED`。manifest 的 `projection_eligibility` 是结构字段、动作
  不得改变数值可取性（`FaultManifest.ProjectionEligibilityIsIndependentFromActions`），
  这是该界成立的前提之一；界本身未实现。
* **证据**：`config/integrity_fault_manifest.yaml`、`validation-manifest.json`（GEO-05/DET-02 未开始）。
* **阻塞/归属**：B/C。

## 3. λ / Λ / τ 的平方约定

* **命题**：生产 `noncentrality_boundary` 是**非中心度** Λ（χ²′ 参数），失败斜率以
  `slope·√Λ` 进入 PL；`protected_slopes` 列存的是 √(gᵀΓ⁻¹g)；`λ=√Λ` 的写法和
  “λ²=Λ”在文档与代码间必须一致。
* **本轮状态**：`LOCKED_BY_TEST`。
* **证据**：
  * `ReferenceFixture.TwoRowScalarLeastSquares`（s²=1/2、Γ=1/2、斜率 √(gᵀΓ⁻¹g) 约定守卫）；
  * `oracle-results.json` O6（scipy `ncx2.cdf(τ, ν, Λ)=p_md` 反解与导出值逐行一致）；
  * `integrity_monitor.cpp` 中 `slope·sqrt(noncentrality_boundary)` 的组合为证据行。
* **阻塞/归属**：无。**P3/B1 更新**：`SquareRootContext.COV02_AuxiliarySubsetProjectionIdentity`
  独立验证 `Z^T Z == D^T (I − H(HᵀH)⁻¹Hᵀ)D`（≤1e-12）与子集 Gamma 形式；`NUM01` 验证
  S/Π/C 同步；oracle O6 在 5 个保留 fixture 上复核 λ* 反解与闭包。

## 4. 三条查询路径的等价性

* **命题**：`snapshotSensitivity`（t=0 快照）、`conditionalSensitivity`、窗口全量
  `H`（历史路径）三条查询在同一批数据上给出同一有限斜率/Z/Γ。
* **本轮状态**：`EVIDENCE_ONLY`（部分）。`ReferenceFixture.FrozenBlockWhiteningAndAggregateAssembly`
  证明冻结块白化后的行拼接严格等于全量 `H`（abs ≤ 1e-12），oracle O3 在 8 个真实
  窗口上复核同一恒等式；三路径的**数值等价**未做逐点对照。
* **证据**：`oracle-results.json`（O3 全 PASS）、`fixtures/frames.json`。
* **阻塞/归属**：B。**P3/B1 更新**：信息求解已统一到同一上下文（`evaluateMapped` 保留为
  参考对照路径），场景重放 7×19 文件离散零差异、数值在分级容差内（`equivalence-summary.json`）；
  三路径的**逐点数值等价**仍待 B2 用同一 D_h 做对照。

## 5. C = [0 | R] 的有限差分锁定（D1 声明相关）

* **命题**：受保护量是机体系原点世界位置 ⇒ `C = protected_state_map` 的前三行在三
  维切空间惯例 `gtsam_pose3_local_rotation_then_body_translation` 下应等于 `[0|R]`；
  任何杠杆臂项都必须被判出。
* **本轮状态**：`LOCKED_BY_TEST`（D1）。`ReferenceFixture.ProtectedMapIsBodyOriginFiniteDifference`
  用中心差分 `C·(δ/h)` 对照 `C` 行，容差 1e-7，并对“含杠杆臂的 C 行”断言失效；
  探针实验（`gtsam::Pose3::retract`：∂t/∂ω=0、∂t/∂υ=R）作为独立佐证。
* **证据**：`test/test_integrity_reference.cpp`；`raw/runs_p2/*/diagnostic_snapshot_identity.csv`
  （`position_reference=body_origin`、`tangent_convention=gtsam_pose3_...`）。
* **阻塞/归属**：无。**P3/B1 更新**：NUM-01 专门覆盖 “C 必须与 H 同步缩放/置换” 的陷阱
  （用未同步的 C 得到的 Σp 会被判 FAIL）。

## 6. IMU 解析灵敏度 vs 重积分 oracle

* **命题**：IMU 故障子空间/灵敏度的解析 Jacobian 必须与“在原始 IMU 样本上叠加故障、
  重积分、再线性化”的数值 oracle 在多个步长下一致（方向、量纲、bias/坐标约定）。
* **本轮状态**：`LOCKED_BY_TEST`（A4-11）。新增
  `IntegrityV2ImuOracle.FiniteDifferenceSweepMatchesAnalyticAcrossStepSizes`：
  5 个 eps、12 次重积分/eps（共 60 次），全有限且 worst ≤ 2e-4；环境变量
  `UWB_IMU_PL_IMU_FD_ORACLE` 的运行路径导出 sweep 列到 v11 诊断。
* **证据**：`raw/run_tests_p2.log`；`raw/runs_p2/*/diagnostic_attempts.csv`
  （`oracle_sweep_*` 列）。
* **阻塞/归属**：无（B 阶段改积分器须重跑）。

## 7. 白化约定（whitened = W·raw 一次）

* **命题**：`H`/`z` 只做一次白化；块级 `J_whitened = W·J_raw`；聚合 H 精确等于各块
  行拼接；不得在历史路径二次白化。
* **本轮状态**：`LOCKED_BY_TEST`。`ReferenceFixture.FrozenBlockWhiteningAndAggregateAssembly`；
  oracle O3 在真实窗口逐块复核。
* **证据**：`oracle-results.json`（O3：assembly/whitening max rel 全为 0 级）。
* **阻塞/归属**：无。

## 8. 历史摘要等价性（Σp/G/Γ/df/代价）

* **命题**：用摘要替代完整历史时，`Σp`、`G`、`Γ`、自由度、代价必须等价，且必须
  记录摘要区间与消失的行；不得静默遗忘。
* **本轮状态**：`DEFERRED`（C 阶段 HIS-01..06）。本轮仅冻结接口：v11 identity
  的 `history_lineage_id`、`boundary_summary_id`、`coverage_epoch` 已就位
  （当前 `NOT_AVAILABLE_IN_SCHEMA`，诚实占位）。
* **证据**：`diagnostic_snapshot_identity.csv`、本文档第 8 项。
* **阻塞/归属**：C。**P3/B1 更新**：上下文接口已预留 `history_lineage_id` /
  `boundary_summary_id` / `coverage_epoch`（v12 identity 表），摘要语义仍未实现。

## 9. Post-FDE 选择风险

* **命题**：FDE 之后的“选择”本身消耗风险；恢复发布必须附带 post-selection 证明，
  否则保持不可用；已排除集不允许悄悄复用。
* **本轮状态**：`EVIDENCE_ONLY`。`C_uwb_fde` 帧导出了动作前后两个冻结窗口
  （attempt 25/26，含 128 动作上限帧），`fde_status=SUCCESS_KEEP_ALL`；
  post-selection 证明未实现（`formal_eligible=false` 保持）。
* **证据**：`raw/replay_p2/C_uwb_fde/exports`、`raw/runs_p2/C_uwb_fde/integrity.csv`。
* **阻塞/归属**：C（FDE-02/03）。

## 10. 0.25 步长门限的物理尺度局限

* **命题**：`max_linearization_step_norm = 0.25` 是混合单位欧氏范数，不能声称是物理
  尺度线性化误差界；G 帧（连续拒绝）暴露了该局限。
* **本轮状态**：`EVIDENCE_ONLY`。配置保留原值与判定（仅追加迁移注释）；
  `G_continuous_rejection` 场景在 epoch 10 导出了拒绝流窗口。
* **证据**：`config-migration.md` §1.3、`raw/runs_p2/G_continuous_rejection`。
* **阻塞/归属**：B/C（物理尺度诊断与门限重设）。

## 11. 128 动作上限口径

* **命题**：候选动作生成存在 `max_candidate_count = 128` 的硬上限（
  `hypothesis_generator.hpp`、`integrity_config.hpp`），任何“全字典”结论必须声明
  该口径，不能用截断后的动作集冒充完备性。
* **本轮状态**：`EVIDENCE_ONLY`。C 帧 attempt 26 的冻结窗口恰有 128 个动作；
  K=20 census 口径的 624 假设/61104 动作组合属基准口径，不作为形成性结论。
* **证据**：`raw/replay_p2/C_uwb_fde/exports/attempt-26.bin`、`census.json`。
* **阻塞/归属**：B/D（性能与完备性口径）。

## 12. Gate J 证据缺口

* **命题**：`formal_eligible=true` 需要 Gate J（校准与独立复核）证据；在此之前所有
  运行必须 `formal_eligible=false` / `IMPLEMENTED_UNVERIFIED`。
* **本轮状态**：`DEFERRED`（保持 false，符合 D4）。`integrity.csv` 中
  `reason` 明确写出 `IMPLEMENTED_UNVERIFIED: Gate J calibration and independent review pending`。
* **证据**：`raw/runs_p2/A_nominal/integrity.csv`（`formal_eligible=0`）。
* **阻塞/归属**：D（真机校准、独立复核、真实 bag）。

---

## 附：本轮新增的可追踪对照（非证明义务，但支撑上面各项）

| 项 | 工具/测试 | 结果 |
|---|---|---|
| 8 窗口 × 8 项独立 oracle | `tools/oracle_compare.py` → `oracle-results.json` | 57 PASS / 0 FAIL / 7 NOT_RUN |
| 解析 fixture（含反例 A/B/C） | `test_integrity_reference` | 8/8 PASS |
| manifest/迁移/分类学/identity/v11 | `test_fault_manifest`、`test_integrity_config` | 全部 PASS（`raw/run_tests_p2.log`） |
| 验证调度与清单 | `tools/run_validation.py`、`validation-manifest.json`、`validation-report.json` | 13 PASS / 0 FAIL / 28 NOT_RUN（未开始项均标注归属） |

## 13.（P3 新增）每假设 Z 响应三态与决策延后

* **命题**：可监测性应来自检测空间响应 `Z_h = Q₂ᵀD_h` 的小型分解（秩揭示），并在
  `ker Z_h ⊆ ker G_h` 时区分「结构性无害零空间」（可给有限界）与「危险零空间」（不可用）；
  数值无法分辨时走参考回退或不可用。
* **本轮状态**：实现为**审计层**（`z_rank/z_sigma_min/z_condition/z_classification` 已导出），
  **不改变任何决策**：数值 Gram 与斜率仍取共享正规方程形式，以保证与 P2 逐位一致
  （`equivalence-summary.json` 离散零差异）。三态逻辑由
  `SquareRootContext.ClassifiesDetectionResponseTriState` 锁定；7 个场景流中
  35,128 行均为 full-rank，未出现 harmless/dangerous/indistinguishable。
* **遗留**：把 harmless 分支接到「有限保守界」（GEO-04 语义）会改变退化情形的离散结果，
  须与 B2/B3 的风险账本一起做，并作为显式的行为变更重跑等价证据。
* **阻塞/归属**：B2/B3。

## 14.（P4/B2 新增）单双故障同 registry、紧凑模式与覆盖包络

* **命题（§5.7）**：order=1 与 order=2 必须共用同一引擎与同一解；双故障交叉项只对
  被引用的模式对计算，且拼接前必须证明参数块结构独立；共享参数/共享方向/不适用族
  一律 fail-closed（不产出 Gamma/斜率，导出原因）。
  * **本轮状态**：`IMPLEMENTED`。`pairFamilySupport` + `hypothesisParametersIndependent`
    在 `evaluateContiguous` 的每假设入口执行；拒绝时 `monitored=false`、
    `plausible=false`、`z_classification=3` 并导出 reason。
  * **证据**：`B2Registry.*`（4 例）、场景计数（`fault_cross_blocks` 按需）、
    `equivalence-summary.json.B2_stage3`（离散零差异）。
* **命题（R2 紧凑计算）**：`HᵀA`、score 与 Gram 交叉项必须从 mode 的**实际非零
  factor blocks** 构建；逐模式 padded 分配不得进入热路径；容量上限须显式且可计数。
  * **本轮状态**：`IMPLEMENTED`。紧凑路径 `mode_dense_allocations=0`（7 场景）；
    回退路径逐模式计数；紧凑与回退**逐位相同**。
  * **证据**：`B2Compact.*`（4 例）、`coverage-envelopes.md` §4/§5。
* **命题（§5.8）**：完整 plausible set 必须被精确枚举或由**通过包含性证明与支配性
  验证**的包络覆盖；覆盖不完备时受保护输出不可用（fail-closed，不许静默）。
  * **本轮状态**：`IMPLEMENTED`（在线为精确遍历；分组包络为可选容量路径，接受时
    打 `UPPER_ENVELOPE`，否则保留 `EXACT`）。
  * **证据**：`B2Coverage.*`（6 例）、oracle O8g–O8i、`coverage-envelopes.md`。
* **遗留/边界**：
  * 包络在线启用会改变导出标签，属显式行为变更，留待与 B3 风险并集界一起做
    （`NOT_RUN`，见 `coverage-envelopes.md` §7）。
  * 时间耗尽的在线提前停止（D）与 K>ν 专项（GEO-03）仍未开始。
  * 危险零空间的有限保守界（GEO-04 语义）仍由包络支配性义务承担，未接入风险账本。

## 15.（P5/B3+B4 新增）§5.9 联合界、风险账本、零空间决定、覆盖证书与惰性 FDE

* **命题（§5.9）**：`P(∃d: |e_d|>L_{h,d}, accept|h) ≤ max(Σ_d α_{h,d}, β_h)`，且每轴尾概率必须
  与账本付费一致。
  * **本轮状态**：`IMPLEMENTED`（每轴等分 `α_h/3`；`Λ` 取保守侧端点；三路径共用同一 helper）。
  * **证据**：`B3Risk.RSK01*`、`B3Risk.RSK03*`、`B3Risk.Boundary*`、`risk-and-lazy-fde.md` §1。
  * **差异**：PL 单侧上升（最大 +0.29 m），7 场景离散差异 0。
* **命题（账本）**：各项有来源与状态，无隐藏零项；未验证项只阻断 formal 资格。
  * **本轮状态**：`IMPLEMENTED`（10 项；attempt v14 导出）。
  * **证据**：`B3Risk.RSK02*`；C 帧 charged 4.0e-5 / declared 5.004e-5。
  * **缺口**：完整 §5.9 计费（miss 通道）未入账，需预算重标定（显式记录）。
* **命题（零空间）**：危险零空间不可用且 reason 指向轴与模式；无害零空间允许有限投影界。
  * **本轮状态**：`IMPLEMENTED`（分类 1/2/3/4 均落到决定；分类 1 行为与 P4 逐帧一致）。
  * **证据**：`B3ZeroSpace.*`（2 例）。
* **命题（覆盖证书）**：发布输出携带覆盖证书，且不绕过历史有效性 gate。
  * **本轮状态**：`IMPLEMENTED`（v14 列 + 叶级标签；在线仍精确遍历）。
* **命题（B4）**：健康普通帧不构造动作实体/桥接块，PL 灵敏度完整；惰性化不破坏健康状态机。
  * **本轮状态**：`IMPLEMENTED`（`ensureActionEntities` 幂等；计数为证）。
  * **证据**：`B4LazyFde.*`；A_nominal 四个计数（0/0/0/5930）与证据路径计数（30/420）。

## 16.（P6/C 包 Stage 0）支配性单侧化与 step gate 归因

* **命题（支配性）**：包络接受必须单侧——`ratio = B_env/B_leaf >= 1−ε`，ε 只吸收共享恒等式的
  binary64 噪声（实测 ~1e-16，取 1e-9，留 7 个数量级余量）；`dominance_margin` 为**原始
  `ratio−1`**（接受时 `>= -ε`），`dominance_ratio` 存同一最小值供审计。
  **P6 更正**：本条曾声称"接受时 margin >= 0 且已实现"，实际实现未落地（仍为绝对容差）；
  P7 提交 `6484b73` 完成，口径以本段为准。
  * **本轮状态**：`IMPLEMENTED`（`CoverageEnvelope::{dominance_margin,dominance_ratio}`，
    接受条件 `ratio >= 1 − dominance_tolerance`）。
  * **证据**：`B2Coverage.*`（6 例，含零核拒绝用例）。
* **命题（gate 归因）**：0.25 步长 gate 的阈值与判定逻辑不变；被拒绝候选的 reason 必须给出
  物理归因（rotation/position/velocity/accel_bias/gyro_bias 幅值与主导历元/块）。
  * **本轮状态**：`IMPLEMENTED`（`stateStepAttribution()` + 监测器仅追加 reason/skip_reason）。
  * **证据**：`GateAttribution.*`；G 场景 40/40 拒绝候选带归因且离散/PL 与 P5 逐帧 0 差异。
* **未执行（NOT_RUN）**：C1 的摘要替换/生命周期/重线性化/oracle/新场景（A1–A6）与
  C2 的双通道检测/PL/fault-span/模型误差通道（B0–B5）；设计与义务已冻结于
  `history-summary-design.md`（C1-a 模块级部分见 §17）。

## 17.（C1-a 新增）历史消元摘要**模块级**恒等式（LOCKED_BY_TEST）

范围：纯算法单元 `history_fault_summary.{hpp,cpp}`（代码提交 `bc5722c`），
**未接入生产管线**（无调用点；检测/PL/风险/阈值零变化）。构造只用两段
Householder 正消元（先消 `x_o`，再按 `x_b` 支持分块）；无正规方程、无求逆；
输出为平方根形式 `(R_b,T_b,d_b,F_b,d_perp)`。详见 `history-summary-module.md`。

* **命题（代价恒等式）**：对一切 `(x_b,f)`，
  `min_{x_o} ‖H_o x_o + H_b x_b + A f − z‖²
  == ‖R_b x_b + T_b f − d_b‖² + ‖F_b f − d_perp‖²`。
  * **本轮状态**：`LOCKED_BY_TEST`（C1-a 模块级）。
  * **证据**：`HistoryFaultSummary.HISM1CostIdentity`、`HISM1OracleTable`
    （多尺寸/多顺序/病态档；实测最大相对误差见 `history-summary-module.md` §4）。
* **命题（边际等价）**：`f=0` 时 `R_bᵀR_b == H_bᵀ(I−P_{H_o})H_b`（`x_b` 边际信息），
  对应边际协方差 `(R_bᵀR_b)⁻¹` 与稠密边际协方差一致。
  * **本轮状态**：`LOCKED_BY_TEST`。
  * **证据**：`HistoryFaultSummary.HISM1SchurEquivalence`。
* **命题（G 响应与符号对偶，§2 冻结约定）**：
  `R_b⁻¹T_b f` 与稠密条件边界均值响应互为负向；
  `boundaryMeanShiftForFault(f)` 与 `conditionalBoundaryMeanDelta(f)` 互为相反数。
  * **本轮状态**：`LOCKED_BY_TEST`。
  * **证据**：`HistoryFaultSummary.HISM1BoundaryResponseSignDuality`
    （两函数各自对稠密 oracle 校验 + 互为相反数断言）。
* **命题（检测审计视图）**：`Ω_b == Aᵀ(I−P_{[H_o H_b]})A`、
  `κ_b == 最小联合残差能量`、`ν_⊥ == 检测行数`（边界满秩时；边界秩亏时由
  代价恒等式承载，模块报告 `rank_boundary` 并不拒绝）。
  * **本轮状态**：`LOCKED_BY_TEST`。
  * **证据**：`HistoryFaultSummary.HISM1FaultGramKappaNu`、
    `HISM1DetectorOnlyAndNominal`。
* **命题（多消元顺序不变量）**：行置换与列置换（`x_o`/`x_b`/`f`）下
  `R_bᵀR_b、F_bᵀF_b、κ_b、ν_⊥`（上至相应置换）与代价恒等式一致。
  * **本轮状态**：`LOCKED_BY_TEST`。
  * **证据**：`HistoryFaultSummary.HISM1EliminationOrderInvariance`。
* **命题（退化显式化）**：非有限输入 / `H_o` 结构性秩亏（无法裁决）/ 空输入
  → `invalid + reason`、不产出数值；`q=0` 给出正确零列（名义边界）；
  纯检测行（无 `x_b` 支持但 `A/z` 非零）进入 `F_b/d_perp`，不被丢弃。
  * **本轮状态**：`LOCKED_BY_TEST`。
  * **证据**：`HistoryFaultSummary.HISM1DegenerateRejection`、
    `HISM1DetectorOnlyAndNominal`。
* **诚实缺口（NOT_RUN）**：管线接入（`buildIntegrityWindow` 块/账本 → `H_o/H_b/A/z`
  适配）、生命周期（§7.6 行 1–7）、容量键、冷启动枚举、重线性化绑定、跨边界新场景
  与管线级 oracle（`HIS-01..06`）待 C1-b/C1-c；C2 未开始。上述模块级恒等式
  **不**构成任何管线等价声明。

## 18.（C1-c 本轮）容量键、缓存身份与版本绑定载体；B1 管线接入阻塞

* **命题（容量键，C1c-C2 配置层）**：`history.{max_summary_rows, max_fault_columns,
  max_perp_rows, capacity_action}` 严格加载：未知键 / 缺键 / 非法 action 硬错误；
  节缺省 = 零容量默认（`REFUSE`）；"绝不静默丢最旧故障"语义冻结。
  * **本轮状态**：`IMPLEMENTED`（配置层；超限执行语义随 C1-b 阻塞）。
  * **证据**：`IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded`；
    `history-summary-b1c-blockers.md §2.1`。
* **命题（缓存身份，B5）**：统计缓存键必须覆盖 summary 绑定与 envelope 全轴——
  `StatisticalBoundKey::history_summary_version` 与 `envelope_kind` 均参与
  `NoncentralKey`（修复 `envelope_kind` 只声明未哈希的静默别名缺陷）；版本变化
  不得命中旧条目。
  * **本轮状态**：`IMPLEMENTED`。
  * **证据**：`B3Risk.HistorySummaryVersionIsPartOfCacheIdentity`。
* **命题（版本绑定载体，C3）**：摘要绑定 = `linearization / whitening / mode_set /
  capacity` 四分量；`digestHistorySummaryVersion` 确定性且对任一分量敏感（改
  human-readable id 不算绑定）。
  * **本轮状态**：`IMPLEMENTED`（载体与摘要；生产侧填充随 C1-b 阻塞）。
  * **证据**：`HistoryFaultSummary.HISM2VersionDigestBindsAllComponents`。
* **阻塞（NOT_RUN，原因+所需输入）**：B1（历史故障参数化 → 摘要接入
  `buildIntegrityWindow`）、B2–B4、B6 生产者、B7、B8、C1c-C1（§7.6 行 1–7）、
  C1c-C4（v15）、D1–D3 均因 B1 阻塞；逐条代码位置、原因与所需输入见
  `history-summary-b1c-blockers.md §1`（影响面清单见其 §5）。本轮**未**做任何
  B1-lite / default-off 半成品集成（理由见同文 §3）。

## 19.（C1-b 解阻：Part A/B）历史故障参数化与抽取定路（LOCKED_BY_TEST）

范围：`history_fault_parameterization.{hpp,cpp}`、`history_summary_extraction.{hpp,cpp}`
（代码提交 `1f05f11`）；**未接线**（Part C 就绪清单见
`history-fault-parameterization.md §6`）。以下均为模块级命题。

* **命题（参数化粒度，D-1）**：UWB 每 (anchor, 历史历元) 常数步列 + 时间线性伴生列；
  IMU 每 (轴, 历元) 区间常值列（与窗口生成器同一解析路径）；双故障不存列。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：
    `HistoryFaultParameterization.ColumnsMatchIndependentSelectionOracle`。
* **命题（窗口-历史一致性）**：同一事件在窗口路径（生成器）与历史路径（新子系统）
  产出的列逐位一致（UWB `norm=0`；IMU `rel=0`）。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：
    `...WindowAndHistoryColumnsAgreeOnSharedMaterial`。
* **命题（模式组合精确性）**：persistent/ramp 历史模式 = 步列精确线性组合
  （构造性 T；ramp ≤ 1e-12 重排级，persistent 精确）。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：`...RampCombinationIsExactOverTheStepBasis`。
* **命题（注入）**：故障键进入 separator（只消状态键）；重建因子不改既有图/不触
  Unrecoverable；列范数与残差在增广误差中保持（二阶差分恒等式）。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：`...InjectionKeepsFaultKeysInSeparator`。
* **命题（视野，A3）**：覆盖 = `[oldest_recoverable+1, window_first)`；视野外
  unrecoverable 且**不得声称覆盖**（gap/omitted 计数 + assumptions/omittedRiskSource
  文本）；生产写入属 Part C。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：`...PlanCoversRecoverableHorizonWithoutOverclaiming`。
* **命题（容量，D-2）**：默认容量 = 实测 q_hist（半跨 220，全跨界 440）→ 512；
  超限 REFUSE（unusable+HISTORY_CAPACITY_EXCEEDED），**无截断路径**；RESET/
  STOP_PROTECTED 仅显式且同样先置不可用。
  * **本轮状态**：`LOCKED_BY_TEST`（构造器层；执行接线随 Part C）。**证据**：
    `...CapacityRefusesWithoutTruncation`、`IntegrityConfig.HistoryCapacityKeysAreStrictlyLoaded`。
* **命题（抽取等价与定路，D-3）**：选定路线 = 线性化行抽取 + 模块正交消元：
  合成系统 `rel=0`（κ/d_b/R_bᵀR_b 全等）；真实窗口 `rel=2.9e-16`（信息）、
  κ 对稠密最小二乘 oracle `rel=6.6e-47`；无特征分解截断（rank=390=columns）；
  秩亏正确（rank_boundary/ν）。对照路线 (i-b)/(ii) 的常数丢失为实测结论。
  * **本轮状态**：`LOCKED_BY_TEST`。**证据**：`HistorySummaryExtraction.*`（3 例）。
* **诚实缺口（NOT_RUN）**：Part C 接线（边界段替换、顺序断言、池化检测、冷启动
  生产、指纹/缓存填充、重基线、v15、新场景）——就绪清单见主证据 §6；本轮未做
  任何半成品接线。

## 20.（C1 收口）管线接线后的命题（LOCKED_BY_TEST / 诚实缺口）

范围：`incremental_estimator.cpp`（边界段）、`integrity_window_snapshot.{hpp,cpp}`
（载体/指纹）、`integrity_monitor.cpp`（诊断 v15）、`common/types.hpp`、
`history_fault_parameterization.cpp`（注入键块宽度）、测试 4 套（pipeline/lifecycle/legacy）。

* **命题（边界即摘要）**：冻结窗口的 `BoundaryPrior` 块 == 模块平方根摘要
  （`R_b` 进窗口列布局、`d_perp` 为零 Jacobian 行、`T_b/F_b` 载体逐项对应）。
  * `LOCKED_BY_TEST`：`HistorySummaryPipeline.BoundaryIsTheSquareRootHistorySummary`。
* **命题（池化口径）**：`T_pooled = ‖r_c‖² + κ_b(+offset)`、`ν_pooled = ν_c + ν_⊥`，
  且 `dof = rows − rank` 恒等式在管线成立。
  * `LOCKED_BY_TEST`：`PooledStatisticAndDofIdentity`（残差能量 == κ_b；
    dof 加性）+ legacy `OneSecondUwbDropAndChangingAnchorSetRecover`
    （`dof == UWB 行 + nu_perp`，整数精确）。
* **命题（跨边界可监测）**：历史故障的模式映射 `[T_b f; F_b f]` 非零，Γ>0 且有限
  （detector-only 内容与 H 行空间正交 ⇒ Γ ≥ ‖F_b f‖²）。
  * `LOCKED_BY_TEST`：`HistoryFaultResponsePersistsAcrossBoundary`。
* **命题（缓存身份/失效）**：摘要身份进入内容指纹；版本变 ⇒ 旧 numerics `stale`
  且不可用。
  * `LOCKED_BY_TEST`：`SummaryVersionBindsWindowFingerprint`。
* **命题（内容完整性/无重复计入）**：`slot_accounting` 精确一次 XOR；边界块不属于
  ledger 组；覆盖率=100% + 唯一性。
  * `LOCKED_BY_TEST`：`RowAttributionExplicitXorBoundary`。
* **命题（顺序与冷启动）**：horizon 内材料被先删 ⇒ 显式不可用
  （`HISTORY_SUMMARY_INVALID`），不从名义边际反推；容量 REFUSE ⇒ 不可用+计数、
  不截断。
  * `LOCKED_BY_TEST`：`DeletedMaterialFailsBeforeSummaryUpdate`、
    `CapacityRefusalIsExplicitAndCounted`。
* **命题（生命周期行 1–7）**：决策级全双向锁定（含 `auto_shrink` 恒关）。
  * `LOCKED_BY_TEST`：`HistorySummaryLifecycle.*`（7 例）。
* **本轮修复的真缺陷（回归守护）**：信息型因子（container）路径（原为硬拒绝）与
  注入键块宽度（原为每键 1 列的静默截断）——两者由 legacy 场景与管线恒等式共同
  守护。
* **诚实缺口**：r0_r1 原始流新场景未执行；context_oracle O8a/O8d 未重跑；管线级
  多消元顺序独立 oracle 未成测；offset 的场景级影响未量化（`history-summary-pipeline.md §11`）。

## 21.（C1 补完）新增命题与缺口状态

* **命题（载体行系统多消元顺序）**：对已交付载体 `[R_b T_b; 0 F_b]`/`[d_b; d_perp]`
  重消元（同序与逆序）必须重现 `R_bᵀR_b`、`R_bᵀT_b`、`R_bᵀd_b`、`F_bᵀF_b`、`κ_b` 与计数。
  * `LOCKED_BY_TEST`：`HistorySummaryPipeline.CarrierInvariantsUnderRowPermutation`（rel≤1e-9）。
* **命题（原始流可监测性）**：`HIP_history_crossing_fault` 原始流中 onset 出窗后，摘要仍携带
  响应与检测内容（q/injected/nu_perp/κ/Ω/ξ 连续且非零）。
  * `LOCKED_BY_RAW_RUN`：`raw/hip_run_capture.md`（60/230 epochs；dev harness
    `marginalizations=0` 已如实记录）。
* **证据缺口状态**：①原始流场景 **已关闭**；③管线级多消元顺序 oracle **已关闭**；
  ②context_oracle O8a/O8d 仍 NOT_RUN（需 raw dump 接入）；④offset 仅一个样本（已导出）。

## 22.（C2/M1）§7.3–7.5 + §8.2 公式评审与义务

* **命题（通道切分精确）**：`T_c+T_b=T_pooled`、`ν_c+ν_b=ν_pooled`；并集界
  `min(1, 2·p_fa·horizon)` 显式导出（阈值未改）。
  * `LOCKED_BY_TEST`：`DualChannelDetector.RealWindowSplitIsExactAndTablesTheDifference`（真实窗口）。
* **命题（§7.4 双通道界）**：`W_h=Σ_j w_j Γ_{h,j}/Λ_{h,j}`，`b_{h,d}=√(g W_h† gᵀ)` 由**细 SVD**
  计算（不形成 W 逆），与显式 `W^{-1}` 形式逐位一致；`ker W_h ⊆ ker G_h` 违反即拒；
  零贡献/无自由度通道不生成 Λ；单有效通道退化为 B3 `s·√Λ`；证书绑定
  detector_id/τ_j/ν_j/Λ_{h,j}/w_j/dim；证明只用二次型界（不依赖通道独立性）。
  * `LOCKED_BY_TEST`：`DET-02`。**评审结论**：公式与 B3 形式在单通道极限相容；
    权重归一化后 `Σw_j=1`；通道丢弃不影响正常化（测试断言 1e-12）。
* **命题（§7.5 fault-span）**：`U` 覆盖声明内全部方向（SVD 基，无 top-K）；恒等式
  `U(Z_bᵀZ_b)Uᵀ=F_allᵀF_all` 在全参数空间验证；截断基必然失败；声明秩亏记录。
  * `LOCKED_BY_TEST`：`DET-03`。管线采用未投影残差（合同允许）并记录 `ν_⊥`。
* **命题（§8.2 模型误差）**：`τ_risk=(√τ_actual+ρ_r)²`，`ρ=0` 退化为标准 χ²；位置 ρ 逐轴；
  ρ 来源与验证状态入证书；负值/非有限拒绝。
  * `LOCKED_BY_TEST`：`DET-02`（含未验证假设路径）。
* **诚实缺口**：fault-span 投影未接入管线（当前用未投影残差，符合合同）；ρ 未进入风险账本
  执行（模块 + 证书状态就绪）；C3（FDE-03..05）与 C4（OUT-01..03）未开始。
## 23.（C3/M2）§8.3–8.5 义务

* **命题（profile 似然用原始量）**：`J=‖r_c‖²+κ_b−t'Γ†t`，输入为原始白化量（w/Λ/W 不进入），
  `constant_used` 可审计；旧先验污染使 J 上升。*`LOCKED_BY_TEST`：FDE-03。*
* **命题（可比较性）**：跨物理单位、跨参数维数、未声明单位的候选**不得**排序；同单位同维才可排。
  *`LOCKED_BY_TEST`：FDE-03（三拒绝 + 一通过路径）。*
* **命题（选择风险并集）**：对全部可能发布动作计费；共享参考（共享证书/接受事件/时间/输出量 +
  三角转移恒等式）构成**一个**失败事件，只计组内最大 ε；不同参考分别计费；证明不闭合 ⇒ 不可用。
  *`LOCKED_BY_TEST`：FDE-03/05。*
* **命题（决策序）**：验证模型 ⇒ 参考估计 + 双通道包络（标记 bounded，不当高斯信息）；
  仅中心 + 参考 + 转移界 ⇒ 保护转移；否则 ⇒ 显式未保护诊断（不是“永久不可排除”）。
  *`LOCKED_BY_TEST`：FDE-04。*
* **命题（移除不等于风险消失）**：被删源仍以 `groups_to_remove` 进记账并自成保证组计费。
  *`LOCKED_BY_TEST`：FDE-04。*
* **诚实缺口（未接线）**：① 模块→`FdeManager::decide` 主链替换；② ε 预算接入风险账本执行；
  ③ IMU 区间移除的数据来源/模型误差字段落盘。C4（OUT-01..03）未开始。

## 24.（C4/M3）§8.6 义务

* **命题（提交绑定）**：发布必须与证书身份一致（快照/解/摘要/清单/健康/检测器/风险证明/
  参考点/时刻/坐标系）；同时间点改解需已证明的中心偏移界；不同时刻需时间传播证明。
  *`LOCKED_BY_TEST`：OUT-01/02。*
* **命题（原子状态机）**：仅白名单迁移；`UNAVAILABLE→PROTECTED` 与 `VERIFIED_CANDIDATE→PROTECTED`
  被拒；完整恢复链可走通。*`LOCKED_BY_TEST`：OUT-01。*
* **命题（时间与新鲜度分离）**：wall 统计仅在单调且未跳变时累加；回放跳变不触发 wall 超时；
  新鲜度用 wall−sensor 滞后量；时钟倒退（无标记）即拒。*`LOCKED_BY_TEST`：OUT-03。*
* **诚实缺口（未接线）**：模块→`integrity_monitor` 发布路径的替换与诊断列/CSV 的 OUT 字段未加。
* **本轮三个里程碑状态**：M1/M2/M3 的**模块 + 测试 + 证据**均完成；M2/M3 的接入项如上。
## 25.（W1：C3 生产接线）义务与状态

| # | 义务 | 状态 | 证据 |
|---|---|---|---|
| W1-01 | 可能集每个假设都有**可比性身份**（单位/维数）与**原始白化 profile 值** | LOCKED_BY_TEST | `hypothesis_evidence.cpp`（`faultUnitKindOf`、`profile_j`）；测试 `SelectionRiskChargedOverPublishableSet`、`MixedUnitPoolIsRecordedAndNotRanked` |
| W1-02 | 证据缺失/非有限 profile ⇒ 拒绝发布（fail-closed，不放宽） | LOCKED_BY_TEST | `FdeManager::decide` 池检查 ⇒ `ModelInvalid`；测试 `ProfilePoolRefusesMissingEvidenceFailClosed` |
| W1-03 | 跨单位/跨维数**不得排序**（拒绝语义被遵守） | LOCKED_BY_TEST | 混单位池 `profile_pool_ranked=false` 且按冻结准则照常选择；测试 `MixedUnitPoolIsRecordedAndNotRanked` |
| W1-04 | 选择风险对**全部可能发布动作**的并集控制，超预算保持不可用 | LOCKED_BY_TEST（当前策略下不可绑定，见 §7.2 诚实说明） | `buildGuaranteeGroups` + `commit_allowed=false` 路径；测试 `SelectionRiskChargedOverPublishableSet` |
| W1-05 | IMU 区间移除的"数据来源/模型误差记录"是显式字段，并在并集动作中保真 | LOCKED_BY_TEST（结构）/ 诊断导出推迟 W2 | `ExclusionAction::{removal_data_source,model_error_record,model_error_validated}`；`actionForMode`/`unite`；`candidate_replay.cpp` 显式说明 |
| W1-06 | 阈值/合同（0.25 gate、128 上限、p_fa/p_md、alert limits、风险预算）不动 | LOCKED_BY_TEST | 无阈值改动；validation 50 PASS/0 FAIL @ b7feb9c |
