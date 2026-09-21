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
