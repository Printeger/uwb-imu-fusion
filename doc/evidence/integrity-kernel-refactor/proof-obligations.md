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
* **阻塞/归属**：无（若要改成立方/平方根内核 B1，须先改本项与测试）。

## 4. 三条查询路径的等价性

* **命题**：`snapshotSensitivity`（t=0 快照）、`conditionalSensitivity`、窗口全量
  `H`（历史路径）三条查询在同一批数据上给出同一有限斜率/Z/Γ。
* **本轮状态**：`EVIDENCE_ONLY`（部分）。`ReferenceFixture.FrozenBlockWhiteningAndAggregateAssembly`
  证明冻结块白化后的行拼接严格等于全量 `H`（abs ≤ 1e-12），oracle O3 在 8 个真实
  窗口上复核同一恒等式；三路径的**数值等价**未做逐点对照。
* **证据**：`oracle-results.json`（O3 全 PASS）、`fixtures/frames.json`。
* **阻塞/归属**：B（B1 平方根内核时需保证三路径同一结果）。

## 5. C = [0 | R] 的有限差分锁定（D1 声明相关）

* **命题**：受保护量是机体系原点世界位置 ⇒ `C = protected_state_map` 的前三行在三
  维切空间惯例 `gtsam_pose3_local_rotation_then_body_translation` 下应等于 `[0|R]`；
  任何杠杆臂项都必须被判出。
* **本轮状态**：`LOCKED_BY_TEST`（D1）。`ReferenceFixture.ProtectedMapIsBodyOriginFiniteDifference`
  用中心差分 `C·(δ/h)` 对照 `C` 行，容差 1e-7，并对“含杠杆臂的 C 行”断言失效；
  探针实验（`gtsam::Pose3::retract`：∂t/∂ω=0、∂t/∂υ=R）作为独立佐证。
* **证据**：`test/test_integrity_reference.cpp`；`raw/runs_p2/*/diagnostic_snapshot_identity.csv`
  （`position_reference=body_origin`、`tangent_convention=gtsam_pose3_...`）。
* **阻塞/归属**：无。

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
* **阻塞/归属**：C。

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
