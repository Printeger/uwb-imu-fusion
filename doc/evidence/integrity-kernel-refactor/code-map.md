# 代码映射 code-map（工作包 A1）

- 仓库根：`/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl`（git root；catkin 工作区 `/home/mint/ws_fusion_uwb`）
- 基线：HEAD `2772b3a1e3584b25c889239421404f550fba9f5d`，`feature/realtime-uwb-imu-pl`，与 `origin` 同名分支 FETCH_HEAD 完全一致（0/0），见 `worktree-before.txt`、`remote-head.txt`
- 证据目录：`doc/evidence/integrity-kernel-refactor/`（本文档所在目录）
- 方法：逐文件阅读实际源码 + 导出真实 replay 窗口独立复算 + 运行诊断日志；**以源码为准**，路线图旧稿路径/数字凡与源码不符处均在 §4 标出
- AGENTS.md：不存在（`agent-instructions.txt`）；约定来源 README、`CMakeLists.txt`、`doc/UWB_IMU_PL_V2_DEVELOPMENT_HANDBOOK.md`、`doc/adr/0001-0003`

## 1. 路线图 §3.2 六个入口逐项结论

### 1.1 `src/uwb_imu_pl/estimation/integrity_window_snapshot.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `finalizeIntegrityWindow()`、`integrityWindowFingerprint()`、`solveFrozenInformation()`、`numericalContractFingerprint()`；结构体在 `include/uwb_imu_pl/estimation/integrity_window_snapshot.hpp`（`LinearizedFactorBlock`、`LinearizedIntegrityWindow`、`FrozenWindowNumerics`） |
| 调用者 | `IncrementalUwbImuEstimator::buildIntegrityWindow()`（`incremental_estimator.cpp:1065`）组装窗口后调用；消费方：`JointWindowDetector`、`RankUpdateEvaluator::factorizeOnce`、`HypothesisEvidenceEvaluator`、`ProtectionLevelV2`、`candidate_replay.cpp`（读回时重算 numerics） |
| 对应测试 | `IntegrityV2Numerics.FrozenWindowSharesOneBaseSvdAndLlt`、`IntegrityV2Numerics.RejectsReuseAfterAnyFrozenIdentityMutation`、`GateDNumerics.*`、`GateDReplay.LosslessRoundTripAllBlocksAndActions`（`test/test_integrity_v2.cpp`） |
| 已实现内容 | 从 blocks 拼装 `H`、`z`；SVD（默认 BDCSVD，`UWB_IMU_PL_DISABLE_WINDOW_BDCSVD` 回退 JacobiSVD）给出 rank/condition/最小最大奇异值；`base_information=HᵀH`、`base_information_rhs=Hᵀz`；LLT(HᵀH) 分解 + 一个 SVD 解作为 canonical 解与 parity；指纹绑定 window/version/全块内容与数值契约；`solveFrozenInformation()` 用 LLT 解右端并带前向误差界，必要时回退谱分解 |
| 本轮确认 | 见下 1.1 Q&A |

**路线图问题回答：**

- **H/z/C 从哪来？** 全部由 `IncrementalUwbImuEstimator::buildIntegrityWindow()` 组装的 `blocks` 拼出（`finalizeIntegrityWindow`），块来源：①边界先验块（历史因子在图外变量上做 partial-QR 消元 → 对窗口内变量做稠密特征分解压缩成 `rows=retained rank, cols=total_columns` 的 `BoundaryPrior` 块，`incremental_estimator.cpp:1178-1245`）；②窗口内历史显式因子组（UWB/CombinedImu）；③当前 pending 的 `CombinedImu`、UWB nominal 组。**C（`protected_state_map`）是 3×total_columns，仅在最后一个 15 维状态块放置 `worldPositionPoseTangentJacobian=[0₃ₓ₃ | R]`**（`incremental_estimator.cpp:1393-1396`），即被保护量是**body 原点世界位置**（manifest scope 亦写 `body_origin_position_world`），不含 UWB tag 的 lever arm。
- **包含哪些先验/因子？** 初始协方差先验、ConstantVelocityRegularizer（作为图中因子）、固定滞后产生的线性约束边界先验、历史 UWB/IMU 因子、当次 IMU/UWB 因子；每块保留 `jacobian_raw/residual_raw/covariance/whitener/jacobian_whitened/residual_whitened/fault_units/whitening_model_id`。
- **是否重复白化？** 否。实测每个块 `||W·J_raw − J_whitened||/max|J_whitened| ≈ 1e-16`（5 帧全部通过；数值见 `census.json` 的 `whitened_eq_whitener_raw_rel`）。`linearizeGroupDense()` 返回的 GTSAM Jacobian 已含噪声模型白化，`linearizePendingGroup()` 用 `jacobian_raw = L·J_whitened` 反推出 raw 表示（`L` 为协方差 LLT 下三角），两种表示同时保留但只白化一次。
- **是否维护稠密 SVD、正规方程、LLT 多套表示？** **是，三套同存于同一 `FrozenWindowNumerics`**：稠密 SVD（rank/condition/canonical 解/谱逆）、正规方程（`base_information`/`base_information_rhs`，供所有块级/candidate 右端求解）、`LLT`（主求解路径 + 前向误差界）。`model_valid` 条件包括：满列秩、`dof>0`、condition≤1e10（配置）、无重复 group、LLT 成功、解残差≤1e-7。**路线图 B/B1 的目标（单一平方根主路径）尚未实现。**

### 1.2 `src/uwb_imu_pl/estimation/rank_update_kernel.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `RankUpdateEvaluator::factorizeOnce/evaluate/buildSharedCache`、`DenseCandidateOracle::evaluate`、`certifyUpdate()`（谱证书）、`certifiedLargeStep()`、`candidateRows()`、`CandidateEvaluation::covarianceTimes/normalCross`；配置 `RankUpdateConfig`（`include/.../rank_update_kernel.hpp`，默认 `max_linearization_step_norm=0.25`） |
| 调用者 | `RealtimeIntegrityPipeline::processUwbBatchImpl`（kernel 阶段 + PL 阶段）；`apps/candidate_replay.cpp`（回放/基准）；`DenseCandidateOracle` 供测试/离线 oracle |
| 对应测试 | `IntegrityV2RankUpdate.MatchesIndependentDenseOracle`、`...RejectsUnresolvedRemovalBeforeSelection`、`...OnlinePathKeepsOnlySharedBaseAndLowRankCorrections`、`GateDNumerics.CompleteReplacementSurvivesSingularDeletionIntermediate`、`GateDNumerics.EuclideanStepGateBothSidesUsesReferenceNearBoundary`、`GateDNumericalCertificate.ClearCandidateAvoidsFinalJacobianSvd`、`GateDCache.*`、`GateDOracle.*` |
| 已实现内容 | KEEP_ALL 直接复用基解（`KEEP_BASE_CANONICAL`）；add/remove 用一次秩 2 更新的 Woodbury 修正（`covariance_plus/minus_factor`），`LOW_RANK_CERTIFICATE` 用特征界证书（`I + R S Rᵀ` 小矩阵、SPD/rank/condition 界与 gate 余量），不确定则回退 `FINAL_JACOBIAN_REFERENCE`（最终 Jacobian 的 JacobiSVD）；早停步长 gate 可在构造最终 Jacobian 前拒绝候选（`QR_CERTIFIED_STEP_REJECTION`）；可选共享块级 `solveFrozenInformation` 缓存 |
| 本轮确认 | 见下 Q&A；实测 A_nominal 30 epoch：`base_svd=30, base_llt=30, candidate_inner_llt=0, candidate_reference_svd=0`（KEEP_ALL 帧没有多余分解） |

**路线图问题回答：**

- **当前数值可靠性与候选更新如何做？** “一次分解 + 每次候选低秩更新 + 保守证书 + 必要时参考回退”。评估路径名写入诊断（`ADD_THEN_REMOVE_SVD`/`KEEP_BASE_CANONICAL`/`LOW_RANK_CERTIFICATE`/`QR_CERTIFIED_STEP_REJECTION`/`FINAL_JACOBIAN_REFERENCE`）。
- **`0.25` 检查什么量？** `integrity_window.max_linearization_step_norm=0.25`，检查的是**候选状态增量（整个窗口所有状态块拼接后）的欧氏范数** `‖δx‖ ≤ 0.25`，单位混合（弧度/米/速度/bias），即路线图 §8.1 说的“混合米、弧度、速度和偏置的整体增量阈值”。它同时出现在：KEEP 路径（`rank_update_kernel.cpp:557-583`）、矩阵无关早停（`:644-661`）、参考路径与最终 valid 判定（`:735-738`）。实测 G_continuous_rejection epoch 10 的唯一候选正是被该 gate 拒绝（`certified step exceeds gate`），导致本历元 discard、无 UWB 提交。
- **候选中心是否等于发布中心？** KEEP_ALL 时 `state_increment=基解`、`statistic=基 parity`，候选与发布中心一致；FDE 提交路径先 commit 后端再以选中候选的 `pl_xyz` 填 `IntegrityOutput`（`integrity_monitor.cpp:2210-2240`），**候选中心未单独作为“发布中心”字段导出**——快照/中心身份字段属 A3 契约范围，当前仅有 `window_id/transaction_id/base_graph_version`。

### 1.3 `src/uwb_imu_pl/integrity/hypothesis_generator.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `HypothesisGenerator::generate()`、`actionsForPlausibleSet()`、`finalizeEffectiveBasis()`、`actionForMode()`、`unite()`、`equivalentActionOperation()`、`healthSourceId()`；结构 `GeneratedFaultModelSet`（`include/.../hypothesis_generator.hpp`） |
| 调用者 | `integrity_monitor.cpp` 的 `model_generation` 与 `health_actions` 阶段 |
| 对应测试 | `IntegrityV2FaultModel.GeneratesCompactUwbModesAndEveryImuAxis`、`IntegrityV2Fde.ActionDedupRejectsSameShapeAndNormDifferentContent`、`IntegrityV2Fde.CoversEntirePlausibleSetBeforeSelection`、`IntegrityV2Transaction.HistoricalUwbReplacementCommitsAtomically` |
| 已实现内容 | UWB：每 anchor、每 onset（窗口内历元）生成 `ANCHOR_BIAS_EPOCH_INDEPENDENT`（仅该历元组）、`ANCHOR_BIAS_PERSISTENT_CONSTANT`（onset→窗口末，起于 detector 首历元时向左扩到 recoverable 边界）、可选 `ANCHOR_BIAS_RAMP`（2 参数，按真实时间戳线性）；IMU：每历元 occurrence 6 个单轴 `*_AXIS_INTERVAL_CONSTANT`（区间常值，参数维度 1）；双故障（开关关闭时无）：UWB×accel、UWB×gyro 组合；动作：`KEEP_ALL` + 每模式单独动作（UWB 删整组并加“排该 anchor 的保留协方差替代组”，IMU 删该历元 IMU 组并加 generic bridge + bias continuity 组），按 `actionKey` 去重后受 `max_candidate_count=128` 限制；健康屏障来源在 `actionsForPlausibleSet` 中强制加入 |
| 本轮确认 | 模式范围只覆盖**显式在窗口内的因子组**：A_nominal 30 epoch 实测 onsets 20..30（11 个）、IMU occurrences 10 个；被拒批历元不产生 occurrence（G 场景 45 epoch 时单 UWB=144=8×9×2）。动作在正常帧实际只保留 `KEEP_ALL`（`all_in.passed && !hardware_barrier` 时 `models.actions={KEEP_ALL}`，`integrity_monitor.cpp:1727-1732`） |

**路线图问题回答：**

- **模式是单轴、整区间还是同时多轴？** 三者不同层：IMU 故障族是**单轴、整区间常值**（accel/gyro 各 3 轴，参数维度 1，时间支撑=一个历元区间 [previous, proposed]）；UWB 是单 anchor、三种时间形状（瞬时/persistent/ramp）；“同一器件共因多轴”只有显式组合族才可能表达，当前 `accel_xyz_interval_bias=false`、`two_uwb=false`，故默认声明只有单轴与 UWB×单轴双故障族。
- **是否同时构造动作/桥接？** 生成阶段同时构造了单模式动作与其 added blocks（UWB 替代组、IMU 的 kinematic bridge + bias continuity 桥接块，`bridgeBlock()` 直接线性化当前冻结值）；但正常帧最终只用 KEEP_ALL，报警/屏障帧才把 plausible 集的单动作并成 union 动作。桥接在候选 PL 中作为 `deterministic_bound`（确定性盒），不是小协方差高斯。
- **实际观测索引如何维护？** 无独立“观测索引”对象：观测身份由 `PendingFactorGroup.source_measurements`（UWB）与 `fault_units` 承载，模式映射 `raw_group_maps{group_id → rows×q}` 在生成时按 batch 内 anchor_id 逐行填充；行序=配置 anchors 顺序（8 行/组），本轮用注入帧残差证实（见 fixtures）。

### 1.4 `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `HypothesisEvidenceEvaluator::evaluateAll()`、`evaluateContiguous()`（共享上下文+低维批量）、`evaluateMapped()`（逐模式投影回退）、`analyzeDynamic/analyzeFixed<>()`、`FrozenHypothesisNumerics`、`completePlausibleHypotheses()`、指纹函数 |
| 调用者 | `integrity_monitor.cpp` 的 `hypothesis_evidence` 阶段（workers=4，`UWB_IMU_PL_DISABLE_HYPOTHESIS_SHARED/BATCH` 可关） |
| 对应测试 | `IntegrityV2ProtectionLevel.*`（与 PL 联动）、`GateDProtectionLevel.SharedModesUseOneCombinedCovarianceSolve`、`GateDNumericalCertificate.*`、`IntegrityV2Risk.EqualAllocationNeverOvershootsAndRealExcessFails`（统计部分） |
| 已实现内容 | 一次性把所有模式的 **whitened 模式矩阵 `dense`（m×Σq）** 与 `[Cᵀ, Hᵀdense]` 右端合并，经 `solveFrozenInformation` **批量求解**（实测帧 `covariance_rhs_solves=1/帧`）；每假设从 **完整 all-mode Gram**（`denseᵀdense − normal_crossᵀcovariance_modes`，一次性 GEMM）取子块；1/2/3 维用栈上定维 LDLT，其他维用动态路径；输出 `FaultModeEvidence`（plausible、conditioned_statistic、log_evidence）与 `FrozenHypothesisNumerics.pl_entries`（σ_min/rank/condition/受保护 slope/gram_spd/valid） |
| 本轮确认 | “是否已批量求解/共享缓存？”**是**；"是否仍形成完整 all-mode Gram？"**是**（注释明言 “One contiguous GEMM pair materializes every self and pair cross term”；单帧 m×Σq 例如 A_nominal: 253×(236 列)，G 场景更小）。健康状态机依赖：单故障假设的 `plausible`（且 `!all_in.passed`）→ `pending_health.observeEvidence()`（`integrity_monitor.cpp:1646-1680`），另有 shadow 恢复计数依赖“非 suspicious” |

### 1.5 `src/uwb_imu_pl/integrity/protection_level_v2.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `ProtectionLevelV2::compute/computeStreaming/computeShared/computeFrozenAllIn`、`detectionBoundaryNoncentralitySquared()`；结果 `ProtectionLevelV2Result`（含 `availability/formal_eligible/risk_budget_valid/allocated_outcome_risk`） |
| 调用者 | `integrity_monitor.cpp` 候选 PL 阶段（KEEP_ALL 用 `computeFrozenAllIn`，其他用 `computeShared`）；风险账本 `auditRiskBudget` |
| 对应测试 | `IntegrityV2ProtectionLevel.RecomputesPostCandidateAndStaysResearchOnly`（及两条相邻用例）、`GateDProtectionLevel.SharedModesUseOneCombinedCovarianceSolve` |
| 已实现内容 | `λ = sqrt(非中心参数边界 Λ)`（`StatisticalBoundsCache::noncentralityBoundary` 解 `F_{χ²_ν(Λ)}(τ) ≤ p_md` 的 Λ，正值二分 120 次）；`fault_component = slopes·λ + k_fault·σ_d`，`k_fault=Φ⁻¹(1−tail/2)`；`nominal_component = k_nominal·σ_d`；逐模式取逐轴最大值；`pl = max(nominal, fault) + bridge_component`；可用性 = 有限 ∧ detector 通过 ∧ ≤ alert limits；`formal_eligible` **恒为 false**（Gate J 未完成），reason 明示 `IMPLEMENTED_UNVERIFIED` |
| 本轮确认 | 三处路径同一数学；post-FDE 语义：对每个 `remaining_hypotheses` 要求在**候选**行空间上 `monitorable`（rank==列数），否则整体不可用并给出具体假设/rank/σ_min/条件的 reason（`computeShared` 的详细 reason 串正是实测 C/G 帧不可用的直接文本） |

**路线图问题回答：**

- **非中心参数的平方约定？** 代码内 `lambda` 变量 = **Λ 的平方根**（乘以 slope 得位置界），残差统计量 `T=‖r‖²` 与阈值同为平方量纲；`λ`（代码命名）与路线图 §5.1 的 `λ=fᵀΓf` 不同名同义，测试需以本文为准记录入参约定（避免“再平方一次”）。
- **逐轴/逐模式预算？** 逐模式：`hmi_allocation`（保守均分，`conservativeEqualRiskAllocation`）→ `fault_tail=hmi/π_h`（下限 1e-15，上限 0.5 取分位）；逐轴：同一 λ 与 k 用于 3 轴，然后逐轴取最大（**不是** §5.9 的联合盒并集公式；当前是保守逐轴 max）。nominal σ 用 `nominal_axis_tail=1e-5`。
- **不可用分支？** detector 未通过 / 余下假设不可监测 / 风险账本不闭合 / PL 超 alert limit / protected covariance 无效 → `availability=Unavailable` + 具体 reason；不会用有限值冒充。
- **post-FDE 有何语义？** 见上；注意 `formal_eligible=false` 使一切在线输出保持“研究态不可用”，与 roadmap “不得靠清字符串改变证据状态”一致。

### 1.6 `src/uwb_imu_pl/integrity/integrity_monitor.cpp`

| 项 | 内容 |
|---|---|
| 符号 | `RealtimeIntegrityPipeline::processUwbBatchImpl()`（主编排，:1211-2440）、`IntegrityMonitor::evaluateSnapshot/evaluateConditional`（旧快照路径，运行时主链未用）、`capture_state_audit`、阶段计时 `stage_timings` |
| 调用者 | `apps/r0_r1_development.cpp`、`tools/run_realtime_integrity.cpp`（ROS 节点） |
| 对应测试 | `test_realtime_incremental.cpp` 全套、`IntegrityV2Transaction.*`、`IntegrityV2Health.*`、`IntegrityV2Reinitialization.*`、`GateDDiagnostics.PreparationExceptionKeepsAttemptTimingAndZeroUpdates` |
| 本轮确认 | 真实时序见下 |

**真实时序（正常/隔离/恢复/候选提交/发布）：**

1. `prepareEpoch()` 冻结交易（graph/values/ledger 版本合一，backend 未变更）。
2. `state_audit`（可选 ledger/health 快照导出）。
3. `buildIntegrityWindow(epochs)` → 窗口 + numerics（SVD/正规方程/LLT/指纹）。
4. `all_in_detector`（`JointWindowDetector`，parity² vs χ²(dof, p_fa=1e-6)，近条件门才做独立 QR 参考）。
5. 任何模型/数值无效 → `discard_fail_closed`（历史污染时要求受控重初始化），发布 `stale_state + Unavailable`。
6. `current_sensitivity`（IMU 解析灵敏度；`UWB_IMU_PL_IMU_FD_ORACLE` 时跑有限差分 oracle）→ `model_generation` → `hypothesis_evidence`（全体假设，实测 236/帧）→ `health_actions`（证据→健康机；quarantine 时形成 hardware barrier 与强制排除组；正常帧动作收缩为 KEEP_ALL）。
7. `base_factorization` → `shared_cache` → `candidate_evaluation`（phase1: kernel+post detector；phase2: bridge 界+PL，仅 post 通过者）→ `fde_decision`（覆盖 plausible 全集的候选才 eligible；优先 cardinality、其次 max(hpl,vpl)、再 logdet）→ `candidate_audit/coverage_audit`。
8. 提交：未选出候选时——`all_in.passed` 则 **best-effort commit**（backend 前进，`protection_level` 保持 inf/Unavailable、reason=decision.reason）；未通过则 discard（可选受控重初始化）。选出候选时：按动作改 commit plan，quarantine 覆盖源，提交后把选中候选 PL 写入输出。
9. 发布：`batch_committed=uwb_committed`；`availability` 恒 `Unavailable`（`formal_eligible=false`，reason=`IMPLEMENTED_UNVERIFIED: Gate J ...`）；bridge 超时 → discard + 重初始化请求。
10. 恢复：健康机 `Quarantined →（连续 shadow 通过）→ RecoveryTest →（连续测试通过）→ Healthy`；隔离源在恢复前不进入正式估计器（`allowedInFormalEstimator`），shadow 观测走 `observeShadowRecovery`。

## 2. 其他实际入口（本轮补记）

| 模块 | 路径:符号 | 角色 | 对应测试 |
|---|---|---|---|
| 增量估计器 | `src/uwb_imu_pl/estimation/incremental_estimator.cpp`：`prepareEpoch/commitEpoch/discardEpoch/buildIntegrityWindow/buildPendingFactorBlock` | iSAM2(fixed-lag) 主估计器；窗口组装；交易生命周期 | `test_realtime_incremental.cpp`、`IntegrityV2Transaction.*` |
| 因子账本 | `estimation/factor_ledger.cpp`：`FactorLedger` | group→entries 溯源（slots/lifecycle/health/replacement/recovery_epoch），`hasCompleteActiveProvenance` 驱动 history provenance | `IntegrityV2Transaction.CommitIsOneAtomicBackendUpdateAndLedgerIsComplete`、`GateDHistoryCache.*` |
| 交易 | `include/.../estimation/epoch_transaction.hpp`：`EpochTransaction`、`EpochCommitPlan`、`CommitReceipt` | 冻结交易/提交计划/回执的字段契约 | 同上 |
| 检测器 | `integrity/joint_window_detector.cpp`：`JointWindowDetector::evaluate/evaluateCandidate` | 平方 parity 统计量 + χ² 阈值 + union-bound 风险叠放 | `IntegrityV2Detector.UsesSquaredParityDofAndUnionBound` |
| FDE 决策 | `integrity/fde_manager.cpp`：`FdeManager::decide` | plausible 覆盖检查 + 排序（cardinality→PL→logdet→id） | `IntegrityV2Fde.*`、`GateDSelection.*` |
| 健康机 | `integrity/health_manager.cpp`：`HealthManager` | Healthy/Suspect/Quarantined/RecoveryTest/Failed 状态转换与计数 | `IntegrityV2Health.*` |
| 风险账本 | `integrity/risk_budget_audit.cpp`：`auditRiskBudget`、`conservativeEqualRiskAllocation`、`conservativeRemainingHypothesisRisk` | 高精度（cpp_bin_float_quad）预算闭合与均分上取/下取 | `IntegrityV2Risk.EqualAllocationNeverOvershootsAndRealExcessFails` |
| 稠密 oracle | `integrity/dense_oracle.cpp`：`evaluateDenseSvdOracle` | 独立稠密白化+SVD 参考（slope、monitorable、∞ 保持语义） | `test_dense_oracle.cpp`、`GateDOracle.*` |
| IMU 故障子空间 | `integrity/imu_fault_subspace.cpp`：`ImuFaultSubspaceBuilder::buildAnalytic/verifyFiniteDifferenceOracle` | 由 `CombinedImuFactor` 的 9 维运动残差对 bias 的 H 得解析灵敏度；oracle 用重积分 ±ε 差分 | `IntegrityV2ImuOracle.*` |
| 桥接因子 | `factors/kinematic_bridge_factor.cpp`：`BridgeFactory::makeGeneric/makeBiasContinuity/uncertainty` | CV+恒姿态桥接（优化协方差）与确定性盒界（完整性）分离 | `IntegrityV2Bridge.*` |
| 统计缓存 | `integrity/statistical_bounds_cache.cpp`：`StatisticalBoundsCache` | χ² 分位、非中心边界（二分）、正态乘子；按键精确位缓存（进程级 static） | `GateDStatistics.ExactBitKeyProducesCacheHitWithoutQuantization` |
| 开发运行器 | `apps/r0_r1_development.cpp` | 确定性 20Hz 流/故障注入/诊断导出/摘要 | 无单元测试；本轮实跑 7 场景 |

## 3. 交叉结论（回答路线图 §3.2 的总问题）

1. **三套数值表示并存**：SVD（canonical/rank/condition）、正规方程 `HᵀH`（全部右端求解）、LLT（主路径 + 误差界）。B/B1 的“单一平方根上下文”尚不存在。
2. **历史只有名义压缩边界**：窗口外的历史因子被压缩为一个 `BoundaryPrior` 块（15 行，来自 partial-QR + 特征分解），**没有任何故障方向/检测信息保留**（`F_b/T_b/d_perp` 摘要在源码中不存在）。所有可监测假设仅覆盖显式窗口内因子组；`history_recovery=active_window_only_maturity_delay`（manifest 自述）。这是 C1 的大缺口。
3. **H 满列秩是硬门**（`rank==columns && dof>0`），任何窗口秩亏直接 discard；与路线图“gauge/nuisance 零空间约化后继续”的目标不同——当前实现没有 gauge 处理，属已识别缺口。
4. **被保护量是 body 原点**（`C=[0|R]` 于当前状态块），与 UWB 测量的 tag lever arm（开发/仿真流使用 `lever_arm_body_m`）不同一；A/A3 需要明确“受保护参考点”契约（roadmap §4.2）。
5. **白化一次、表示分离**已由独立复算证实；`fault map` 的白化约定为 `whitened = W·raw`，UWB 模板 raw 值为 1.0/行，IMU 解析映射为 `L·analytic`（raw）→ `W·raw`（whitened）。
6. **诊断 v10 的 slope 列存在真实缺陷**：`integrity_monitor.cpp:1793` 从 `evidence[i].monitorability.protected_slopes` 取 slope，但批量路径只在 `pl_entry/hypothesis.monitorability` 写入 slope，`evidence` 副本保持结构体默认 `+inf`。实测 A_nominal 全部 236 行 `slope_x/y/z=inf`。**仅影响审计导出，不影响 PL 计算**（PL 走 `pl_entries`）。修复与回归测试建议放 A3/A4（属诊断结构变动）。**→ P2/A3 已修复**：`hypothesis_evidence.cpp` 在 `analyzeDynamic/analyzeFixed` 中同步写 `evidence->monitorability.protected_slopes`；P2 重跑 A 场景 5930 行假设全部有限（`oracle-results.json` O7 为回归守卫，oracle O5 逐帧对照生产斜率）。

## 4. 路线图旧稿与源码不符/过时条目

| 旧稿条目 | 源码事实 |
|---|---|
| `integrity_window.epochs` 语义不明 | 实测为**区间数**：节点数=epochs+1，`total_columns=15(epochs+1)`；epochs=10→165 列，epochs=20→315 列（同一 attempt 30） |
| 624 假设 = “最近报告” | 可由构造公式精确复现：`S = Na·(K+1)·(2+ramp) + 6K`。Na=8、K=20、ramp 开 → 504+120=624；SD=624+2·504·60=61,104。当前研究配置（K=10、ramp 关）实测 236/帧；SD 实测（census 变体）3,588/帧（6 epoch 帧） |
| “模板阶跃/斜坡以 nominal epoch index”——检查 | 斜率用真实时间戳 `measurement->timestamp.seconds()-onset_time`，非 epoch index；区间常值 IMU 用 epoch 名称 `:interval:<epoch>` 做物理源名 |
| 排除动作需要 kernel 按 excludable 分派 | 当前 kernel 由代数条件决定；动作提供方（generator）决定 added blocks；`test_integrity_v2` 已覆盖“同形状不同内容不可去重” |
| 40ms 正常帧目标严重不达标（629ms 参照） | 本轮开发运行器实测 core ~30-45ms/帧（Release，8 anchor、K=10、单故障；见 `baseline-report.md`）；629ms 属旧 `realtime_performance_benchmark` 口径（K=20、ramp 开、624/61,104 假设），需在 D 重测后再断言 |

## 5. 本轮未做/无法从本仓库确认

- 真机传感器数据、真实先验/包络证据（A4/C/D 输入）。
- GTSAM 版本只从本机 config.h 读出 major=4；manifest 记录 `4.2a5`（来源于构建时宏），详见 `runbook.md`。
- 旧结论“683/787 等计数”未在源码中出现，不再引用。

## 6. P2（A3+A4）新增接口与修复

### 6.1 新增/修改的公共契约

| 契约 | 位置 | 状态 | 证据 |
|---|---|---|---|
| 失败分类学（12 码 + `None` + `UnknownText`），线性化/超时/资源码标记 `check_implemented=false` | `include/uwb_imu_pl/common/failure_reason.hpp`、`src/.../failure_reason.cpp` | 已冻结 | `FailureTaxonomy.MapsExistingReasonTexts` |
| 快照身份（19 字段 + `identity_digest`，FNV-1a64；`kNotAvailableInSchema` 诚实占位） | `include/uwb_imu_pl/common/integrity_identity.hpp` | 已冻结（字段可扩展，语义不可改） | `ResultIdentity.DigestIsStableAndSensitive`、`oracle-results.json` O0 |
| 故障清单（词表/结构规则/未实现声明/动作不改变数值可取性） | `include/uwb_imu_pl/config/fault_manifest.hpp`、`config/integrity_fault_manifest.yaml` | 已冻结 | `FaultManifest.*`、`fault-manifest-resolved.yaml` |
| 规范配置键：`integrity_window.intervals`、`fault_models.max_fault_order`、`manifest_path` | `src/uwb_imu_pl/config/integrity_config.cpp` | 已冻结（冲突拒绝，无静默双写） | `config-migration.md`、`ConfigMigration.*` |
| 诊断 v11：attempts 追加 `primary_failure/all_failures/not_evaluated_checks/oracle_sweep_*`；新增 `diagnostic_snapshot_identity.csv` | `src/uwb_imu_pl/io/run_logger.cpp` | 已冻结 | `DiagnosticsV11.LoggerExportsFailureAccountingAndIdentityColumns`、`raw/runs_p2/*/diagnostic_snapshot_identity.csv` |
| 冻结候选 replay v5（v1–v4 仍可读；v5 = v4 布局 + identity 块；v5 保留 factor_inventory） | `src/uwb_imu_pl/estimation/candidate_replay.cpp` | 已冻结 | `GateDReplay.*`、`tools/replay_io.py` 解析 8 个 v5 bin 全 PASS |
| IMU 有限差分 sweep（多步长、重积分 oracle，导出列） | `include/uwb_imu_pl/integrity/imu_fault_subspace.hpp` | 已冻结（慢路径开关 `UWB_IMU_PL_IMU_FD_ORACLE`） | `IntegrityV2ImuOracle.FiniteDifferenceSweepMatchesAnalyticAcrossStepSizes` |

### 6.2 已修复的 P1 发现

| 缺陷 | 修复 | 回归证据 |
|---|---|---|
| G9：`hypotheses.csv` 斜率恒为 inf | `hypothesis_evidence.cpp` 同步写 evidence 侧 slope | `oracle-results.json` O5/O7；A 场景 5930 行全有限 |
| `BoundaryPrior.window_column_indices` 为空（块级列映射缺失，影响审计/复算） | `incremental_estimator.cpp` 填充 `iota(total_columns)` | `GateDReplay.*`、oracle O3（块行拼接 == H） |

### 6.3 B/C 所需接口的就绪状态

| B/C 需求 | 状态 | 说明 |
|---|---|---|
| 受保护量/参考点契约（C=[0\|R]、body_origin） | **已冻结** | 第 5 项证明义务；`ProtectedMapIsBodyOriginFiniteDifference` |
| 白化约定（一次、`W·raw`） | **已冻结** | 第 7 项证明义务 |
| Λ/λ 约定与失败斜率口径 | **已冻结** | 第 3 项证明义务；O6 |
| 失败分类/诊断导出（供 trace/复算） | **已冻结** | v11 + identity 表 |
| 历史摘要等价接口（`history_lineage_id`/`boundary_summary_id`/`coverage_epoch`） | **已映射（占位）** | 字段已存在，值 `NOT_AVAILABLE_IN_SCHEMA`；语义留给 C |
| 分组包络/coverage（G5） | **延迟** | 无接口实现；manifest 不声明该能力 |
| 惰性 FDE（B4） | **延迟** | 当前 eager；FDE-01/02 未开始 |
| 平方根内核上下文（B1） | **延迟** | 三套数值表示并存（§3.1）未变；改前须先锁定 §3 约定与测试 |

### 6.4 本轮明确未动

- PL/detector/evidence/fault-model/rank-update 的数学与全部阈值（0.25 gate、128 动作上限、`p_fa/p_md`、alert limits、风险预算）。
- `formal_eligible` 保持 `false`（Gate J 未完成）。
- 未实现平方根内核、历史摘要、分组包络、惰性 FDE。

## 7. P3（B1）统一平方根上下文

### 7.1 新增模块

| 模块 | 路径 | 角色 |
|---|---|---|
| `FrozenSquareRootContext` | `include/uwb_imu_pl/estimation/square_root_context.hpp`、`src/uwb_imu_pl/estimation/square_root_context.cpp` | 每冻结窗口一次的列置换 QR（默认 unit/natural）、隐式 Qᵀ、`informationSolve`（(HᵀH)⁻¹）、`leastSquaresSolve`、`faultResponse`（Y/Z）、`protectedMap`/`protectedCovariance`（Σp）、证书、符号分析缓存 |
| v12 诊断 | `src/uwb_imu_pl/io/run_logger.cpp`、`include/uwb_imu_pl/common/types.hpp` | 新表 `diagnostic_square_root.csv`（证书与上下文统计）；`hypotheses.csv` 追加 `z_rank/z_sigma_min/z_condition/z_classification` |
| Z 响应分类 | `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp:classifyDetectionResponse` | 每假设 Z_h=Q₂ᵀD_h 的小型 SVD：full / harmless nullspace / dangerous nullspace / indistinguishable（审计与 B2 输入） |

### 7.2 消费者切换

* detector：parity/statistic 由上下文提供（SVD 值仅作证书参考与回退）。
* hypothesis evidence：Σp/G/Γ 的求解改走上下文 `informationSolve`（R 三角回代）。
* 候选 KEEP：块缓存与内层交叉改走上下文 solve；`base.state_increment` 由上下文提供。
* **显式保留的旧路径**：`evaluateMapped`（`enable_shared_context=false` 的参考对照路径）
  继续使用 LLT；`solveFrozenInformation(numerics, …)` 作为证书不满足时的参考回退（计数）。
* **删除/降级**：LLT 正规方程 + 谱回退不再位于主路径（429→15 次求解 / 92,262→1,695 列）。

### 7.3 与 P2 契约的关系

`FrozenWindowNumerics` 增加 `spectral_state_increment`（参考解）与 `square_root_*` 摘要字段；
`LinearizedIntegrityWindow` 增加 `square_root`（进程内对象，不参与 replay 序列化，
读回后由 `finalizeIntegrityWindow` 重建）。replay 架构仍为 v5（无新增字段）。

## 附：P4（B2）新增/修改的代码与入口（2026-09-21）

| 文件 | 变更 | 说明 |
|---|---|---|
| `include/uwb_imu_pl/integrity/fault_model.hpp` | 新增 | `PairFamilySupport`、`pairFamilySupport()`、`hypothesisParametersIndependent()`（接收 mode id 列表，避免不完整类型） |
| `src/uwb_imu_pl/integrity/hypothesis_generator.cpp` | 新增/修改 | 族分类（UWB×UWB、IMU×IMU → Unsupported；同 `physical_source_id` → SharedParameters）、堆叠 map 秩守卫（`1e-10·max(1,diag)` 门限）、双故障候选计数（considered / rejected_unsupported / rejected_shared） |
| `include/uwb_imu_pl/integrity/hypothesis_generator.hpp` | 修改 | `GeneratedFaultModelSet` 追加三个候选计数 |
| `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp` | 重构 | 紧凑模式描述符（行跨度 + 紧凑块 + `HᵀA` + `Aᵀparity`）；交叉块按需预计算（单线程、只读缓存，修复首版多线程写 map 的崩溃）；容量回退路径（padded，逐模式计数）；共享方向/参数守卫 fail-closed；假设维数容量 fail-closed；`context->compact_*` 度量 |
| `include/uwb_imu_pl/integrity/hypothesis_evidence.hpp` | 修改 | `CompactModeCapacity`（模式数/窗口列数/单模式行数/总紧凑行数/假设维数上限）、`FrozenHypothesisNumerics` 追加紧凑度量 |
| `include/uwb_imu_pl/estimation/numerical_work_counters.hpp` | 新增计数器 | `fault_mode_columns`、`fault_cross_blocks`、`fault_cross_block_cache_hits`、`all_mode_gram_columns`、`mode_dense_allocations(+rows/cols)`、`compact_mode_rows/columns`、`compact_padded_equivalent_rows`、`compact_capacity_fallbacks`、`hypothesis_capacity_refusals` |
| `include/uwb_imu_pl/integrity/coverage_envelope.hpp`、`src/uwb_imu_pl/integrity/coverage_envelope.cpp` | 新增 | 覆盖证书：精确遍历、分组包络（包含性证明 `A_leaf = A_group·T` + 支配性义务）、容量上限、标签与 envelope id 查询 |
| `include/uwb_imu_pl/common/types.hpp`、`src/uwb_imu_pl/io/run_logger.cpp` | 修改 | `HypothesisAuditRecord` 追加 `coverage_label/coverage_envelope_id`；`hypotheses.csv` 追加两列；诊断 schema `v12 → v13` |
| `src/uwb_imu_pl/integrity/integrity_monitor.cpp` | 修改 | 每窗口构建精确覆盖证书并按假设导出标签 |
| `apps/r0_r1_development.cpp` | 修改 | 摘要追加 B2 计数器（11 个字段） |
| `test/test_integrity_v2.cpp`、`test/test_integrity_reference.cpp` | 新增用例 | `B2Registry.*`（4）、`B2Compact.*`（4）、`B2Coverage.*`（6）、`ReferenceFixture.GEO05*`（1）；更新 `FixedLowDim...`（守卫拒绝语义）与 v13 头部断言 |
| `CMakeLists.txt` | 修改 | 加入 `coverage_envelope.cpp` |
| `tools/gate_d_diagnostics.py`、`tools/validate_run_schema.py` | 修改 | 接受 `v13` 诊断 schema 与 B1/B2 两种 `hypotheses.csv` 头部 |
| `doc/evidence/.../tools/context_oracle.py`、`equivalence_compare.py` | 修改 | 新增 O8g–O8j（标签域/id 一致性/UNCOVERED 禁止/包络内部量 NOT_RUN）；schema bump 注释改为版本无关 |

## 附：P5（B3+B4）新增/修改的代码（2026-09-21）

| 文件 | 变更 | 说明 |
|---|---|---|
| `include/uwb_imu_pl/integrity/risk_budget_audit.hpp`、`src/.../risk_budget_audit.cpp` | 新增 | `RiskTermStatus`/`RiskLedgerTerm`/`RiskLedger`/`buildRiskLedger()`（10 项、charged 仅 VALIDATED）、`axisTailSplit()`（§5.9 每轴等分与计费） |
| `include/uwb_imu_pl/integrity/statistical_bounds_cache.hpp`、`src/.../statistical_bounds_cache.cpp` | 重写 | complement 分位数（小尾不再消减）、`noncentralityBoundaryVerified()`（收敛/残差/bracket 检查 + 保守侧端点）、`StatisticalBoundKey`（detector/contract/包络版本进键）、`normalTwoSidedMultiplierVerified()`、`clear()`、统计扩展（invalid_inputs/non_converged/policy_mismatches） |
| `include/uwb_imu_pl/integrity/protection_level_v2.hpp`、`src/.../protection_level_v2.cpp` | 修改 | `faultAxisBounds()` 单一来源（三路径共用）；结果新增 `hypothesis_tail_used/axis_tail_used/fault_multiplier_used/noncentrality_used`；无效输入拒绝；PL 门接受 `bound_from_projected_path` |
| `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp`、`include/.../hypothesis_evidence.hpp` | 修改 | `classifyDetectionResponse()` 增加每轴残差与 §5.5 投影界输出；三态决定落到证据路径（危险/不可分辨→fail-closed + 轴级 reason；无害→投影有限界）；`FrozenHypothesisPlEntry::bound_from_projected_path` |
| `include/uwb_imu_pl/integrity/hypothesis_generator.hpp`、`src/.../hypothesis_generator.cpp` | 修改 | `lazy_action_entities`（默认开）、`ensureActionEntities()`（幂等、按需）、`GeneratedFaultModelSet` 计数（constructed/bridge/deferred/built） |
| `src/uwb_imu_pl/integrity/integrity_monitor.cpp` | 修改 | 报警/屏障路径显式触发实体构造并计数；健康路径计数 deferred；每帧构建精确覆盖证书 + 风险账本并写入 attempt 诊断；`evidence_calls_*` 计数 |
| `include/uwb_imu_pl/estimation/numerical_work_counters.hpp` | 新增计数器 | `action_entities_constructed`、`bridge_blocks_built`、`candidate_graph_built`、`action_entities_deferred`、`evidence_calls_fault_path`、`evidence_calls_health_path` |
| `include/uwb_imu_pl/common/types.hpp`、`src/uwb_imu_pl/io/run_logger.cpp` | 修改 | v14：attempt 行新增账本（charged/declared/closes/all_validated/三项计数/terms 字符串）与覆盖证书列（status/叶计数/包络/证明/在线标志） |
| `apps/r0_r1_development.cpp`、`apps/gate_d_development.cpp` | 修改 | 摘要追加 B4 计数器；诊断导出 app 保持 eager 实体（`lazy_action_entities=false`） |
| `test/test_integrity_v2.cpp`、`test/test_fault_manifest.cpp` | 新增/修改 | `B3Risk.*`（4）、`B3ZeroSpace.*`（2）、`B4LazyFde.*`（2）；v14 断言 |
| `tools/gate_d_diagnostics.py`、`tools/validate_run_schema.py` | 修改 | 接受 v14 诊断 schema |

## 附：C1-b 边界构造状态（2026-09-21 更新）

* 现状：`buildIntegrityWindow` 的边界段仍为 partial-QR + 特征分解
  （`src/uwb_imu_pl/estimation/incremental_estimator.cpp:1187-1271`）；
  `boundary_summary_id = "partial_qr_schur_boundary"` 与
  `validity_assumptions = "history_nominal_boundary_only"` 未变化且仍然正确。
* **已就绪（模块级，未接线）**：`history_fault_parameterization.{hpp,cpp}`
  （历史故障列粒度/组合/注入/视野/容量）与 `history_summary_extraction.{hpp,cpp}`
  （定路：线性化行抽取 + 模块消元；实测 rel=2.9e-16、κ oracle 6.6e-47、无截断）。
* 替代关系（待 Part C 落地时按此记录）：`BoundaryPrior` 特征分解块 →
  `extractBoundaryRows(linearized)` + `buildHistoryFaultSummary()` 的
  `(R_b,d_b)`（含 `T_b` 故障响应与经 `DetectorOnlyRows` 进入的 `(F_b,d_perp)`）。
* Part C 就绪清单与残留风险：`history-fault-parameterization.md §3/§6`。
