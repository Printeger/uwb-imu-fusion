# C3（M2）：FDE 证据、post-FDE 选择风险与桥接/IMU 条件修复

范围：roadmap §8.3–8.5。合同：现有阈值（0.25 gate、128 上限、p_fa/p_md、alert limits、风险预算）
**不动**；`formal_eligible=false`；不 push。

本轮里程碑：**M2 = C3**（模块 + 测试 + 证据）。M3（C4）状态与续跑起点见 §6。

---

## 1. §8.4 profile likelihood（与 PL 度量分离，用原始白化似然）

实现：`fde_post_selection.cpp:profileLikelihoodEvidence`

$$
J_h^{\text{profile}}=\|r_c\|^2+\kappa_b-t_h^\top\Gamma_{h,\text{full}}^{\dagger}t_h,\qquad
t_h=Z_{h,c}^\top r_c+\xi_h,\qquad
\Gamma_{h,\text{full}}=Z_{h,c}^\top Z_{h,c}+\Omega_h
$$

| 要求 | 实现事实 | 证据 |
|---|---|---|
| 用**原始**白化似然，不得用 C2 风险缩放量 | 输入即 `fault_gram`（Γ_full）、`t_h`、通道 1 残差、**原始** κ_b；`constant_used` 回写审计；实现中不出现 w/Λ/W | FDE-03 `constant_used == 0.5` |
| Γ 的伪逆与秩 | 小 SVD + 1e-12 相对地板；记录 `rank`、`condition_proxy` | FDE-03（2×2 精确解 13.0 / −7.5） |
| 通道口径映射（与 C2 一致） | Z_c 仅取通道 1 行；Ω/ξ 即 C1 载体的 detector-only 交叉项 | 文档 + C2 的 `omega()/xi()` |
| 结构化候选池 | `StructuredCandidate`（unit/param dim/physical source/groups/bounded/centre-only/reference/bound） | FDE-03/04 |
| **禁止跨单位/跨维排序** | `rankStructuredCandidates`：混合单位 ⇒ `mixed_units` 拒绝；混合维数 ⇒ 拒绝；未声明单位 ⇒ 拒绝 | FDE-03（三种拒绝路径 + 同单位排序路径） |
| 旧先验污染 | 污染落在故障方向上（先验方差膨胀）⇒ 解释能量下降 ⇒ J **上升**；实测 −7.5 → +1.5 | FDE-03 |
| 隔离准则 | 保留既有 FDE 隔离准则（未改动）；本模块只提供证据与排序的结构化约束 | 结构 |

## 2. §8.5 post-FDE 保守保证（`buildGuaranteeGroups`）

$$
L_{a,d}=L_{\text{ref},d}+|p_{a,d}-p_{\text{ref},d}|
\;\subseteq\;\text{参考失败事件}
$$

| 要求 | 实现事实 | 证据 |
|---|---|---|
| 对**所有可能发布动作**做并集控制 | 预算对每个保证组各计一次（成员取该组最大 ε），再求和；超预算即 `valid=false` | FDE-03（2e-5+2e-5 vs 3e-5 拒绝、vs 4e-5 通过） |
| `GuaranteeGroup` 共用失败事件 | 需**同时**满足：共享参考证书/接受事件/时间/输出量 + 三角转移证据 + **恒等式成立**（1e-9） | FDE-05 |
| 多中心共用参考 | 3 个动作同参考 ⇒ **1 组**，只计一次（max 1.2e-5，非 3.3e-5 求和） | FDE-05 `charged_once=1.200e-05` |
| 不同参考不得少记 | 两个不同参考（结构相同）⇒ 两组独立计费（1.0e-5+1.2e-5 > 1.5e-5 拒绝） | FDE-05 |
| 缺一共享标志 ⇒ 不成组 | `shared_time=false` ⇒ 两组单元素（0 shared / 2 singleton） | FDE-05 |
| 三角恒等式被破坏 ⇒ 不得复用 | 动作自报界过乐观 ⇒ 单元素组 + 记录残差（9.334e-01） | FDE-05 |
| 找不到合格候选 ⇒ 保持不可用 | 预算不闭合即 invalid（调用方必须保持 unavailable，不放松标准） | FDE-03/04/05 |

**证书/记录字段表**（`GuaranteeGroup`）：`group_id`、`reference_certificate_id`、`action_ids`、
`charged_budget`、`worst_triangle_residual`；结果级：`total_charged_budget`、`singleton_groups`、
`shared_groups`、`valid`、`reason`。

## 3. §8.3 桥接候选决策序（`decideCandidateHandling`）

| 输入条件 | 处置 | 保护可用 | 说明 |
|---|---|---|---|
| 有验证随机/有界模型 | `UseInReferenceEstimate` | ✅ | 进入参考估计 + 双通道包络；`uses_bounded_model=true`（**不得**当高斯信息用） |
| 仅可用于估计中心 + 有有效参考 + 有时间/量转移界 | `TransferToValidReference` | ✅ | 用同一时刻同参考量的有效参考解做保护转移 |
| 仅中心 + 有参考 + 无转移界 | `UnprotectedDiagnosticOnly` | ❌ | 明确未保护 |
| 仅中心 + 无有效参考 | `UnprotectedDiagnosticOnly` | ❌ | 无参考可转移；**不是**"永久不可排除"，只是本动作不可保护 |
| 无模型无参考 | `UnprotectedDiagnosticOnly` | ❌ | 仅诊断 |

**IMU 条件修复（复用既有机制，未另造）**：本模块只做**决策与记账**——区间移除/桥接候选仍走既有
重积分/回滚/桥接路径；候选 PL 必须按**完整物理事件集合**计算（由调用方传入），被删源仍以
`groups_to_remove` 进入记账，**不得从风险账本移除**（FDE-04 断言：移除候选自成保证组并计费，
不会"消失"）。原始输入替换/重积分的数据来源与模型误差记录由既有路径提供；本模块要求其模型
被标记为 bounded（未验证 ⇒ 不得作为高斯信息）。

## 4. 决策差异表（对 M1 基线）

| 决策点 | M1（C2 后） | M2（C3 后） | 保守方向/依据 |
|---|---|---|---|
| FDE 证据量 | 各动作的统计量/Γ 并列，无单位约束 | profile likelihood + **结构化池**；跨单位/跨维显式拒绝 | 更保守：不可比的对象不再被排序 |
| 选择风险计费 | 只有"最终候选"的预算 | 对**全部可能发布动作**并集计费；共享参考只计一次 | 双向：同参考不再重复计费（不虚高），不同参考必须分别计费（不少记） |
| 候选可否发布 | 取决于 kernel/PL 结果 | 追加 post-selection 证明（组预算闭合） | 更保守：证明不闭合即不可用 |
| 桥接/仅中心候选 | 无统一处置序 | 三段式（验证模型 / 中心转移 / 未保护诊断） | 更保守：无转移界即明确未保护 |
| 被删源风险 | 无显式记账 | 自成保证组计费；不得移除 | 更保守：源删除不删除风险 |
| 单位混排 | 无检查 | 显式拒绝 | 更保守 |

对齐检查：**无覆盖缩水**——所有新增路径只把"原本可能被发布"的情况改为"带证明才发布"或
"显式未保护"；没有把任何原本被接受的判定放宽。

## 5. 计数与测试

| 量 | 值 |
|---|---|
| 新增模块 | `fde_post_selection.{hpp,cpp}`（≈300 行） |
| 新增测试 | `test_fde_post_selection.cpp`：FDE-03/04/05 = **3 用例** |
| 全量测试 | **392 / 0 errors / 0 failures**（基线 386 + 3 用例×2）；日志 `raw/run_tests_c3c4_baseline.log` |
| validation | `UWB_IMU_PL_VALIDATION_SHA=ecbaa8e --all` → **47 PASS / 0 FAIL / 6 NOT_RUN**（FDE-01..05 全 PASS）；日志 `raw/validation_c3c4.log` |
| 提交 | M2 checkpoint `wip(M2)` = `ecbaa8e`；证据提交见 `hashes-C1C2.txt` 头部 |

## 6. 里程碑状态与续跑起点（W1 后更新）

| 里程碑 | 状态 | 说明 |
|---|---|---|
| M2 = C3（模块） | 完成 | §8.4/§8.5/§8.3 语义与测试齐全 |
| **W1 = C3 生产接线** | **完成（本轮）** | ① profile 证据 + 结构化池接入 `FdeManager::decide`；② `GuaranteeGroup` ε 记账接入选择主链；③ 区间移除的"数据来源/模型误差记录"落到 `ExclusionAction`（详见 §7） |
| M3 = C4（模块） | 完成 | `publication_identity` 模块与测试已交付并提交（8c5330f） |
| W2 = C4 生产接线 | **未开始** | 续跑起点：`integrity_monitor.cpp` 发布路径（身份检查/状态机/看门狗）、诊断 v15→v16（含 §7.3 的三个 provenance 列）、场景重跑 + 发布差异表、OUT-01..03 升级到 pipeline 级 |

W1 的三项没有"半成品"：每项都改了生产主链、加了针对性与回归测试、并跑过全量套件与
validation（见 §7.4）。

## 7. W1：C3 接入生产主链（本轮）

### 7.1 ① profile 证据与结构化池接入 `FdeManager::decide`

* 证据侧：`hypothesis_evidence.cpp` 为每个被分析假设填充**可比性身份**
  （`FaultModeEvidence::unit_kind` / `parameter_dimension`）与**原始白化 profile 值**
  （`profile_j = conditioned_statistic = all_in − t'Γ†t`，`profile_valid` 标记可用性）。
  单位由 `FaultKind`/`SensorType` 显式映射（锚点偏差/斜坡 → 米；加速度 → m/s²；
  陀螺 → rad/s）；同假设各部件单位不一致 ⇒ `Unknown`（不猜）。
* 决策侧：`FdeManager::decide` 对**完整可能集**构造结构化池并调用
  `rankStructuredCandidates`：
  * 证据缺失或 profile 非有限 ⇒ `FdeStatus::ModelInvalid` **fail-closed**（不放宽）。
  * 池混单位/混维数 ⇒ 记录 `profile_pool_comparable=false`、`profile_pool_ranked=false`，
    **不做任何 profile 排序**（模块的拒绝语义被遵守），既有动作排序不受影响。
  * 可比的池记录 `profile_pool_ranked=true` 与逐假设 `plausible_profile_j`（顺序与
    `plausible_hypotheses` 一致，不改变其顺序）。
* **诚实说明（为何没有"替换排序逻辑"）**：`decide` 的可发布候选必须覆盖**同一个**
  完整可能集，因此所有合格动作的 profile 依据相同，任何 profile 键都不可能区分它们
  ——用 profile 值排序既无信息又违反"跨单位不得比较"的约束。故本轮把
  `rankStructuredCandidates` 用作**可比性审计 + fail-closed 门**（"包住"既有排序），
  而不是引入一个假的 tie-break；该结论由新测试
  `MixedUnitPoolIsRecordedAndNotRanked`（混单位仍按冻结准则成功选择）与
  `SelectionRiskChargedOverPublishableSet`（可比池记录 profile 值）锁定。

### 7.2 ② 选择风险按"可能发布的动作并集"记账

* ε_a = 动作 a 覆盖的假设的 `hmi_allocation` 之和（并集语义，每个假设只计一次）。
* 事件类：覆盖**同一可能集**的合格动作是同一参考事件的不同保护方案 ⇒ 折叠为一类
  （一类一收费）；覆盖不同并集的动作各自成类、分别收费。类 id 由覆盖并集哈希生成，
  `triangle_transfer_evidence` 保持 false（本轮没有三角传递证明），因此模块对每类分别
  收费，绝不把未证明的共享当成折扣。
* 收费 > `risk.p_hmi_total` ⇒ `commit_allowed=false` 且 `integrity_available=false`
  （"保持不可用，不放宽标准"）；账本（charged/available/classes）落在 `FdeDecision`。
* **诚实说明**：在冻结的分配策略下，并集收费 = 可能集总额，已被 `auditRiskBudget`
  的同一并集约束保证 ≤ `p_hmi_total`，所以健康场景不会触发拒绝；它现在的价值是
  （i）显式记账（ii）不一致即拒绝（非有限 ε、预算 ≤0、可能集超出审计集合）(iii) 为
  未来"按动作族分配"提供已接线入口。

### 7.3 ③ IMU 区间移除的数据来源/模型误差记录

* `ExclusionAction` 新增显式字段：`removal_data_source`（如
  `imu_interval:<physical_source_id>` / `uwb_range:<physical_source_id>`）、
  `model_error_record`（`whitening:<model_id>` 或 `model_error:none`）、
  `model_error_validated`。
* `actionForMode` 填充；`unite` 对并集动作做**并集合并**（`a|b`），使每个被移除区间的
  来源在合并后仍可追溯；`model_error_validated` 目前一律 false（本管线尚无 trial 级
  模型误差验证），并以此防止消费者把"已声明模型"误当"已验证模型"。
* 诊断导出**有意推迟**到 W2 的版本化步骤（v15→v16 附加列）：`candidate_replay` 的
  codec 是无版本号的往返格式，就地扩展会静默改变 replay 哈希，故在
  `candidate_replay.cpp` 留下显式说明而不做未版本化的线格式变更。

### 7.4 决策差异表（对 M3 基线 ff0dea2）

| 面 | 变化 | 依据/方向 |
|---|---|---|
| 证据记录 | 新增 `unit_kind`/`parameter_dimension`/`profile_j`/`profile_valid` 并全部填充 | 加性字段；无既有语义依赖 |
| `decide` 早退 | 证据缺失/非有限 profile ⇒ `ModelInvalid` | 保守（fail-closed）；健康管线不可达 |
| `decide` 记录 | 池可比性/逐假设 profile/每候选 disposition/并集账本 | 加性字段，不参与既有判据 |
| 动作排序 | **无变化** | 见 §7.1 诚实说明；设计使然 |
| 已发布动作/状态 | **无变化**（404/0，validation 50/0） | 全部场景与 oracle 一致 |
| 手工夹具 | `test_integrity_v2.cpp` 三个 FDE 夹具补身份/证据字段 | 契约变严（现在要求证据完整），非放宽 |
| `ExclusionAction` | 三个 provenance 字段 + 填充/合并 | 加性；旧构造路径默认空值 |

### 7.5 计数与验证

* 全量套件：**404 tests / 0 errors / 0 failures**（基线 398，新增 3 个用例；日志
  `raw/run_tests_cwire_w1.log`）。
* validation @ `b7feb9c`：50 PASS / 0 FAIL / 6 NOT_RUN（与 M3 完全相同，报告
  `validation-report.json` 已入库）。
* 新增用例：`ProfilePoolRefusesMissingEvidenceFailClosed`、
  `SelectionRiskChargedOverPublishableSet`、`MixedUnitPoolIsRecordedAndNotRanked`。
