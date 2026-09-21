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

## 6. 里程碑状态与续跑起点

| 里程碑 | 状态 | 说明 |
|---|---|---|
| M2 = C3 | **完成（模块 + 测试 + 证据）** | §8.4/§8.5/§8.3 语义与票据齐全；IMU 条件修复以"决策 + 记账"落地并复用既有重积分/回滚/桥接 |
| M2 未接线的部分（如实记录） | — | ① 结构化候选池/profile 证据**接入** `FdeManager::decide` 的实际序列（当前为模块级，未替换既有选择逻辑）；② `GuaranteeGroup` 的 ε 预算来源接入既有风险账本执行；③ IMU 区间移除的"数据来源/模型误差记录"字段落到 `ExclusionAction`/诊断（现由既有路径隐含携带） |
| M3 = C4 | **未开始** | 续跑起点：`integrity_monitor.cpp` 发布路径（提交绑定表/状态机）、`types.hpp`（`AttemptDiagnostics` 身份字段）、`run_logger.cpp`（新增列）；验收 OUT-01..03（同一时间点改解、不同时间/坐标系、过期/队列/时钟跳变） |

未把 M2 的①–③当作已完成：它们需要改动选择/账本主链并重跑全部场景与 oracle（与 M1 同量级），
在本窗口内无法完成"实现 + 完整验证"闭环，按纪律记录为待办而非半成品。
