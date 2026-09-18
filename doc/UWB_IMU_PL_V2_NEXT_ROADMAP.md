# UWB–IMU PL V2 下一阶段路线图：有效闭环与实时性

日期：2026-09-15。代码审计基线：`e6ded30`；最近实现 checkpoint：`8ada4f7`。

本文是下一阶段开发建议，不是已完成工作或新的验收结论。依据当前源码、
[V2 handbook](UWB_IMU_PL_V2_DEVELOPMENT_HANDBOOK.md)、
[P0–P2 报告](evidence/gate-d-p0-p2/report.md)和
[P3–P6 报告](evidence/gate-d-p3-p6/report.md)整理；本次未重新运行性能测试。
既有性能结果来自开发工作区，不能直接当作当前 clean SHA 的正式证据。

## 1. 结论与优先级

下一阶段主线：**建立真实可用的闭环基线 → 消除重复数值计算 → 改为稀疏/局部求解 → 完成实时与统计验收。**

目前 V2 的 transaction、window、ledger、joint detector、结构化 IMU modes、
rank update、bridge、health、post-FDE PL 已有实现。Handbook §3 描述的是早期
基线，不能再据此把 `prepareEpoch/commitEpoch` 等列为待从零开发的功能。
P3–P6 中的证书、共享 mode、PL 合并求解和常驻四线程池也已实现。

真正的两个阻塞是：

1. **性能离目标仍有数量级差距。** P6 `core_total p99=1353.91 ms`，
   20 Hz 对应 50 ms 周期，按 handbook §30 的 80% 预算目标为 40 ms，差约 33.8 倍。
2. **现有压力测试没有代表性成功闭环。** 测量段所有候选的 `post_passed`、
   `pl_evaluated`、`selected` 均为零；正常提交、成功排除和成熟 fixed-lag
   下的实际 PL 成本尚未建立。

建议暂停扩大故障模型、正式大矩阵和新 bridge 模型的开发投入，先完成下述 R0–R4。
保持 `IMPLEMENTED_UNVERIFIED / formal_eligible=false`，直至相应证据完成。

## 2. 已确认的成本与代码缺口

### 2.1 性能观测

P6 为 140 次输入、100 次预热、40 次测量、四 worker、单数值线程。

| 阶段 | p50，ms | 含义 |
|---|---:|---|
| 整轮 core_total | 1292.29 | 尚不含完整 ROS 排队与发布延迟 |
| candidate_evaluation | 542.73 | 数值拒绝路径，未测到成功 PL |
| integrity_window | 165.29 | 包含窗口收尾及其 dense SVD，不能全归因于边界提取 |
| base_factorization | 164.47 | 后续又对同一窗口做 SVD/LLT |
| hypothesis_evidence | 164.68 | unique modes 已合并 RHS，但仍有全窗口 dense map |
| health_actions | 67.73 | 候选与健康数据处理 |
| model_generation | 55.41 | 故障模型及动作相关构造 |
| hypothesis_audit | 42.70 | 仅构造审计记录已接近整个实时预算 |

各阶段分位数不可相加得到整轮分位数，也不能用各阶段 p99 相减预测优化后的 p99。
上述多个非候选阶段各自已超过 40 ms，单独加速候选或增加线程不足以达标。

### 2.2 源码确认

- `estimation/integrity_window_snapshot.cpp::finalizeIntegrityWindow()` 做一次 dense SVD。
- `integrity/joint_window_detector.cpp::evaluate()` 再做 QR 和基础状态求解。
- `integrity/hypothesis_evidence.cpp::evaluateAll()` 再构造 LLT、基础状态和残差。
- `estimation/rank_update_kernel.cpp::factorizeOnce()` 再做 SVD/LLT。
  因而“候选共享一次基础分解”已经实现，但“整条流水线只构建一次基础数值结果”尚未实现。
- `incremental_estimator.cpp::buildIntegrityWindow()` 已使用部分 multifrontal QR；
  仍逐次构造并线性化 boundary graph，且完整窗口按 dense 315 列状态矩阵表示
  （20 个区间、21 个 15D 状态）。后续应测量边界子阶段，不能直接认定它占全部 165 ms。
- `HypothesisEvidenceEvaluator` 为每个 mode 构造全窗口零填充矩阵。
  现有组合 Gram 缓存可以保留，继续去除零块运算和重复索引。
- `integrity_monitor.cpp` 先执行候选 kernel/post/PL，再进入
  `FdeManager::decide()` 判断是否覆盖完整 plausible set。
  P6 5120 个实际内核中，5080 个最终 coverage rejected。
- 正常 detector pass 且无健康屏障时，代码已经只保留 `KEEP_ALL`；
  不能把“增加 nominal 只算一个动作”当作新优化。但此前的模型生成、evidence、audit 仍会执行。
- `ImuFaultSubspaceBuilder::build()` 每次调用都做六轴正负扰动，共 12 次重新预积分。
  Handbook §12.4 要求正式路径使用 analytic sensitivity、有限差分作为测试 oracle；
  这是可移出每历元生产路径的验证成本，但接口和验证状态必须一并调整。

### 2.3 行为问题必须先诊断

P0–P2 报告显示，输入走到 7 s 时状态仍停在 3 s，pending 达 4 s、801 个 IMU
样本、连续拒绝 80 次。P6 也仅覆盖持续拒绝路径。由此推断，长 pending 可能
进一步增加重积分成本并破坏线性化适用范围；这条因果链仍需按历元实测验证。

优先核对以下具体疑点，不预先认定它们是全部失败的根因：

- 性能程序的轨迹初始真速度是 `(0.45, 0.45, 0) m/s`，初始化只设置位置，
  `NavigationState` 默认速度为零。分别保留“与真值一致的稳态基线”和
  “有初始误差的鲁棒性用例”，观察初始误差对拒绝的影响。
- `relinearize_skip=200` 与全窗口 `||delta||≤0.25` 同时存在。
  记录各状态的姿态、位置、速度、bias 增量以及重线性化历史，定位超限来源。
- `RISK_BUDGET_INVALID` 需要输出分项预算、hypothesis 数目和求和误差。
  当前多处直接累加后与总预算比较，应区分真实超支、分配/重复计数问题与浮点累加问题。
  不得通过扩大风险预算或比较容差直接掩盖失败。

## 3. 分阶段执行路线

阶段按退出条件推进，时间只作排期参考，不承诺 40 ms 必然可达。

### R0 — 建立行为与性能基线（建议 2–4 个工作日）

**工作内容**

1. 新建 development 场景集，保留旧的强制报警压力入口及其历史证据。
2. 至少覆盖：nominal、单 anchor 成功排除、IMU 成功 bridge、联合/并集排除、
   合理 ambiguity unavailable、持续拒绝与超时恢复。
3. 运行真正经过 integrity 决策的正常序列，至少超过 200 次提交并跨过边缘化；
   再在成熟窗口注入故障。205 次直接提交的机制测试不替代此项。
4. 为每种分支保留 raw-stream replay 和完整 post/PL 证据。
   当前 `candidate_replay` 的冻结数值内核重放不等于完整风险集重放。
5. 分开记录：成功 action、PL 有限、PL 在 AL 内、formal eligible。
   正确排除但 PL 超 AL 是合法 unavailable，不能强制把它计为算法失败。
6. 增加窗口行列数、变化秩、模式数、动作生成/实际求解数、分解次数、
   pending 时间、状态年龄、原始样本数、连续拒绝及边缘化次数。

**退出条件**

形成可复现的小场景集；所有预期可恢复正向场景进入 post/PL 和真实提交，
负向场景明确 fail closed。若正向场景失败，先提交根因和最小修复，
不得把无成功输出的 smoke 继续作为唯一优化基线。

### R1 — 合并整条流水线的基础数值计算（建议 3–5 个工作日）

**工作内容**

引入不可变的 `FrozenWindowNumerics` 共享上下文：

```text
冻结窗口与版本
  → 一次精确谱信息 + 基础求解器
  → base delta / parity energy / rank / dof / logdet
  → joint FD、mode evidence、KEEP_ALL、rank-update candidates 共用
```

- 第一小步保留现有数值方法，只消除重复 SVD、LLT、QR 与基础状态求解。
  保留稳定参考路径；若使用不同求解形式共享结果，必须验证 detector/PL 等价。
- 用计数器验证同一冻结窗口只计算一次所需基础分解，KEEP_ALL 直接引用基础结果。
- 共享 immutable 数据，避免 `base.window = window` 等大对象复制扩散到候选。
- 把有限差分拆为显式 oracle 接口，生产构建保留 analytic validity checks；
  不能把 `analytic_verified` 无条件写为 true。测试覆盖 handbook §12.4 的运动、
  bias、不同积分时长，并用开发 replay 持续核对 analytic 与 oracle。
- 把审计中的整数/数值证据与字符串/CSV 格式化分离，使用完整性可校验的字典引用。
  benchmark 当前在生成 hypothesis audit 后清空它，应从生成端明确日志模式。
  保留候选、风险与事务可追溯性；同时报告包含日志写入的端到端成本。

**退出条件**

R0 场景的 detector、candidate、PL、risk、action 和事务结果等价；
沿用 ADR 0002 的 `1e-9 / 1e-7 / 1e-6` 分级容差；实际计数证明重复分解消除。
记录收益，不预先宣称可因此达到 40 ms。

### R2 — 动作资格前置与故障 mode 紧凑计算（建议 3–5 个工作日）

**工作内容**

- 在完整 plausible set 和物理覆盖关系冻结后，先检查动作是否具备最终选择资格，
  再执行 kernel/post/PL。复用 `FdeManager` 的同一套覆盖逻辑，避免两套标准。
- 保留所有风险 hypotheses、监测性和预算计算；这里只跳过可证明不会被选择的 action。
  被跳过动作记录 `SKIPPED_COVERAGE` 及覆盖证据，不伪造数值有效性。
- 保留独立 exhaustive 模式：所有 128 个动作真实求解，供旧 Gate D 压力合同和等价性检查使用。
  在线资格检查节省的耗时不能计作旧 128-kernel 合同的达标收益。
- 从 mode 的实际非零 factor blocks 构建 `HᵀA`、score 和 Gram，复用唯一 mode
  的解与组合交叉项。完整相关 UWB covariance 必须重白化替换，不能删除已白化子行。
- nominal 的 hypothesis/PL 风险仍须完整；仅在证明不影响 health、PL 和风险语义后
  才延后纯 isolation 诊断，不做“无报警就省略故障 PL”。

**退出条件**

在线调度与 exhaustive 重放得到相同 selected action、PL、ambiguity 和风险结果；
不可覆盖场景也验证一致。成功 UWB、IMU、联合场景中有实际 PL 计时。

### R3 — 实现 handbook §16.5 的稀疏/局部求解（建议 1–2 周，主要技术投入）

**工作内容**

1. 将窗口因子按 key/block 稀疏表示，使用一次局部稀疏平方根分解或等价稳定求解器。
   不能仅把 dense 矩阵换容器、随后继续 dense 化。
2. candidate 只提取 `protected keys ∪ removed keys ∪ added keys` 所需的 solves，
   对变化块做低秩更新；PL 只求 protected、mode、bridge 所需 RHS。
3. 保留全窗口统计量、rank/dof、logdet 和完整 delta 的检验能力。
   Schur 消元必须保留被消元残差能量与恢复映射，不能把局部维数直接当作 detector DOF。
4. 条件证书继续针对原数值合同；变量缩放或局部化后的条件数不能替代原窗口条件门限。
   近边界、不确定或不稳定情况仍用精确参考路径并计时。
5. 用 trace 把 `integrity_window` 拆为 boundary graph、线性化、消元、组装、谱检查。
   在确认成本后，复用冻结边界/分隔变量信息与可验证未变化块。
   每次 commit、重线性化、噪声变化、历史替换、边缘化和 reinit 都要检查缓存依赖。

**关键边界**

持久 anchor 或历史 union action 可能接触窗口内很多状态，“低秩/局部”并不保证
每个动作都是很小的矩阵。必须按实际 touched dimensions 和变化秩分组测量，
保留最坏情况；不能把 20-interval detector 偷换成 current-only detector。

**退出条件**

全部 R0 场景及病态/相关噪声/历史替换 oracle 通过；1/4 worker 结果一致。
测出 nominal、成功 FDE、128-action exhaustive、成熟 fixed-lag 四类完整耗时。
若主要阶段仍远超预算，在此评估结构复杂度和支持范围，不继续仅靠线程微调。

### R4 — 有界拒绝恢复与 ROS 实时闭环（建议 3–5 个工作日）

R0 发现的实际行为 bug 应立即修复；本阶段系统完成持续运行验证。

- 明确 consecutive rejection、最大 pending duration 与状态年龄的策略。
  达到限值后通过独立 bridge 或受控 reinit 恢复，无法满足模型则继续 unavailable；
  不允许为了推进时间而提交未经批准的 suspect IMU。
- 全程保持 prepare/discard 零 backend 更新，commit 一次更新，bridge 超时与
  `HISTORY_PRIOR_CONTAMINATED` fail closed。
- 保留 best-effort 与 integrity available 的独立标签，旧状态不能以当前时间冒充新状态。
- 用真实 200 Hz IMU + 20 Hz UWB 输入测试排队、deadline miss、发布延迟与 RSS。
  为有界队列/过期输入制定可追溯策略；丢弃 IMU 会改变预积分和模型，不能静默处理。

**退出条件**

正常运行不形成持续积压；故障/拒绝序列内存和 pending 有界，恢复状态可解释。
额外报告 capture/arrival-to-publish 延迟和 state age；不能只报告 core_total。

### R5 — 分层验证，再执行正式 campaign

采用以下开发反馈层级，减少每次修改都跑长实验的成本：

| 层级 | 触发 | 证据 |
|---|---|---|
| 单元/oracle | 每次相关数学或事务修改 | 核函数数值等价、事务、版本和覆盖 |
| 冻结内核 replay | 每次性能修改 | 同一窗口/动作的耗时、分配和路径计数 |
| 小型 raw-stream 场景 | 每个可评审 checkpoint | 正常/成功排除/恢复的全链行为与 PL |
| 延长稳态运行 | 小场景通过后 | 超过 lag、真实 marginalization、尾延迟与 RSS |
| 正式冻结运行 | 功能/性能预检具备通过条件后 | 原协议规定的样本、seed、SHA 和统计分析 |

复用现有 bulk worker、四分片、断点续跑与流式分析，不重复开发 campaign 基础设施。
先用 pilot 实测不同 cell 的成本，估算完整矩阵的计算量与磁盘，再安排正式运行。
不根据 1.35 s 的拒绝 smoke 外推所有统计场景的耗时，也不因少量零 HMI 宣称风险闭合。

旧 Gate D 合同保持：Release、fixed-lag 200、四 worker、单数值线程、
每轮 12000 输入、100 预热、11900 测量、128 个实际求解动作、三个 repeat
各自 `core_total p99≤40 ms`。另外提供生产调度与成功 PL 路径性能结果。
若需修改 benchmark 初始化/故障流程或调度合同，应建立新协议与明确差异，
不能覆盖旧失败记录或把新协议结果直接标为旧 Gate D PASS。

性能预检通过后推进 E–I 的边界扫描、bridge held-out、persistent recovery
与 ROS 等价证据；Gate J 的噪声/风险/bridge 标定和独立审查单独闭合。

## 4. 40 ms 工程预算与决策点

按 handbook §30 将 40 ms 分配如下；这是设计预算，不是当前测量值或统计可加性结论。

| 环节 | 预算，ms |
|---|---:|
| prepare/preintegration | 6 |
| window/boundary | 6 |
| shared base + joint FD | 6 |
| hypothesis evidence | 4 |
| candidate rank updates | 8 |
| post-FDE PL | 4 |
| commit/finalize | 6 |

模型生成、health 和必要审计必须纳入相应环节，不能在预算外隐藏。
最终只以实测整轮 p99 和 ROS 端到端结果判定。

在 R3 完成后设置决策点：若保持完整监测范围和原 128-action 合同仍不可达，
先给出按窗口维数、变化秩、mode/组合数拆解的成本证据，再讨论目标硬件、
运行频率或监测范围的研究取舍。缩短窗口、降低频率、改变风险集均需要新的
scope/ADR/协议与统计证据，不能通过关闭监测项获得表面上的 V2 达标。

## 5. 下一次开发直接从这里开始

建议第一批按三个可独立评审的提交组织：

1. **场景基线与拒绝诊断**：补正确初速度的 development nominal、成熟 lag
   场景和失败分解，保留原 benchmark；找出无成功 FDE/PL 的实际阻塞。
2. **共享窗口数值上下文**：消除重复基础 SVD/LLT/QR，增加分解次数断言与 oracle 对比。
3. **oracle 与生产计算分离**：拆出在线 12 次扰动重积分，完善 analytic 验证状态；
   推迟审计格式化，保留可审计数值证据和完整端到端计时。

随后执行 R2 的动作资格前置与 compact modes，再投入 R3 的稀疏/局部求解。
这条顺序既能尽早取得可解释的收益，也能避免继续优化一个始终拒绝、从未完成 PL 的闭环。
