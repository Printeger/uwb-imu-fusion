# UWB–IMU–PL：统一完整性内核开发路线图

**用途：供 Codex 在 `Printeger/uwb-imu-fusion` 的 `feature/realtime-uwb-imu-pl` 分支上逐项实施。**  
**编制日期：2026-09-21**  
**输入：**《UWB_IMU_PL_方案复核与综合改进.md》（下称 G0）与《Claude roadmap 3.md》（下称 C3）。  
**交付目标：**保留增量 FGO、slope-based RAIM、UWB/IMU 故障监控与条件性 FDE；默认单故障，同一实现支持配置开启已声明的双故障族；减少重复数值计算，补全历史、风险与发布语义。

> **代码证据边界，必须保留：本轮没有成功取得 GitHub 当前分支源码或 HEAD。** 分支页、raw 文件访问未取得内容；容器中的 `git ls-remote` 因域名解析失败；项目文件检索找到的是讨论稿而不是源码快照。本文不是“当前 HEAD 已审计通过”的报告。旧稿涉及的路径、624/120/787 等计数及 629 ms 等性能数字均不得当作最新代码事实。
>
> 为避免再写一份只能讨论、不能开工的评审，本文将**对齐实际工作树、读取当前实现、复用已完成能力**写成工作包 A 的首批操作。Codex 完成这部分后即可按真实路径实施剩余任务；不是要求用户再接受一个“普查版本”，也不是先重做一套过渡算法。

## 阅读与执行顺序

先读 §1 的审查结论、§2 的四个工作包和 §3 的代码对齐规则，再按 §6 执行。§4–5、§7–9 是实现时必须遵守的接口、数学与配置规格；§10 是测试清单；§11–12 是性能与合并验收。§13 可直接作为 Codex 的任务指令。

**本文中 `[G0 §x]`、`[C3 §x]` 指上传文档的章节；`[R1]` 等指文末本轮核查的公开原始资料。** “本轮修订/推导”表示本文新增内容，不冒充附件或当前代码已有的实现。

---

## 1. 对 Claude roadmap 3 的审核：采纳什么，改掉什么

C3 正确撤回了“120 个备选模式等于 120 维同时故障”、错误的冗余计数、对幅值分区界的否定和仅靠 `B_b` 保存历史完整性的主张。它增加了系统普查、独立参考和验收条件，这些应保留。[C3 §1、§3]

但它新增的五条原则不能原样变成代码分支；其 S0–S8 也不需要对应九次分阶段开发。

### 1.1 审查决策表

| C3 的主张 | 审核结论 | 本路线图中的处理 |
|---|---|---|
| 逐假设考察自由度，不把所有模式维数相加 | 采纳；但 `q_h ≤ ν` 不是有限位置 PL 的完整判据 | 同时检查状态输出可估计性与 `ker Z_h ⊆ ker G_h`；参数计数仅用于诊断。 |
| `excludable/bound_only` 决定数值内核 | **不采纳** | 数值路径由故障子空间的代数性质、白化与历史来源决定；动作策略独立。 |
| 对 IMU 做辅助子集协方差会自动得到无穷，说明 IMU 物理上不可排除 | **不采纳** | 删除某些过程约束后，当前位置仍可能被绝对测量确定。应检查具体输出与剩余模型，不按传感器名称判断。 |
| 只借用子集协方差算 slope，不顺便切换为 SS detector | 采纳 | 保持残差检测事件；辅助子集只是数值查询。不能混用 SS/RB 阈值。[R4] |
| 历史能量不断池化，检出能力随任务时间单调下降 | 有条件成立，原表述过强 | 新增历史行没有故障均值时可能稀释检测；一般情况下历史非中心参数也会改变，不能宣称普遍单调下降。 |
| 两个“独立”历史/当前统计量即可解决问题 | 不充分 | 必须定义残差空间、相关性、阈值和对应 PL 证明；本稿给出不依赖两个统计量独立性的保守分区构造。 |
| `template ⊆ recent-k ⊆ free` 是通用覆盖阶梯 | **错误** | 早于 recent-k 的模板通常不在最近 k 个历元的自由子空间里。改成带显式包含证据的覆盖图。 |
| 源基站固定，启动时即可检查所有后续可监测性 | **错误** | 启动检查模式、维数约束、依赖和风险；每帧检查实际几何、噪声、数据缺失、历史与数值秩。 |
| 运行时不再出现启动报告未预告的 `inf` | 不合理验收条件 | 运行时退化是合法状态；验收要求原因明确、保护声明不缩水、输出不误标可用。 |
| `dof_margin: 4` 是可采纳性硬门 | 无通用依据 | 不作安全定理；最多作为可用性预警参数，不能替代零空间判据。 |
| `O((Nb)^3)/O(Nb^3)=N²`，因此结构性加速约 440 倍 | 只保留渐近阶数差异 | 不能把大 O 的比值当成固定尺寸的速度比例；分解常数、投影、概率计算与其他成本仍在。 |
| 所有普通帧、所有模块必须零动态分配 | 目标范围过宽 | 稳定容量下的模式循环和小矩阵热区要求零分配；外部稀疏分解与 ROS 分配单独计量和约束。 |
| 风险闭合放到 S7、双故障到 S8 才打开 | **改开发组织** | 风险接口、单双故障与正常帧内核同时开发；历史风险与历史摘要同包；FDE 选择风险与发布事务同包。 |

本表被评审主张的定位：C3 §2.1–2.5、S1/S3/S4/S6/S7/S8。G0 中已有的“历史覆盖与检测能力不同”及“可监测性须面向保护输出”也继续有效。[G0 §3.3、§6.5]

### 1.2 三个可直接写成单元测试的反例

**反例 A：动作标签不能选择内核。** 两个标量时刻状态的线性模型为

\[
H=\begin{bmatrix}1&0\\0&1\\-1&1\end{bmatrix},\quad
A=e_3,\quad C=\begin{bmatrix}0&1\end{bmatrix}.
\]

前两行是两个时刻的绝对位置观测，第三行是过程约束。删除第三行后 `H_-S=I₂`，位置仍可估计，且

\[
s^2=C(P_{-S}-P)C^T=1/3.
\]

这不是完整 IMU 模型，但足以否定“删除过程约束必然产生无穷位置 slope”这个普遍命题。真实 IMU 场景必须按实际模型检验。

**反例 B：所谓覆盖阶梯不一定嵌套。** 四个测量时刻，早期瞬时模板为 `e₁`，最近两时刻自由剖面为 `span(e₃,e₄)`；前者不包含于后者。两者都包含于全时域自由空间，但不能按线性阶梯排序。

**反例 C：固定基站不等于固定几何雅可比。** 四个基站位于 `(±1,±1,0)`；目标位于 `(0,0,0)` 时测距位置雅可比秩为 2，位于 `(0,0,1)` 时可为 3。IMU 姿态、运动激励、观测可用性和边界先验也会继续改变融合几何。

另一个必须纠正的 S1 推断：**一维假设也可以完全不可检测**，例如 `H=(1,0)^T、A=(1,0)^T、C=1`。因此第一级诊断就得到 `inf`，不能排除故障几何问题；第二级失败也不自动证明模板代码有 bug。

**无害零空间解析 fixture：**取 `H=[[1,0],[0,1],[1,0]]、A=[e₂,e₁]、C=[1,0]`。此时 `q=2>ν=1`，但第一个故障参数只改变第二个 nuisance 状态；`G=(0,1/2)`，有限位置 `s²=1/2`。该 fixture 防止把 `q≤ν` 或额外的 dof margin 错写成所有位置输出的硬性必要条件。

---

## 2. 一个目标实现，四个合并工作包

### 2.1 工作包与原 S0–S8 的对应

| 工作包 | 合并范围 | 一次完成的内容 | 完成证据 |
|---|---|---|---|
| **A：工作树对齐、声明与参考基线** | S0、S1、S2、S3，以及 S7 的事件/预算定义 | 读最新本地代码、普查、失败诊断、manifest、接口、独立参考、测试 fixtures | 代码映射、真实 census、冻结配置、解析与稠密参考测试 |
| **B：统一在线数值与保护流水线** | S4、S5、S8 的单双故障，加正常帧风险闭合 | 平方根上下文、精确字典、按需交叉项、分组覆盖、单双故障同引擎、惰性动作、风险计算 | 逐假设参考对照、覆盖证据、无误删模式、热区分配与基准 |
| **C：历史、故障处置与原子发布闭环** | S6、S7 的历史/非线性/post-FDE/发布内容 | 故障与检测摘要、检测事件对应 PL、历史生命周期、IMU 条件修复、选择风险、输出证书 | 跨边界一致性、残差分布检查、错排/失效/异步回归 |
| **D：端到端验收与有证据的性能收尾** | S8 的整体验收、必要的局部增量优化 | 多轨迹、单双故障、耗时/内存/队列、证明义务检查、清理旧路径 | 可重复命令、数据报告、测试结果、剩余限制、发布验收记录 |

**依赖是 `A → {B,C} → D`。** A 冻结接口后，B 与 C 的局部实现/测试可并行；最终合并必须共同通过。每个工作包可以一个 PR，也可以几个相邻提交，不需要四个发行版本，更不需要 V0/V1/V2/V3 并行维护。

### 2.2 不再单独安排的“阶段”

- 不先写一套单故障算法，再为双故障重写类型和缓存；在 B 中同时测试 order=1、2。
- 不先做稀疏 Cholesky，再整体推翻成 QR；统一平方根接口，选择适合现有依赖的生产实现。
- 不先把所有故障改成 free-profile，再为了可用性退回模板；精确声明与计算包络从开始分离。
- 不先丢掉历史检测代价，再补历史风险；摘要结构与检测/PL 一起实现。
- 不等到最后才考虑风险预算、版本标识和发布身份；A 定义、B/C 实施、D 验收。
- 不强制在任何测试之前做全面跨帧 rank-update；B 已复用索引/符号结构/版本缓存，进一步数值增量更新由实际热点决定。

### 2.3 最终必须保持的产品语义

保留主估计器的增量 FGO，不另建一个主 KF 估计器来回避问题。正常健康帧不构造排除图；但当次 PL 必须覆盖当次 manifest 要求的全部故障，或者由有效分组上界覆盖。报警、历史失效、数值无法确认或超时后，估计器可继续运行，**受保护输出必须明确不可用**。

模型下有限的计算结果和真实部署有效性使用同一实现、不同证据字段区分；不能因为单元测试通过就自动宣布传感器先验、非线性包络或完整性认证已经成立。这不是拆“工程版/正式版”，而是避免错误发布声明。

---

## 3. 与最新代码对齐：Codex 的第一组操作

### 3.1 源码与版本规则

Codex 在实际工作区执行，不假设本文编制时的网络限制仍存在。

1. 读取仓库根及相关子目录的 `AGENTS.md`、构建说明和贡献约定。
2. 记录 `git status --short`、当前分支、完整 HEAD、remote、未提交改动和子模块版本。不得覆盖用户修改，不使用 `reset --hard` 或 `clean -fd`。
3. 能联网时 fetch 目标分支，比较本地 HEAD 与远端；fetch 不改变工作树。不要自动将开发工作树 checkout 到另一分支。
4. 若本地工作树不是目标分支的最新提交，写清选用的 base SHA 与差异；有用户修改则使用获准的独立 worktree，或在当前树保留补丁继续核对。不能把旧 SHA 称为“最新”。
5. 若既没有源码又无法获取源码，只能完成文档/数学检查，停止任何“已修改代码/已通过仓库测试”的声明。

**可执行的非破坏性信息采集示例：**在预期仓库根运行；生成目录属于新的证据目录。

```bash
set -eu
ROOT=$(git rev-parse --show-toplevel)
cd "$ROOT"
OUT="doc/evidence/integrity-kernel-refactor"
mkdir -p "$OUT"
{
  date -u '+%Y-%m-%dT%H:%M:%SZ'
  git branch --show-current
  git rev-parse HEAD
  git status --short
  git remote -v
  git submodule status
} > "$OUT/worktree-before.txt"

if git remote get-url origin >/dev/null 2>&1; then
  if git fetch origin refs/heads/feature/realtime-uwb-imu-pl; then
    git rev-parse FETCH_HEAD > "$OUT/remote-head.txt"
    git rev-list --left-right --count HEAD...FETCH_HEAD \
      > "$OUT/local-vs-remote.txt"
  else
    printf '%s\n' 'REMOTE_NOT_VERIFIED' > "$OUT/remote-head.txt"
  fi
fi

git ls-files > "$OUT/tracked-files.txt"
find . -name AGENTS.md -not -path './.git/*' -print \
  > "$OUT/agent-instructions.txt"
```

执行后先读日志，不把远端比较结果自动当成允许更新工作树的授权。记录版本完成后才将证据目录加入提交；不要提交原始敏感数据或机器上的凭据。

### 3.2 找真实入口，不按本文虚构文件

以下路径只来自旧稿。[G0 §9.4；此前 GPT 讨论稿§1] 对每项定位实际符号、调用链和测试；已完成的能力直接复用，不重新创建同名模块。

| 旧稿提到的入口 | 必须读取并回答的问题 | 对应工作包 |
|---|---|---|
| `src/uwb_imu_pl/estimation/integrity_window_snapshot.cpp` | H/z/C 从哪里来？包含哪些先验/因子？是否重复白化？是否维护稠密 SVD、正规方程、LLT 多套表示？ | A、B、C |
| `src/uwb_imu_pl/estimation/rank_update_kernel.cpp` | 当前数值可靠性与候选更新如何做？`0.25` 检查什么量？候选中心是否等于发布中心？ | A、B、C |
| `src/uwb_imu_pl/integrity/hypothesis_generator.cpp` | 模式究竟是单轴、整区间还是同时多轴？是否同时构造动作/桥接？实际观测索引如何维护？ | A、B |
| `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp` | 是否已批量求解/共享缓存？是否仍形成完整 all-mode Gram？健康状态机依赖哪些证据？ | A、B、C |
| `src/uwb_imu_pl/integrity/protection_level_v2.cpp` | 非中心参数的平方约定？逐轴/逐模式预算？不可用分支？post-FDE 有何语义？ | A、B、C |
| `src/uwb_imu_pl/integrity/integrity_monitor.cpp` | 正常保护、隔离、恢复、候选提交、发布的真实时序？ | A、B、C |
| `config/realtime_uwb_imu_pl_research.yaml` | YAML、launch、ROS 参数和命令行的优先级？epochs 指节点还是区间？有效 order/family？ | A、D |
| `doc/adr/`、`doc/evidence/` | 哪些旧决策/基准仍适用？当前 `IMPLEMENTED_UNVERIFIED` 的依据与未完成项目？ | A、D |
| 边缘化、IMU 预积分、输出消息、模拟器与测试模块 | 使用哪些 GTSAM API？是否保存原始 IMU？实际消息身份和构建/回放入口是什么？ | 全部 |

若文件迁移，用 `git grep` 搜索符号/字段，而不是新建第二条 pipeline。填写 `code-map.md`：

```text
责任 | 当前路径:符号 | 调用者 | 对应测试 | 证据 SHA
     | 已实现内容 | 本轮增量修改 | 不再需要新增的模块
```

### 3.3 构建、测试与依赖入口也要从实际工作树读

检查 `CMakeLists.txt`、`package.xml`、CI、现有 test target、launch 参数及数据要求。ROS1/catkin、ROS2/ament 或纯 CMake 的实际入口以当前源码为准，不能因为旧稿出现 `roslaunch` 就硬编码命令。

在证据目录记录 `BUILD_CMD`、`UNIT_TEST_CMD`、`INTEGRATION_TEST_CMD`、`REPLAY_CMD`、GTSAM/Eigen/编译器版本、构建选项、线程配置。Eigen/GTSAM 新文档里的接口不能直接视为本机已安装版本支持。[R1–R3、R5]

旧稿的模拟器命令只作为待核对入口，不视为本轮跑过：

```text
roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  trajectory:=figure_eight rviz:=false
```

先核实 package、launch 文件和参数均存在，再写进可重复的 runbook。构建与回放出现失败时，记录真实错误，不以“未运行”替换成 PASS。

---

## 4. 先冻结公共契约，防止四个包反复改接口

下列名称是逻辑职责，不强制每个都变成独立类或独立源文件。优先扩展现有数据结构，避免一个简单流水线变成大量工厂/插件层。

### 4.1 必须严格区分的对象

| 对象 | 含义 | 不允许的混淆 |
|---|---|---|
| `FaultEvent` | 物理故障事件：源、轴/共因组、时间支撑、形状与幅值集合、概率依据 | 不能等同于字典一列或一次排除动作。 |
| `FaultHypothesis` | 某组允许同时存在的事件及其参数共享关系 | 参数列数不是 fault order。 |
| `FaultBasis / T_h` | 用于表达实际故障影响的计算字典与模板 | 字典可有很多列，不代表它们同时故障。 |
| `CoverageEnvelope` | 包住一组假设的计算上界及包含证据 | 外包络不是自动改变对外声明，也不是更低维的等价模型。 |
| `FdeAction` | 对因子、原始数据、状态或输出的可执行处置 | 数值上能屏蔽某些方向，不代表物理上可以执行相同排除。 |
| `ProtectedOutput` | 某一时刻、某坐标系、某参考点的状态和保护证书 | 不是一个可以贴到下一帧位置上的 PL 数字。 |

为当前快照记录 `m, n, rank(H), ν, K, L`；为每假设记录 `q_h, rank(A_h), rank(Z_h)` 和保护输出零空间检查。`ν=m−rank(H)` 仅针对正确白化的当前残差投影，不对未白化重复约束直接计数。

### 4.2 `IntegritySnapshot` 的最低字段

```text
snapshot_id, source_revision, config_digest, manifest_digest
state_solution_id, sensor_timestamp, frame_id, position_reference
state_block_ids, tangent_convention, state_scale, output_jacobian_C
factor_ids, factor_revisions, raw_measurement_ids, sample_time_supports
whitening_id, noise_model_id, weight_policy, linearization_id
boundary_summary_id, history_lineage_id, protected_reference_center
active_observation_index, coverage_epoch, validity_assumptions
```

快照生成后不可变。不能只给可变指针加一个时间戳。缓存必须检查实际依赖的版本；跨线程查询时不得混用不同版本的 `H、D、C、R、边界先验`。

**特别注意保护位置的定义。** UWB tag 和 IMU body 原点可能存在 lever arm；Pose3 的局部平移坐标也不必等于世界坐标增量。C 应为“实际受保护位置关于实际状态局部增量”的 Jacobian，不硬编码为挑出某三列；其实现须有有限差分测试。

### 4.3 故障模型契约

保留实际代码已声明的 IMU 单轴/单区间模型，不为了消除 `inf` 自动改成整窗六维 bias step。固定起点的 bias step、scale error 或丢样可以作为明确的新故障族，但必须有数据到因子残差的映射和独立测试。

manifest 至少包含：

```text
source_id / common_cause_group / event_id
model_type / parameter_units / parameter_sharing
onset_domain / end_condition / physical_time_support
amplitude_model: unbounded_subspace | validated_bounded_set
prior_bound_source / prior_time_basis / evidence_status
included_hypothesis_families / omitted_event_set
history_retention_or_reset_rule
allowed_actions（与数值 kernel 无关）
```

“单故障”默认指一个已定义的物理事件；一个设备共因多轴故障可在其事件定义下是一阶，但不能为了节省预算任意这样命名。历史事件发生在不同时间，不等于它们在当前位置的影响不会同时存在。

模板阶跃/斜坡以实际时间戳和实际测量支撑构造，不用 nominal epoch index 代替秒。零列、重复列和零时长末端 ramp 必须处理；删除计算上的重复列不能删除相应事件概率或改变有界参数集合。

### 4.4 启动与每帧验证分开

**启动验证：**配置 schema、支持的阶数与组合族、单位、先验/包络证据引用、符号依赖、明确不可能的维数约束、资源上限、构建能力。可以做工作域采样预检，但采样通过不是全工作域证明。

**每帧验证：**实际可用因子、白化、秩/条件、`C` 可估计性、逐假设危险零空间、历史来源、余项包络、当前风险预算、发布对象与期限。

`q_h > ν` 可以报警“全参数辨识不可能”，不能直接拒绝全部位置 PL；`q_h ≤ ν−4` 也不能保证位置可监测。启动报告只写它实际证明的内容。[G0 §3；本轮对 C3 §2.4 的修订]

### 4.5 结果与失败原因

统一结果中至少区分：

```text
computation_status: OK | UNAVAILABLE | NOT_EVALUATED
model_claim_status: VALID_UNDER_DECLARED_MODEL | ASSUMPTIONS_UNVALIDATED
                   | UNSUPPORTED_DECLARATION
coverage_status: COMPLETE | CERTIFIED_ENVELOPE | INCOMPLETE
primary_failure, all_failures, not_evaluated_checks
slope / PL / covariance / risk_bound / detector_thresholds
assumption_ids / proof_ids / snapshot_id / timing
```

失败原因至少覆盖：

```text
STATE_OUTPUT_UNOBSERVABLE
DANGEROUS_FAULT_NULLSPACE
NUMERICAL_CERTIFICATE_UNRESOLVED
LINEARIZATION_UNCERTIFIED
HISTORY_SUMMARY_INVALID
HISTORY_CAPACITY_EXCEEDED
RISK_BUDGET_INFEASIBLE
FAULT_MODEL_UNSUPPORTED
POST_FDE_UNCERTIFIED
OUTPUT_IDENTITY_MISMATCH
DEADLINE_MISSED
RESOURCE_LIMIT_EXCEEDED
```

`NOT_EVALUATED` 不能写成通过，也不能写成已确认秩亏。在线遇到 gate 可以提前返回；同一快照的离线诊断模式继续检查后续原因，避免“首因”遮住第二个缺陷。

若保留有限的 `pl_model_only` 供研究绘图，但模型证据未满足部署要求，必须同时标记它不是有效受保护输出。不得靠清除 `IMPLEMENTED_UNVERIFIED` 字符串改变证据状态。

---

## 5. 数值与风险规格：B/C 必须实现的同一套数学对象

本节的精确关系建立在**同一固定、正确白化的线性模型**上。实际非线性轨迹、数据依赖权重与部署模型是否满足这些前提，需由 §7–8 的证据和有效性检查承接，不能因为“把最后一次矩阵冻结”就自动成立。

### 5.1 统一符号，先解决平方/符号混乱

使用生成模型

\[
z=H\delta x+A_hf_h+\varepsilon,\qquad \varepsilon\sim N(0,I).
\]

名义估计为 `δx̂=H†z`，其故障均值响应为 `+H†A_h f_h`。若现有代码导出的是 `r₀+Jδx`，在 adapter 中转换到相同约定；不得在 UWB、IMU 和历史摘要中各使用一种符号。

定义：

\[
r=Q_2^Tz,\quad Z_h=Q_2^TA_h,\quad
G_h=CH^\dagger A_h,\quad \Gamma_h=Z_h^TZ_h.
\]

**约定 `λ=f_h^TΓ_hf_h` 是非中心参数本身，即残差均值范数的平方。** `T=‖r‖²`、阈值 `τ` 也是平方范数。检测边界 `Λ` 与 λ 同量纲；PL 使用 `√Λ`。所有外部统计函数的入参写进测试，防止把 λ 与 λ² 再平方一次。

### 5.2 平方根主路径与列置换

令 S 为有物理意义的状态尺度矩阵，`δx=SΠy`。对 `H S` 作带列置换 QR：

\[
H S\Pi=Q_1R,\quad \widetilde C=C S\Pi.
\]

通过三角求解

\[
R^TU^T=\widetilde C^T
\]

取得

\[
\Sigma_p=UU^T,\quad
Y=Q_1^TD,\quad Z=Q_2^TD,\quad B=UY.
\]

对于 `A_h=DT_h`：

\[
G_h=BT_h,\qquad Z_h=ZT_h.
\]

实现要求：

- 主路径不形成完整 `P=(HᵀH)⁻¹`、Q、Q₂ 或 `I−HPHᵀ`；用隐式正交变换和少量右端项。
- 可按 dictionary block 分批应用 `Qᵀ`，防止为减少函数调用而创建超大临时矩阵。
- 状态尺度与列置换必须同步作用于 C；只给 H 置换、忘记 C，是必须专门测试的错误。
- 已白化的 Gaussian 因子不能再除一次 sigma。残差、状态 Jacobian、故障映射三者采用同一白化变换。
- 精确硬约束、零方差约束与随机软先验分别处理；硬约束不能作为一条单位噪声行增加卡方自由度。需要时先约化确定性约束，并同步转换 C 与故障映射。
- 若主估计器的分解、权重、边界、顺序和当前快照完全相同，可以复用；否则使用同一快照单独建立完整性上下文，不能无条件取 iSAM2 的某个 R。

采用现有 Eigen 依赖时，可实现 `SparseQR` adapter；若真实拓扑满足局部链，可使用局部块 QR 以保留局部结构。不要为了本计划强制升级 GTSAM 到 develop。Eigen 的正交变换和列置换接口、符号/数值分解拆分已有官方定义，但须核对本地版本。[R3]

### 5.3 拓扑普查不能只数“一个因子碰几个变量”

`ImuFactor` 在 GTSAM 中连接五个变量，`CombinedImuFactor` 连接六个变量，但都可能只跨两个相邻时刻。前者不建模 bias 时间演化，后者包含 bias 演化与相关协方差；这只能帮助核对真实结构，不能替代实际代码 census。[R1、R2]

分类依据应为：时间块跨度、是否相邻、全局共享变量、边界分隔宽度，以及白化后实际耦合。两个相隔很远时刻之间的因子也会破坏简单链；一个共享外参或基站状态会形成全局耦合；相关噪声白化还可能改变原始因子的局部性。

保存 `epoch_span`、`global_key_set`、`separator_width`、`nnz(H)`、`nnz(R)` 和 fill-in。只有满足条件的结构才采用 `O(Nb³)` 的链式分解成本描述。

### 5.4 零状态行不等于可以丢掉的行

若某一白化行在 H 中为零，但 z 或 A 非零，该行可能是纯检测信息。Eigen 的部分 `SparseQR::analyzePattern` 实现要求没有空行，因此 adapter 可将其移到 `DetectorOnlyRows`，但必须保留其残差、故障响应、噪声与自由度；不能为满足求解库的输入要求而删除它。[R3]

这是历史摘要的直接相关测试：优化器看不到的常数代价，完整性检测仍可能需要。

### 5.5 可监测性与 slope：对 Z 作小型分解，不直接求 Γ 逆

先验证状态输出是否可估计：

\[
\ker H\subseteq\ker C.
\]

满列秩 H 自动满足；有纯 nuisance/gauge 零空间时，可以在经过验证的可估计子空间上计算输出。若当前内核尚未支持这种约化，返回明确的不支持/不可用，而不是加一个小正则项假装变满秩。

再对每个模式检查：

\[
\boxed{\ker Z_h\subseteq\ker G_h.}
\]

用 `Z_h=U_hΣ_hV_hᵀ` 的小型 SVD/秩揭示 QR 得到有效空间。若无危险零空间，则

\[
s_{h,d}^2=\left\|g_{h,d}V_r\Sigma_r^{-1}\right\|^2.
\]

这比显式形成病态 Γ 再求逆更适合主路径。Γ 可以作为必要的可审计输出，但不是必须使用的求解介质。

**严格区分三种情况：**有结构性证明的无害零空间；存在危险零空间；有限精度下无法分辨。第三种走高可靠参考/局部回退或不可用。不得把小奇异值截成零、再把对应的小位置响应也截成零，就宣称无限幅值故障无害。Eigen 官方也提醒数值列主元的 rank-revealing 策略不是所有秩亏问题上的万能保证。[R3]

**上述无界核判据只适用于无幅值上限的子空间。** 对 manifest 中已有有效幅值集合的故障，检测看不到它也不一定意味着位置无界。最低支持可以直接使用集合支撑函数：

\[
b_{h,d}=\sup_{f\in\mathcal F_h}|g_{h,d}f|.
\]

盒集 `f=f₀+diag(w)u、|u_i|≤1` 对应 `b=|gf₀|+Σ_i|g_i|w_i`；椭球 `f=f₀+Lu、‖u‖≤1` 对应 `b=|gf₀|+‖gL‖`。加上名义噪声尾界可以在不利用检测的情况下给出保守风险上界；集合越界概率另记账。未实现的集合类型应拒绝，而不是自动当作无限子空间或任意 Gaussian。

这只是同一 RiskEvaluator 对参数域的处理，不是另建一个完整性版本。含有界集合的降维必须同步变换集合，不得只保留列空间。

### 5.6 三条查询路径，但只有一个保护模型

| 代数条件 | 可用查询 | 限制 |
|---|---|---|
| 单个白化行的任意加性误差 | leverage，或统一 dictionary 投影 | `s²=(cᵀPh_i)²/(1−h_iᵀPh_i)`；分母相消不可靠时回投影路径。 |
| 一般物理故障映射/时间模板 | dictionary + 隐式 Q 投影 + 小型 Z 分解 | 生产必需路径；保持准确时间支撑和参数共享。 |
| 故障恰为独立白化行子集 S 的完整自由剖面 | 辅助子集的输出协方差差 | 必须满足固定线性化、行子空间等价、正确先验与可估计性；不由 `excludable` 决定。 |

第三条在满秩条件下为：

\[
G_S\Gamma_S^{-1}G_S^T=C(P_{-S}-P)C^T.
\]

对 template 直接删掉整个 anchor，只能在集合包含成立时得到**外包络**，不是 template 的精确值。一般物理 IMU 输入可以使用 dictionary 路径；若其实际映射恰好满足另一条等价路径条件，也不应被传感器标签禁止。[G0 §5；C3 §2.1 的修订]

不需要为三条路径写三个完整监控器。先由统一上下文正确服务全部模式，再根据已存在的实现和局部基准启用低成本查询。辅助子集路径至少作为独立恒等式测试；没有性能收益时，不强制在热路径维护它。若协方差差因相消出现不可靠负值，回退到正交投影求值，不能简单截为零。

### 5.7 单双故障一次实现

单事件 `h₁`、`h₂` 组合时应首先按 manifest 处理参数共享，只有参数独立时才可直接拼接 `A_{12}=[A₁,A₂]`。对应检测度量包含交叉块：

\[
\Gamma_{12}=\begin{bmatrix}
\Gamma_{11}&Z_1^TZ_2\\Z_2^TZ_1&\Gamma_{22}
\end{bmatrix}.
\]

两个单故障的最大 PL 不等于双故障 PL。两个源可以在检测空间抵消而共同移动位置。

`max_fault_order: 1` 和 `2` 共用 registry、projection、risk evaluator、cache 与测试入口。order=2 只加入明确启用的 family，默认示例为 `uwb_imu`。其他族未经实现/测试时启动拒绝，不能默认为已支持。

单故障只计算模式内部所需交叉项；双故障按组合需求计划交叉块。可以在小字典上使用连续批量乘法，是否计算完整**基础字典** Gram 由基准决定；不要继续无条件计算展开后所有模板之间的大 Gram。

### 5.8 分组上界与覆盖证据

覆盖关系是带证明的有向图，不是按 `template/recent_k/free` 名称推断的固定阶梯。

一个 envelope 记录：成员模式 ID、共同快照 ID、完整时间支撑、`range(A_leaf) ⊆ range(A_group)` 的构造证据、参数约束处理、风险参数映射和上界状态。对无界线性空间可通过精确构造 `A_leaf=A_group T` 证明；仅凭浮点最小二乘近似残差很小不够。

正常帧可以先评估源级包络。所有未精算叶节点必须始终有一个有效上界覆盖；不能把“计算了前 K 个”当作保护完整。包络为无穷而声明只是其内部模板集合时，可以细化；若声明本来就是完整自由剖面，则不能用只覆盖部分形状的模板替代。

不同叶节点的风险参数不同时，组上界不能只取最大 slope：还应保守处理检测边界和名义裕度。例如固定单检测器且各模式共享 σ 时，可使用组内 `max √Λ_h` 与 `max k_h`，或预先给组分配统一条件风险上界。改变分组不自动改变原先事件概率与风险账本。

超时前只能输出覆盖完整的有效上界；某未处理组连有效上界都没有，则输出不可用。不得按残差大小自动丢弃低概率/未报警模式。

### 5.9 正常帧风险必须随 B 一起闭合

对模式 h、轴 d，采用 G0 已证明的幅值分区界：

\[
L_{h,d}=s_{h,d}\sqrt{\Lambda_h}+k_{h,d}\sigma_d,
\quad k_{h,d}=\Phi^{-1}(1-\alpha_{h,d}/2),
\]

其中

\[
F_{\chi^2_\nu(\Lambda_h)}(\tau)\le\beta_h.
\]

对三维盒状保护、各轴共用 Λ：

\[
\boxed{P(\exists d:|e_d|>L_{h,d},\ \mathrm{accept}\mid h)
\le\max\left(\sum_d\alpha_{h,d},\beta_h\right).}
\]

边界内由位置尾部并集控制，边界外由一次漏检控制；不要求三个位置轴相互独立。取各模式的逐轴最大值，并单独覆盖 nominal case。假设概率与遗漏事件、包络失效、post-FDE/输出选择预算写入同一账本。[G0 §7；C3 §1.3、S7]

数字实现要求：

- 用 survival/complement 或等价稳定函数求小尾分位数，避免 `1−tiny_probability` 的消减；沿用仓库已验证统计库，不无理由引入第二套库。
- 解 Λ 时使用单调 bracket；返回满足接受概率上界要求的保守侧解。禁止固定粗网格与默认迭代次数后不检查误差。
- 缓存键包含 detector ID、ν、τ、β、包络参数与其版本，不能只按模式名称缓存。
- 模式未报警仍必须进入 PL；nominal σ 不是带故障 PL。
- 非中心卡方所需的噪声模型必须成立。把 covariance 调大不自动证明所有故障下的漏检尾界。
- 保留更紧的一维最坏幅值风险计算作为离线参考即可，不要求先开发第二个在线风险引擎；数值搜索必须有上包络和终止标准。

对 β=0、β≥1、无正数 Λ、零自由度或非法先验先进行显式边界处理；不能把它们传入求逆后再截断。未定义的参数组合拒绝，零风险目标不能用有限精度近似假装达成。

名义统计模型下的预算可写为

\[
\pi_0\alpha_0+
\sum_h\pi_h\max(\alpha_{h,x}+\alpha_{h,y}+\alpha_{h,z},\beta_h)
+\epsilon_{\rm omitted}+\epsilon_{\rm envelope}+\epsilon_{\rm selection}
\le\epsilon_{\rm target}.
\]

这是示意账式；π 是否互斥、故障族重叠、各项预算是否已涵盖同一事件必须在 A 定义。不能从同一事件收取两份“精确概率”，也不能凭模式合并少记概率质量。

---

## 6. Codex 按工作包实施

### 工作包 A — 工作树对齐、声明、诊断和独立参考

**修改责任：**配置解析/manifest、快照导出、诊断结构、现有测试与证据目录。参考实现优先放在现有测试工具目录；不得调用生产 PL/evidence 的同一计算函数来充当独立 oracle。

#### A1. 锁定实际代码并减少重复开发

- [ ] 完成 §3 的 SHA/工作树/依赖信息与 `code-map.md`。
- [ ] 阅读现有 ADR、测试、基准和未验证清单，建立“已有能力/真正缺口/本稿过时条目”表。
- [ ] 读取正常帧与故障帧调用链，确认哪些缓存、批量求解和单候选路径已经存在。
- [ ] 将真实构建和回放命令写入 `runbook.md`，先跑原有测试并保存结果；不要求不存在的测试 target 通过。

#### A2. 一次性普查与诊断 fixtures

- [ ] 导出每类因子的误差维数、变量 key、所属历元、全局变量、白化方式、先验来源。
- [ ] 确认 bias 是单个共享状态还是逐历元状态；读取其过程约束。检查物理上相同的信息是否重复加入。
- [ ] 从实际 H 记录 m/n/rank/ν、块结构、fill-in；从每假设记录 q/rank/时间支撑。不要填入推算的 363 或 483。
- [ ] 导出有效配置全量值及 hash，包含 launch/ROS/命令行覆盖结果；观察窗口移入移出验证 `epochs` 的实际含义。
- [ ] 增加首因、全部检查状态与离线 shadow diagnostic。保留至少：普通帧、当前 inf 帧、缺测帧、边缘化帧、报警帧。
- [ ] 做一维瞬时/现有模板/源级自由剖面三个对照，但**固定 H、C、噪声、历史和风险参数进行代数诊断**，另做改变声明后的完整风险实验；两类结果不混合。

该阶梯对照只定位“哪个修改触发失败”，不自动证明原因。检查失败时逐项列出状态输出、Z 零空间、风险、历史与线性化证据。

#### A3. manifest 与公共接口同时确定

- [ ] 将现有支持的事件、形状、时间域、组合族和概率来源迁入 registry；保留现有范围。
- [ ] 固定 `max_fault_order=1` 的产品默认与 `order=2 + declared families` 的含义。
- [ ] 拆开 `allowed_actions` 与 `projection_eligibility`，不要接受 C3 的 kernel/action 绑定。
- [ ] 冻结 §4 的 snapshot/result/summary/action 身份字段及风险事件定义；B/C 共用。
- [ ] 初始化 `proof-obligations.md`，每项明确假设、参考、测试、需要真实数据的证据和当前状态。
- [ ] 为旧配置建立一次解析适配：冲突值拒绝，含义清楚的别名警告；禁止两套参数默默覆盖。

#### A4. 独立参考与解析测试

- [ ] 从 `raw Jacobians + raw covariance + raw fault mapping + C` 独立白化；与生产快照白化一致性另测。
- [ ] 用小型稠密 QR/SVD 实现名义输出、协方差、G、Z/Γ、slope、检测与风险参考，测试代码不能复用生产求解器。
- [ ] 解析测试：`H=(1,1)^T、A=(1,0)^T、C=1`，应有 `ν=1、P=1/2、G=1/2、Γ=1/2、s²=1/2`，且给出有限模型 PL。
- [ ] 加入 §1.2 的反例，以及“q>ν 但只影响 nuisance 的零空间”算例，防止错误硬门。
- [ ] 对 IMU 在原始加计/陀螺样本端注入，做正负扰动、多个步长、重积分对照；不能只改预积分残差。

**A 的合并判据：**真实 census 与代码映射完整；原有测试结果有记录；解析与独立参考通过；缺口/假设状态诚实；B/C 所需接口已定义。没有要求此时真实数据所有 PL 都有限，也不允许为了通过 A 调大门限或删故障。

**A 交付物：**`code-map.md`、`census.json`、`effective-config.yaml`、`baseline-report.md`、`runbook.md`、`proof-obligations.md`、最小 fixtures 与 oracle 测试。

### 工作包 B — 统一平方根、单双故障、风险与正常帧惰性处理

**修改责任：**实际 snapshot/kernel/generator/evidence/PL/monitor 模块及对应测试。保留一条生产上下文，不增加第二个主估计器。

#### B1. 数值上下文与缓存一次替换

- [ ] 按 §5 建立平方根 adapter；处理尺度、列置换、输出 C、零状态检测行和可估计状态约化。
- [ ] 所有名义修正、Σp、故障投影和检测残差来自同一上下文；删除已被替代的重复在线分解。
- [ ] 使用真实观测的不可变块构建 D/T；UWB 时域模板复用投影列和时间加权和；IMU 映射按原始样本/PIM/bias/线性化版本缓存。
- [ ] 同稀疏结构可复用符号分析；pattern 改变必须重新分析；数值改变不能只复用旧 R。[R3]
- [ ] 保留每帧低成本证书和局部回退，不用“每隔若干帧完整 SVD”代替中间帧有效性检查。

#### B2. 同一 registry 服务 order=1 和 2

- [ ] 一次实现模式选择、参数共享、pair-family 过滤与组合交叉项；测试同时覆盖两种 order。
- [ ] 减少 all-mode 展开，保存紧凑 descriptor 与受容量约束的连续缓冲；模式数变化不触发每模式大对象分配。
- [ ] 分组 envelope 附包含证据与风险参数映射；若现有代码没有分组能力，从完整 exact traversal 开始，但这是同一接口下的正确执行计划，不另发一个覆盖不同的版本。
- [ ] 基准确有收益时启用 leverage/辅助子集查询；否则统一投影仍可完成目标。所有路径给出 `EXACT` 或 `UPPER_ENVELOPE` 标签。
- [ ] 对错配模式、未实现 family、共享参数误拼接、跨模式抵消建立失败测试。

#### B3. 风险与证书同步落地

- [ ] 统一 λ/Λ/τ 的平方约定、nominal 与 fault 风险、逐轴联合风险和遗漏账本。
- [ ] 实现稳定分位数与非中心参数求解、保守收敛检查、版本正确的概率缓存。
- [ ] 对不能证明无害的零空间保持不可用；记录决定是哪一个模式/输出方向触发，而非全局字典维数。
- [ ] 正常帧产生完整 coverage certificate；原子发布接口已接收证书，即使 C 的历史改造尚在开发，不能绕过现有历史有效性 gate。

#### B4. 惰性 FDE 与健康状态机同时改

- [ ] 将“故障描述/灵敏度”与“排除动作实体/桥接/回滚图”拆开。
- [ ] 健康且未报警普通帧不构造动作、不建候选图。
- [ ] 报警、隔离状态恢复检查、历史屏障或待提交候选等真实需要 evidence 的路径才计算细粒度隔离证据；不要只按 `all_in.passed` 一刀删除健康管理所需调用。
- [ ] `monitor_only` 仍计算 PL 灵敏度；`conditional repair` 的 IMU 保留动作接口，完整修复与提交证明在 C 完成。
- [ ] 固定动作目录/动作 ID/预算映射及可证明共享参考的保证组与 C 对齐，避免 C 再改生成器一次。

**B 的合并判据：**全部支持模式在 well-conditioned fixtures 上与独立参考一致；单双故障同时通过；不可用原因正确；无正常帧 eager actions；热区分配有计数；没有覆盖缩水；历史与发布旧保护机制未被绕过。

**B 不要求：**一次实现所有可能的优化查询、全局数值 rank-update、所有传感器组合或新 KF 后端。

### 工作包 C — 历史摘要、检测证明、IMU 处置与发布事务

**修改责任：**实际边缘化/PIM 保留、snapshot、detector/risk、候选评估、健康管理与发布模块。详细算法见 §7–8。

#### C1. 历史接口与真实边缘化同时接入

- [ ] 实现 `R_b,T_b,d_b,F_b/d_perp` 的平方根摘要，或其经验证的紧凑等价表示；保留 Ω/ξ/κ/ν 的审计视图。
- [ ] 摘要更新先于原始信息删除；完整历史 oracle 与摘要路径逐项比较，不只比较优化轨迹。
- [ ] 为故障出生、窗口内外转移、持续事件、组合交叉项、配置变化和重线性化定义生命周期。
- [ ] 引入资源上限与明确溢出动作；不能声称任意历史只占六维，也不能 cap 后静默遗忘故障。
- [ ] 在没有原始历史或有效既有摘要的运行中，明确初始化不可认证状态；不得从已有 nominal marginal 反推出缺失检测信息。

#### C2. 检测事件与 PL 一起实现

- [ ] 默认目标为 §7.3 的分离残差检测布局，并使用 §7.4 的联合接受事件分区界；不直接沿用单检测器 Γ/df 公式。
- [ ] 池化检测作为短序列参考布局保留，用于验证摘要的代数信息保持与检出能力比较，不作为另一套产品版本。
- [ ] 对历史纯噪声方向，可在声明覆盖和统计前提成立时做 §7.5 的 fault-span 投影；未证明或维数不可靠则不启用该压缩。
- [ ] 列出两个统计量的协方差/来源；不因为叫“历史”和“当前”就假定独立；实际风险证明不依赖它们相互独立。
- [ ] 把模型误差的位置/残差双通道、实际阈值和参考阈值一起加入证书；ρ 必须来自有效模型或有标识的未验证假设。

#### C3. IMU 条件性 FDE 与 post-selection 闭合

- [ ] 读取并尽量复用现有重积分/历史回滚/桥接机制，保留真实能执行的动作。
- [ ] 对原始输入替换与重积分、区间移除与有证据的动力学替代分别建立数据来源和模型误差记录。
- [ ] 不默认 IMU 永久不可排除；也不因为子集位置有界就认为全状态可修复。
- [ ] 依据完整物理事件集合计算候选 PL；被删除源可能仍污染边界，不能从风险账本直接移除。
- [ ] 候选选择采用 §8 的保证组/固定动作联合风险控制；预算不可仅按最后选中的候选分配。

#### C4. 提交与发布作为同一事务

- [ ] 候选更新使用 copy-on-write/不可变记录；验证失败则不改变可用的主状态/历史账本。
- [ ] 成功提交将状态、历史摘要、模式配置、健康状态和保护证书一起更新；检测—证书之间发生后端改解则拒绝旧证书或显式计入同时间中心偏移。
- [ ] watchdog/状态通道独立于耗时 FDE，能够及时报告不可用；模拟时间与单调壁钟延迟分别计量。
- [ ] 过期、不同时间、不同坐标系的证书不得重用。无中间时刻传播证明时，高频估计输出标记未保护。

**C 的合并判据：**历史摘要在固定模型下等价；检测布局与 PL 证明一致；跨边界故障不丢失；非法重线性化/配置改变会失效而非误用；错排、桥接失败、超时与异步输出测试通过；没有未经证明的 post-FDE 恢复发布。

### 工作包 D — 端到端、性能与合并清理

**修改责任：**回归/基准工具、CI、runbook、必要的热点优化和兼容清理。不得在 D 才首次决定 fault order、风险公式或历史接口。

- [ ] 在固定数据、声明、先验、权重与环境下，运行全部指定轨迹和 §10 测试矩阵，order=1/2 分别报告。
- [ ] 区分普通帧、报警帧、边缘化、重线性化、回退、启动和恢复；记录 p50/p95/p99/max、峰值内存、队列等待、超时率与有限有效 PL 比例。
- [ ] 用原始逐帧 trace 比较，不用各组件中位数之和预测总中位数。
- [ ] 若实测瓶颈在 symbol/index/hash、重复 PIM 映射、概率反演或临时分配，就优化实际热点；不预设一定在 factorization。
- [ ] 若仍需要数值增量更新，用 A 的 oracle 与 B 的版本接口验证移入、移出、重线性化和回退；不另写一套不受相同证书约束的 quick mode。
- [ ] 清理生产中已替代的 dense/eager 路径，但保留独立测试 oracle；更新 ADR 和唯一配置入口。
- [ ] 列明未通过的部署模型证据；不把性能 PASS 等同于完整性认证 PASS。

**D 的合并判据：**§12 的全部必要门通过；所有测试/基准有实际命令和产物；资源超限不缩小声明；功能改动无未解释漂移。达不到性能目标则记录 blocker 和热点，不调低故障覆盖来伪造成功。

---

## 7. 历史摘要与检测器：C 中必须同时完成的细节

### 7.1 固定模型下的消元摘要

将旧状态 `x_o` 消元、保留分隔状态 `x_b`，同一物理故障参数 f 下的历史代价为：

\[
\|H_ox_o+H_bx_b+Af-z\|^2.
\]

对旧状态列和剩余边界状态列依次作正交消元，得到：

\[
\boxed{\|R_bx_b+T_bf-d_b\|^2+\|F_bf-d_\perp\|^2.}
\]

名义估计取 `f=0`，不是把全部故障当作新状态拟合掉。摘要至少支持以下信息的恢复：

\[
\Omega_b=F_b^TF_b,\quad \xi_b=F_b^Td_\perp,
\quad \kappa_b=\|d_\perp\|^2,\quad \nu_\perp=\dim d_\perp.
\]

生产实现尽量以平方根/正交变换保存 `F_b,d_perp` 的等价紧凑表示，Ω/ξ/κ 为审计接口，不强制长期累加易相消的正规方程。必须保持消元噪声与其他残差的关系。[G0 §6.3；C3 §1.4]

`R_b⁻¹T_b` 表示正向故障注入的边界均值响应；在固定 z 的代价中将 f 作为条件变量求最优 x，则导数为 `−R_b⁻¹T_b`。两个问题符号不同，并不矛盾；代码与测试必须写明是哪一个。

### 7.2 不得丢失、不得重复计入

摘要与新观测组合时，状态侧白化系统为：

\[
H_c=\begin{bmatrix}R_b&0\\H_{\rm active}\end{bmatrix},\quad
A_c=\begin{bmatrix}T_b\\A_{\rm active}\end{bmatrix},\quad
z_c=\begin{bmatrix}d_b\\z_{\rm active}\end{bmatrix},
\]

实际列位置按边界状态嵌入当前 ordering。对它作 QR 得到 `r_c、Z_c、G、Σp`，另有历史纯残差 `d_perp、F_b`。

完整历史池化参考满足：

\[
T_{\rm pooled}=\|r_c\|^2+\kappa_b,\quad
\Gamma_{\rm pooled}=Z_c^TZ_c+\Omega_b,\quad
\nu_{\rm pooled}=\nu_c+\nu_\perp.
\]

这是在固定正确白化、独立原始噪声或已正确联合白化、无重复数据的模型中的等价关系。不能既加入 estimator 已边缘化的同一先验，又重复加入同一批原始历史测量。

每条原始测量/IMU 样本到因子/摘要的归属必须可追踪。跨帧窗口重叠本身不违规，但不能因此声称各历元检验独立；任务风险另外按时间事件处理。

### 7.3 主路线的检测布局：分离残差，不凭名称假定独立

本路线图选定目标：**当前状态因子系统的残差检测 + 历史状态消元后的残差检测**。两个均接受才允许该布局下的保护发布：

\[
\mathcal A=\{T_c\le\tau_c\}\cap\{T_b\le\tau_b\}.
\]

这里“当前”系统包含带历史信息的边界因子，不能误称它只依赖新数据。历史检测块是已经正交消除状态的残差块，不是再次对历史边界均值做一个未经联合建模的测试。

每个检测通道返回：

```text
detector_id, residual_energy, residual_dof
fault_metric（Z_j 或其平方根表示）
actual_threshold, nominal_fa_allocation
noise/whitening lineage, model_error_residual_bound
```

在固定正交模型且新增噪声独立时可以验证通道噪声正交；实现仍使用并集控制虚警、用 §7.4 控制漏检，不依赖额外的通道独立性假设。对相关噪声、数据重用、鲁棒权重等情况，不满足单通道分布前提时先联合建模/有效包络，否则不可认证。

将两个检验分开不保证任何故障下检出能力永不下降；历史故障的非中心参数可能增加，新故障也可能被无关旧噪声稀释。实验应分别测近期瞬时、长期持续、窗前故障，不报告一条无条件“随时间单调”结论。[本轮对 C3 §2.2 的修订]

### 7.4 两个检验量对应的 PL：不能继续只套一个 χ² 公式

以下为本轮在 G0 幅值分区证明上的扩展，需作为独立公式审查项加入 `proof-obligations.md`。

对于固定模式 h，通道 j 有

\[
\lambda_{h,j}=f_h^T\Gamma_{h,j}f_h,
\qquad T_j\sim\chi^2_{\nu_j}(\lambda_{h,j}).
\]

取**同一个模式漏检上界** β_h，为每个有效通道求 Λ_h,j：

\[
F_{\chi^2_{\nu_j}(\Lambda_{h,j})}(\tau_j)\le\beta_h.
\]

选固定正权重 `w_j>0、Σ_jw_j=1`，两通道默认等权，构造

\[
W_h=\sum_jw_j\,\Gamma_{h,j}/\Lambda_{h,j}.
\]

若所有 `λ_h,j≤Λ_h,j`，则 `f_hᵀW_hf_h≤1`。若 `ker W_h ⊆ ker G_h`，输出故障项可由

\[
\boxed{b_{h,d}=\sqrt{g_{h,d}W_h^\dagger g_{h,d}^T}}
\]

控制。于是

\[
L_{h,d}=b_{h,d}+k_{h,d}\sigma_d
\]

满足

\[
P(\exists d:|e_d|>L_{h,d},\ \mathcal A\mid h)
\le\max\left(\sum_d\alpha_{h,d},\beta_h\right).
\]

**证明要点：**所有检测均值在各自边界内时，用上述椭球外包络控制位置；至少一个通道超过边界时，联合接受蕴含该通道接受，因此漏检概率不超过 β_h。无需两个检测量独立。该椭球上界可以保守，但逻辑闭合。

实现时把各 `Z_h,j` 按 `√(w_j/Λ_h,j)` 缩放后堆叠，用 §5.5 的小型分解求解，不显式求 W 的逆。无残差自由度/无故障检测贡献的无效通道不能凭空生成 Λ；只有一个有效通道时，本式退化为单通道的 `s√Λ`。无任何通道能看到危险方向时仍不可用。

不要用 `Γ_c+Γ_b`、`ν_c+ν_b` 和某个随意选出的阈值，替代这里的联合接受事件；那对应另一种检测器。detector ID、阈值和证明 ID 必须一起进入证书与缓存。

### 7.5 可合并实现的历史降噪投影：去掉声明内纯噪声方向

这项是本轮新增的明确计算选项，不来自 C3 的“两个独立检验”口号，也不应标为已有文献定理。

设 `F_all` 的列空间覆盖**本运行声明中全部保留历史故障方向**，U 为其正交列基。可对历史检测使用

\[
r_b=U^Td_\perp,\quad Z_b=U^TF_{\rm all},
\quad \nu_b=\operatorname{rank}(F_{\rm all}).
\]

因为 `(I−UUᵀ)F_all=0`，该变换保留所有声明故障的历史非中心二次型：

\[
Z_b^TZ_b=F_{\rm all}^TF_{\rm all}.
\]

被舍弃方向在该固定声明模型下不含故障均值。它们的能量仍保留供完整历史代价审计，但不必全部加入面向这些故障的历史检测。正确白化和固定投影下，保留残差噪声协方差为单位阵。

**限制：**这改变了检测器，不保证最终 PL 与池化参考相等；须重新设阈值、使用 §7.4 风险构造并检验噪声模型。U 必须覆盖全部声明方向，不能按观测到的故障或 top-K 模式挑选；数值近零不是结构性无均值证明。模式新增、manifest 变化、线性化变化或只保存了部分交叉项时可能需要重建。

这项压缩仅消除当前声明内无用的历史噪声方向；`rank(F_all)` 仍可能随保留故障族增长，不能据此承诺固定内存或永久不稀释检验。若无法验证该投影，使用未投影的历史残差通道并正确记录自由度，不降低覆盖。

### 7.6 故障生命周期与资源边界

| 事件 | 必须采取的处理 |
|---|---|
| 未来时刻新增故障参数 | 仅当其物理 onset 确实晚于已压缩数据时，旧摘要中的相应列可为零；迟加入一个“从任务开始就可能存在”的参数不能补零。 |
| 模式进入历史 | 将对应输入响应与检测代价一起消元，保留事件 ID、时间支撑、参数含义和概率依据。 |
| 活动窗口内已无该源观测 | 不自动删除模式；历史边界仍可能携带其状态影响。 |
| 故障结束 | 结束注入不等于清除累积状态误差；保留响应直到有有效清除/重初始化证据。 |
| 准备淘汰历史模式 | 需要对未来保护范围也成立的无影响证明、验证过的包络和风险处理，或真正清洁重置；当前时刻 G 很小/为零不一定足够。 |
| 在运行中从 order=1 改为 2 | 若摘要未保存必需的历史交叉项，则从原始数据/检查点重建；不能只修改 config 值继续使用旧摘要。最简单的可靠策略是本运行 manifest 不热切换。 |
| 历史参数/源组超过容量 | 合法选择只有经过证明的更粗包络、有效重置/重建或停止受保护发布；不能丢最旧故障以维持性能。 |

容量包括参数列、模式数、组合交叉块、原始数据重放长度、分隔宽度和内存上限。必须在 A 按真实 manifest 设定，在 C/D 验证达到容量时的行为。不要把 `q=6` 的固定起点算例作为整段任意历史的容量证明。

### 7.7 重线性化、时间与原始数据保存

摘要绑定被消元因子的线性化点、局部坐标、白化和物理故障映射。对可证明的线性坐标变换，可以一致地转换；对已消元非线性因子的 Jacobian 发生实际变化，必须从原始数据重建，或有明确的余项包络。不能仅重新标记 `linearization_id`。

保留重积分所需 IMU 样本、时间边界和版本；预积分名义 bias 改变后，故障映射缓存也必须根据其依赖更新。若原始数据已释放且没有有效包络，摘要无效，受保护输出不可用。

**在摘要证明与测试完成前，`auto_shrink` 保持关闭。** 完成后缩窗也须固定声明、先验和噪声模型比较：Σp/G/Γ/检测量是否等价，或差异是否由已证明的保守包络承担。覆盖保持与可用性改善分别报告。

---

## 8. 模型有效性、FDE 与发布：不允许只改最后一条 PL 公式

### 8.1 线性化和鲁棒权重

混合米、弧度、速度和偏置的整体增量阈值应替换为有物理尺度的诊断，但小步长不等于非线性保护证明。检查应覆盖声明未检故障下所需的有效区域，不仅是最后一步优化增量。[G0 §10]

当前残差决定的鲁棒权重、数据选择、重线性化点和校准参数，可能使设计矩阵依赖同一批噪声。冻结矩阵只保证代数可重复，不自动恢复 χ² 分布。运行证书必须列明适用条件；未建立有效性证据的鲁棒统计模型不能标成已验证高斯 RAIM。

可直接开工的边界处理包括：保持现有可靠噪声模型；明确 raw/whitened 残差；验证 Jacobian；记录迭代和线性化漂移；实现有来源的模型误差接口。不能用“待研究”跳过接口，也不能编造实际传感器的有效包络。

### 8.2 双通道模型误差及虚警阈值

若参考模型之外的附加误差 u 在证书有效事件内满足

\[
|[CH^\dagger u]_d|\le\rho_{p,d},\qquad
\|Q_{2,j}^Tu\|\le\rho_{r,j},
\]

实际检测接受必然包含于无 u 参考残差的扩大接受区域。因此在 PL 风险计算中使用

\[
\boxed{\tau_{j,\rm risk}=(\sqrt{\tau_{j,\rm actual}}+\rho_{r,j})^2}
\]

求 Λ，再给每轴加上 `ρ_p,d`。模型包络不成立的风险另记账。[G0 §10.3]

**同时处理虚警，不只处理漏检。** 若名义运行也有相同有界 u，可将名义高斯阈值 `τ_j,0` 的平方根加上 `ρ_r,j` 作为实际门限，以保证所需的保守名义虚警界。实际门限与风险参考门限是两个不同量；不能把它们都命名为 τ 后忘掉一次变换。

当 ρ=0 时回到标准卡方模型。ρ 的物理来源、适用域、失效风险与版本必须进入 certificate。对于含故障的大范围非线性模型，无法在所需区域内证明一个有限 ρ 时，不得只靠调参生成有限“可信”PL。

### 8.3 不把没有概率依据的桥接伪装成独立高斯测量

匀速/恒姿态是运动假设，不是新的传感器。给桥接因子设一个小 covariance，不会自动使它满足独立高斯白化模型。纯有界确定性约束和独立高斯残差也不是同一种噪声；不能无证明地按一个高斯因子增加 ν。

对桥接候选采取以下决策顺序：

1. **有验证的随机/有界组合模型：**按其真实模型进入参考估计与双通道包络，计算 post-FDE PL。
2. **桥接只用于得到估计中心，而无可用于 RAIM 的概率模型：**不把它当作可信随机信息；使用同一时刻、同一参考量的有效参考解，将保护转移到候选中心。
3. **没有有效参考解或转换界：**候选可以供未保护估计器诊断使用，但不允许恢复受保护发布。

参考保护转移为

\[
\boxed{L_{{\rm candidate},d}=L_{{\rm reference},d}
+|\hat p_{{\rm candidate},d}-\hat p_{{\rm reference},d}|.}
\]

它不要求两个估计器独立，甚至不要求桥接中心有正确协方差；要求参考 PL 自身有效、保护同一个物理量/时刻/坐标系。若参考是已有的 UWB-only 位置内核，它必须独立满足相应 UWB 故障覆盖，不能共享一个未经修复的 IMU 污染先验再称其独立于 IMU。不同参考点之间还需有可验证转换界。

这是一条与既有单帧 UWB RAIM 资产相容的候选校验接口，不是要求另建一个主定位系统。较大的中心偏移会自然增大 PL，不能为了可用性裁掉该项。[G0 §10.2；本轮桥接实施补充]

### 8.4 FDE 证据与 PL 检测度量不可混用

故障解释/隔离可以使用完整白化代价的 profile likelihood。含历史摘要时，固定模型下的参考量为

\[
J_h^{\rm profile}=\|r_c\|^2+\kappa_b-
 t_h^T\Gamma_{h,\rm full}^{\dagger}t_h,
\]

其中

\[
t_h=Z_{h,c}^Tr_c+\xi_h,\qquad
\Gamma_{h,\rm full}=Z_{h,c}^TZ_{h,c}+\Omega_h.
\]

它依赖原始白化似然度量。§7.4 为风险边界缩放过的 W 不是该似然的 Γ，不能拿来替代。不同参数维数的 profile 残差也不能不加说明直接横向排序；保留或明确修订当前隔离准则，并用独立选择风险保证兜底。

候选池存放结构化故障解释和动作，不将 UWB 米残差与 IMU 不同单位残差直接比较。局部 FD 可帮助优先搜索动作，不能自动删除未报警传感器对应的在线 PL 故障。

### 8.5 post-FDE 的保守可实现保证

先固定**本次配置下所有可能被发布的动作类别与索引集合**，为动作 a、模式 h 建立预算 `ε_a,h`。事件为

\[
\{\mathrm{select}\ a,\ \mathrm{accept}_a,
\ \exists d:|e_{a,d}|>L_{a,d}\}.
\]

一个便于实施的充分方案，是为每个固定动作建立统一故障模型下的接受且超界上界，再对所有可能发布动作做并集控制：

\[
\sum_a\sum_h\pi_h r_{a,h}
+\text{nominal/omitted/envelope terms}\le\epsilon_{\rm target}.
\]

丢掉“select a”只会放大概率，因此选择方法可以保留实际工程准则，但预算不能只给最后被选中者。动作集合若由数据变化，必须覆盖所有可能产生的发布动作，或另有对选择规则的证明；只按实际搜索过的 top-K 候选数分预算不够。[G0 §10.5；R6]

被排除的源仍可能通过历史影响候选输出，继续进入该动作下的 G/A 与风险账本；只有能证明其响应消失或被有效包络，才可在该动作的条件模型中收紧。动作恢复后再跑单帧 nominal 检验，不等于排除风险已闭合。

**共享参考证书可以避免对每个中心重复支付选择风险。** 若一组候选都使用同一个有效参考证书、同一个真实位置量，并对所有轴取 `L_a,d=L_ref,d+|p_a,d−p_ref,d|`，则不论数据选择了哪一个中心：

\[
\{\exists d:|p_{a,d}-p_d^*|>L_{a,d}\}
\subseteq
\{\exists d:|p_{{\rm ref},d}-p_d^*|>L_{{\rm ref},d}\}.
\]

因此这一组可以由同一个参考失败事件控制，而非按组内中心数量重复做并集。实现 `GuaranteeGroup`：共同参考证书/接受事件/时间/输出量/模式身份及成员的三角转移证据。无法证明共享保证的动作自成单元素组，最后对各保证组做并集预算。这是一个统一的选择风险接口，不是允许任意把动作合并少记风险。

若从多个不同参考中数据依赖地挑最小 PL，仍需对那些参考的可能失败事件做选择风险控制；不能继续沿用“同一个参考”的论证。所有发布路径都必须要求所用参考的有效接受条件成立。

对于极大动作空间，不强制每帧构造全部动作。只需预定义可能动作及预算/统一上界，报警时实例化必要候选。找不到满足要求的候选就保持不可用，而不是放松标准。

### 8.6 原子提交与时间契约

一次有效提交绑定：`snapshot_id、state_solution_id、history_summary_id、manifest_digest、health_state、detector_ids、risk_proof_id、PL、position_reference、timestamp、frame_id`。

建议最小状态机：

```text
READY → EVALUATING → PROTECTED
                  ↘ UNAVAILABLE
UNAVAILABLE → FDE_EVALUATING → VERIFIED_CANDIDATE → ATOMIC_COMMIT → PROTECTED
                            ↘ UNAVAILABLE
```

仅在证书对应的估计中心和输入身份仍匹配时提交。后端同一时间点修改位置时，可用已证明的中心偏移界重新绑定；不同时间点需要时间传播证明，不能只用两估计值之差修补。

受保护输出与未保护的高速估计输出在消息字段或 topic 契约中明确区分。watchdog 以单调壁钟计期限，数据新鲜度另以 sensor timestamp 检查；回放中的 `/clock` 跳变/暂停不能使 wall-time 统计失真。

如果现有消息不能容纳证书身份，可扩展现有消息或增加一个绑定到相同 ID 的证书消息，但消费者必须验证两者配对。不能靠两个未绑定 topic 的发布顺序假装原子性。

---

## 9. 配置定稿与迁移规则

### 9.1 新 schema 示例

以下是**待实现的目标 schema 示例，不是声称当前 YAML parser 支持**。`intervals: 20` 是明确的实验口径；迁移时先读取原配置含义，不强制覆盖当前用户参数。风险数值是目标，不能替代先验/噪声/包络证据。

```yaml
integrity:
  schema_version: 1
  protected_quantity: position_xyz
  position_reference: uwb_tag
  risk_time_basis: per_protected_epoch
  update_trigger: uwb_batch

  window:
    intervals: 20
    auto_shrink: false

  fault_model:
    manifest: config/integrity_fault_manifest.yaml
    max_fault_order: 1
    allowed_pair_families: [uwb_imu]
    reject_unsupported_family: true
    hot_reload: require_full_rebuild

  computation:
    backend: auto_square_root
    projection_policy: algebraic_eligibility
    traversal: exact_with_verified_envelopes
    use_actual_measurement_support: true
    reuse_symbolic_pattern: true
    cache_by_dependency_revision: true
    unresolved_numerics: reference_fallback_or_unavailable

  admissibility:
    startup_checks: schema_structure_evidence_resources
    per_snapshot_checks: required
    sampled_geometry_check_is_proof: false

  history:
    summary: fault_and_detection_preserving
    detector_layout: split_state_and_history
    history_projection: verified_fault_span_or_full
    invalid_summary_action: unavailable
    require_lineage: true

  fde:
    instantiate_actions: on_alarm_or_health_demand
    uwb_action_policy: exclude_supported_measurements
    imu_action_policy: conditional_interval_repair
    require_sensitivity_for_all_monitored_faults: true
    selection_risk_policy: union_over_certified_guarantee_groups
    unvalidated_bridge: reference_transfer_or_unavailable

  risk:
    p_hmi_target: 1.0e-7
    construction: detector_partition_bound
    axis_rule: joint_box_union_bound
    omitted_mass: derive_from_event_manifest
    require_model_evidence_for_protected_publish: true

  output:
    bind_state_and_certificate: true
    normal_deadline_ms: 40
    on_deadline_miss: unavailable
    high_rate_estimates_without_propagation: unprotected

  resources:
    # 示例容量；A 按实际模式、硬件和摘要增长上界核实后锁定。
    max_history_parameters: 1024
    max_active_dictionary_columns: 4096
    max_history_bytes: 67108864
    max_pending_snapshots: 2
    on_limit_exceeded: unavailable
```

`union_over_certified_guarantee_groups` 按 §8.5 的共享参考证明分组；没有该证明的动作各自计账。

`auto_square_root` 只在已实现的块链/一般稀疏平方根 adapter 之间选择，不表示可以自动选择任何未实现算法。`verified_fault_span_or_full` 的回退必须同步改变 detector ID、df、阈值和风险缓存，不能仅修改能量值。

禁止让 `fault_classes.uwb_range.mode=excludable` 自动强制 `subset_covariance`，尤其不能对一个 template 使用删整源的协方差差后标记为 EXACT。也不要同时存在 `imu.mode=bound_only` 和“该源允许 conditional exclusion”两个互相冲突的控制项。

### 9.2 迁移必须自动、可审计且只做一次

| 旧字段/旧行为 | 新处理 |
|---|---|
| `integrity_window.epochs` | A 通过实际节点数量与移窗行为确定含义，再转换为 intervals；转换后的有效配置必须导出。 |
| `single_fault/double_fault` 或旧阶数字段 | 映射到 `max_fault_order` 与 family；两个新旧值冲突时拒绝，而非按加载顺序覆盖。 |
| `excludable/bound_only` | 只作为动作政策别名；`bound_only` 映射为 `monitor_only`，与数值路径无关。 |
| `max_linearization_step_norm` | 保留迁移警告；转换为有尺度的诊断与明确有效性检查，不能仅删除 gate 后视为可信。 |
| 人工 `unmonitored_fault_mass` | 拆成目标上限与有证据的事件概率上界；未知不是零。 |
| `history residual budget` 而无摘要/来源 | 记录为未完成证据，不因预算数值小就允许继续保护。 |
| 报警后丢掉已排除源的风险 | 用候选动作下的完整物理事件模型与历史响应重新评估。 |
| 后台双故障审计 | 可保留为诊断，但不能计入已经发布的单故障保护声明。 |

冷启动配置开启双故障时必须构建足够的历史交叉能力。运行中切换若没有完整历史重建能力则不支持，明确返回状态；不能保留 order=1 的摘要却宣称 order=2 已生效。

---

## 10. 测试规格：一张矩阵覆盖数学、工程与历史

### 10.1 必测项

下列 ID 是本路线图建议的测试标识；复用现有测试时建立映射，不要求全部新建文件。

| ID | 测试场景 | 必须验证 | 主要工作包 |
|---|---|---|---|
| CFG-01 | 新旧 YAML 一致/冲突、launch 覆盖 | 生效值、错误分支、hash 与运行一致 | A |
| CFG-02 | order=1、order=2、未知 family | 单双故障集合准确；未支持族拒绝 | A/B |
| CFG-03 | 固定基站、运动位置/姿态变化 | 启动通过不掩盖运行时退化 | A/B |
| GEO-01 | §6 A4 两行解析 LS | P/G/Γ/s² 精确对应，PL 有限 | A/B |
| GEO-02 | 一维完全不可检测故障 | 返回危险零空间，不误诊成大字典问题 | A/B |
| GEO-03 | 多个备选一维模式，K 大于 ν | 不因 K 或总字典列数而统一拒绝 | A/B |
| GEO-04 | q>ν 的无害 nuisance 零空间 | 在结构性证据下保留有限位置 bound | A/B |
| GEO-05 | 双故障残差相消 | 单故障有限、组合危险，交叉项不可省略 | B |
| NUM-01 | 状态缩放、变量/因子排序、列主元 | 物理输出与 bound 不变；C 同步转换 | B |
| NUM-02 | 纯检测行 H=0、A/z 非零 | 不丢检测信息与自由度 | B/C |
| NUM-03 | 极小奇异值、近零 covariance difference | 不做任意 regularization/负值截零；可靠回退 | B |
| NUM-04 | SparseQR pattern 不变/改变 | 合法复用与失效；不能使用旧 R | B |
| MOD-01 | UWB 瞬时/阶跃/ramp，丢包与非均匀时刻 | 真实时间支撑、末端退化、单位与白化 | A/B |
| MOD-02 | IMU 原始样本 ± 故障、多步长重积分 | 映射方向、量纲、bias/坐标约定正确 | A/B |
| MOD-03 | 区间内部 onset、跨窗口持续 step | 非整区间模板不能误用完整 bias Jacobian | B/C |
| MOD-04 | 同器件多轴共因、两个事件共享参数 | 事件阶数、参数拼接、先验账本一致 | A/B |
| MOD-05 | 有效有界幅值、但检测零空间危险 | 用集合支撑界而非误套无界判据；集合失效风险记账 | B/C |
| COV-01 | template 与有效 free envelope | 相同模型/风险下 slope 支配；不假定 recent-k 包含早期模板 | B |
| COV-02 | 辅助子集恒等式 | 适用行子集下 covariance difference 与投影相符 | A/B |
| COV-03 | 不适用子集：相关白化/历史污染/template | 拒绝 EXACT 或标记合法外包络 | B/C |
| COV-04 | 分组提前停止与时间耗尽 | 全部叶节点始终有有效覆盖；否则不可用 | B/D |
| HIS-01 | 完整历史与摘要，多个消元顺序 | Σp/G/Γ/代价/df 等价，不只比轨迹 | C |
| HIS-02 | 原始 IMU/UWB 故障跨左边界 | 故障影响与检测信息持续保留 | C |
| HIS-03 | old/new 数据归属重复或漏记 | lineage 检查失败；无“增加冗余”的假象 | C |
| HIS-04 | 历史重线性化、权重/白化改变 | 重建/有效包络/不可用，不重新贴版本号 | C |
| HIS-05 | 运行中阶数/family 改变 | 缺交叉历史时必须重建或拒绝 | C |
| HIS-06 | 参数/内存容量满、故障结束后残留 | 不静默遗忘，不把停止注入视为恢复 | C/D |
| DET-01 | 单/双通道，ν=0 边界 | 能量、阈值、Λ 与 risk proof ID 一致 | B/C |
| DET-02 | 只有合并检测方向才能监测的故障 | W 的核测试与联合分区 bound 正确 | C |
| DET-03 | 历史 fault-span 投影 | Γ/score 保持，df 正确，漏掉声明方向时拒绝 | C |
| DET-04 | 最近故障/长期故障/旧纯噪声增长 | 定量比较池化与分离布局，不宣称普遍单调 | C/D |
| RSK-01 | 多轴相关 Gaussian、各模式预算 | 联合盒风险不重复花预算；小尾求解保守 | B/C |
| RSK-02 | 模式合并、事件重叠、遗漏质量 | 不因模式数减少凭空得到风险红利 | A/B |
| RSK-03 | 非线性/历史误差抵消故障残差 | 位置裕度与残差门限均生效 | C |
| FDE-01 | 健康普通帧 | 无动作/桥接 eager 构造，灵敏度仍在 | B |
| FDE-02 | 隔离后恢复需 evidence | 惰性化不破坏健康状态机 | B/C |
| FDE-03 | 错排、漏排、两故障、旧先验污染 | 未满足 post-selection 证明不得恢复发布 | C |
| FDE-04 | IMU 区间修复/桥接失败 | 合法参考转移或不可用；不增加虚假的 Gaussian 信息 | C |
| FDE-05 | 共享参考的多个候选中心/多个不同参考 | 同参考共用失败事件；不同参考选择不能少记风险 | C |
| OUT-01 | 后端同时间改解、证书到达晚 | ID 检查/中心转移；不复用旧证书 | C |
| OUT-02 | 不同时间/坐标系/tag-body 混用 | 拒绝或有明确变换界 | C |
| OUT-03 | 过期、队列拥塞、模拟时钟跳变 | 及时不可用，wall/sensor 时间分开 | C/D |
| PERF-01 | 同覆盖、同环境、两种 order、多轨迹 | 全流程时延/资源/有限有效 PL 比例 | D |

### 10.2 数值误差与统计验收的口径

**不能对所有场景一律使用相对误差 ≤1e-10。** 接近零、病态、非线性有限差分和保守上界需要不同标准。

建议初始测试规则：对经过尺度归一化且条件良好的纯线性 oracle，用 `atol=1e-12、rtol=1e-10`；对生产数据派生矩阵采用预声明尺度与条件数分层阈值。阈值改变必须有依据，不能为了让失败测试通过临时放宽。

对解析同一结果的路径检查 equality；对 envelope 检查**保守方向**与覆盖证据；对不可用场景检查原因及应保留/禁止的输出。两个实现都返回 `inf` 不算对照通过。

IMU 有限差分用多个合理扰动步长观测收敛区间，验证正负号与单位；不用单个过小 h 导致的舍入误差否定解析映射。接近奇异时优先验证失效分类/数值界，不比较两个不稳定的大 PL 是否相等。

Gaussian/χ² 分布检查可以使用具有已知模型的合成样本，对均值、协方差、卡方能量矩和接受率给出统计置信区间。它们是模型实现测试，不能用有限样本证明 `10⁻⁷` 部署风险。真实故障先验与包络证据仍单独审查。

### 10.3 一个测试驱动入口，避免各包新增不同脚本体系

优先扩展现有测试 runner；若没有，A 新增统一入口，例如：

```text
tools/integrity/run_validation.py
  --suite oracle|manifest|projection|history|fde|output|replay|benchmark|all
  --config <effective config>
  --output <evidence directory>
```

该路径与 CLI **是建议的新交付物，不是当前仓库事实**。runner 调用 A 记录的真实 test target/回放命令，不硬编码本文推测的包名。

每项结果必须为 `PASS / FAIL / NOT_RUN / SKIP_WITH_REASON`；必要套件未运行时不返回“全通过”。JSON 至少含：`base_sha、run_sha、config_digest、manifest_digest、case_id、seed、command、exit_code、status、metrics、artifact_paths、assumptions`。

流程中使用一个持续更新的验证 manifest，将已有测试与上表 ID 映射。每包复用该入口，不再单独做“单故障测试框架”“历史测试框架”“双故障测试框架”。

### 10.4 证据目录建议

```text
doc/evidence/integrity-kernel-refactor/
  code-map.md
  worktree-before.txt
  remote-head.txt
  census.json
  effective-config.yaml
  fault-manifest-resolved.yaml
  baseline-report.md
  proof-obligations.md
  config-migration.md
  runbook.md
  validation-manifest.json
  oracle-results.json
  history-results.json
  fde-output-results.json
  benchmark-summary.json
  final-review.md
```

大的逐帧 trace、原始 bag 和二进制 fixture 按仓库现有数据管理规则存放，只在报告中记录可追溯 ID/hash，不把大文件无意加入源码库。

---

## 11. 性能目标与资源策略

### 11.1 衡量同一个问题

比较必须固定：被保护的点/时刻、故障事件和时间域、阶数与 family、先验与风险目标、noise/whitening、边界模型、数据集、构建方式与硬件。改变其中任一项就另列实验，不能把范围变小的结果叫“等价加速”。

旧稿 629 ms 等数字可以写为历史参照，但不能当成本轮代码基线。A/D 重新测得的 trace 才是当前报告的基准。普通/报警/边缘化/回退帧分开报告，不能通过丢弃慢帧提高 p99。

### 11.2 核心目标，而不是速度保证

沿用讨论中的 **40 ms 正常受保护输出期限**作为待验证目标。验收中建议要求：在冻结环境和负载下，正常帧端到端 p99 不超过该目标；实际每次期限由 watchdog 执行。p99 达标不等于硬实时 WCET 证明，也不允许其余帧晚到后仍标为当次及时保护。

若运行目标另为 20 ms/10 ms，作为单独的负载档测试，而不是在规划阶段宣称不可能或保证能达到。FDE 恢复期限另设；报警后不可用标记不得等到整个 FDE 算完才发布。

每轮报告至少含：

```text
snapshot/export/whitening/factorization/projection/risk/evidence/FDE/commit
queue_wait / computation_time / sensor_age / end_to_end
p50 / p95 / p99 / max / sample_count / warmup_rule
alloc_count_hotloop / alloc_count_solver / peak_bytes
factorization_count / projection_columns / evaluated_modes / covered_modes
fallback_count / deadline_miss_rate / finite_valid_PL_ratio
```

### 11.3 预分配的边界

稳定配置/容量下，模式 descriptor、组合索引、小矩阵求解缓冲和普通帧遍历要求无逐模式动态分配。稀疏 QR 的内部申请、快照导出与 ROS 序列化分开测，不在没有 instrumentation 的情况下声称“正常帧零分配”。

如果零分配需要完全重写求解库，应先比较收益与风险；内存有界、无每模式大分配及尾延迟达标优先于为了口号引入复杂池化系统。达到任何预设容量必须保持安全失败语义。

### 11.4 增量化不是最后再写一套算法

B 已完成因子/字典版本缓存和符号结构复用；C 已完成历史增量摘要。D 仅在 trace 证明需要时增加数值局部更新。对修改的 Jacobian/权重/边界，影响范围由依赖图和版本决定，不按“这帧没有报警”决定。

更新后必须恢复与同一冻结模型重新分解相同的结果或有效保守界。若证书不确定，重分解/参考回退或不可用；不能只定期纠偏、中间帧用未知误差的旧矩阵。

**不采用“链式结构，所以必快 N² 倍”的验收指标。** 链式分解改变一个工作项的复杂度，不能解释全部 pipeline。把分解做快之后，故障投影、尾概率、构图、校验、日志与队列都可能成为主导。[本轮对 C3 §2.5 的修订]

---

## 12. 合并验收与证据等级

四个工作包服务一个目标，以下是最终必要条件，不是四个版本的验收。

| 门 | 通过条件 | 不能作为替代的东西 |
|---|---|---|
| **代码与配置门** | 已绑定实际 base/run SHA；变更与现有结构对齐；有效配置/manifest 可复现 | 旧文档里写过某路径或某提交 |
| **代数正确性门** | 名义输出、G/Z/Γ、slope、摘要和参考一致；边界/反例分类正确 | 新旧都返回 inf；仅比较轨迹 RMSE |
| **覆盖门** | 每个声明事件在线被 exact 或合法 envelope 覆盖，组合交叉项完整 | top-K、抽样起点、只测最可能故障 |
| **风险门** | detector 接受事件与 PL 推导一致；逐轴、模式、时间、遗漏与选择风险闭合 | 预算相加小于目标；一个权威算法名字 |
| **历史门** | 故障和检测信息均有来源；无重复/丢失；重线性化/配置变化/容量失效正确 | nominal marginal、单独 B_b、故障离窗即删除 |
| **FDE/发布门** | 候选选择与修复可认证；状态和证书原子绑定；超时/错配拒绝 | 排除后 residual 变小；发旧 PL 保护新状态 |
| **实时与资源门** | 同覆盖实测满足已声明档位；上限/回退/队列行为经过测试 | 渐近阶数比；另一台机器的文献耗时 |
| **模型证据门** | noise/先验/非线性或桥接包络对所声明场景有有效依据，限制写清 | 合成测试通过；把假定概率写进 config |

通过软件与数学门，可证明的是实现满足已声明模型中的规格。能否标记真实部署的受保护输出有效，还取决于模型证据门。独立公式评审和实际数据校准没有完成时，状态必须继续如实显示；Codex 不得自动把整个项目从 `IMPLEMENTED_UNVERIFIED` 改成“已正式认证”。

最终 `final-review.md` 至少回答：实际改了哪些模块、复用了哪些已有能力、默认与双故障分别覆盖什么、仍可能出现哪些正常的不可用原因、实际性能、未完成的证据、复现命令和哪些旧结论已废止。

---

## 13. 可直接给 Codex 的执行指令

```text
请在当前 UWB–IMU–PL 仓库中执行本路线图，而不是重新写方案。

先读取 AGENTS.md 和当前构建/测试说明。记录工作树、完整 HEAD 与远端
目标分支状态；保留用户未提交改动。本文编制者未取得当前源码，不能据此
假定任何旧路径、矩阵尺寸、配置默认值或性能数字仍正确。

按 A/B/C/D 四个工作包执行：
A 同时完成实际代码映射、census、声明/接口、配置适配和独立参考；
B 同时实现统一平方根、精确字典、单双故障、覆盖计划、普通帧风险与惰性动作；
C 同时实现历史摘要、对应检测/PL、IMU 条件处置、选择风险和原子发布；
D 做端到端回归、实测与确有收益的局部增量优化，并清理重复生产路径。

已经实现且通过所需验证的功能直接复用，不另建 V3，不先做单故障简化版，
不把风险/历史补丁推迟到一个“正式版”。B/C 共用 A 定义的接口；测试与
代码一起提交，不把失败隐藏成 NOT_RUN 或跳过。

禁止：把模式数当维数；按 excludable 标签选 kernel；静态启动检查替代
每帧几何证书；按 template/recent-k/free 名称假定包含；省略双故障交叉项；
仅保存 B_b；用小 covariance 把无依据桥接当独立 Gaussian 信息；运行中
扩展故障声明却不重建不兼容历史；选择候选后删除未证明已清除的风险；
使用旧 PL 保护不同时间/身份的输出；用缩小覆盖换取性能。

每完成一组相关修改，运行真实测试入口，更新统一 evidence/validation
manifest。报告实际修改文件、测试命令、PASS/FAIL/NOT_RUN、证据路径与
剩余阻塞项。缺少原始数据或模型证据时仍实现所需接口和安全失败测试，
但不能编造通过结果，不能提升保护声明。

若发现当前源码已经改变本文假设，先更新 code-map.md 和适配记录，再完成
同等目标；不因路径不同重复造模块，也不在有源码可查时先要求用户解释。
```

---

## 14. 本轮确实做过的检查与没有做过的检查

**已完成：**完整阅读当前两份附件；尝试取得 GitHub 分支、raw 和 git 远端；检索项目可用资料；核查下列官方资料；在本地运行固定种子 `20260921` 的小型合成代数检查。

| 本地检查 | 结果 |
|---|---:|
| 早期模板对 recent-2 子空间的正交残差 | 1，验证不包含反例 |
| 相同四基站、两个目标位置的测距 Jacobian 秩 | 2 与 3 |
| 删除标量过程约束的 slope² / 子集方差增长 | 均约 1/3 |
| q=2、ν=1、无害 nuisance 零空间例 | `G ker(Z)=0`，slope² 约 1/2 |
| 100 个随机系统的历史消元代价最大相对误差 | 约 `1.04e-15` |
| 历史 fault-span 投影的 Gram 最大绝对差 | 约 `6.62e-14` |
| 历史 fault-span 投影的 score 最大绝对差 | 约 `1.10e-14` |
| 投影后噪声协方差与单位阵最大差 | 约 `2.05e-15` |
| 10,000 个方向的双检测边界内椭球界检查 | 未出现正向违反 |

这些检查支持本文反例和部分线性代数构造，没有代替解析证明，也没有验证数据依赖线性化、极小尾概率、真实 IMU 故障或完整性部署要求。

**没有完成：**确认该分支当前 HEAD、读取最新源码、编译仓库、运行 ROS 模拟器/数据集、统计真实 inf 首因、测量新内核运行时间。因此本文不能填写这些实测结果；它们是 A/D 在实际源码环境中必须交付的内容，而不是已经完成的审核。

---

## 来源

### 当前上传文档

- **[G0]** `UWB_IMU_PL_方案复核与综合改进.md`。主要采用：§3 数学对象与核判据，§4 覆盖/计算区分，§5 平方根与辅助子集，§6 历史摘要，§7 幅值分区风险，§8 IMU/FDE，§9–12 架构与验证。其自身声明没有核实当前 GitHub HEAD。
- **[C3]** `Claude roadmap 3.md`。主要审核：§1 更正、§2.1 内核分派、§2.2 历史检测、§2.3 覆盖阶梯、§2.4 启动可采纳性、§2.5 性能比例，及 S0–S8 与 YAML。原文部分表格位置未包含表格内容，本文没有补称“原表建议了什么”。

### 本轮核查的公开原始资料

- **[R1] GTSAM 官方 ImuFactor 文档。** 核对九维残差、连接变量、bias 演化需另外建模及预积分的一阶 bias correction。只用于依赖定义，不证明当前项目使用该因子。`https://borglab.github.io/gtsam/imufactor/`
- **[R2] GTSAM 官方 CombinedImuFactor 文档。** 核对十五维误差、六变量连接及预积分与 bias 的相关协方差；不能在使用 combined 模型时机械重复加入同一 bias 约束。`https://borglab.github.io/gtsam/combinedimufactor/`
- **[R3] Eigen 官方 SparseQR 与稀疏线性系统文档。** 核对 `A*P=Q*R`、隐式 Q、列置换、symbolic/numeric 分离、空行输入限制与数值秩的局限。nightly 文档不代表目标机版本，开发必须核对本地头文件。`https://libeigen.gitlab.io/eigen/docs-nightly/classEigen_1_1SparseQR.html`；`https://libeigen.gitlab.io/eigen/docs-3.4/group__TopicSparseSystems.html`
- **[R4] Joerger, Chan, Pervan (2014), Solution Separation Versus Residual-Based RAIM, NAVIGATION 61(4):273–291，DOI 10.1002/navi.71。** 本轮核查出版方摘要，确认 RB/SS 的比较取决于构造与实现约束；没有取到作者托管 PDF 全文，不据此宣称逐式核对 C3 引用的所有编号公式。本文的协方差差关系由所列线性代数与本地测试支撑。`https://onlinelibrary.wiley.com/doi/10.1002/navi.71`
- **[R5] GTSAM 官方 Fast Covariance Recovery with Bayes Trees，2026-03-29。** 支持按查询局部结构恢复协方差的实现方向；不要求目标仓库升级到该文章所述的新版本，也不提供本项目速度保证。`https://gtsam.org/2026/03/29/bayes-tree-covariance-recovery.html`
- **[R6] Zhu, Meurer, Joerger (2023), Integrity Analysis for Greedy Search Based Fault Exclusion with a Large Number of Faults，PLANS，DLR 托管稿。** 本轮取得解析正文，用于确认多故障排除需要独立完整性分析；截图未成功，本文未依赖其图表或复制编号公式。`https://elib.dlr.de/201771/1/PLANS_2023_Zhu_v3_IEEE_compatible.pdf`

**最终执行原则：把需要共同证明、共同维护版本和共同测试的改动放进同一个工作包；把真正改变故障声明或检测事件的改动显式记录。不要用增加阶段掩盖接口未定，也不要用取消阶段跳过证明与回归。**
