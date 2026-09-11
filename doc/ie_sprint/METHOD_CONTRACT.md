# UWB-IMU-IE 方法合同（T01）

状态：`T01_DONE`

冻结依据：[`../v2/paper_structure.tex`](../v2/paper_structure.tex)、[`../v2/v2_roadmap.md`](../v2/v2_roadmap.md)

环境事实：[`REPO_AUDIT.md`](REPO_AUDIT.md)

适用范围：T02–T13 的 paper path；legacy 路径只要求兼容，不据此补写新能力。

本文是后续实现的唯一方法合同。每条规则标明来源类别：

- **FROZEN**：冻结论文已经规定的数学或证据边界，修改前必须在
  [`STATUS.md`](STATUS.md) 登记 amendment 并获负责人确认。
- **SELECTED**：roadmap 建议且 T01 选定的工程实现方式；当前仅完成合同，尚未实现或验证，
  也不表示已获负责人审查确认。
- **PENDING**：需要数值测试、验证集或外部来源闭合后才能锁定的事项；在对应准入条件满足前
  不得用于正式 test 或论文结果。

## 1. 符号、单位、输入和代码对应

| 符号/对象 | 单位 | 合同定义 | 拟对应代码对象与当前事实 | 来源/状态 |
|---|---:|---|---|---|
| `i` | — | 一条原始 UWB 观测；由稳定 `obs_id` 唯一标识 | **拟新增** `ObservationRecord`；当前 `UwbRange` 只有 anchor、range、RSSI，未实现稳定 ID | FROZEN；T02 `NOT_RUN` |
| `k` | — | 策略无关关键帧编号 | 当前 `UwbFrame` 经旧入口预滤后下采样；paper path 的冻结计划未实现 | FROZEN；T02 `NOT_RUN` |
| `m` / link | — | link 至少是 `(tag_id, anchor_id)`，不能只凭 factor index 标识 | 当前 `UwbFrame.tag_id` 与 `UwbRange.anchor_id` 可提供原始字段 | SELECTED；T02 `NOT_RUN` |
| `t_i` | s | 传感器时间；排序、gap、prefix cutoff 均使用同一声明时基 | 当前 loader 的具体时间语义见 `REPO_AUDIT.md`，不同 adapter 尚未统一验证 | FROZEN/PENDING；T02/T09 |
| `z_i` | m | 不可覆盖的原始测距；任何 bias 只在 factor 模型中出现 | 当前 `UwbRange.dist` | FROZEN；T02 `NOT_RUN` |
| `sigma_i` | m | 在任何策略 mask 前冻结的独立名义标准差，`w_i=sigma_i^{-2}` | 当前 `GraphBuilder::AdaptiveSigma()` 会按保留下来的时序计算；paper 快照对象未实现 | FROZEN；T02 `NOT_RUN` |
| `X_k=(R_k,p_k,v_k,b^a_k,b^g_k)` | rad, m, m/s, m/s², rad/s | keyframe 导航状态；其中 `b^a,b^g` 是 IMU bias | 当前 GTSAM `X(k)` Pose3、`V(k)` Vector3、`B(k)` ConstantBias | FROZEN；现有对象已声明，paper 流程未验证 |
| `a_m` | m | 世界系 anchor 位置；主配置要求独立 survey 后固定 | 当前 `AnchorConfig.pos`；可选 `A(m)` correction 是现有扩展 | FROZEN/PENDING provenance |
| `ell` | m | IMU/body 到 UWB antenna 的杆臂；主配置独立标定后固定 | 当前 `Config::lever_arm_init`；可选 `L(0)` 在线变量 | FROZEN/PENDING provenance |
| `beta_m` | m | 有符号、按 link/anchor 的固定静态测距偏置；主配置来自独立 LOS 标定 | 当前只有可选在线 `Z(m)`，关闭时固定非零值会漏用；固定入口未实现 | FROZEN；外部值 `PENDING_EXTERNAL`，T02 |
| `b_i` | m | Stage 1 的逐观测非负动态 excess path，`b_i>=0` | T06 `DiscoveryObservation::bias_m`/chain solver 已实现，不是 IMU bias | FROZEN；T06 `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| `c_s` | m | Stage 2/4 中段 `S_s` 共享的非负动态幅值，`c_s>=0` | T04 oracle-debug 已实现独立 `C(segment_ordinal)`；T08 final graph 对 accepted segment 保留同一独立 `C`，不复用 `B(k)` 或 `Z(m)` | FROZEN；T04 `REVIEW_ACCEPTED`；T08 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| `C` | obs_id 集 | 所有候选段观测的并集 | **拟新增** mask/segment 元数据 | FROZEN；T02/T06 |
| `A` | segment 集 | 时间重叠图的一个连通分组 | T05 `SegmentOverlapGroup` 已实现并在测试支持域独立复审接受 | FROZEN；T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `F,G` | 白化残差对相应局部坐标的 Jacobian | `F` 对全部 nuisance，`G` 对组内 `c_A` | T05 从同一 debiased graph/Values 线性化并保存 key-column、factor-row/obs 映射；评分前完整校验 graph/metadata/support/plan | FROZEN；T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `N_A` | m^-2 | `G^T G`，已知 nuisance 时的名义 bias precision | T05 score dump 已实现；弱信息不再被单位尺度地板覆盖 | FROZEN；T03/T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `R_c,A` | m^-2 | `G^T(I-P_F)G`，消去 nuisance 后的 rank-aware profile/marginal information | T05 production prototype 用 sparse least-squares residual 构造，不显式形成稠密 projector | FROZEN + SELECTED；T03/T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `eta_A` | 1 | `lambda_min(N_A^{-1/2} R_c,A N_A^{-1/2})`，相对可分离性 | 不是 accuracy 或绝对不确定度；不可用时主字段为 unavailable，原始诊断独立保存 | FROZEN；T03/T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `s_A` | m | `R_c,A` 正定时为 `lambda_min(R_c,A)^(-1/2)`，否则 `+inf` | 区分 rank-deficient 的 `+inf`、short/boundary 的不适用和数值失败的 unavailable | FROZEN；T03/T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| `gamma_s` | 1 | 段内平均平方 nominal-normalized post-fit residual | T05 从 Values/raw/fixed beta/`c_s`/nominal sigma 重算，不读 residual CSV | FROZEN；T05 `REVIEW_ACCEPTED_VALIDATED_DOMAIN` |

测距残差统一为

```text
r_i(X,beta,c) = h_i(X) + beta_link(i) + c_segment(i) - z_i,
h_i(X) = ||p_k + R_k ell - a_m||_2.
```

非候选观测取动态项 `0`。固定 `beta` 是已知常量，在线 `beta` 是 nuisance 状态；两者不能同时应用。
`beta`、`c_s` 与 `B(k)` 的 IMU bias 是三类不同物理量、不同 key 空间和不同日志列。缺失固定
`beta` 标定时状态必须为 `MISSING_CALIBRATION`，不得以 `0 m` 伪装为已标定值。

## 2. 原始观测、ID、mask、关键帧和名义噪声

### 2.1 不可变观测账本

**FROZEN。** paper path 在任何 NLOS 策略前建立只追加的观测账本。`obs_id` 由
`recording_id + source message ordinal + range ordinal` 确定；timestamp 相同或同一帧同一 link 重复时仍唯一。
过滤、重建图、缓存和不同策略不得重编号。factor index 只在一次 graph 构建内有效，并通过
`FactorMeta(factor_index, obs_id, factor_type, keys, graph_id)` 映射。

每条观测至少保存下列 mask/状态，均以布尔列或原因码记录：

1. `valid_format`, `valid_range`, `known_anchor`：只表示输入有效性；
2. `suspected_nlos`：提示信息，不能在候选阶段前删除；
3. `in_keyframe_plan`：策略无关采样计划；
4. `candidate`, `active_support`, `segment_id`, `eligible`, `group_id`；
5. `decision_use`, `final_use`, `fallback_use`；
6. 每次失败的原因码，不用缺行表示拒绝或失败。

### 2.2 策略无关计划

**FROZEN。** 有效输入池、初始化规则、关键帧时刻、每帧观测关联和 `sigma_i` 必须先冻结并计算
hash，之后才施加 all-range、baseline、候选、Use/Suppress 等 mask。所有比较策略复用同一计划。
当前 `AdaptiveSigma()` 的结果只能在完整固定输入顺序上快照一次；删观测后不得重新计算
`dt_since_last`。`gamma` 与信息矩阵只用此名义 `sigma_i`。

**SELECTED。** T02 用显式 `InputPlan`/等价不可变对象承载上述记录；paper runner 若暂只支持单 tag，
遇到其他输入必须显式报 `UNSUPPORTED_TAG_CONFIGURATION`，不能静默混合 link。

## 3. 四阶段输入、输出与候选状态流

| 阶段 | 输入 | 输出 | 不变量/失败状态 | 来源 |
|---|---|---|---|---|
| Stage 1 support discovery | 固定观测账本、关键帧计划、名义 `sigma_i`、独立标定、仅 prefix 内数据 | 逐观测 `b_i>=0`、候选 mask、change points、冻结 segments、求解诊断 | 正式模式不读 GT/oracle；`D` 只连同 link 且不跨 gap | FROZEN；T06 |
| Stage 2 debiased refit | 冻结 segment partition、原始 ranges、完整当前 graph | 无 L1/TV 的受约束联合 `X,beta,c`、完整 graph/Values、factor metadata、boundary/short 状态 | partition 不变；去正则不等于去除 support-selection bias | FROZEN + SELECTED solver；T04 |
| Stage 3 score | Stage 2 的同一 debiased linearization、共同 `G0`、冻结 groups、名义权重 | decision-time `F,G,N,R_c,eta,s,gamma,rank,status,linearization_id` | `G0` 排除全部候选；每组只加自己的 raw candidate factors | FROZEN + SELECTED sparse solve；T03/T05 |
| Stage 4 decision/final | 冻结 support/group、锁定 gate、Stage 2 graph/Values | 组级决定、accepted `c_s` 的最终受约束联合图、final scores、covariance/status；必要时一次 fallback | raw range 至多一次；无 re-admission；所有最终输出同一 graph/Values | FROZEN；T08 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；正式锁定 gate/RQ `NOT_RUN` |

一个观测/候选段的完整状态流为：

```text
raw observation (stable obs_id)
  -> validity only (suspected NLOS retained)
  -> fixed keyframe/input plan + nominal sigma
  -> Stage 1 active support
  -> maximal per-link segment, or INACTIVE
  -> boundary/short/gap checks
  -> temporal-overlap connected group
  -> Stage 2 constrained refit
  -> ELIGIBLE or INELIGIBLE(reason)
  -> common-reference decision score
  -> USE(group) or SUPPRESS(group)
  -> final joint graph: original accepted factor + live c_s exactly once
     or absent candidate factor
  -> final audit
  -> OK, FALLBACK_OK after one all-candidate suppression, or ESTIMATION_FAILED.
```

## 4. 支撑、分段、gap、短段和非负边界

以下结构规则在 T01 选定；数值参数见第 8 节，未锁定前不得跑正式 test。

### 4.0 Stage 1 交替求解与停止审计

**FROZEN objective；SELECTED implementation；T06 development engineering scope 独立复审接受。** 固定导航状态时，每条
chain 在声明容差内求解（不宣称代数精确）

```text
min_{u>=0} 0.5 sum_i w_i (u_i-e_i)^2
           + lambda_1 sum_i u_i + lambda_TV ||D u||_1,
e_i = z_i-h_i(X)-beta_i.
```

外层从 input-plan 的初始 navigation `Values` 和全零可行 `u` 开始，交替执行：(1) 固定当前 `u`，用
现有 GTSAM graph/LM 更新全部 navigation 状态；(2) 固定该导航状态，按 link/time/obs_id 及 gap 分出的
每条 chain 求上式，以上一轮非负 `u` warm-start；(3) 在最终可行 `u` 上计算原始 L1/TV 目标和 navigation
驻点。没有平方 L2、softplus、GNC、截断式无约束替代或另一套 Python estimator。

chain 子问题使用 consensus ADMM：约束 `b=u`、`Db=v`，`y,q` 是对应 scaled dual，`p=rho*q` 才是
TV 的物理对偶/次梯度，单位为 objective/m。取 `rho=rho_scale*median(w_i)`，固定 rho；初始化
`b=u=warm_start`、`v=Du`、`y=q=0`。每次迭代依次解

```text
(W+rho I+rho D^T D)b = We+rho(u-y)+rho D^T(v-q),
u = max(0, b+y-lambda_1/rho),
v = soft(Db+q, lambda_TV/rho),
y <- y+b-u,  q <- q+Db-v.
```

最终声明解始终是可行 `u`，辅助 `b/v` 仅用于 residual。primal residual 与阈值（单位 m）为

```text
r_p = ||[b-u; Db-v]||_2,
eps_p = sqrt(2n-1)*eps_abs,m
        + eps_rel*max(||[b;Db]||_2, ||[u;v]||_2).
```

dual residual 与阈值（单位 objective/m）为

```text
r_d = rho ||(u-u_prev)+D^T(v-v_prev)||_2,
eps_d = sqrt(n)*eps_abs,obj/m + eps_rel ||rho(y+D^Tq)||_2.
```

两者不得共用米单位绝对阈值。另在最终 `Du` 上检查 `|p_j|<=lambda_TV`，非零差分时
`p_j=lambda_TV sign((Du)_j)`；令

```text
g = W(u-e)+lambda_1*1+D^T p,
```

内点检查 `g_i=0`，非负边界检查 `g_i>=0`，均允许各自声明的 objective/m 容差。只有 primal、dual、
TV 次梯度和原始问题 KKT 全部通过才返回 chain `CONVERGED`。factorization/nonfinite/invalid/max-iteration
均为显式失败。

外层停止同时要求原始完整目标相对变化、组合尺度步长、每条 chain 上述审计和同一最终 navigation state
的物理尺度化驻点全部通过。组合步长先由 GTSAM
`Values::localCoordinates(values_before,values_after)` 得到导航 `X/V/B` 的局部坐标差，按 pose rotation
`1 rad`、pose translation `1 m`、velocity `1 m/s`、accelerometer bias `1 m/s^2`、gyroscope bias
`1 rad/s` 缩放；再与 `max_i |u_i-u_i^prev|/observation_bias_scale_m` 取最大。trace 必须分别导出
navigation、observation-bias 和 combined 三个值及 `step_ok`；只有 combined 值不超过
`discovery_scaled_step_tolerance` 才满足该条件。目标只允许
`64*binary64_epsilon*max(1,|J_before|,|J_after|)` 的舍入上升，超过即
`OBJECTIVE_INCREASE`；LM、chain 或外层上限、非有限值、graph/Values 不一致均失败。该工程常数与下述
development 参数不是已验证的正式 gate/科学参数。

若 Stage 1 冻结 partition 为空，Stage 2 仍在共享的无 `C(s)`、无 L1/TV raw-range graph 上执行导航
refit。其 trace 明确把 segment KKT 标为 `NOT_APPLICABLE`，并逐轮记录原始 raw 目标、GTSAM local-coordinate
尺度步长、最终导航驻点与各条件布尔值；只有全部适用停止条件通过才返回 `NO_CANDIDATES` 并导出该同一
graph/Values 的中间估计。上限、非有限或驻点未达阈值均显式失败且不导出有效估计；score 为空并标
`NOT_APPLICABLE_NO_CANDIDATES`，不触发任何 T08 fallback。

**A02 SELECTED 实现边界（默认关闭；仅 DEVELOPMENT_IMPLEMENTATION_AND_DIAGNOSTIC_RUNS_ONLY）：**
Stage 1 可显式选择版本化的 conditional-navigation stationarity qualification。默认策略保持原行为；开启
A02 策略时，每次 actual linked `gtsam::checkConvergence` 返回 true 后，使用同一 conditional graph、当前
optimizer Values、上述 navigation scales/tolerance/roundoff 规则调用既有 `AuditNavigationStationarity`。只有
generic convergence 与有效且通过的 conditional stationarity 同时成立才返回条件求解成功；驻点未通过则在
原总 inner budget 内继续同一 LM optimizer，不重建 optimizer、不重置 lambda。audit 无效/非有限/异常或预算
耗尽均显式失败，不进入 chain 或导出有效估计。外层四项 AND、FROZEN objective、正则项、初始化、block
顺序、全部数值参数和 gate 不变。policy/version 必须进入实际 solver/support/cache identity、effective config、
trace 与失败诊断；旧策略与 A02 策略不得共享误导性身份。A02 是数值求解策略开发对照，不是已证明的
correctness fix、默认迁移、正式 validation 或科学 claim 支持；后续迁移需另行审查。
当前实现状态为 `A02_R01_REVIEW_ACCEPTED_DEVELOPMENT_IMPLEMENTATION_SCOPE`：默认关闭与 refit 回归、身份隔离、
失败诊断及 step/ramp 各一次 cap=50 development 对照已完成。T10-A02-R01 又明确落实了“内外层使用同一
参数”：当且仅当 A02 policy 开启时，`AutomaticSupportProvider` 与独立
`BuildAutomaticSupportPartition` 在入口精确核对 conditional/outer 两份 navigation stationarity tolerance、
gradient roundoff safety factor 和 pose rotation/translation、velocity、accelerometer-bias、gyroscope-bias
五类尺度；任一冲突分别返回 `INVALID_INPUT` 或抛出 `invalid_argument`，不得仅写入 hash 后继续。
默认 `GTSAM_CHECK_ONLY_V1` 不使用这些 inner 字段，保持原行为/身份；独立 `RunCheckedConditionalLm` 仍可由
其他调用者直接配置，不依赖 discovery outer options。focused config 11/11、discovery 26/26、refit/scoring
23/23 与 runner contract 已本地通过；A02-R01 与下述 A03 已由独立复审接受，接受范围仍只限
development implementation/diagnostic，不构成默认迁移或科学 claim。

A03 只是一项已完成的 `DEVELOPMENT_BUDGET_DIAGNOSTIC`，不是方法参数迁移：两份隔离 A02 step/ramp
配置将 outer cap `50→500`，其余参数不变且各运行一次。两条在前 50 轮保持数值 trace 精确一致，但分别在
outer 121/127 的原 inner cap 因 conditional stationarity 未达标失败，均未进入 Stage 2。该结果不修改上述
objective、四项 AND、默认 cap 或任何合同数值，也不授权继续增加预算。

后续 read-only 停滞诊断不改变本合同：step outer 121 的 50 次 wrapper check 已拆解为 1 次 accepted
state update 与 49 次 linked GTSAM `tryLambda` small-cost-change 无更新返回；后 49 次没有 lambda search，
所以 optimizer iterations 保持 1。最大梯度坐标的预声明有限差分在连续 8 个步长上与解析梯度一致。
诊断 observer 默认关闭、排除于 scientific config/identity，旧三参数 overload 的源码/API 调用形式保留；
其作用仅为保存 branch/Values/factor audit，不参与接受、停止或更新。A04/A05 扩展了公开 C++ diagnostic
struct，受影响二进制必须重新编译并链接，因此不声称 binary ABI 兼容。建议的 fail-fast 显式失败行为尚未
实施；任何改变 A02 solver 行为或身份版本仍须单独审查。

A04 是已完成且已独立复审接受的 `DEVELOPMENT_FIXED_CHECKPOINT_LM_RECOVERY_DIAGNOSTIC`。它在上述 outer-121
停滞后的同一 conditional graph、Values 和 `lambda=1e-6` 上运行两个隔离 shadow optimizer：A 保持内部
`relativeErrorTol=1e-6`，50 calls 均无更新且不驻点；B 仅把 optimizer 内部该值改为 `0`，明确改变
small-change lambda-search 终止语义，同时让外部 generic check 继续使用原 relative `1e-6`/absolute
`1e-8`。B 在三次 lambda trial、两次 wrapper call 中真实接受两个 objective-decreasing update，并以原
stationarity `1e-6`、roundoff `8` 和尺度达到资格。model-fidelity 门槛、lambda 上界 `1e5`、50-call cap
均未改变，shadow Values 未写回 estimator。该结果只证明固定小问题存在有界恢复路径；不改变 A02 V1、
默认策略、冻结 objective、外层停止或正式方法。

**A05 SELECTED development 实现边界（默认关闭）：** 新增版本化
`GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2`。V2 只把同一 optimizer 内部
lambda search 的 `relativeErrorTol` 设为 `0`；wrapper 外部 generic convergence 继续使用调用者原
relative/absolute tolerance，并仍与原 conditional stationarity 资格作 AND。V2 不强制接受增加 objective
或 model-fidelity 不合格的候选，不重建或重置 optimizer，保留 linked GTSAM 接受规则、`lambdaUpperBound=1e5`、
50-call cap、stationarity tolerance、roundoff、五类尺度和外层四项 AND。lambda 搜索在上界耗尽而无 accepted
update 时显式失败为 `CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`，不得冒充 generic convergence 或驻点成功。
V2 进入 effective config、solver/support/snapshot 与 scheduler Stage-2 producer identity；两份 discovery
入口对 V1/V2 都执行 A02-R01 七字段精确一致性检查。默认 V1、A02 V1 和 standalone conditional LM 的调用
行为保持；但 V2 是明确的 solver 数值语义变化，不是“语义完全不变”。

A05 两次预登记 development 运行均使用 A03 的 outer=500、inner=50 且只把 policy 从 V1 改为 V2。V2 恢复
了旧 step outer 121 与 ramp outer 127 checkpoint，并分别达到原 conditional stationarity；随后 step 在
outer 123、ramp 在 outer 168 因原 lambda 上界耗尽而失败。两条均未满足外层四项 AND、未通过 Stage 1、未
进入 Stage 2。故 A05 仅建立默认关闭的 development 集成与失败边界，不支持默认迁移、正式 validation、
科学 claim 或继续扩大预算；独立复审前状态为本地实现/定向运行完成。

1. **链与 gap（FROZEN/SELECTED）：** 按 `(tag_id,anchor_id,t_i,obs_id)` 排序。`D` 只连接同 link
   相邻有效、进入固定 input plan 的观测。当 `Delta t > T_gap` 时断链；`Delta t = T_gap` 保持连接。
   无观测区间不会被插值成 support。
2. **active support（FROZEN/SELECTED）：** constrained discovery 解满足 `b_i>=0`；仅
   `b_i >= b_min` 记 active。数值边界附近保持 `BOUNDARY`，不能凭截断制造正幅值。
3. **change point（SELECTED）：** 在同一 active chain 内，当相邻 discovery 幅值差的绝对值
   `>= delta_change` 时切段。inactive 样本和 gap 必然切段。
4. **segment（FROZEN）：** preliminary segment 是上述规则产生的最大连续 active 集合；每个观测最多
   属于一个 preliminary segment。允许的 merge 仍只限同 link、无 gap、无 inactive 间隔的相邻段，
   不得跨 gap/inactive。确定性执行语义见下节已技术接受的 A01；实现与 U11 已在 T06 development
   engineering scope 内独立复审接受，但科学参数未 validation，不能作为正式 partition hash。
5. **短段（FROZEN/SELECTED）：** `n_s < n_min` **或** `duration_s < T_min` 即 `SHORT_SUPPORT`，
   不进入 gate 的 eligible 分子；它仍留在 candidate coverage 的分母并保留原因码。
6. **边界（FROZEN/SELECTED）：** Stage 2/4 refit 后 `c_s <= epsilon_boundary` 或未满足边界 KKT/
   projected-gradient 条件时记 `BOUNDARY_OR_KKT_INVALID`，不赋予有限 Gaussian 不确定度，不 eligible。
7. **重叠分组（FROZEN）：** segment 闭区间 `[t_start,t_end]` 相交即有边；group 是该图的传递闭包
   连通分量。端点相等视为重叠。组整体 Use/Suppress，不能在组内重新选段。

### 4.1 一次 merge 的确定性规则（A01）

**A01 / TECHNICALLY_ACCEPTED_IMPLEMENTED_REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE。** 本节补足原合同“一次 merge”的代表幅值、权重、
快照和顺序；它影响 partition 定义。本轮用户指挥/审查会话已技术接受该规则，真实确认来源已在
[`STATUS.md`](STATUS.md) 登记。T06 已实现 immutable snapshot、单遍重算、等号合并、哈希父子 ID、
`mu_merge_snapshot`、obs 唯一归属和 partition hash，并通过独立 U11 fixture 复审；冻结的不跨
gap/inactive、Stage 2 固定 partition 等边界保持不变。该接受只覆盖 reviewed development engineering
scope，不锁定正式科学参数。

输入是 Stage 1 达到其已锁定停止条件后、任何 merge 之前的不可变 `support_snapshot`：逐观测
`(obs_id, link, t_i, b_i^disc, w_i)`，其中 `b_i^disc` 是该次 discovery 解的非负幅值，
`w_i=sigma_i^{-2}` 来自策略前冻结的 nominal sigma。快照记录 `support_snapshot_id`、solver/config/input
hash；不得改用 Stage 2 的 `c_s`、post-fit residual、robust weight 或 GT。snapshot、partition、父子 ID
使用 SHA-256，并把 source/config/input plan/calibration/solver config 纳入 discovery context；任一上下文
改变都必须改变 snapshot/partition hash。preliminary segment `q` 的代表
幅值定义为

```text
mu(q) = sum_{i in q}(w_i * b_i^disc) / sum_{i in q}(w_i),  unit: m.
```

实现以当前最大权重缩放后累计，避免有限 `w_i` 与 `b_i` 的乘积或总权重溢出；输入、缩放累计权重、
缩放累计加权幅值、分母或最终结果任一非有限，或分母非正时，partition 构造失败，不以等权替代。
确定性算法如下：

1. 每个 link 独立处理；按 `(t_start, t_end, provisional_segment_id)` 升序排列 preliminary segments。
   `provisional_segment_id` 由 `(partition_rule_version, link, first_obs_id, last_obs_id)` 确定。
2. 将首段置为 accumulator。只对**原排序中的下一个段**做一次从左到右扫描，不回看已输出段，
   也不进行第二轮扫描。
3. 先检查 accumulator 与 next 之间没有 gap、没有 inactive observation；任一不满足即输出 accumulator，
   以 next 新建 accumulator。
4. 若满足邻接条件，使用 accumulator 当前包含的全部观测重算 `mu(accumulator)`，与 next 在原始
   `support_snapshot` 上的 `mu(next)` 比较。`|mu(accumulator)-mu(next)| <= delta_merge`（**等号合并**）
   时合并 next，按时间/`obs_id` 拼接其观测并立即以全部 child observations 重算 accumulator 代表幅值；
   否则输出 accumulator，以 next 新建 accumulator。
5. 扫描结束输出最后 accumulator。输出 segment 的 `parent_segment_ids` 按上述排序保存；
   `segment_id = hash(partition_rule_version, support_snapshot_id, link, ordered_parent_segment_ids)`。
   每个 child 和 obs_id 只归属一个输出 segment，不复制、遗漏或重编号。输出保存 observation count、
   duration、`mu_merge_snapshot` 和 partition hash。
6. Stage 2 接收该输出 partition 后，support、边界和 obs_id 归属全部冻结；Stage 2/3/4 的 refit、
   `gamma` 或 gate 结果不得触发再 merge、split 或重新分配。

三段链式例子：同 link 的 `q1,q2,q3` 无 gap/inactive、各自总权重相等，snapshot 代表幅值依次为
`0.40 m, 0.60 m, 0.80 m`，`delta_merge=0.25 m`。先比较 `q1/q2`，差 `0.20 m`，合并并重算
`mu(q1+q2)=0.50 m`；再比较 accumulator 与 `q3`，差 `0.30 m`，不合并。唯一结果为
`[q1,q2]` 与 `[q3]`。不能因原始 `q2/q3` 的差也为 `0.20 m` 而传递式合并三段，也不能从右向左得到
另一 partition。

### 4.2 非负 refit 求解器

**SELECTED；T04 oracle-debug 已实现并由独立复核接受。** 采用 roadmap 的交替受约束 refit，不宣称全局收敛：

1. 从可行 `c>=0` 开始；固定 `c`，在完整 graph 上对全部不固定 nuisance/navigation 状态执行条件 LM；
2. 固定这些状态，对每个不相交常值段作精确非负加权更新
   `c_s=max(0, sum_i w_i[z_i-h_i(X)-beta_i] / sum_i w_i)`；
3. 在闭式 `c` 更新后的同一个无 L1/TV 最终联合 graph/Values 上重新计算目标，并线性化该联合图；
   对每个自由 `X/V/B` 局部坐标检查尺度化目标梯度，对 `C(s)` 检查非负 projected gradient/KKT，
   同时检查可行性、目标非增、相对目标变化和尺度化状态更新；
4. 只有所有已启用停止条件同时满足才返回 `CONVERGED`。达到 `max_refit_iterations` 仍不满足、
   denominator 非正、出现非有限量、条件 LM 失败、目标超过数值容差上升、graph/Values key 不一致，
   均返回明确失败状态。

无约束 LM 后截断不是允许的替代实现。条件子图只是一个迭代步骤；条件 LM 的目标停滞不能代替闭式
`c` 更新后最终联合状态的驻点检查。最终必须保存含 `c_s` 的完整联合 graph/Values。非负约束不通过
巨大 prior、jitter 或 LM damping 变成报告信息。

## 5. 共同参考、完整 nuisance 和分组评分

### 5.1 参考图与列集合

**FROZEN。** `G0 = priors + IMU + all noncandidate raw range factors`，排除 `C` 中全部候选。
对 group `A` 的评分图为 `G0 + A`，且 A 的每条 raw range 只加一次。其他 candidate group 不在图中，
因此不能互相担保。固定并记录 reference factor mask、非负 robust weight（若确有）和 linearization ID。

`delta y` 包含评分图中所有未固定且非 `c_A` 的变量：

- 每帧 Pose3 `X(k)` 的 GTSAM 局部姿态/平移、速度 `V(k)`、IMU bias `B(k)`；
- 若启用在线估计，则包含 anchor correction `A(m)`、lever `L(0)`、静态 range bias `Z(m)`、
  time offset 及任何真实存在于该 graph 的校准变量；
- 固定 survey/标定常量不建列；初始化值本身不算独立信息；prior 只有真实存在且 provenance/强度已记录
  时才进入 `J_ref`。

主实验预期固定 anchor、lever、clock 与独立 LOS `beta`。可选在线 `beta` sensitivity 若执行，必须把
`Z(m)` 及其真实 prior 同时放入 nuisance。当前 `calib_td` 是否真正进入 factor 尚未验证；不得因配置字段
存在便宣称支持。

### 5.2 `F/G/N/R_c` 的确切构造

在 Stage 2 debiased `Values` 上，将参考因子和 group range factor 的**已白化** Jacobian 按明确 ordering
堆叠：

```text
F = [ J_ref ; W_A^(1/2) H_y ]
G = [   0   ; W_A^(1/2) H_c ]
N = G^T G
Y = argmin_Y ||F Y - G||_F
E = G - F Y
R_c = sym(E^T E).
```

RHS 不属于状态列；噪声不得白化两次。生产路径 **SELECTED** 使用本机可验证的稀疏 rank-revealing
QR/等价平方根最小二乘求 `Y`，不形成全轨迹稠密 `P_F` 或 `A^{-1}`。T03 小矩阵使用 SVD projector，
满秩时另与 Schur complement 对照。

`F` 存在与 `G` 无关的零列/零空间时，不能自动拒绝；判据是是否有非零 `v` 使 `Gv` 落入
`col(F)`。伪逆只可用于数学 projector/golden reference，绝不能把 `R_c` 的不可辨识方向报告成有限
或零方差。

### 5.3 列尺度、秩和容差

**SELECTED；T03 已接受，T05 在已测试 sparse 支持域独立复审接受。** 生产 QR 前对 nuisance 列做一次确定性 2-norm equilibration：非零列
乘 `d_j=1/||F_:j||_2`，精确零列保持零并记录。该可逆列尺度只改变数值坐标，不改变 `col(F)`，不作为
prior 或信息加入。`G` 和物理单位为 metre 的 `c` 列不作归一化，因此 `N,R_c` 保持 `m^-2`。

scaled `F` 的数值秩使用
`sigma_j > max(rank_abs_tol, rank_rel_tol * sigma_max)` 的规则；稀疏 QR 的同等阈值必须与 T03 SVD
fixtures 对照。`R_c` 先检查对称误差、PSD、`R_c <= N` 和最小二乘正交残差；仅容许在已记录的
roundoff tolerance 内对称化。`lambda_min(R_c)` 不高于锁定的 PD tolerance 时，`s=+inf` 并给
`RANK_DEFICIENT`，不能加 diagonal jitter。

`eta` 由对称 generalized eigenproblem `(R_c,N)` 求最小值；`N` 非正定或数值检查失败时分数无效。
`eta` 理论范围 `[0,1]`，越界超过数值容差即 `NUMERICAL_FAILURE`；仅处于容差内的舍入越界可在日志
保留 raw 值后裁到区间。良态 fixture 的初始比较目标为 relative `1e-7`，这是 roadmap 的测试建议，
将在 T03 锁定为测试配置；近奇异例必须报告谱与阈值敏感性，不能靠放宽容差通过。

**T03 NUMPY GOLDEN TEST SETTINGS（已锁定，仅限 L0 数值测试）。** `comparison_rtol=1e-7`、
`comparison_atol=1e-10`；scaled-`F` 的 `rank_abs_tol=1e-12`、`rank_rel_tol=1e-10`；symmetry 与 retained
projection orthogonality 均用 absolute `1e-12`、relative `1e-10`；`R` PSD、`N` PD 的 absolute
为 `1e-12 m^-2`、relative 为 `1e-10`，`R` PD 另取下述 projector 舍入信息地板；`eta` 区间舍入容差
为 absolute `1e-12`。fixture 浮点叶字段按上述 comparison `atol/rtol` 比较，case ID、状态、维度、类型、
键、列表长度与顺序严格比较；同环境 byte-identical 重生成只作为复现检查，不是跨 BLAS 正确性要求。

`N-R` PSD 不使用固定绝对地板，也不以可能因消减而接近零的 `N-R` 自身谱作尺度。对 binary64
`u=2^-53`，令 `gamma_k=ku/(1-ku)`，小矩阵行数为 `m`、retained nuisance rank 为 `r`、amplitude
列数为 `q`，T03 使用 safety factor 8 和
`delta_NR=8[(gamma_m+gamma_q)(||N||_2+||R||_2)+2 gamma_(m+2r+q)||N||_2]`；
只允许 `lambda_min(N-R)>=-delta_NR`。投影残差可分辨界为
`delta_E=8 gamma_(m+2r+q) sqrt(||N||_2)`，`R` 的 PD 阈值再取
`max(pd_abs,pd_rel||R||_2,delta_E^2)`；因此落在投影舍入界内的完全混淆方向不能产生有限 `s`。
这些界仅适用于当前 binary64、小稠密 SVD golden reference 且要求 `ku<1`；任一关键中间量非有限、
误差模型不适用或范数无法可靠表示时为 `NUMERICAL_FAILURE`。极端 nuisance 列采用 max-abs 后再除
scaled 2-norm 的两阶段实现，数学上仍是 `d_j=1/||F_:j||_2`，精确零列只按原始元素判定。以上值随
T03 fixture 保存并经 sensitivity/跨 BLAS 测试，不是 `tau_eta/tau_s/tau_gamma` 或任何实验 gate。
T05 production prototype 已对照同一 fixtures，但不把 Eigen QR pivot threshold 称作冻结 SVD rank 的
等价实现，也不直接用 QR rank 决定可报告分数。精确零列与 QR 可确认的精确重复/结构依赖只从 active
QR view 移除并保留完整列映射；数值谱与冻结阈值有充分间隔且 condition/orthogonality/roundoff 审计
通过时才允许 sparse `E/R` 进入评分。QR pivot 可能与冻结 SVD 不同或临界区间无法排除时返回
`SPARSE_RANK_UNCERTAIN`，不得输出有限 `s`。指挥反例
`F=[[1,1],[0,1.5e-10]],G=[[0],[1]]` 的 frozen reference 为 rank 1、`R≈1`，naive QR 为
rank 2、`R=0`，production prototype 按上述规则显式不确定。

T05 review 后不再使用经验 pivot ambiguity band。对去除精确零列及精确同号/反号重复列后的 active
`R11`，通过逐单位向量稀疏 triangular solve 累计 `||X||_F` 和 `||I-R11 X||_F`；当后者小于 1 时，
`||R11^{-1}||_2 <= ||X||_F/(1-||I-R11 X||_F)` 给出可审计上界。完整 scaled-`F` 每个非零列单位范数，
故 `sigma_max<=sqrt(n_nonzero)`；重复列的完整数量进入该上界和冻结相对阈值，而不是只看 active view。
只有扣除已记录 QR backward-error bound 后的 `sigma_min` 下界仍严格高于完整矩阵冻结阈值上界，才确认
frozen rank 等于 active 列数并继续评分；否则为 `SPARSE_RANK_UNCERTAIN`。这会保守拒绝无法证明的
低于阈值、临界、上三角 pivot 和重复列数量敏感案例，不用 QR rank 替代冻结 rank，也未修改合同定义。

T05 nuisance QR 的条件数候选定义为 retained triangular `R11` 的
`kappa_1(R11)=||R11||_1||R11^{-1}||_1`，inverse norm 由 Hager–Higham 型 1-norm estimator 通过
稀疏 `R11/R11^T` triangular solves 估计，不形成 inverse。assembly、equilibration、Householder QR、
列置换、triangular solve、`FY`、Gram 与小矩阵 eigensolver 的 roundoff 模型，以及
`8*kappa_1_est*gamma_(m+2r+q)` 的 resolution 判据，当前仍为
`PENDING_NUMERICAL_PROPOSAL`，不是正式实验锁定参数。任一量非有限、`ku>=1`、condition estimator 未
收敛、rank 临界、orthogonality 未通过、误差地板不小于 `||G||` 或审计失去分辨力时返回
`NUMERICAL_RESOLUTION_LOST`/不可用不确定度；不得加入 jitter、LM damping 或放宽阈值。零秩 `F`
直接取 `E=G,R=N,eta=1`（若 `N` 有效），不运行 triangular solve。若要让不支持域产生有效分数，
必须先登记并裁定修改合同秩定义的 amendment；本轮没有此 amendment。

T05 review 数值审计移除了 `N/R` 谱尺度中的 `max(1,...)`。`N` rank/PD 共用
`max(pd_abs,pd_rel||N||_2)`，`R` rank/最终有限 `s` 判定共用
`max(pd_abs,pd_rel||R||_2,delta_E^2)`；`N-R` 只使用随 `N/R` 谱和 binary64 运算次数缩放的误差界，
无固定绝对地板。正交审计若报告归一化残差，则容差同样归一化为
`orth_abs/(||F||||G||)+orth_rel+roundoff`。上述 sparse 误差模型仍为
`PENDING_NUMERICAL_PROPOSAL`，review 修复通过不等于正式 gate 参数已锁定。

评分前必须逐项核对 factor index、实际 key 集、允许的 factor type、graph/Values key 集、plan 中唯一
obs ownership、candidate 完整覆盖、support/refit segment identity 和每组实际 factor mask。未知类型不进入
`G0`，任何不一致均在构造有效分数前失败。`linearization_id` 是规范序列化后的 SHA-256，覆盖实际 factor
mask、ordering/dimension、factor-row/key-column 映射及白化 `F/G/RHS`；其他组被排除的 factor 变化不得
改变当前组 ID。short/boundary 或数值失败时主 `eta/s` 字段为 unavailable；数学中间诊断只进入明确的
`debug_*` 字段/审计文件，rank-deficient 的 `s=+inf` 则保留为独立可用语义。

### 5.4 三个分数与信息边界

```text
eta_A = lambda_min(R_c,N generalized eigenproblem)
s_A   = 1/sqrt(lambda_min(R_c)) m, if R_c PD; otherwise +inf
gamma_s = (1/n_s) sum_{i in S_s} (r_i(X,beta,c_s)/sigma_i)^2.
```

`eta` 是相对 separability，`s` 是绝对最弱方向尺度，`gamma` 是经验 post-fit 拟合诊断；三者都不是
全局安全证书。decision-time 使用 Stage 2 的 linearization 和 residual；final-time 使用最终联合解，
必须分文件/列并带不同 `linearization_id`。

报告信息只来自冻结 factor 集合的物理 Gauss–Newton quadratic。以下内容一律排除：LM damping、Stage 1
已去除的 L1/TV、用于数值稳定的人造 prior/jitter、未实际存在或来源不明的校准 prior。非凸 robust
factor 只允许使用已冻结并记录的非负 Gauss–Newton weight；`gamma` 仍使用原始名义 `sigma_i`。

## 6. 冻结决策、最终图、审计和 fallback

eligible group 的主政策是

```text
Use(A) iff eta_A >= tau_eta
           and s_A <= tau_s
           and max_{s in A} gamma_s <= tau_gamma.
```

rank failure、数值失败、短段、边界/KKT 失败和支持不足均明确 Suppress。gate 参数由 validation 锁定，
test 期间不可修改。`tau_s` 是政策采用的局部不确定度限值，不是 correction error 上界保证；实验中的
`epsilon_bad_m` 是另一个预声明标签阈值。

**FROZEN。** support、segments、groups 和 Use/Suppress 决定在 final refit 前冻结。Use group 的原始
range factor 以 live `c_s` 状态进入最终受约束联合图；不得另加 corrected pseudo-range。Suppress 的
candidate range 不进入；noncandidate 按共同 reference mask 进入。用 `obs_id` 断言每条 accepted raw
range 恰好一次、每条 suppressed candidate 零次。

最终联合优化返回单一 `InferenceResult(final_graph, final_values, masks, status, score_sets, timing)`。
轨迹、导航/IMU/static/segment bias、逐 factor post-fit residual 和 covariance 只能从该对象的同一个
graph/Values 读取。全图 covariance 与 reference-group `R_c` 分开命名，计算不了就输出状态而非占位值。

在 final linearization 对同一 reference-plus-group 定义重新审计，并保留 decision/final 两套分数。
若 final 解 rank-invalid、违反冻结 acceptance check、受约束求解失败或 graph/Values 不一致，只允许一次
`ALL_CANDIDATES_SUPPRESSED` 重跑。fallback 成功返回 `FALLBACK_OK` 且保留 recovery attempt failure；
fallback 再失败返回 `ESTIMATION_FAILED`，不得输出未优化轨迹为有效结果，也没有迭代 re-admission。

**T08 实现状态（不改变上述冻结定义）。** `FinalInferenceEngine` 接受来源中立的
`SupportPartition`/decision score，返回唯一 `InferenceResult`；final exporter 只接受该对象。最终图重建接口
同时接收 frozen full candidate set 与 accepted ordinal set，因此 suppressed candidate 不会被误归入
noncandidate。decision/final score 使用独立 linearization ID；fallback 从 frozen raw/reference graph
重新构造且计数硬限制为一次。实际全图 marginal covariance 单列为
`FULL_FINAL_GRAPH_MARGINAL_COVARIANCE`，失败时只有 `UNAVAILABLE` 原因。定向 12/12、完整 CTest 17/17 与
development runner 已本地通过，证据见
[`evidence/t08_20260907T083437Z/`](evidence/t08_20260907T083437Z/VERIFICATION.md)。
当前配置只允许显式 `T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION` 工程值，不能冒充 T10 locked gate。

T08 review-fix 进一步把 Stage 2、recovery attempt、fallback 与 selected final result 的 solver/iteration/stop/
score 证据分离；fallback-final 的 score 不适用行不再覆盖失败 recovery 的 group/segment 分数、原因与
linearization ID。runner 总耗时统一为 main entry 到 final status preparation 的墙钟，正常、失败和 fallback
共用同一语义常量，Stage 4 engine 耗时另列。final identity schema v2 规范绑定实际 Values binary64 内容、ordered factor type/keys/error/最终
白化线性化 Jacobian，以及 input/config/plan/support 等必要身份；它不同于 run ID 和 scoring
linearization ID，exporter 在落盘前重新核验内容绑定。实际 final re-score acceptance 失败的 success/failure
fallback 两路、双 group Use/Suppress 和等结构/等总 error identity collision 反例均本地通过。修复后定向
18/18、完整 CTest 17/17，证据见
[`evidence/t08_review_fix_20260907T092037Z/`](evidence/t08_review_fix_20260907T092037Z/VERIFICATION.md)。最终独立复审
重跑 focused GTest 18/18、runner contract 与非法 manifest 反例，并确认 R01/R03 源码未变；R02 为
`REVIEW_ACCEPTED_EXCEPTION_TIMING_SCOPE`，T08 为 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`。
本轮未独立重跑 package build、完整 CTest、T07 step/ramp、正式 RQ、held-out validation 或性能/RSS；见
[`evidence/t08_final_review_20260907T121510Z/`](evidence/t08_final_review_20260907T121510Z/VERIFICATION.md)。
以上只修正实现与可追溯性，不改变冻结数学定义。

## 7. Prefix 与 oracle 边界

正式自动端到端 estimator 始终不可读取 GT、synthetic bias truth、obstruction label、oracle support，
也不可读取下列诊断的 manifest/cache。三类诊断用途和标签互不替代：

1. `T04_ORACLE_SUPPORT_DEBUG_ONLY`：只用于 T04 在已知候选 support 下验证 segment factor、非负
   constrained refit 和残差语义。oracle-support 输入只可给 candidate time interval/`obs_id` partition，
   不给真实 bias amplitude、GT pose 或 GT-derived initialization。
2. `RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY`：只用于 constant/ramp 固定分段的 model-mismatch 和
   matched-cache gate 诊断；support 来源及禁止输入与上一项相同，但其问题、cache namespace 和结果表
   独立，不能作为 automatic discovery 证据。
3. `RQ4_FUTURE_COMMON_LINEARIZATION_DIAGNOSTIC`：固定同一 historical support/amplitude estimate 与
   common linearization，仅改变 future factors 来诊断信息变化；它不提供 GT amplitude/initialization，
   也不与 T04/RQ3 共用标签、manifest、cache 或证据表。

prefix 先按 cutoff 同时裁剪 UWB/IMU，再执行初始化、input plan、discovery、refit、score、decision 和
final。独立 calibration 与 locked gate 可复用；full-run states、support、noise fit、auto-anchor fit、
未来插值样本及其他未来信息不可复用。固定-support/common-linearization 未来信息实验必须标
`diagnostic_fixed_model`，不能冒充端到端结果。

## 8. 参数登记与锁定

`PENDING_VALIDATION` 不是默认值。对应值写入带 hash 的 locked config 前，正式 test 准入失败。

| 参数 | 单位 | 用途 | 来源 | 锁定阶段 | 验证方法/当前值 |
|---|---:|---|---|---|---|
| `beta_link` | m | 固定静态 LOS bias | 独立 LOS calibration | calibration 后、所有 dev/val/test 前 | `PENDING_EXTERNAL`；估计不确定度并查每 link 覆盖 |
| `sigma_i` / sensor noise rule | m | range 白化与 gamma | 独立 LOS/noise calibration；策略前快照 | calibration 后 | `PENDING_VALIDATION`；残差尺度、时间间隔规则与策略 hash 一致性 |
| `lambda_1` | m^-1 | discovery 稀疏幅值 penalty | development/validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.05`，仅用于实现/测试，不是 formal lock |
| `lambda_TV` | m^-1 | discovery 相邻差 penalty | development/validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.10`，constant/ramp fixture 不用于按标签调参 |
| `T_gap` | s | 同 link 差分链断开 | 数据采样协议 + validation | discovery 实验前 | `PENDING_VALIDATION`；T06 development smoke `1.0 s`，严格 `>` 断链 fixture 本地通过 |
| `b_min` | m | active support threshold | validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.02 m`，无候选路径以独立高阈值仅作合同测试 |
| `delta_change` | m | change-point threshold | validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.05 m`，等号切段 fixture 本地通过 |
| `delta_merge` | m | A01 单遍相邻段合并 | validation；A01 规则已技术接受 | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.05 m`，三段链/等号/归属 fixture 本地通过 |
| `n_min` | samples | eligible 最小样本数 | validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `2`，单点明确 short |
| `T_min` | s | eligible 最小持续时间 | validation | test 前 | `PENDING_VALIDATION`；T06 development smoke `0.01 s`，短段 fixture 本地通过 |
| `rho_scale` | 1 | chain ADMM 数值 penalty 相对 median weight | solver fixture，不改变原始目标 | T06 数值复审后 | development `1.0`；`0.1/1/10` 解一致性本地通过，仍 `LOCAL_NUMERICAL_PROPOSAL` |
| `admm_primal_abs/rel_tol` | m / 1 | `b=u,Db=v` primal residual | solver fixture | T06 数值复审后 | development `1e-8 m / 1e-6`；不与 dual 绝对阈值共用 |
| `admm_dual_abs/rel_tol` | objective/m / 1 | scaled-dual residual | solver fixture | T06 数值复审后 | development `1e-8 objective/m / 1e-6` |
| `admm_kkt/tv_subgradient_tol` | objective/m | 原始非负 KKT 与最终 `Du` 上 TV 对偶审计 | analytic/independent reference fixture | T06 数值复审后 | development 各 `1e-8 objective/m`；`LOCAL_NUMERICAL_PROPOSAL` |
| `admm_max_iterations` | iterations | chain 硬失败上限 | solver nonconvergence fixture | T06 数值复审后 | development `10000`；上限耗尽显式失败 |
| `discovery_rel_obj_tol` | 1 | 外层原始目标变化停止 | solver/runner trace | T06 数值复审后 | development `1e-8`；目标上升另用 binary64 派生舍入界 |
| `discovery_scaled_step_tolerance` | 1 | Stage 1 导航与逐观测 bias 组合尺度步长停止 | solver/runner trace | T06 数值复审后 | development `1e-6`；GTSAM local coordinates 与 bias 尺度项取最大，`LOCAL_NUMERICAL_PROPOSAL` |
| `discovery_observation_bias_scale_m` | m | Stage 1 逐观测 bias 步长归一尺度 | solver fixture/config | T06 数值复审后 | development `1 m`；必须有限且正，`LOCAL_NUMERICAL_PROPOSAL` |
| `discovery_navigation_stationarity_tol` | objective / normalized coordinate | 最终可行 `u` 下 navigation 驻点 | 共享 T04 审计工具/runner trace | T06 数值复审后 | development `1e-6`、roundoff safety `8`；`LOCAL_NUMERICAL_PROPOSAL` |
| `discovery_max_outer_iterations` | iterations | Stage 1 硬失败上限 | actual short development run | T06 数值复审后 | 默认 development `50`；20 轮失败和 34 轮成功 trace 均保留。A03 隔离诊断曾仅在两份配置取 `500`，step/ramp 在 outer 121/127 的原 inner cap 先失败，未形成默认或正式参数迁移 |
| `epsilon_boundary` | m | 判定 active constraint 边界 | solver 数值精度 + T04 | T04 实现时 | `1e-9 m`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；解析边界/KKT GTest |
| `refit_rel_obj_tol` | 1 | 交替 refit 相对目标停止 | solver tests | T04 实现时 | `1e-8`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；联合目标 trace |
| `refit_scaled_step_tol` | 1 | 状态更新停止 | solver tests | T04 实现时 | `1e-6`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；GTSAM local coordinates 按 rad/m、m/s、IMU bias、m 单位尺度归一 |
| `refit_projected_grad_tol` | objective/m | 非负 KKT 停止 | solver tests | T04 实现时 | `1e-8 objective/m`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；内点 `|g_s|`、边界 `max(0,-g_s)` |
| `refit_navigation_stationarity_tol` | objective / normalized coordinate | 最终联合自由导航状态驻点停止 | solver tests + runner trace | T04 review 修复时 | `1e-6`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；同一最终 graph/Values 的 `g=J^T r`，逐坐标以 `1 rad` pose rotation、`1 m` pose translation、`1 m/s` velocity、`1 m/s^2` accel bias、`1 rad/s` gyro bias 物理尺度相乘；未知自由 key 类型显式失败 |
| `refit_gradient_roundoff_safety_factor` | 1 | 最终联合梯度求和舍入余量 | binary64 error model + trace | T04 review 修复时 | `8`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；每坐标余量为 `8*(gamma_n+u)*sum_f |g_fj|*scale_j`，`u=eps/2`、`gamma_n=n*u/(1-n*u)`，只豁免可解释的求和舍入量 |
| `max_refit_iterations` | iterations | 硬停止 | T04 runtime/收敛测试 | T04 实现时 | 默认 `20`，`T04_DEBUG_NUMERICAL_LOCKED_REVIEW_ACCEPTED`；成功 run 与显式一轮超限失败 run；`-0.4` fixture 的 20 轮非收敛原样保留 |
| `rank_abs_tol` | 1（scaled F） | QR/SVD 绝对秩阈值 | T03/T05 fixtures | T05 前 | T03 NumPy golden `1e-12`；T05 完整 scaled-F 证书在已测试支持域 `REVIEW_ACCEPTED`，但数值模型仍为 `PENDING_NUMERICAL_PROPOSAL`，QR rank 不替代 frozen rank |
| `rank_rel_tol` | 1 | 相对秩阈值 | T03/T05 fixtures | T05 前 | T03 NumPy golden `1e-10`；T05 完整列数量/谱界在已测试支持域 `REVIEW_ACCEPTED`，但仍为 `PENDING_NUMERICAL_PROPOSAL`，临界区显式失败 |
| `psd/pd/numerical tolerances` | m^-2 或相对量 | `R,N` 数值审计 | T03/T05 | T05 前 | T03 NumPy golden：`R` PSD、`N/R` PD abs `1e-12 m^-2`/rel `1e-10`；`N-R` 使用 binary64 `gamma_k` 的 `N/R` 谱尺度界且无固定绝对地板；`R` PD 加 projector error squared floor；roundoff safety factor 8；symmetry/projection abs `1e-12`/rel `1e-10`，eta abs `1e-12`，comparison abs `1e-10`/rel `1e-7`；T05 已移除 unit floor、统一 rank/PD 状态阈值并修正归一化正交容差，在已测试支持域 `REVIEW_ACCEPTED`，sparse condition/roundoff 仍为 `PENDING_NUMERICAL_PROPOSAL` |
| `tau_eta` | 1 | relative separability gate | validation risk–coverage | test 前 | `PENDING_VALIDATION`；T08 工程 fixture 显式 `0` 仅验管线，不是 T10 lock；与相同缓存 gate 比较 |
| `tau_s` | m | absolute uncertainty gate | validation + application tolerance | test 前 | `PENDING_VALIDATION`；T08 工程 fixture 显式 `1000000 m` 仅验管线，不是 risk–coverage 结果 |
| `tau_gamma` | 1 | empirical fit gate | validation，按段长检查 | test 前 | `PENDING_VALIDATION`；T08 工程 fixture 显式 `1000000` 仅验管线，不是 constant/ramp validation |
| `epsilon_bad_m` | m | accepted correction 的 bad 标签 | application criterion，预声明 | 看 test label 前 | `PENDING_OWNER/VALIDATION`；与 `tau_s` 分离 |
| prefix horizons `H` | s / `full` | RQ4 future context | 记录时长/采样率 | prefix manifest 前 | roadmap 候选 `{0,2,5,full}`；最终为 `PENDING_VALIDATION` |

## 9. U01–U14 验收映射

T03 NumPy、T04 oracle-debug factor/refit 与 T05 已测试 sparse 支持域均由独立复核接受。T06 Stage 1、
A01 和 automatic discovery/refit/score 闭环的 R01–R05 已在 reviewed fixed-calibration development
engineering scope 内独立复审接受，T06 为 `DONE`；
T08 已按 `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` 收口；T09 及后续仍 `NOT_RUN`。

最终复核见 [`evidence/t06_final_review_20260907T043727Z/`](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md)：
独立重跑 14/14 discovery、23/23 refit/scoring、6/6 recoverability、8/8 config、5 cases x 3 rho
reference 和 14/14 preflight probes。真实短输入仍为 17 segments/13 short/0 boundary、3 groups 全部
score unavailable；有效 score 只由 synthetic engineering GTSAM fixture 验证。旧默认空 fixture 的 20 轮
非收敛、Stage 1/Stage 2 失败和所有 unavailable 结果均保留。full package/build、T03 双 BLAS、
T04/T05/T02/legacy 只复核开发者证据，未由该独立复审重跑。

| ID | 合同规则 | 后续任务 | 验收证据 | 当前状态 |
|---|---|---|---|---|
| U01 | 已知导航时 `R=N, eta=1` | T03 | NumPy fixture + log | `T03_NUMPY_PASS_REVIEW_ACCEPTED` |
| U02 | `z=x+c` 即使 nuisance block 可逆也可 `R=0,s=inf` | T03/T05 | 解析 fixture、C++ dump 对照 | T03 NumPy 与 T05 C++ sparse `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| U03 | 与 bias 无关 nuisance 零列不触发虚假判定 | T03/T05 | zero-column fixture、rank status | T03 NumPy 与 T05 C++ sparse `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| U04 | 弱信息可同时高 `eta`、大 `s` | T03 | `N=R=1e-4 m^-2` fixture | `T03_NUMPY_PASS_REVIEW_ACCEPTED` |
| U05 | 多幅值近退化暴露最弱方向与阈值敏感性 | T03/T05 | spectrum/tolerance sweep | T03 NumPy 与 T05 C++ sparse `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN`，临界/不可证明域显式失败 |
| U06 | fixed/online `beta` 恰好一次，online 进入 nuisance 和真实 prior | T02/T05 | factor residual/key/column tests | T05 test-only online `Z(m)` + real prior 与去 prior混淆测试 `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN`；正式 fixed value 仍缺失 |
| U07 | 良态小图 projector/Schur/C++ sparse 在锁定容差内一致 | T05 | F/G/N/R dumps + comparison log | T03 Python projector/Schur 与 T05 C++ sparse 支持域对照 `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| U08 | Jacobian 符合 GTSAM 局部参数化 | T04 | manifold finite difference GTest | T04 actual GTSAM retract/local-coordinate whitened finite difference 与 R01 最终联合 `X/V/B` 驻点 `PASS_REVIEW_ACCEPTED` |
| U09 | 同 graph/Values 改 LM damping 不改报告信息 | T05 | paired score test | 两组实际 GTSAM LM trial step 不同、同物理 graph/Values 重评分完全相同，`PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN` |
| U10 | reference 排除全部 candidate；无跨组担保；factor 唯一 | T05/T08 | mask/factor-meta assertions | T05 组间 candidate 排除、当前组不变/另一组变化、factor 唯一与 linearization ID 隔离 `PASS_REVIEW_ACCEPTED_VALIDATED_DOMAIN`；T08 双 group 完整 engine 的 accepted/suppressed/noncandidate 唯一性、live `C` 与无 re-admission `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| U11 | gap/短段/边界不偷连，单包不自动接受；验证 A01 immutable snapshot、加权代表幅值、左到右单遍重算、等号合并、父子 ID/obs 唯一归属与三段链唯一结果 | T06 | segmentation fixtures/reason codes/partition hash；A01 三段链、溢出与非均匀权重 fixture | `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`；Stage 2 partition 冻结，科学参数仍待 validation |
| U12 | accepted `c` 留图，一次 fallback 全链可追踪 | T08 | graph key/factor/fallback tests | accepted `C`/raw factor、冻结 mask、实际 final re-score 失败、单次 fallback success/failure、recovery/fallback/final 分相 trace、同一内容绑定 `InferenceResult` 导出均 `REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE` |
| U13 | cutoff 后数据改变不影响 prefix | T11 | metamorphic hash/numeric test | `NOT_RUN` |
| U14 | 无候选/全拒绝的 coverage/risk 分母正确 | T09/T10 | evaluator fixtures | `NOT_RUN` |

## 10. 已知未决项与阻塞范围

| 未决项 | 当前事实 | 阻塞范围 |
|---|---|---|
| 独立 LOS 固定 `beta` | 未找到专用 manifest 或可信数值；不得填 0 | T02 固定值正式配置；全部依赖 fixed-beta 的正式 RQ2/RQ3/RQ4 与主 claim |
| 数据许可/发表权限 | 自有与外部数据均未知 | 公开 artifact、论文可发表数据表/图、C1/C3 release 表述 |
| GT 点、杆臂、时钟及标定 provenance | 自有数据与部分外部数据未闭合 | raw-frame ATE、真实 measured bias reference、跨数据公平比较 |
| 当前机器缺自有多链路 NLOS | 只有一条自有 no-obstacle recording；两种运动和重复证据未建立 | controlled UGV 主 RQ2、C2/C3 的真实多链路与重复性证据 |
| IMU preintegration API/test 冲突 | T00 为 38 tests、2 failures；构造参数未采用 `gravity_world` | 进入依赖该语义的 T02/T04 前 correctness；受影响的新实验均不得正式化 |
| linked-devel 同名仿真库碰撞 | 三个库同名且 hash 不同；本次 baseline 未失败 | 后续环境可复现性、仿真/T07 与批量 runner 的二进制 provenance |

这些是证据或准入缺口，不是 scope amendment。partition merge 的执行澄清 A01 已由本轮用户指挥/
审查会话技术接受；它保留原冻结边界，T06-R01–R05 修复和 U11 fixture 已在 development engineering
scope 内独立复审接受。

## 11. 合同审查点

交替非负 refit 已在 T04 独立复核中接受。T05 的确定性列 equilibration、guarded sparse QR、完整输入
校验、规范 linearization ID 与不可用输出语义在已测试支持域独立复审接受；sparse rank/condition/roundoff
仍是 `PENDING_NUMERICAL_PROPOSAL`，不能用于正式 gate。一次 merge 的确定性执行规则 A01 已技术接受并
实现通过 U11；溢出稳定性、SHA-256/context 与组合尺度步长修复已在 T06 development engineering
scope 内独立复审接受。这些选择没有改变冻结四阶段、评分、gate 或证据边界。其他审查意见若要求改变
模型/指标，继续先在 `STATUS.md` 登记新的 `PROPOSED` amendment，保留原定义，获确认后再实施。

## T10-A14 显式 IMU 条件协方差 amendment（opt-in development）

本指挥会话技术接受[A13唯一草案](T10_A13_AMENDMENT_DRAFT.md)，实施边界和测试判据见[A14协议](T10_A14_AMENDMENT_PROTOCOL.md)。
新增PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1：likelihood条件于未知live Bi/Bj；biasHat仅预积分一阶修正的线性化点，非已知真实bias。
无额外独立积分bias随机源，显式biasAccOmegaInt=0。LEGACY_GTSAM_COMBINED_DEFAULT_V1仍为缺省I6；配置仅枚举opt-in，无任意K调参。
A10 P1数值：Qa=4e-6 I3，Qg=4e-8 I3，Qba=1e-4 I3，Qbg=4e-10 I3，Qi=1e-9 I3。
Qa单位(m/s²)²·s、Qg单位(rad/s)²·s，为测量白噪声密度平方，采样协方差Q/dt；sigma单位分别(m/s²)/sqrt(Hz)、(rad/s)/sqrt(Hz)。
Qba单位(m/s²)²/s、Qbg单位(rad/s)²/s，bias random-walk增量协方差Q*dt；sigma单位分别(m/s²)/sqrt(s)、(rad/s)/sqrt(s)。
Qi单位m²/s，native位置协方差注入Qi*dt，是保留的engineering积分近似，非实测独立标定。
旧K对角块在实际代码中分别与Qa/Qg同样按1/dt注入；本模型不引入该辅助源。原B0 prior方差1e-4（acc/gyro各自状态单位平方），是一次状态约束，不作K替代；原启发式pose/velocity/bias priors不变。
测量/IMU bias常值生成与估计器非零RW的差异为已声明synthetic建模假设；已知anchor/lever/beta/noise并非真实独立标定。
完整15维theta,p,v,ba,bg协方差及交叉项、native tangent/Rot3/retract、重力世界系和原采样时刻规则保留；不修改GTSAM或其他传播近似。
白化权重和目标数值语义改变，不以跨模型objective高低宣称改进；damping不进入信息量。
模型版本、实际六矩阵、重力、native预积分convention进入实际common/discovery/support/Stage2 producer-cache-request/final身份兼容检查。
原raw身份不变；模型相关PIM、graph、linearization、估计/评分/协方差缓存跨模型全部失效，旧checkpoint不能新模型warm start。
旧artifact保留为历史模型证据；旧未绑定此版本的cache也不作为新运行的兼容输入。仅显式opt-in development，非默认迁移/validation准入。

A14本地实施/工程回归及唯一development运行已完成，见[evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md](evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md)。工程门通过不等于科学通过：首block lambda耗尽且不驻点，Stage2/score NOT_RUN；默认/正式准入不迁移。

## T10-A17 certified pair reduction 限定原型 amendment

本轮指挥授权[A17协议](T10_A17_AMENDMENT_PROTOCOL.md)：仅默认关闭、独立paper development首conditional block入口实施PAPER_CERTIFIED_PAIR_REDUCTION_V1；固定333bit P/D区间、保守fidelity比较/转换、明确失败和原generic AND stationarity。原数学模型/priors/方向/初始化/容差/lambda预算不改；这是显式接受/分辨率数值语义变化。
本轮不铺开A16草案中的Stage2/cache/final生产集成：原型schema/策略身份独立、consumable=false且不输出正式缓存/结果，旧reader必须拒绝。静态checkpoint仅工程回归，pilot从A14原raw重新初始化，唯一step seed10101、首block50calls/120s，门失败pilot NOT_RUN，无retry/精度搜索。正式validation/test/T11/scheduler/gate与claim升级均未授权。

A17本地限定实施与唯一pilot已完成，见[A17证据](evidence/t10_a17_certified_prototype_20260910T002211Z/VERIFICATION.md)：工程门通过，fresh step首block20calls/43trials后满足原generic AND stationarity，外部89.244s；chain/Stage1后续/Stage2均NOT_RUN。仅首block原型能力与本输入观察，不升级正式准入/claim，不自动迁移默认或扩展Stage2/cache/final集成。

## T10-A18 certified policy进程内实现与Stage1限定amendment

本指挥授权[A18实施前协议](T10_A18_AMENDMENT_PROTOCOL.md)。保留A17 PAPER_CERTIFIED_PAIR_REDUCTION_V1全部333bit证书/接受/失败语义，C++进程内实现版本与实际MPFR/GMP重新绑定身份。默认关闭的显式development入口接入现有AutomaticSupportProvider完整Stage1，条件range常数取实际beta+当前u，chain/partition/四项AND不变。输出独立schema、consumable=false，不进入Stage2/cache/final。43固定pair一致性、C++异常/legacy/非零bias与完整批次<=10s工程门全部通过后，自动唯一fresh step seed10101 Stage1 pilot，outer500/conditional50/整进程树900s，首次失败或Stage1完成即停止。无retry/参数或精度搜索，非正式validation/test，不升级claim。

A18限定实现及唯一development运行本地完成：[证据](evidence/t10_a18_certified_stage1_20260910T010000Z/VERIFICATION.md)。工程门通过，fresh step完整Stage1于90outer达到原四项AND；仅Stage1，Stage2/正式准入/claim不迁移，等待review。
# T10-A19 development-only certified Stage1→Stage2 amendment

`PAPER_CERTIFIED_PAIR_REDUCTION_V1` 可在显式 opt-in 的 paper development 路径中用于 Stage1 与 Stage2 的 conditional navigation。Stage2 证书必须使用实际冻结 partition 和当前迭代的 `beta+c_s` range 常数；GTSAM 方向、native retract、无正则 refit、`c_s>=0` 更新、四项 AND、最终 joint graph 中的 live `c_s` 及 reference-plus-group score 定义均保持原合同。该接口默认关闭，诊断输出不可由正式 cache/final 路径消费。详见 `T10_A19_AMENDMENT_PROTOCOL.md`。

### T10-A19-R01 接线修复记录

R01 按 [`T10_A19_R01_PROTOCOL.md`](T10_A19_R01_PROTOCOL.md) 将同一 Stage2 conditional graph 内全部
reference/candidate range 的实际 raw measurement、sigma、anchor/lever、fixed beta 或 fixed beta+live
`c_s`、pose key、obs/factor identity 与残差传给既有证书 consumer；未改变本节方法定义、数值判据或
最终 joint graph。真实 mixed 工程门通过，但唯一 fresh 运行在 Stage1 前因 launcher 路径失败，故没有新的
方法级 development 结果，正式合同与 C1--C3 均不升级。

## T10-A19-R03 Stage2 受限 inexact handoff amendment

本轮按 [`T10_A19_R03_PROTOCOL.md`](T10_A19_R03_PROTOCOL.md) 技术接受并登记默认关闭的
`PAPER_STAGE2_INEXACT_HANDOFF_V1`。它只允许 Stage2 非空 live-`c_s` 交替 refit 的 conditional block 在
原 lambda upper exhaustion 时，且已有 certificate-accepted update、最后接受更新满足原 generic convergence
及 `1e-6` scaled step、最后搜索全部拒绝试步都有有效 `P.lo>0,D.hi<=0` 证书、拒绝步不超过 `1e-6` 且
`P.hi` 不超过原 binary64 objective allowance、身份/状态/梯度审计均有效时，返回最后已接受 Values 的
独立 `INNER_NUMERICAL_STALL_INEXACT` 交接状态。inner 明确保持未收敛，被拒绝端点永不进入状态。

调用方仅可在该精确状态下继续原闭式非负 `c_s` 更新。最终 joint graph/Values 的相对目标、scaled step、
KKT、navigation stationarity 四项 AND 与所有有限性、非增、可行性和 key 检查保持不变。Stage1、legacy、
空 `c` 路径、方向/retract、333-bit 证书、lambda、模型、初始化、partition、score 与 final 原始 range + live
`c_s` 定义均不改变。组合 solver identity 必须同时绑定原 A18 certificate policy 与本 amendment。

## T10-A19-R04 crash-root-cause boundary

R04 does not amend estimator mathematics. It authorizes only a stack-supported
software/build root-cause repair, a prepare-only stop before Stage1, and
process orchestration needed to enforce the registered automatic and per-final
budgets. `PAPER_STAGE2_INEXACT_HANDOFF_V1`, all convergence thresholds, lambda
behavior, 333-bit certificates, raw initialization, partition, score, final
live-`c_s` graph, and fallback semantics remain exactly as registered by R03.
Any compatibly rebuilt binary receives a new implementation identity; old R03
outputs remain nonconsumable.

## T10-A19-R07/R08 no-C policy and validation-role cross-reference

R07 extends the already certified 333-bit development numerical policy to an
accepted-empty candidate-excluded recovery and its single fallback. The no-C
path retains all noncandidate raw ranges, contains no `C` key or corrected
pseudo-range, forbids inexact handoff, and uses the same objective/step/KKT/
navigation-stationarity AND. See [`T10_A19_R07_PROTOCOL.md`](T10_A19_R07_PROTOCOL.md).

R08 does not change that mathematics. It adds an opt-in, truth-free validation
role envelope bound to the old split reservations and carries its content hash
through Stage1, Stage2 and final graph/Values identity. Legacy development
identity remains unchanged. The fixed P1 and gate working point are evaluated,
not selected or locked, under [`T10_A19_R08_PROTOCOL.md`](T10_A19_R08_PROTOCOL.md).

## T10 单工作包收口授权 amendment

用户本轮明确授权以 `T10_CLOSEOUT_MANIFEST.json` 的有限诊断及 C2 裁决替代旧 T10 扩展计划。
模型、评分、P1、solver 与阈值不改；仅普通 correctness 修复、两条既有 validation step2 的 A/B/C、LOS 验收及失败分类。
不执行正式 held-out RQ3，不据此声称完成原完整实验合同。普通修复在同包内完成并保留旧失败/受影响重跑身份。
C2-C 优先：恢复收益缺少跨两基础轨迹重复支持则降为探索性；否则按冻结证据决定 A 或 B。
T10 可冻结负结论，工程失败不能标验收通过，T11/T12 仍 NOT_RUN。

本收缩工作包已冻结为 **C2-C**，见 `T10_CLOSEOUT.md`。η与现有gate代码保留作诊断/探索性比较；
稳定恢复收益和η增量操作收益未获支持，不授予正式held-out gate准入。T11/T12尚未运行。
用户另授权删除可再生成的中间转储/重复二进制；原完整归档的当前保留范围以retention账本为准。

## 0911-STEP1 用户授权 amendment（2026-09-11）

本轮用户实施计划授权第一步 LCB 固定部分补偿及同集合全额消融，替代旧下一任务边界。
仅 `lcb_partial` / `lcb_fixed_full` 允许按段冻结动态 offset；旧方法 accepted live C 规则保留。
复用 Stage2 共同参考 R 与幅值列映射，独立有限性、满秩、正定检查后由单位向量求解得到
`sigma_c_local=sqrt(diag(R^-1))` 米，沿用原容差，不加 jitter/damping/prior。
按段 `delta=max(0,c_hat_stage2-2*sigma_c_local)`，结构/数值不合法或 delta=0 suppress；
不串联 eta/s/gamma gate。全额 variant 严格复用上述集合，仅 delta 改为 Stage2 幅值。
最终 raw factor 残差 h+beta+delta-z，每个恢复观测一次，无 live C；sigma 为局部诊断，
不是校准置信保证或最终 bias 后验。导航、残差、协方差来自同一 final graph/Values；
保留求解判据与一次 suppress fallback，固定模式无 live-C final rescore。
仅实现、工程测试、SFUISE Walk1 起始后 [8,11]s 固定 smoke、六输入四方法加载/启动检查；
第二步精度矩阵 NOT_RUN，不按结果调 kappa/区间/阈值，不自动 push。
T10=C2-C、T11=C 与 C1–C3 限制不变，旧结果/默认/用户材料保护。

## 0911-FDE-STEP1 residual Stage1 amendment（2026-09-11）

用户本轮明确授权论文主代码路径以 `imu_aided_fde` 替代上述 L1/TV Stage1；上述
`automatic_discovery` 数学与历史证据继续作为 legacy/development 保留，不被重写。FDE 的 production
输入仅为同一 physical graph/initial Values、factor metadata、冻结 input plan、配置、calibration/common
preparation identity；禁止 GT、oracle support、obstruction label 或 future-only 数据。

FDE reference 是与 fixed-rejection 共享的 preliminary tightly-coupled LM。对每个且仅每个
`valid && planned` 的真实 `uwb_range` factor，核对 obs/factor/graph/Values 一一身份并直接读取
`r=factor->unwhitenedError(reference_values)[0]=h+beta-z` 与
`sigma=factor->noiseModel()->sigmas()[0]`。定义 `q=r/sigma`、`T=q^2`；固定 1 DoF、
`p=0.99`、阈值 `6.6348966010212145`。只有严格 `T>threshold` 为双边 fault，等号保留；
`positive_excess=(r<0)`，candidate 当且仅当 fault 与 positive excess 同时成立。正 residual fault
只保留诊断，不进入 candidate。

Temporal aggregation 按 `(tag_id,anchor_id,timestamp,obs_id)` 稳定排序。同 link 的健康 planned
观测立即结束当前 run；相邻 candidate 的 gap 只有严格 `>T_gap` 才断开，等号不分。最大 run 仅在
`count>=n_min && duration>=T_min` 时进入 `SupportPartition`；短 run 仍在 observation artifact 中保存
candidate 与过滤原因。segment ordinal/ID、partition hash 必须确定性生成，不使用幅值、change-point、
merge、L1/TV、ADMM 或 outer loop。reference LM 成功且全部 observation test 完成即返回 SUCCESS，
包括空 partition；随后空 partition 走既有 raw Stage2。reference 或输入校验失败显式失败且不发布 Stage2
cache。该方法是 RAIM/FDE-family residual front end，不是完整 ARAIM、保护级计算或 certified integrity。

本 amendment 不改变 Stage2 无正则受约束 refit、共同 candidate-excluded `R_c`、local sigma、LCB、
固定补偿、suppress/final graph/Values 与一次 fallback 定义。`structured_bias_only` 只属于旧 Stage1，
不得与 FDE 组合。FDE provider/version、p/DoF/threshold、temporal 与 preliminary LM、physical graph/
Values、plan/input/preparation/calibration identity 必须进入 producer/cache identity；final replay 同时核对
mode、Stage1 hash 与 `SupportPartition.provider=imu_aided_residual_fde_v1`，legacy/FDE cache 不得互用。
