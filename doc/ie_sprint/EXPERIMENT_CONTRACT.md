# UWB-IMU-IE 实验合同（T01）

状态：`T01_DONE`

方法前置：[`METHOD_CONTRACT.md`](METHOD_CONTRACT.md)

数据事实：[`REPO_AUDIT.md`](REPO_AUDIT.md)

冻结依据：[`../v2/paper_structure.tex`](../v2/paper_structure.tex)、[`../v2/v2_roadmap.md`](../v2/v2_roadmap.md)

本文固定 T09–T12 的数据角色、划分、RQ、指标、预算、缓存和报告规则。T01 没有运行新方法实验；
所有未执行项保持 `NOT_RUN`。

## 1. 数据角色与当前准入状态

角色具有排他用途边界：

- `calibration`：只估计独立 anchor/lever/clock/static `beta`/nominal noise；不进入方法收益统计。
- `development`：实现调试、failure discovery、候选参数范围；不能成为最终 test 证据。
- `validation`：在预声明有限预算内选择 support、segment、gate、LOS tolerance 等锁定参数；不报告为 test。
- `test`：锁定代码、数据协议、参数和缓存规则后一次性评估；labels 不得反向调参。
- `legacy`：证明旧后端在特定条件构建/运行/重跑；不自动支持 v2 方法 claim。
- `portability`：证明某 adapter/后端输入可运行；只有标定、GT 和许可闭合后才能升级为正式轨迹证据。

| 当前数据 | 候选角色 | 已证实事实 | 缺口与禁止升格条件 |
|---|---|---|---|
| 自有 UGV/Vicon no-obstacle bag | development LOS smoke 候选；未来 calibration 候选 | 文件健康 exit 0 | 独立距离参考、GT 刚体点、anchor/lever/clock 来源及许可未知；不能仅凭文件名当独立 LOS calibration/test |
| 独立 LOS `beta` calibration | calibration | **不存在已闭合输入** | 未有 manifest、独立参考与不确定度；阻止 fixed-beta 正式配置 |
| `sim_circle` 旧 bag | legacy/development smoke 候选 | 文件健康 exit 0 | 无生成 commit、锁定 seed、逐 `obs_id` bias truth；不能作 formal latent-bias test |
| SFUISE Walk1 | legacy；portability/development 候选 | legacy 两次 exit 0、trajectory 字节一致、aligned ATE `0.169642 m` | 许可、GT 点/标定独立性未知；不是 v2 结果或 raw-frame benchmark |
| SFUISE Walk2/3 | portability/development 候选 | 文件健康 exit 0 | estimator `NOT_RUN`；与 Walk1 相同 provenance 缺口 |
| MILUV random/circular | external portability 候选 | bag/adapter 材料健康 | estimator `NOT_RUN`；一手标定、GT 点、许可未核实 |
| NTU VIRAL `tnp_03` | external portability 候选 | bag 健康、仓库有 sensor YAML | estimator `NOT_RUN`；anchor/lever/time/许可与公平评估未闭合 |
| awesome-uwb example | discovery-only | 找到文件 | loader 不兼容且来源未知；不进入本 sprint 正式矩阵 |
| 计划 deterministic step/ramp input | development/validation/test（按 base trajectory/seed 隔离） | 尚未生成 | T07 前 `NOT_RUN`；truth 必须 evaluation-only 且按 `obs_id` 对齐 |
| 计划受控多链路 NLOS、两种运动、重复记录 | formal controlled test 候选 | 当前机器缺失 | 直接阻塞 RQ2 真实主证据和相关 C2/C3 claim |

来源、许可、标定或 GT 点未闭合的数据保持候选角色。角色升级必须更新 manifest 与本合同，并在
`STATUS.md` 留证据；普通缺口不构成 scope amendment。

## 2. 划分与泄漏控制

1. 划分单位是完整 `recording_id`、基础 trajectory 和 seed 的组合。一个基础轨迹的相邻 packet、时间窗、
   prefix 或由同一 seed 派生的版本只能属于同一 split。
2. calibration recording 不与 validation/test 重用。development 可查看 labels；validation 仅用于预声明
   的有限参数/operating-point 选择；test labels 在 gate 与评估协议 hash 锁定前不可读取。
3. 固定基础轨迹上的多个 noise seed 只说明该运动条件下的扰动重复，不能写成跨运动泛化。
4. 同一真实 recording 的不同方法是配对 run；不能把 packet 或 segment 当独立样本。bootstrap/区间以
   recording/base trajectory/seed 为重采样单位；独立 run 太少时展示逐 run 结果，不宣称统计显著。
5. split manifest 必须列出 recording、trajectory family、seed、role、provenance hash。任何重分配在看
   test labels 后发生都使 test 失效，必须换未见 test 或明确降级为 exploratory。

当前没有足够且 provenance 完整的数据可锁定正式 calibration/validation/test split，状态为
`PENDING_DATA`. 这不阻止 T01 合同完成，但阻止 T10–T12 正式 test。

## 3. RQ、证据与最小矩阵

| RQ | 问题 | 最小合格证据 | 不足时的 claim 边界 | 当前状态 |
|---|---|---|---|---|
| RQ1 | 同一后端/评估/provenance 是否跨 simulation、controlled、一个 public UWB–IMU 输入工作 | 每类至少一个实际 run；build/run 条件、规模、stage time、score overhead、peak RSS、重跑差异 | 单 adapter smoke 只支持该输入；T00 只支持 legacy SFUISE Walk1 | `NOT_RUN`（v2） |
| RQ2 | 锁定配置能否改善持续多链路 NLOS 且 LOS 不超过预声明退化容忍 | repeated LOS + 1/2/3-link step + ramp；配对 trajectory metrics、fail/fallback；受控真实主证据 | 缺多链路自有记录时 C2 真实主证据不足，不能用 legacy ATE 替代 | `NOT_RUN` |
| RQ3 | `eta+s+gamma` 在 matched coverage/cost 下是否比简单 gate 有增量，并有何 downstream 影响 | 两条分离路径：预声明 fixed-partition constant/ramp mismatch diagnostic；automatic support discovery end-to-end。各路径内部共享自己的 Stage 2 cache，比 fit-only、`s+fit`、full、nominal-curvature；held-out risk–coverage 与冻结工作点 final refit | fixed partition 只作 `DEBUG_DIAGNOSTIC_ONLY`，不能代替自动路径；无增量则按冻结 stop rule 收窄/简化 claim | `NOT_RUN` |
| RQ4 | 增加 future context 对同一历史 burst 有何影响 | 两个预声明 burst；`H={0,2,5,full}` 或记录允许的锁定等价值；端到端 prefix + 单独 fixed-model diagnostic | 只报告有证据的 regime；不写实时/fixed-lag 性能 | `NOT_RUN` |

### 3.1 场景和重复上限

roadmap 的最小主场景为 6 类：LOS、1/2/3-link sustained step、双链路缓 ramp、双链路陡 ramp。
每类最多先安排 2 个 development/validation seed 和 3 个未见 test seed；有多条独立基础轨迹时按轨迹
分组，不能把 seed 数冒充轨迹数。额外诊断只以双链路主场景为基准，各做一个因素：弱几何、较高
真实 measurement noise、较少实际 observation count、另一 burst duration。每一因素不得展开交叉组合。

正式下游主比较、代表子集和缓存诊断按第 4 节分层。roadmap 的最低主运行仍先覆盖锁定的
robust/rejection reference、`structured_refit` 和 `full_gate`，LOS 另含 `all_range`；其余冻结消融没有
删除，而是进入测试前预声明的代表子集或同一 cache 的 gate diagnostic。不能为 threshold sweep 重跑
estimator，也不能把全部方法、阈值、场景、seed 和 prefix 展开成笛卡尔积。

上述数量是执行上限建议和最小证据规划，不保证统计充分。缺场景必须列为 missing evidence，不能以
扩大 seed 或 packet 数替代。

### 3.2 计算预算规则

T03/T04/T06/T09 得到代表场景的实测 median/slow runtime 和 stage breakdown 后，负责人登记可用总计算
预算 `B_total`（wall-clock 或等价 CPU-hours）。正式排程必须满足

```text
B_scheduled <= 0.75 * B_total
B_reserve   >= 0.25 * B_total
estimated cost = sum(uncached run estimates) + aggregation/rerun allowance.
```

T01 没有 v2 单 run 实测，故 `B_total=PENDING_PROFILING`；T00 的约 7 s legacy wrapper runtime 只作旧
路径观测，不作为新 pipeline 预算实测。超预算时依次删额外参数水平、诊断重复、可选第二 public dataset；
不得删除失败场景、某个 RQ 的最小证据或表现差的 seed。并行度仅由实测 peak memory 决定，每个 run
仍使用独立输出目录。

预算记录必须分 `estimated_*` 与 `measured_*` 字段，不得把公式估计写成实测耗时。

## 4. Baseline 与消融合同

所有模式共享：raw observation pool、validity mask、keyframe/input-plan hash、初始化规则、独立标定、
nominal `sigma_i`、solver 精度和 evaluation interval。

下表的 baseline/ablation 项目来自冻结论文，均必须保留；canonical mode 名、初版建议名映射以及三层
执行安排是本次选定的工程表达，状态为 `SELECTED_NOT_YET_REVIEWED`，不表示负责人已经确认。

| 冻结论文/roadmap 项 | canonical mode 与确定语义 | 与初版建议名的对应 | 执行层级 | 状态 |
|---|---|---|---|---|
| all-range batch | `all_range`：固定有效 raw ranges 全部进入共同后端，无 NLOS suppression/correction | 同名 | LOS 必跑；其他场景仅预声明需要时 | 未实现/`NOT_RUN` |
| preselected Huber/Cauchy batch | `robust_huber`、`robust_cauchy`：同一后端只施加预锁定 Huber 或 Cauchy loss，不作后续 hard rejection；validation 前声明候选，test 前选择一个 `robust_loss_primary`，另一项保留为代表子集消融 | 初版遗漏；不能用 `gnc_rejection` 代替 | primary 进入主 trajectory matrix；另一个只进代表子集 | 未实现/`NOT_RUN` |
| fixed rejection policy | `fixed_rejection`：预先锁定 residual/quality rule 与阈值后作二元保留，不在线按 test label 改规则 | roadmap 的 `gnc_rejection` 只有在输入合同一致且其 GNC/rejection 全流程被锁定并明确记录时才可作为该项；否则只叫 legacy system reference | primary main matrix | 现有组件存在，匹配语义未验证/`NOT_RUN` |
| structured bias only | `structured_bias_only`：运行 Stage 1 非负 L1/TV 联合目标并直接使用其 regularized bias/trajectory，不执行 Stage 2 debias；明确展示 shrinkage baseline | 初版 `structured_refit` **不对应**此项 | test 前预声明代表子集；论文 ablation row 保留 | 未实现/`NOT_RUN` |
| structured bias + debias | `structured_debias`：自动 discovery 后固定 partition，运行无 L1/TV 的 constrained Stage 2 refit，不施 gate | 初版 `structured_refit` 是该项的兼容别名；manifest canonical name 用 `structured_debias` | primary main matrix | 未实现/`NOT_RUN` |
| fit-only gate | `fit_only`：相同 Stage 2 cache，仅按锁定 `gamma` fit rule 作组级决策 | 初版同名 | 两条 RQ3 路径的缓存诊断；冻结工作点在代表子集 final refit | 未实现/`NOT_RUN` |
| absolute-uncertainty + fit | `s_fit`：相同 cache，以 `s+gamma` 作组级决策 | 初版同名 | 缓存诊断；test 前代表子集补下游 trajectory | 未实现/`NOT_RUN` |
| complete module | `full_gate`：冻结 `eta+s+gamma` 组级政策和最终联合 refit/fallback | 初版同名；文中 `full` 均解析为此模式 | primary main matrix + 两条 RQ3 缓存诊断 | 未实现/`NOT_RUN` |
| simulation eta-only | `eta_only`：仅在 synthetic RQ3 diagnostic 用 `eta` gate，暴露尺度和 mismatch 局限；不作为真实数据主政策 | 初版遗漏 | 缓存诊断；至多在预声明 simulation 代表子集 final refit | 未实现/`NOT_RUN` |
| nominal-curvature comparator | `nominal_curvature`：以 bias nominal block `lambda_min(N)` 为量纲一致的未消元 comparator；不是某论文完整复现 | 初版同名，来自 roadmap | 两条 RQ3 路径的缓存诊断；通常不单独重跑后端 | 未实现/`NOT_RUN` |
| oracle correction/rejection | `oracle_reference`：仅 synthetic，使用 evaluation-only truth 并标 `DEBUG_REFERENCE` | 初版 oracle row | 仅限仿真的参考对照，不保证性能上下界；不进入自动端到端 claim | 未实现/`NOT_RUN` |

baseline 不得因不同 RSSI 前置删除、keyframes、beta、noise、初始化或时间配对而被写成只差一个 gate。
不能公平匹配时，明确列出系统差异并降为额外参考。

执行分层固定如下，不删除上表任何项目：

1. **主 trajectory matrix：** 每个主 test 场景先跑 `robust_loss_primary`、合格的
   `fixed_rejection`/明确标注的 `gnc_rejection` reference、`structured_debias`、`full_gate`；LOS 加
   `all_range`。若 roadmap 的资源预算要求更小，删减必须在 test 前记录，不能看结果后删方法。
2. **测试前预声明代表子集：** `structured_bias_only`、未选作 primary 的另一个 robust kernel、
   `fit_only`/`s_fit` 的 final trajectory，以及 simulation `eta_only`。子集按 scenario/seed/geometry 在
   查看 test metric 前写入 locked manifest。
3. **缓存诊断：** `fit_only`、`s_fit`、`full_gate`、`eta_only` 和 `nominal_curvature` 读取各自 RQ3
   路径的同一候选/partition/Stage 2 score cache，threshold/ranking 点不触发 discovery/refit。只对锁定
   operating points 和上述主/代表子集做 final joint refit。

Huber/Cauchy、fixed rejection、Stage 1-only comparator 具有不同优化目标或 factor mask，不能伪装成
从 Stage 2 gate cache 免费导出；它们各自的实际后端 run 才算 trajectory evidence。当前不建议删除任何
冻结 baseline/ablation，因此没有为 baseline 清单登记 deletion amendment。

## 5. 参数锁定和 test 准入

需要 validation 锁定的参数包括：`lambda_1,lambda_TV,T_gap,b_min,delta_change,delta_merge,n_min,T_min,
tau_eta,tau_s,tau_gamma,epsilon_bad_m`、LOS degradation tolerance、trajectory failure 条件、prefix horizons、
time association/interpolation tolerance、gate operating points，以及 sensor/domain-specific nominal noise rule。
数值与完整方法参数表见 `METHOD_CONTRACT.md`。

锁定流程：

1. 完成 calibration provenance、split manifest 和 evaluation protocol；
2. 在 development 只确定有限候选范围与 failure；在 validation 为每个策略使用相同有限评估预算；
3. 生成 `locked_gate.yaml`/等价配置，记录参数、选择准则、输入 role IDs、代码/配置/hash、时间与负责人；
4. 冻结 input plan、discovery/refit cache schema、指标实现和 baseline 定义；
5. 运行 U01–U14 中所有适用 correctness tests，且先解决 IMU preintegration 语义冲突；
6. 审查者确认数据许可/角色、GT 点/标定与 locked manifest 后，才允许读取 test labels 和正式运行；
7. test 后 correctness fix 必须使受影响 cache/result 失效并完整重跑；方法/证据边界变化先走 amendment。

当前 test 准入为 `DENIED_PENDING_IMPLEMENTATION_AND_EXTERNAL_PROVENANCE`，并非实验失败结果。

## 6. 轨迹、bias 与 residual 指标

### 6.1 轨迹指标

- `raw_frame_ATE`：在独立于待评估 trajectory 确定并锁定的 map-to-GT rigid transform 下计算；记录 GT
  刚体/测量点、杆臂、坐标系、时间关联/插值、matched count。没有独立 transform/provenance 时为
  `UNAVAILABLE`，不能补造。
- `aligned_ATE`：对每条估计 trajectory 单独作 scale=1 SE(3) alignment 后的 ATE；只能以该名称报告，
  不能冒充 raw-frame 定位误差。T00 的 `0.169642 m` 属于 SFUISE Walk1 legacy aligned ATE。
- RPE、per-axis error、P95/P99/max 与 failure 另报；失败 run 留在分母，不能以缺值或零误差进入均值。
- LOS degradation tolerance 与 trajectory failure condition 必须在看 test 结果前锁定。

### 6.2 三种 bias 量的边界

| 量 | 定义 | 允许用途 |
|---|---|---|
| synthetic `b_i*` | 生成器通过 evaluation-only sidecar 给出的 latent injected bias | bias-field RMSE、bad correction label；estimator 禁止读取 |
| measured `b_ref=z-h(X_GT)-beta_LOS` | 含 range noise、GT、survey、lever、beta、timing 误差及相关性的带噪参考 | 真实数据 agreement 与不确定度报告；不是 noise-free truth |
| post-fit residual `r_i` | 同一 final graph/Values 的 factor residual，按定义可 normalized | `gamma`/fit diagnostics；不得当 bias truth 或 measured reference |

ramp 下按固定 historical obs_id 集计算 `hat b_i-b_i*` 的 bias-field RMSE；单段均值误差不能替代时变场误差。
`bad_correction_rate` 仅在 accepted correction 上以预声明 `epsilon_bad_m` 定义。只有实际配对比较 Use 与
Suppress 的下游 trajectory error，才可称 `trajectory_harm`；bad correction 不能自动推断 trajectory harm。

## 7. Coverage、空集合、失败和 fallback

以 measurement count 为主并同时输出 group/segment 数量与长度：

```text
candidate_use_coverage = accepted candidate observations / all candidate observations
eligible_use_coverage  = accepted candidate observations / eligible candidate observations
overall_retained_fraction = final-used raw observations / fixed valid input observations.
```

第一项分母包含 ineligible/short/boundary candidate；第三项分母不因策略而变。另可报告 group-level coverage，
但名称和分母必须显式，不能替代三项主定义。

| 情况 | 必报状态 | 指标处理 |
|---|---|---|
| 零候选 | `NO_CANDIDATES` | candidate/eligible coverage 为 `UNDEFINED`；overall 正常计算；非胜利/失败 |
| 有候选、零 eligible | `NO_ELIGIBLE_CANDIDATES` | candidate use 为 0；eligible coverage `UNDEFINED`；列原因 |
| eligible 非空、零接受 | `ZERO_ACCEPTED` | 两种 use coverage 为 0；accepted conditional risk `UNDEFINED`，绝不能记 0 |
| recovery attempt 失败、fallback 成功 | `FALLBACK_OK` + 原 failure | fallback trajectory 单列；run 仍计 recovery failure/fallback frequency |
| fallback 也失败 | `ESTIMATION_FAILED` | 所有 trajectory/bias metric `UNAVAILABLE`；run 计 failure 分母，不以 0 进入均值 |
| timeout/resource failure | 对应明确状态 | 同样保留在 run-level denominator；不得删除 |

decision-time risk/coverage（共同 Stage 2 cache）与 final-time bias/ATE（各冻结工作点 joint refit）分表。
fallback 结果和 recovery attempt failure 不混成一列。

## 8. RQ3 公平比较

RQ3 必须同时执行下面两条路径。它们复用同一 raw input/calibration/input plan 与锁定评分定义，但
partition 来源不同、cache namespace 不同、回答的问题不同，结果必须分表。跨路径比较只能描述自动
discovery 带来的 partition/coverage 差异，不能写成 gate-only 因果比较。

### 8.1 路径 A：fixed-partition constant/ramp mismatch diagnostic

标签固定为 `RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY`。它用于隔离“同一个常值段模型面对 constant
和 within-burst ramp 时，`gamma/eta/s` 与 gate 怎样变化”，不是正式自动端到端证据。

1. partition 来源必须在运行任何 estimator、查看任何 estimated residual/score/test metric **之前**写入
   `fixed_partition_manifest`。对 synthetic injected scenario，由已经冻结的 scenario recipe 给出 link 与
   `[t_start,t_end]`；按固定 input plan 将区间映射为 obs_id。constant 与对应 ramp 使用相同 link/区间，
   每个 scripted burst 固定为一个 constant-amplitude segment，从而故意让 ramp 构成 model mismatch。
2. 该 manifest 可使用 scripted support interval，因此属于 oracle-like diagnostic 输入；只提供
   obs_id/partition，不向 refit 提供 true amplitude、GT pose 或 trajectory initialization。所有 run、表和
   cache 带 `DEBUG_DIAGNOSTIC_ONLY`，不得进入自动 discovery 成功率、正式四阶段 coverage 或 C2
   end-to-end 完成证据。
3. 跳过 Stage 1，仅从同一 fixed partition 执行 Stage 2 constrained refit 与 Stage 3 score。路径内的
   `fit_only/s_fit/full_gate/eta_only/nominal_curvature` 共享一个 immutable cache：

   ```text
   rq3_fixed_partition_cache_key = common_input_key
     + scenario_recipe_hash + fixed_partition_manifest_hash
     + partition_rule_version + stage2_score_config_hash + solver_version.
   ```

   constant/ramp 是不同 input hash；同一 input 内所有 gate comparator 的 obs_id、partition、refit Values、
   `F/G/N/R`、nominal sigma 和 residual 完全相同。threshold sweep 只读缓存。
4. 必须输出 manifest 规定的 segment count、每段 obs count/duration、segment/group IDs、partition hash，
   以及每段 decision-time `gamma_s`；同时输出 `eta/s`、risk/coverage 和空接受状态。若对锁定 operating
   point 做 final joint refit，另输出 final-time gamma 与 trajectory metric，并继续标 DEBUG。

### 8.2 路径 B：automatic support discovery end-to-end

标签固定为 `RQ3_AUTO_DISCOVERY_END_TO_END`。这是 RQ3 对自动四阶段系统的正式路径。

1. 每个完整 recording/base trajectory/seed 从正常 UWB/IMU 输入重新运行 Stage 1 discovery；estimator
   不读 scripted support、bias truth、GT 或路径 A 的 partition/cache。Stage 1 后按已锁定 change-point、gap、
   short/boundary 及 merge 规则产生自动 partition，再进入同一个 Stage 2–4 实现。
2. 路径内 gate comparator 共享该 input 的自动 discovery/Stage 2 cache：

   ```text
   rq3_auto_discovery_cache_key = common_input_key
     + discovery_config_hash + partition_rule_version
     + stage2_score_config_hash + solver_version.
   ```

   cache payload 另存 discovery result hash、support/partition hash 和 linearization ID。不得复用路径 A 的
   fixed partition cache；改变 A01 merge 规则、support/change-point 参数或 discovery solver 使其失效。
3. `fit_only/s_fit/full_gate/eta_only/nominal_curvature` 在同一自动 cache 上比较，因此每个 input 内候选、
   segments、Stage 2 refit、`F/G/N/R`、residual 和 nominal sigma 一致。只有 gate policy 不同；locked
   operating point 的 final joint refit 才可用于 downstream trajectory comparison。
4. 每个 run 必须输出 discovery active count、自动 segment count、每段 obs count/duration、长度分布、
   short/boundary 数、group count/size、每段 decision/final `gamma_s`、`eta/s`、三种 coverage、failure 和
   fallback。constant/ramp 同时报告 fragmentation/merge 结果；不能只报通过 gate 的段。

### 8.3 两条路径的共同公平性与 RQ4 隔离

两条路径均使用相同 raw observation pool、validity mask、input-plan/keyframe hash、independent calibration、
nominal sigma、navigation initialization rule、Stage 2/3 solver 精度、gate definitions、split 与 evaluation
interval。路径 A 与 B 的 gate 比较各自在路径内部匹配 acceptance coverage/cost；不得把两条路径不同
partition 下的 segment 当成配对独立样本。统计仍以完整 run/seed 为单位。

先比较 accepted bias-field error、bad-correction rate、good-correction rejection 与空接受；再只对锁定
operating point 运行 final joint refit，独立比较 trajectory effect。correction error 不自动等于 trajectory
harm。单 amplitude 且 `N` 固定时 `eta` 与 `s` 有确定关系，实验不预设 `eta` 必须独立胜出；无增量/
负增量均完整报告并按冻结 claim fallback 处理。

RQ3 路径 A 不得与 RQ4 的 `RQ4_FUTURE_COMMON_LINEARIZATION_DIAGNOSTIC` 合并：RQ3 固定 scenario
partition 并改变 constant/ramp bias shape，检验 model mismatch；RQ4 固定同一历史 support/amplitude 和
共同 linearization，只增加 future factors，检验 future information。两者使用不同 manifest、cache key、
图表标题和结果表，不共享“fixed model diagnostic”这一含混标签。

## 9. Prefix/full-batch 合同

历史 burst `B=[t_a,t_b]`，run 的可用终点为 `t_b+H`；`full` 表示完整 recording。所有 run 只在同一
历史 obs_id/evaluation interval 上评分。

端到端 prefix 必须在 initialization 前裁剪 UWB 和 IMU，并重新运行 input plan、初始化、discovery、refit、
score、decision、final。禁止继承 full-run state/warm start、support/segments、auto calibration、future-dependent
noise、cutoff 后插值或任何未来 observation。独立 calibration 和 test 前锁定的 gate 可以共享。

每个 prefix manifest 保存每种输入最大 timestamp 与 cutoff。必须通过 U13：修改/删除/打乱 cutoff 后输入，
prefix 结果在锁定数值容差内不变。不同 prefix 的 segment ID 不直接比较；以相同历史 obs_id 上的 bias field
与 trajectory error 比较。fixed support/common linearization 的信息单调性试验另标
`RQ4_FUTURE_COMMON_LINEARIZATION_DIAGNOSTIC`，使用独立 manifest/cache namespace，与 RQ3 的
constant/ramp fixed-partition diagnostic 及非线性端到端结果分别成表。

## 10. 隔离输出、manifest、hash 与缓存失效

每个实际运行使用唯一 `runs/<run_id>/`，禁止共享 `latest`、固定 `trajectory.txt` 或同一目录并行写入。
最低产物为：

```text
input_manifest.json
config_original.yaml
config_effective.yaml
run_status.json
observations.csv
segments.csv
scores_decision.csv
scores_final.csv
decisions.csv
trajectory.tum
metrics.json
stdout.log
stderr.log
```

manifest 至少记录：run ID/role/split、commit 与 dirty diff hash、binary/dependency/overlay、input/calibration/
config hashes、seed、units/frames、GT point/time policy、cutoff、keyframe plan、solver/discovery/refit/gate versions、
parent cache IDs、start/end time、exit/status。evaluation-only truth 单独存放且不列入 estimator input。

discovery/refit cache key 至少为

```text
input_hash + calibration_hash + keyframe_plan_hash + initialization_rule
+ solver_version + discovery_refit_config_hash.
```

任何输入/validity、beta/sigma、keyframes、initialization、support/segment rule、reference mask/weight、factor
语义、solver correctness、linearization convention 或 discovery/refit 配置变化均使 cache 失效。只改 gate threshold
可复用 decision-time cache；改变冻结工作点后必须重跑对应 final joint refit。禁止跨 commit 混用未声明 cache。

T10-A05 的 `DEVELOPMENT_CONDITIONAL_LM_RECOVERY_V2_INTEGRATION` 只是一项已预登记的 development
solver 对照：两份隔离 A03 step/ramp 配置各运行一次，outer=500、inner=50、每进程上限 120 秒，唯一配置
变化是默认关闭的 V2 policy。V2 的 optimizer 内部 relative tolerance 为 0，外部 generic relative/absolute
tolerance 不变；该语义和 policy version 必须进入 solver/support/cache producer identity。两条均在 Stage 1
失败且没有生成实际 Stage-2 cache；因此现有证据只证明 producer identity 可区分，不能声称缓存已产生、
可用于正式评分或获得 validation 资格。该对照不占用或定义尚未锁定的正式 `B_total`，不授权重复、参数搜索、
test、gate lock 或默认迁移。

## 11. Run-level 统计和报告

1. 统计表保留每个预登记 run；`OK/FALLBACK/FAILED/TIMEOUT/NO_CANDIDATES/ZERO_ACCEPTED` 都占相应分母。
2. 方法比较使用同 recording/base trajectory/seed 的配对结果；missing metric 保持 NA，不填零。
3. 主表给逐 run 或 run-level 汇总与区间；segment/packet 只作诊断，不作为独立重复。
4. 所有 headline 数字最终只能从 locked machine-readable metrics 生成；T01/T00 手写事实不能进入 v2 主结果。
5. 预算估计、实测 runtime、score overhead、fallback cost、peak RSS 和 rerun difference 分字段报告。
6. 来源不闭合、被排除场景及原因、低 coverage、不利结果和全部 failure 进入公开 evidence ledger。

## 12. T01 未执行项与外部阻塞

| 项目 | T01 状态 | 阻塞内容 |
|---|---|---|
| v2 build/test/runner/NLOS implementation | `NOT_RUN` | T02–T09 尚未实施；T01 不修改源码/CMake/config/test |
| U01–U14 | 全部 `NOT_RUN` | 各自在 T02–T11 的方法验收 |
| calibration/validation/test split | `NOT_RUN/PENDING_DATA` | 缺独立 beta、权限、GT/杆臂/clock provenance 和足量独立记录 |
| RQ1–RQ4 | 全部 `NOT_RUN` | v2 pipeline/locked params/正式输入未具备 |
| 自有多链路两运动重复实验 | `NOT_RUN/MISSING_LOCAL_DATA` | RQ2/C2/C3 真实主证据 |
| public portability run | `NOT_RUN` | 先核实一个数据集的一手许可与标定，再由 T09/T12 执行 |
| raw-frame ATE、bias truth/reference | `NOT_RUN/UNAVAILABLE` | 独立 frame transform、GT 点与 calibration provenance 未闭合 |
| IMU correctness | T00 已运行但 `2 failures` | 进入依赖 IMU 语义的新实现/正式实验前处理 |
| linked-devel collision | `OBSERVED_RISK` | 后续 runner 的 overlay/binary provenance 与仿真可复现性 |

影响 partition/hash 的 merge 执行澄清 A01 已由本轮用户指挥/审查会话技术接受，但尚未实现或测试；
确认来源和边界登记在 `STATUS.md`。若未来无法取得冻结主证据，只能另行登记 `PROPOSED` amendment
并收窄 claim；不能把 portability、legacy aligned ATE 或普通证据缺口静默改写成正式证据。

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
# T10-A19 bounded development run

A19 只允许一次冻结 P1 step seed10101 的 fresh Stage1→Stage2→eligibility/score development 运行，整进程树 900 秒，Stage1 outer500/conditional50、Stage2 outer200，无 retry/warm start/搜索。结果 schema 为 `A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY` 且 `consumable=false`；不得作为正式 validation/test、gate lock 或 C1-C3 证据。完整身份、工程门、失败记账和 NOT_RUN 规则见 `T10_A19_AMENDMENT_PROTOCOL.md`。

## T10-A19-R03 development 运行与比较边界

R03 的唯一方法 amendment、工程门、身份与预算在
[`T10_A19_R03_PROTOCOL.md`](T10_A19_R03_PROTOCOL.md) 实施前登记。工程门通过后只允许一个新 fresh P1
step seed10101 自动 Stage1→Stage2→score 进程树（900 秒），从 raw 原始初始化且不读旧 checkpoint、oracle
support 或 truth。首次算法失败停止该科学分支，无 retry 或数值参数变化。

只有最终 joint 四项 AND、身份与 score 完整时，才在冻结的同一自动 partition/Stage2 graph/Values/scores 上
运行预登记 `suppress_all/structured_debias/fit_only/s_fit/full_gate`；阈值固定为 gamma `1`、s `0.10m`、
eta `0.10`，每策略 900 秒、final 合计 4500 秒、整个科学计算 5400 秒，至多一次原合同 fallback。零 eligible
停止 final。决定与输出冻结后独立 evaluator 才读取 evaluation-only truth，`epsilon_bad=0.20m`；报告全记录
和 `[3,6]s` 配对 raw-frame ATE、bias/risk/coverage/failure/fallback/cost。新 schema 保持
`development/consumable=false`，由显式 adapter 读取，正式 reader 必须拒绝；结果不锁 gate、不进入正式
validation/test，不升级 C1--C3。

## T10-A19-R01 接线修复与运行记录

R01 的限定范围、身份失效、工程门和一次性运行预算在
[`T10_A19_R01_PROTOCOL.md`](T10_A19_R01_PROTOCOL.md) 预登记。R1--R3 工程门全部通过；新的唯一
fresh ticket 已消费，但进程在 Stage1 前因隔离输出父目录缺失 exit2，Stage1/Stage2/scoring 全部
`NOT_RUN`，无 retry。失败后的 launcher 修正属于下一身份，尚未执行 estimator；它不能复用已消费 ticket
或将本次入口失败改写为成功。正式 validation/test/cache/final、locked metrics 与 RQ 仍 `NOT_RUN`。

## T10-A19-R04 crash repair and rerun boundary

R04 separates at most `60 s` original-stack plus, only if required, one
`120 s` targeted diagnostic from engineering-fixture and science accounting.
One post-fix prepare-only run (`<=120 s`) must reach the explicit pre-Stage1
stop with zero optimizer calls. Before the single fresh science ticket, the
same final build must prove that an independent evaluator consumes real mixed
score->decision->final artifacts. Science preserves automatic `900 s`, five
separate final process trees at `900 s` each, final total `4500 s`, and overall
`5400 s`; no shared `900 s` wrapper may replace these limits. Truth remains
unavailable to estimator, decisions, and final optimization. All outputs stay
development-only and `consumable=false`; no gate or C1--C3 claim is upgraded.

R04 actual execution is closed at its preregistered first algorithm failure.
The ABI-matched prepare path and real mixed-final evaluator gate passed, after
which the sole fresh ticket completed raw initialization and a 371-factor graph
but Stage1 rejected the R04 output schema at its producer identity allowlist.
It performed zero outer/conditional calls and exited 1 without timeout. There
is no retry under this ticket: Stage2, score, five science finals and truth
evaluation are `NOT_RUN`, and all corresponding metrics are unavailable rather
than zero. The complete record is
[`R04 verification`](evidence/t10_a19_r04_crash_score_compare_20260910T072815Z/VERIFICATION.md).

## T10-A19-R07/R08 finite validation boundary

R07 supplies the valid all-suppressed common reference for the single exposed
development seed; its result is not validation evidence. R08 is the first
limited synthetic validation characterization and is restricted to the two
unused validation reservations and six frozen scenarios in the A10 split.
Its role/context/ancestry identity must reach the C++ producer and final output;
all estimator artifacts are frozen before an independent evaluator opens the
scenario truth. The 12-input matrix, serial budget, fixed working point, LOS
all-range reference and stop rules are preregistered in
[`T10_A19_R08_PROTOCOL.md`](T10_A19_R08_PROTOCOL.md). It cannot lock a gate,
admit held-out test, or upgrade C1--C3 without a separate review.

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

## 0911-FDE-STEP1 实验与 artifact amendment（2026-09-11）

本轮开发/portability 运行把论文主 producer 的 Stage1 固定为 `imu_aided_fde`，旧
`automatic_discovery` 配置、测试、cache 与结果仅作 legacy/development 对照并继续保留。FDE 配置必须
显式给出 gap/min-count/min-duration，并要求 `solver.chi2_reject_prob=0.99`；拒绝其他概率、oracle
support 与 `structured_bias_only` 组合，不要求 L1/TV、ADMM、activity/change/merge/outer-loop 参数。

每个 FDE producer 不论成功或失败都必须写 `fde_status.json`、`fde_observations.csv` 和
`support_partition.json`；成功 producer 另写与后者字节相同的兼容 `partition.json` 并按既有 Stage2
schema发布 cache。`fde_status.json` 至少保存 provider/version、reference solve、p/DoF/threshold、
planned/tested/fault/positive-candidate/raw/filtered/retained 计数、partition identity 和 `gt_read=false`。
`AUTO_DISCOVERY` 调度/cache namespace 名称为历史兼容保留，但准入按 provider-aware Stage1/context/
payload hash 隔离；FDE payload 强制包含三个 FDE artifacts，legacy payload不追加该要求。

工程门后仅在新隔离目录运行 Walk1 起始后 `[8,11]s` smoke，再串行运行已有六输入的
`robust_cauchy`、`suppress_all`、`structured_debias`、`lcb_partial`、`lcb_fixed_full`。每个数据集的
structured producer 只运行一次，其四个 candidate-dependent final 必须引用同一 cache ID、partition hash
与 Stage2 Values；Cauchy 保持独立 disabled path。估计器运行与 hash 冻结后才由 evaluator 读取既有锁定
GT，报告 aligned RMSE/P95/horizontal/vertical RMSE 与 trajectory coverage；LCB improvement 定义为
`suppress_all-lcb_partial`，百分比以 suppress 为分母，零分母记 unavailable。所有失败、fallback、空候选、
zero-coverage 原样保留，不按结果调参或重跑算法。该批次不是正式 held-out test，不升级 T10=C2-C、
T11=C 或 C1–C3。

## 0911-RECOVERY-FDE-V2 amendment

本轮用户授权 [RECOVERY_FDE_V2_PROTOCOL.md](RECOVERY_FDE_V2_PROTOCOL.md) 的完整定义与验收边界。该版本替代 paper 共同初值、FDE 空候选重复 refit、旧 r/sigma detector 规则；历史文本作为旧版本保留。实施状态 IN_PROGRESS，未经运行的门均 NOT_RUN。非空 Stage2/LCB/live-C/fallback 原规则保持。

0911-RECOVERY-FDE-V2 收口：定位门、完整CTest29/29、固定smoke/full Walk1五方法和30项prepare已通过；真实零候选只支持空集合工程链。实际阈值保留旧查表6.6349；详见 [结果与限制](../ie_0911/RECOVERY_FDE_V2_RESULT.md)。历史条款不回写，C1–C3不升级。

## 0911 NLOS injection experiment amendment

用户授权 [NLOS_INJECTION_PROTOCOL.md](NLOS_INJECTION_PROTOCOL.md) 限定 semi-synthetic 实验与评价接口；算法基线46d37f6冻结。旧合同及历史结果保留，C1–C3不升级。

## 0912 Windowed FDE v4 locked Walk1 amendment

用户授权 [`WINDOWED_FDE_E2E_PROTOCOL.md`](WINDOWED_FDE_E2E_PROTOCOL.md) 的单一 locked Walk1
development E2E。只复用旧 manifest 中 normal clean/injected、seed 911、固定 link/window/+0.5m 与
同一 input/truth hashes；只在隔离 effective config 启用 v4。先 truth-hidden clean detector-only，
要求零 retained segment；再 truth-hidden injected detector-only，artifacts 冻结后由独立 evaluator 读取
truth，要求 target-link production support 有 temporal overlap。任一 detector gate 失败立即停止 estimator，
不调 probability/window/temporal/kappa/threshold。

两个 screen 通过后才在同一 injected input 串行运行六方法；四个 candidate-dependent final 必须共享
同一 v4 Stage2 cache、partition 与 Values，并核对 E2E support 与 screen support 完全一致。轨迹只报告
aligned ATE 及配套 P95/horizontal/vertical/coverage；定位收益只按协议中的严格 paired RMSE 规则裁决。
本轮不运行 Walk2/3 或低冗余矩阵，不构成正式 held-out、总体 detector 保证或 C1--C3 claim 升级。

## 0912 PL conditional RAIM/FDE locked Walk1 amendment

用户授权 [`PL_CONDITIONAL_RAIM_PROTOCOL.md`](PL_CONDITIONAL_RAIM_PROTOCOL.md) 的单一 locked
Walk1 development preflight 与条件式 production E2E。复用旧 manifest 的 normal clean/injected、
seed 911、link `27956:20276`、闭区间 `[1664959678.3077347,1664959686.3077347]`、
`+0.5m`、30 planned affected IDs 与原 keyframe plan。clean/injected detector 在不接收 truth
参数且 truth 路径隐藏的独立进程中运行；产物封存后独立 evaluator 才可读 truth，
并依次要求：(1) clean support 为 0；(2) affected groups 至少一次 alarm；(3) 至少一次
unique isolation 命中 target anchor；(4) truth-blind persistent support 命中 target link 且与 truth
在时间或 obs IDs 上重叠。唯一通过状态为 `PL_CONDITIONAL_PREFLIGHT_PASS`；任一门失败则
production 和 E2E 全部 `NOT_RUN`，不调概率、temporal、kappa、区间或阈值。

Preflight 通过后才可新增显式互斥配置 `nlos.pl_conditional_raim_fde_test=true`，并先逐
epoch 证明 production 与 sealed preflight 在 obs/order、alarm、hypotheses、isolation、commit policy 和
support 上精确一致，浮点量在 `1e-12+1e-10*scale` 内。production clean 必须 0 support，
injected 必须与 sealed support 一致并由 truth evaluator 证明 target overlap。通过后串行
`all_range`、`robust_cauchy`、`suppress_all`、`structured_debias`、`lcb_fixed_full`、
`lcb_partial`；后四者共享单一新 provider support、partition、Stage2 Values 和 payload identity。

独立 evaluator 报告 RMSE/P95/horizontal/vertical/coverage/optimizer/fallback 以及各 recovery 相对
suppress 差值。缺指标为 `NOT_EVALUABLE`；否则只以严格 `lcb_partial RMSE < suppress_all RMSE`
判定收益。分开输出 preflight、production detector、backend 和 localization 四项 verdict。本轮不运行
Walk2/3/低冗余，不构成 formal held-out、总体 detector 保证或 C1--C3 升级。

0912 preflight 实际结果：clean detector-only 运行 0 retained support，第一门 PASS；injected 的
30/30 planned affected IDs 均被分组检测，但 affected alarm 为 0，第二门 FAIL。独立 evaluator 在
detector artifacts 封存后才读取 truth，file-open trace 的 forbidden truth/oracle open 为 0。第三门
target unique isolation、第四门 persistent target overlap 按串行门标为
`NOT_RUN_PREVIOUS_GATE_FAILED`；production detector、六方法、backend 与 localization 全部
`NOT_RUN/NOT_EVALUABLE`。不允许根据该结果调整任何冻结参数。

## 0912 PL threshold / persistent-signal diagnostic amendment

本轮只复用既有 locked Walk1 clean/injected scenario cache 和 f2ee3f0d causal shadow replay，不重新生成
输入、不运行 Stage2/recovery/E2E。先在两个不接收 truth 的隔离进程中复现原 224 clean groups/0 alarms、
30 affected groups/0 alarms 和最大 `T`；同时生成逐 row `nu,R,HPH^T,S,marginal z,conditional z` 与
quadratic decomposition 并封存。独立 evaluator 封存后才读旧 injection truth，按 group/anchor/obs identity
精确配对。

离线 sweep 固定为 `P_FA={1e-5,1e-4,1e-3,1e-2,0.05,0.10}`，每档按实际 DoF 报 threshold、clean
alarm fraction 与 affected recall；不是 parameter selection。persistent statistics 固定为 N、mean、median、
sample std、P10/P90、NLOS-direction sign fraction、max absolute、`sum(z)/sqrt(N)` 与逐 epoch cumulative
signed sum。target 与同组 healthy anchors同时报告。最终只允许协议 A/B/C/D 裁决，不产生 trajectory、
support、cache 或论文收益 claim；所有下一 detector 均 `NOT_RUN`。

0912 diagnostic 实际完成：clean 224/0 与 affected 30/0、最大 T=4.86052305024 均复现；六档
`P_FA=1e-5...0.10` 的 affected recall 全为 0。statistics seal 后独立 truth evaluator 精确配对 30 target
rows，得到 injected target conditional mean 1.0926 sigma、正号 1.0、`Z_sum=5.9846`，paired
`Z_delta=6.6220`；healthy 最大正向 paired shift 为 0。唯一裁决为 B。file-open forbidden count=0；
Stage2/recovery/E2E、support/cache、threshold 写回和下一 detector 均未执行。

## 0912 PL persistent CUSUM frozen/dynamic admission amendment

本轮数据角色仅限同一 sealed Walk1 clean conditional artifact 内预先冻结的 temporal calibration 与
held-out validation，以及 locked injected development gate。split 固定为有效 clean 时间跨度 60% 点，
两侧各留 0.5s、总 1s guard；manifest 在任何 CUSUM statistic 前 seal。calibration 只能决定
`G_calibration_max` 与 `h=max(5,G_calibration_max+1)`，不得访问 validation statistic、injected、truth、
target 或注入 metadata；validation CUSUM state 从零开始。

Frozen admission 依次要求 integrity、held-out clean 0 alarm/segment、target index<=15 且 alarm 在注入区间、
healthy 0 alarm/segment、target precision/recall 各>=0.80。仅全部通过才运行 raw CONTROL/SHADOW dynamic
always-commit replay，并依次要求 non-interference、dynamic held-out clean、target latency、specificity 和
同样 support quality。Truth 仅由 detector artifacts seal 后的 evaluator 读取。失败后禁止修改 split、
`kappa/h`、gap/reset/backfill/termination、注入或 gates。本轮没有 recovery、trajectory metric、正式
production integration 或 claim 升级。

0912 CUSUM admission 收口：F0/F1/F2/F3 通过、F4 失败。target TP/FP/FN=30/37/0，
precision=0.4477611940、recall=1.0；held-out clean 与 healthy anchors 均 zero-alarm/zero-segment。
按预登记顺序，dynamic CONTROL/SHADOW、D0--D4、scientific PASS 后的 package-wide tests 以及所有
recovery/localization 工作均为 `NOT_RUN_DUE_TO_EARLIER_FAILURE`。

## 0912 PL bidirectional CUSUM frozen/dynamic admission amendment

本轮 byte-verify 上一轮 forward evidence 并复用其 sealed clean split/calibration，禁止重新切分或重算
forward threshold。backward calibration/held-out validation/injected/intersection 分属 truth-blind A--D
进程，support seal 后 E 才读 target/interval/30 IDs。Frozen B0--B4 固定要求 forward regression、backward
clean zero alarm/segment、intersection target precision/recall 各>=0.80 且 healthy segment=0。只有全过才
执行 always-commit dynamic CONTROL/SHADOW non-interference、clean、detection、support gates。首门失败
即封存并停止，不 adaptive rescue；production、Stage2/Rc/final/ATE/RMSE 均不运行。

0912 bidirectional admission 收口：B0--B4 全部通过；backward held-out clean 为 0 alarm/0 segment，
冻结交集 TP/FP/FN=30/0/0。随后按协议执行的 dynamic CONTROL/SHADOW scientific artifacts 全部
SHA-256 exact、最大 state 差 0；dynamic clean 为 0/0，dynamic target alarm 仍为 affected #15，最终
TP/FP/FN=30/0/0，healthy segment=0。唯一成功裁决为
`BIDIRECTIONAL_CUSUM_SUPPORT_PASS_FOR_PRODUCTION_ADMISSION`。production provider、Stage2/Rc/final、
recovery 和定位指标仍为 `NOT_RUN/NOT_EVALUATED`。
