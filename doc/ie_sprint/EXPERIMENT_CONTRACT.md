# UWB-IMU-IE 实验合同（T01）

## 2026-09-14 runner initialization-linearization migration audit

T04/T06/T08只允许在下列审计全部通过后迁移当前线性化哈希：raw/selected `obs_id`、
state timestamps/count、factor count/type/key sequence、UWB range/sigma/anchor/lever arm、IMU/PIM interval/duration以及
physical topology与历史合同一致；并且同一当前physical graph在历史pre-common reference Values上精确复现
旧Values和graph-linearization指纹。旧值保留为`PHYSICAL_GRAPH_IDENTITY`，新common-initializer Values及其
线性化指纹作为`INITIALIZATION_LINEARIZATION_IDENTITY`。任一物理语义不同则停止，不修改expected hash。

## 0913 DEV-DENSE-01 Walk1 state-density feasibility amendment

本轮是与正式 R0--R6 隔离的 development-only 可行性实验，只回答 Base FGO clean ATE 中
低频离散状态/UWB keyframe subsampling 的可能贡献。正式 Walk1 B0 的实际 step=4 记为
`D0_CURRENT`，另做 step=2/1；若实际值重复则不制造重复 variant。所有 variant 复用同一
Walk1 raw measurement cache、beta、anchor/lever、IMU/UWB noise、初始化、all-range Gaussian
graph、LM solver/budget/tolerance、同一已有 runner binary、measurement-only isolation 与 CPU/thread
policy。禁止 robust、reject、detector、recovery 和 fresh SFUISE，也禁止为高密度失败改变求解参数。

R1 `FROZEN_EVALUATOR` 的 Walk1 common-GT timestamps、nearest tolerance、scale-fixed SE(3)
alignment 与 1s RPE 定义保持；如任一成功输出不能覆盖完整冻结集合，则另取所有成功 variant 的
共同子集并显式报告，不能把输出点数差异写成精度收益。raw-frame provenance 未闭合，未对齐
坐标差只能作 diagnostic。每 variant 恰好一次完整 scientific run；runtime重复 NOT_RUN。
预登记解释阈值：material ATE improvement 同时要求 `>=0.010 m` 且 `>=5%`；obvious UWB increase
要求 used multiplier `>=1.5`；acceptable cost 要求 runtime `<=5x` 且可用时 peak RSS `<=3x`；
`>20x` runtime、solver failure 或 factor/temporal integrity failure 视为不可接受。PROMISING 要求
material+acceptable+integrity；MARGINAL 要求成功、used明显增加且改善 `>0.001 m`，或 material
但成本在5--20x；否则 NOT_WORTH_IT。该规则只裁决本开发实验，不迁移正式 backbone。

输出根固定为 `experiments/icra2027/dev/DENSE_STATE_WALK1/`，保存隔离 run、配置、命令、hash、
逐 observation/anchor utilization、逐 IMU interval 审计、solver status/iterations、estimator runtime、
total wall 和 GNU time peak RSS。正式 metrics/tables/figures/paper_assets 与 canonical manifest 不写。
实验结论不升级 C1--C3、T10/T11 或正式 R2/R3 状态；只将结果登记为 development evidence。

## 0913 R3–R6 连续执行 amendment

本轮用户冻结B0_CURRENT，授权三条ISAS clean及3×5个+1m/10s/constant单anchor ToA case，
并将offline treatment送入未改SFUISE；不运行B1/B2/B3或R7之后。沿用此前已完整暴露的Walk数据，
本轮formal指预登记受控benchmark，不把它们重新标为独立未见held-out，不声称physical NLOS。
用户本轮全部流程与停止条件优先。科学freeze后detector/recovery/solver/evaluator/injection均不调参。
SF三方法使用同一每recording官方config；仅支持集合的删除或final accepted range的减补偿，
不传GT/truth/oracle、c估计或support文件到SF。未接受的SF_RECOVER observation保持原样。
原final native raw factor不重复，导航/残差等仍取同final Values。downstream corrected range是声明的
measurement-treatment实验，不将它等同于原联合graph的后验，也不向SF添加bias prior/variance模型。
缺producer/cache/final的策略保留failed及dependency原因；普通failure继续所有其余case。
使用R1/R2锁定common GT sets和原score/association，fault窗口仅取同一全段SE3拟合后的误差子集。
R4预登记等价容差position1mm/rotation1mrad/ATE1mm。所有正式CSV采用用户method IDs与终态，
记录git/config/input/fingerprint，旧结果保护。R6结束分类并停止，无自动R7或参数迭代。

## 0913 R2.5 作者 static ToA values input-parity amendment

用户明确解除R2对本次B1 development对照的独立标定来源停止门；这不证明独立calibration provenance。
仅将官方升序anchor offsets的负值写入现有fixed_beta_by_link，三条Walk clean各fresh一次，B0/SF复用。
冻结R1/R2已有common GT timestamps、evaluator、alignment；任何缺失匹配都不能通过缩小共同集合补救。
仅fixed beta常量变化，raw/state schedule/topology/IMU/算法策略保持；不运行B2/B3/注入，不结果调参。
报告 delta=B1-B0，relative=100*(B1-B0)/B0；NA/失败保留，原始审计/指标文件不覆盖。
这些offsets是SFUISE作者配置提供的dataset-specific values；repository/paper没有证明它们来自
独立calibration recording，因此本实验用于input-parity analysis，而不宣称独立calibration provenance。

## 0913 ICRA v6 R0–R2 当前范围 amendment

本轮用户授权只执行 ISAS Walk1/2/3 clean provenance、frame/reference-point、static calibration 和
range-density 公平性审计。替代旧下一任务边界；历史合同/结果保留，T10=C2-C、T11=C、C1–C3不升级。
见 `../../experiments/icra2027/README.md` 与 `../../experiments/icra2027/audits/backbone_parity_report.md`。
R1 在原 nearest-GT≤0.02s 基础上，每个GT时刻保留最接近的一条estimate，再取比较方法的共同GT交集；
不插值estimate，不拟合时移/尺度。未知world/point外参时 raw-frame ATE 保持UNAVAILABLE，未对齐坐标
误差单列diagnostic，SE3 aligned ATE以tracker/body共点近似的development误差报告，拟合不回写estimator。
RPE采用共同GT时刻上1s后首个≤0.02s误差的配对和现有SE3相对位姿误差公式；未知tracker姿态外参限制保留。
官方static offset仅在独立标定来源准入后才能作为独立标定B1；来源未闭合时不得静默应用或宣称独立性。
B2不得为了利用不同timestamp的range而静默引入nearest-state近似/插值；更改kf_step会同时更改state rate，
不计为纯density收益。B3与R3之后工作保持NOT_RUN，等待本轮结果后的明确决定。所有输入GT隔离、配置/hash、
实际final factor计数、失败/未运行记录、源产物复用与新运行身份按本合同保留。

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

## 0912 own_vicon 15-31-28 single-recording flow amendment

本轮只用 `data/own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle.bag` 做一次完整
development 流程检查。输入为完整 header-stamp UWB/IMU；每个原始 node range 保留 message/range/observation
序号和稳定 `obs_id`。Livox acceleration 依现有 loader 固定乘 9.81 从 g 转为 specific force m/s²，gyro 为
rad/s，不用 GT 拟合轴、时移或 bias。四个 anchor 取同 bag 各静态 Vicon topic 中位位置，tag0、IMU 和 rig
采用零杆臂近似；moving tag Vicon 仅在 estimator artifacts 封存后由 evaluator 读取。

方法固定为一个 `pl_bidirectional_cusum` producer、共享 Stage2 cache 的 `suppress_all`/`lcb_fixed_full`、
`robust_cauchy` 和原生 `SFUISE-ToA`。冻结 Walk1 clean 参数、Cauchy scale 2.3849、SFUISE zero ToA offset；
最多5个科学进程树、1800s/树、无算法重试。评价沿用10 Hz、estimate 0.02s、GT bracket 0.05s、scale=1
SE3 和正误差窗口定义。该单条曾在 evaluator-only 审计中出现一个2.240277s正误差段，但几何仅
`USER_APPROXIMATE_COLOCATION`；流程成功不能升级为精度、NLOS truth、跨数据集或 Recover 优越性结论。

## 0912 PL CUSUM production integration / accuracy amendment

本轮按
[`PL_CUSUM_E2E_INTEGRATION_ACCURACY_PROTOCOL.md`](../ie_0911/PL_CUSUM_E2E_INTEGRATION_ACCURACY_PROTOCOL.md)
执行单一 locked Walk1 clean/+0.5 m injected development E2E。先 byte-verify prior seal，再要求 production
clean zero support/no-op 与 injected support 对 sealed dynamic shadow 在 obs/link/segment/alarm 上 exact；
任一失败按最早 verdict 停止，不重算 calibration 或调 detector。其后只复用当前冻结 Stage2、score、
gate、fixed compensation、final factor audit 与一次 fallback。

primary recovery 为 `lcb_fixed_full`，唯一 rejection comparator 为共享同一 support 的 `suppress_all`；
range evaluator 在 scientific artifacts seal 后按 obs_id 精确配对 30 条 clean/injected measurement，要求
recovered RMSE 严格改善。trajectory evaluator 复用 `evaluate_runs.py` 的 scale=1 SE(3) aligned ATE/RPE，
并在 common matched GT timestamps 上要求 primary RMSE 严格优于 suppress 且 p95 不差，才能给
`E2E_FULL_SYSTEM_PASS_DEVELOPMENT`。链路技术通过后执行 six-input non-gating diagnostic，不用其结果调参、
换 primary 或扩大 claim。所有失败、zero/fallback/unavailable 原样进入结果；本轮不是 formal held-out test。

## 0912 own_vicon initialization correctness amendment

用户授权在重跑同一 own-vicon LOS recording 前修复初始化的三个直接阻塞。Trilateration 的 LM 候选点
必须先计算同一 range least-squares 目标；仅有限且严格降低目标的步可以写回状态，拒绝步只提高 damping。
返回 `true` 必须对应有限解和数值收敛，迭代耗尽、非有限分解或未收敛返回 `false`，且不得发布未收敛点。

静止判定继续使用首2秒 acceleration norm variance 与 gravity magnitude，但方差阈值固定为当前
IMU 配置的 `sigma_a^2`，不再使用源码常数 `0.01 (m/s^2)^2`。这只是将既有判据绑定到预先配置的
IMU noise，不按本 recording 的残差或 GT 调整 `sigma_a`。

每次初始化从当前 anchor 几何自动判断是否近共面，不增加 recording 侧别配置。固定判断为中心化 anchor
最小/最大 singular-value ratio `<=1e-3`，且最佳拟合平面的法向与世界 Z 夹角不超过约25.8度
（`|n_z|>=0.9`）。满足时在平面内用首批原始 range 做差分最小二乘，再由非负法向距离平方的中位数生成
两侧候选；先选择实际3D raw-range平方目标严格较小的一侧。若两侧目标在数值容差内不可分，只在anchor
平面不穿过世界 `z=0` 且两候选到 `z=0` 距离可分时选择更接近 `z=0` 的一侧，作为由anchor坐标系给出的
非GT高度侧信息；仍不可分则明确失败。禁止读取 moving-tag Vicon、GT 高度、逐 recording 拟合高度、beta
或噪声。非近共面几何保持普通3D初始化。

工程测试和构建通过后，按
[`OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md`](../../experiments/OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md)
在新隔离目录重跑原四方法流程。CUSUM/LCB/Cauchy scale/SFUISE offset/评价/预算均沿用上一冻结协议，
无算法参数重试；保留全部失败、fallback、空 support 和负收益。

实际 fresh run `own-vicon-flow-initfix-20260912A` 通过工程门与锁核验。自动静止判定启用重力对齐；
anchor singular ratio `0.000325028` 触发2.5D，raw-range objective选择下侧，最终trilateration
`p0=[-0.0216184,-0.0852544,-0.526389]m`。raw reference收敛并发布成功状态，证明原初始化阻塞已解除。
producer随后在既有Stage2固定checkpoint机制耗尽总预算，未发布cache；253候选/8段仍被冻结，两个RR方法
依合同NOT_RUN、差值NA。Cauchy成功导出918 pose完整时域轨迹，ATE RMSE `0.951271317m`；SFUISE ATE
`0.297529563m`。本结果只支持初始化correctness与基础FGO可运行，不支持Stage2或Recover收益。

## 0913 REFACTOR-GATE-03 engineering-only amendment

本轮只实现和验证 `doc/v3/v3.md` Prompt 3 的 integrity/correlation 语义及既有鲁棒 baseline 的求解入口
保护，不运行正式数据、精度矩阵或科学比较，也不据结果调 robust scale。新的 input-plan identity 必须包含
integrity plan/hash，避免与 Gate02 或历史 cache 混用。`all_range` 继续是 plain-Gaussian 对照；
`robust_huber` / `robust_cauchy` 继续是独立方法，并沿用其显式 scale/provenance。任何历史 run、metric、
claim 或论文数字都不得重解释。Gate04 solver certificate 为 `NOT_RUN`，本轮仍不得把 generic optimizer
termination 升级为 scientific success。

## 0913 REFACTOR-GATE-04 engineering-only amendment

本轮只实现和验证 `doc/v3/v3.md` Prompt 4 的 solver-success 语义。Base FGO 与 final IE 的 run/artifact
必须同时记录原始 optimizer termination 和独立 `PAPER_SOLVER_CERTIFICATE_V1`；只有
`CERTIFIED_SUCCESS` 才能令 `valid_estimate_exported=true`。证书不得读取 GT/ATE/truth/oracle，也不使用
旧 D2 的 ATE 作为判据。若旧 D2 不能低成本重放，则只运行确定性 synthetic/unit regression；不运行正式
数据、dense rerun 或完整实验矩阵，不改变 LM、初始化、noise、robust kernel、NLOS 方法或阈值，不重解释
历史结果。certificate 语义改变使受影响的旧 success/cache 身份不能被静默当作新 certified evidence。

## 0913 REFACTOR-GATE-05 engineering-only amendment

本轮只执行 `doc/v3/v3.md` Prompt 5 的 behavior-preserving 结构清理和确定性工程回归。改造前后至少核对
raw/selected observation IDs、state/factor 数量、candidate/support/segment/group、Stage2 refit、score、
decision、trajectory/objective 与 `PAPER_SOLVER_CERTIFICATE_V1`；可使用既有 deterministic unit/integration
fixtures 与失败语义快照，不运行正式数据、dense-state 重跑、精度矩阵、sweep 或参数选择。运行时动态耗时
字段不作为逐字节相等要求，其余可稳定 artifact 优先作精确比较。

旧 YAML/CLI/artifact schema 继续可解析；任务卡、debug label、scientific lock 和 provenance 属于实验协议
边界，不得反向决定 estimator/IE core 数学。所有历史结果、metric、claim 与论文数字保持原解释，不因文件
重排而升级、作废或重算。Gate05 完成后必须停止，等待新的明确指令才可执行 Prompt 6。

## 0913 REFACTOR-GATE-05R-B test-fixture-only amendment

本轮只执行现有 T04/T06/T08 runner CTest 内的双子例合同恢复。test-only fixed-sigma 正例必须记录来源、
确定性、sigma、observation/state/UWB factor 数、support/candidate 与成功/certificate 条件；不得用 GT/ATE
调 fixture，不做参数 sweep。原 sim-circle 输入保持完整，只作为当前已诊断 fixed-sigma 数值失败的负例，
断言 fail-closed 语义且不依赖 wall-clock。验证顺序固定为单项 T04、T06、T08，三项组合，Gate05 focused
core，architecture guard，最后 full CTest；只有零失败才更新 Gate05 历史报告的追加 resolution。

不得修改 checked-in production config/solver/noise/threshold、不得运行正式数据矩阵或 dense/Prompt6、不得
重解释任何历史科学结果。fixture 和 contract 产物只支持软件行为，不支持论文 claim；不自动 commit/push。

## 0913 REFACTOR-GATE-06 single-run correctness amendment

本轮只允许一个 canonical SFUISE Walk1 clean 主运行：复用 DEV-DENSE-01 的同一 t07 cache、
anchor/extrinsic/IMU/solver 配置，只使用已存在的 step=1 配置，方法为 `robust_cauchy`、
scale=2.3849。运行前锁定 commit、dirty-worktree hash、runner/library/config/input/evaluator/GT hash 和单 cell
batch；一个方法一次 attempt，1800s 超时，不 retry、sweep、tuning 或 runtime replicate。可开启
`UIFGO_BASELINE_DIAGNOSTIC=1` 以完整记录 LM call/iteration 和 factor-error 诊断；该开关不参与
求解状态或停止判定，runtime 明确标记为 instrumented run。

估计器进程不接收 GT/truth/oracle；scientific artifacts 封存后才由现有 Walk1 evaluator 读取
既有 GT，使用冻结 common-GT timestamps 和 scale=1 SE(3) alignment 报 ATE RMSE/P50/P95。历史
D0 直接复用 DEV-DENSE-01 已封存数字，不重跑；对照必须标记 `OLD PIPELINE vs REPAIRED PIPELINE`，
不得作为受控 state-density ablation，也不复用旧 `DENSE_STATE_RECOMMENDATION` 裁决新系统。

运行前固定 correctness 判定：唯一 attempt 必须 exit 0、导出有效轨迹，optimizer termination
成功且 `PAPER_SOLVER_CERTIFICATE_V1/CERTIFIED_SUCCESS`；所有 final state/objective/UWB residual 有限；
state_step=1 下每个 selected correlation group 恰好一个 final UWB factor，不允许 stale repeat 倍增信息；
23.94m glitch 两条原始记录必须在 provenance 中，但只允许一个 likelihood，且任一 glitch
likelihood 的 Cauchy 贡献不得超过 total final objective 的 50%(“dominates”的严格多数定义)。
最大位置模、aligned ATE/P50/P95 作为物理合理性诊断完整报告，不增加 Gate04 证书未定义的
轨迹误差硬门。任一硬门失败则 verdict 为 `ALL_UWB_CORRECTNESS_FAIL`，只定位最窄机制并停止；
全部通过才能为 `ALL_UWB_CORRECTNESS_PASS`。无论结果均不推导“all UWB 更好/更差”或修改 paper claim。

## 0913 REFACTOR-GATE-06D diagnostic-only amendment

本轮只复用 Gate06 已锁定的单一 Walk1 clean step=1 cell 做 optimization-basin 归因，不读取 evaluator、
GT、ATE、truth 或 oracle，也不生成论文指标。先从首个 optimizer update 前的同一 graph/initial Values
导出状态、逐 UWB 关联、逐 anchor residual/weight 与 graph/frame/IMU interval 审计；发现关联或构图
correctness bug 即停止后续 solver 对照。

在无 correctness bug 的条件下，允许三个隔离 diagnostic replay：R0 既有 Cauchy `2.3849`、R1 既有
Huber `1.345`、R2 plain Gaussian。三者除 UWB loss wrapper 外的 ledger、usable/representative mask、
states、initial Values、factor topology、IMU、priors、sigma、solver 参数与 100-call budget 必须相同；
每个 replay 恰好一次，不 sweep、不 tuning、不 retry。未收敛 terminal Values 只作标明 uncertified 的
诊断，不能导出有效估计。可选 larger-budget Cauchy 本轮默认 `NOT_RUN`，只有在前三项仍不能归因时才可
一次性执行，且不得写回配置或解释为生产预算修复。最终只形成
`docs/refactor/ALL_UWB_OPTIMIZATION_DIAGNOSIS.md`、隔离机器可读证据和 `STATUS.md` 记录；不修改生产算法、
scientific result、Gate06 verdict 或 C1--C3 claim。

## 0913 REFACTOR-GATE-06R-A initialization-only engineering amendment

本轮只实现并验证因果 UWB-aided progressive common initializer。配置只新增一个
`initialization.progression_horizon_s=0.25` 秒工程参数；不得依据 GT、ATE、最终残差或本轮输出调整，
也不得 sweep。单个局部块仅允许 METHOD_CONTRACT 登记的一次同图、同参数 continuation，并须计入
retry；不得做第二次 retry。确定性测试必须覆盖：future measurement 不影响已完成 prefix、ledger 与独立
selected obs IDs 不变、实际 UWB factor 与固定 sigma 不变、prefix 失败无整段 open-loop fallback、局部
drift fixture 被周期性 UWB 校正、给定相同 returned Initial Values 时最终图构建不变，以及 Gate05
architecture guard。

工程门后只允许重建 Gate06D 同一 SFUISE Walk1 clean、`state_step=1`、all-usable correlation
representatives、fixed `0.15 m` sigma 的初始化，并在无 GT/truth/oracle 环境中审计 state/position/velocity/
displacement、raw/standardized UWB residual、最终 Cauchy `2.3849` 将看到的 weight、per-anchor 分布、
finite、prefix/failure/retry/runtime。该运行到共同 Initial Values 即停止；Base FGO final LM、certificate、
ATE/evaluator、Stage1/Stage2/final IE 全部 `NOT_RUN`。PASS 不使用 ATE，且要求消除原百至千米 open-loop
tail、全段有限、最终 Cauchy 不再几乎沉默全图、全部定向/core/architecture 回归通过。结果只写
`docs/refactor/COMMON_INITIALIZATION_REPAIR_REPORT.md` 与隔离 engineering evidence，不覆盖 Gate06/D
旧证据，不升级 C1--C3，不自动 commit/push。

实施期勘误记录：原预登记 `1.0 s` 在 synthetic drift fixture 的第二局部块耗尽 100-call LM 预算；未接触
Walk1/GT/ATE 前，仅一次收紧为上述 `0.25 s` 并冻结，不继续试档、不依据后续初始化 audit 调整。

## 0914 REFACTOR-GATE-06R-D diagnostic-only amendment

本轮只运行同一锁定 SFUISE Walk1 clean step1 的 frontier 45--51 初始化诊断。独立 diagnostic executable
可保存局部 state/UWB/LM/Jacobian 证据；不得由生产入口消费或导出 trajectory。顺序固定为：重放并审计
45--50 趋势；审计51首块和既有 continuation；从其 exact failed terminal 运行有界 identical continuation
诊断；最后运行唯一 `0.25 s` causal joint-window comparison。额外 continuation 不是 retry 参数实验，窗口
不是 sweep；所有输出目录隔离且记录命令、输入/config、`gt_truth_oracle_read=false`。

若相同 continuation 越过原 `1e-5`，只按任务卡判断是否满足 marginal cliff；若 joint window 改善，仍不
自动授权生产改动。Gate06 final、GT/ATE/evaluator、IE stages 全部 `NOT_RUN`。最终只写
`docs/refactor/COMMON_INITIALIZATION_FRONTIER51_DIAGNOSIS.md`、机器证据、STATUS/claim-evidence，并以
`GATE06R_D_ROOT_CAUSE_IDENTIFIED` 或 `GATE06R_D_UNRESOLVED` 停止。

## 0914 REFACTOR-GATE-06R-E fixed-lag repair amendment

本轮只实现 common initialization 的 bounded fixed-lag formulation 和确定性工程测试；唯一真实输入运行
是同一锁定 SFUISE Walk1、state_step=1、全部 estimator-usable UWB representatives 的 initialization-only
audit。lag 固定沿用 0.25s，不调 threshold/retry/Huber/sigma/LM，不读取 GT/ATE，不运行 Base-FGO final
solve、certificate 或 IE stages。测试必须覆盖 frontier future invariance、lag 外 state immutable、one-state
失败而 fixed-lag 同参数通过、ledger/selected/sigma/final graph 不变、失败无 full Values、local graph 不改
physical graph、无 future factor 与 Gate05 architecture guard。

Walk1 只允许一次完整 audit；若任一后续 frontier 失败立即停止且不得调参。PASS 必须返回全部913 states、
每窗通过原 `1e-5` 资格、全状态有限、无百至千米 open-loop residual tail、final Cauchy seed 不再几乎全部
沉默，并通过 initializer/core/architecture 回归。输出隔离目录与
`docs/refactor/COMMON_INITIALIZATION_FIXED_LAG_REPORT.md`；PASS 仅解锁 Gate06 下一独立 Prompt，不能在本轮
执行 Gate06 或升级论文 claim。

## 0914 IE-CORE-STABILIZATION-FINAL closure amendment

本轮授权以同一锁定 SFUISE Walk1 clean、state_step=1、全部2276 estimator-usable independent UWB
representatives、固定0.15m sigma与相同physical graph完成一次 full initialization。局部非驻点不再单独STOP，
但结构/数值/catastrophic local seed失败仍立即停止；全序列只执行一次无GT seed-quality gate并完整输出状态、
窗口termination/stationarity、UWB residual/q、production-Cauchy weight与per-anchor统计。不得用Walk1 ATE调
seed gate、lag、Huber、sigma、LM、retry或任何参数。

若full seed gate失败即STOP；若通过，自动运行原Gate06 unchanged Cauchy。仅当该solve未获未修改的
Solver Certificate成功时，授权恰好一次 Huber1.345 warm-start→unchanged Cauchy final fallback，两阶段仍
无GT且预算/图/测量完全相同。只有certified trajectory导出后现有evaluator才可读取GT计算ATE/P50/P95，
不得据此改参数。工程测试、core regression、Gate05 architecture和full CTest必须先通过；任何回归失败STOP。
最终只交付 `docs/refactor/IE_CORE_STABILIZATION_FINAL_REPORT.md`，不启动IE实验、不commit/push。

## 0914 FIXED_LAG_BOUNDARY_SEMANTICS_FIX execution amendment

本轮仅修复fixed-lag boundary/free-state选择语义，不改0.25s、fixture/state cadence或任何数值/科学参数。
必须以确定性测试覆盖稀疏1s cadence退化为previous fixed boundary + current free state、Walk1-like dense
multi-state free suffix、lag外不可重开、无future factor、跨旧boundary物理IMU bridge，以及缺失/非法IMU
interval fail closed。依次要求initializer、T04/T06/T08、core、Gate05 architecture与full CTest全部通过。

只有全部回归零失败才执行同一锁定Walk1一次913-state initialization；full truth-free seed gate通过才继续
原Gate06 Cauchy，必要时仅一次既授权Huber1.345→unchanged Cauchy。只有未修改Solver Certificate成功并
导出trajectory后才可评价GT；不得调参或自动追加micro-fix。新结构/数值失败立即STOP，交付
`docs/refactor/FIXED_LAG_BOUNDARY_SEMANTICS_FIX_REPORT.md`，不运行IE实验、不commit/push。

## 0914 COMPLETE_HUBER_TO_CAUCHY_WARM_START execution amendment

本轮先以确定性测试证明：未认证但有限的 Huber terminal 可仅作中间种子，不可导出或
评价 GT；非有限/非法 key/灾难性 terminal 被拒绝；Cauchy final 仍须原 certificate；
initializer 与 physical graph 不变。依次运行 focused、initializer、T04/T06/T08、core、Gate05
architecture 与 full CTest，任一失败即停止 scientific run。

全部回归通过后，仅运行一次锁定 SFUISE Walk1 913-state chain：已资格化初始种子 →
Huber 1.345 → truth-free intermediate gate → unchanged Cauchy 2.3849 → 原 Solver Certificate。
三阶段独立记录 objective/state/residual/q/weight/termination/stationarity/finite 证据。Huber
certificate 失败可与 intermediate gate PASS 并存，但 GT 必须保持未读；只有 Cauchy
certificate 成功并导出有效 trajectory 后才运行既有 evaluator。若 final Cauchy 失败，
精确记录首个最终数值失败并 STOP，不调参、不追加 fallback/micro-fix。

## 0914 BASE_FGO_THREE_CELL_STABILITY_AUDIT amendment

用户新授权在不运行 IE 的边界内执行三个隔离 Base-FGO cell：冻结 sim-circle clean
fixture、SFUISE Walk1 `keyframe.step=4`、SFUISE Walk1 `keyframe.step=1`。三者统一复用当前
common initializer、Huber 1.345 intermediate gate、unchanged Cauchy 2.3849 与原 Solver Certificate；
每个 cell 只运行一次，不调整 state step 外任何冻结 config，不增 budget/retry/fallback，不读
GT/不评价ATE，不运行 Stage1--4。Simulation 保留 fixture 的 sigma=0.1m，Walk1 保留
sigma=0.15m。“稳定工作”仅当 Cauchy final raw solve 有有限 terminal 且原 certificate 成功；
raw termination、intermediate gate、certificate、trajectory export 分别报告，不以一次通过声称统计可靠性。
