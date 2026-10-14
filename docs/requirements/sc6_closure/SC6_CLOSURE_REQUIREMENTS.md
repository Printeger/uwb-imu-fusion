# SC6 Closure — 需求与验收规范

**副标题：故障恢复、全状态精度、FDE 收益、Stanford 完整性图和 P95 实时性**  
**性质：下一轮开发合同，不是现有功能的通过声明。**  
**版本：SC6-CLOSURE-R1 / 2026-10-14**

| 字段 | 约束 |
|---|---|
| 仓库/分支 | `Printeger/uwb-imu-fusion` / `feature/realtime-uwb-imu-pl` |
| 文档编写时核对的远端 HEAD | `6b780101e6ff61cf614df1b457111502e0248d4e`（2026-10-14，`chore: ignore SC6 direct-run artifacts`） |
| 上游实际计算/结果 | SC6 报告中冻结计算版本 `adab2da91a1448422c3982ef3dd5500cd16c6405`；SC6 本地结果交付提交 `984f9cc58c41d2a6fde402740fd18451c68703e0`。以启动时本地 `git rev-parse HEAD` 为准，不擅自覆盖用户本地新提交。 |
| 权威历史证据 | `docs/fde_next/SC6_RESULTS.md`，并保留 SC5/SC1–SC4 既有回归；新一轮不篡改旧结论 |
| 新结果 | `docs/fde_next/SC6_CLOSURE_RESULTS.md`，一个持续维护的结论文件 |
| 启动方式 | 一个连续 Goal，内部 W0→W1→W2→W3；每个工作包完成真实开发、必要测试和本地 commit 后继续 |
| 新实时判据 | **P95(core) ≤ 40 ms；P95(measurement-to-sink) ≤ 50 ms**；不再要求零超时；超时/拒绝/排队和可用率必须保留 |
| 新必交成果 | 同原始输入的 FDE ON/OFF 成对精度对照；水平/垂直 Stanford Plot + 区域表；D1/晚期 IMU 漏检机制；全状态与资源/实时复验 |
| 保持不变 | 源噪声、原始独立来源及共享 ADC、时间一致风险、任务运动域身份、实际 FGO/单次原生提交、生产 AL/发布开关和原故障覆盖 |
| 不宣称 | 真实硬件/应用 AL/正式 HMI 资格、任意弱故障都可检测、全部15D可观测、无限资源/WCET、P95 即严格每帧实时 |

## 0. 执行摘要：当前证据及为什么需要 Closure

SC6 已完成 W0–W4 的代码与仿真，结果是 `SC6_EVALUATED_WITH_GAPS`。本 Closure **不是重做 SC6**，也不是再次实现滚动证书：利用现有 `Sc6Service`、`Sc6Provider`、`Sc6Result`、`epoch_raim_runner`、`sc6_system_research.py`，完成两个新增导师可验收的成果（成对 FDE 精度 + Stanford 图）并有依据地收敛原先未通过的功能缺口。

**冻结的上游事实（不是新成果）：**

- S0 健康 tag RMSE：OFF/ON `0.0500571/0.0500571 m`。
- S1 故障：OFF `0.22139 m`、ON `0.090906 m`；相对下降 **58.94%**。
- S2 故障：OFF `0.224942 m`、ON `0.102563 m`；相对下降 **54.40%**。
- S2 姿态 RMSE：OFF `1.74072 rad`、ON `2.21514 rad`，**FDE ON 恶化**；不能以位置收益代替全状态收益。
- L0 5000：`5000/5000` raw 有效/按时，core P95 `21.229 ms`，sink P95 `21.411 ms`；11 次 core 超40、0次 sink 超50。
- L1 10000：`10000/10000` raw 有效、`9996/10000` 按时，core P95 `21.497 ms`，sink P95 `21.641 ms`；23次 core 超40、4次 sink 超50。
- 40/40 250：core P95 `31.600 ms`、sink P95 `31.761 ms`；3次 core、1次 sink 超时。
- D0 六轴早期强故障真实修复有正例；D1 gyro 和晚期固定强度故障多次漏检、没有当次原生 bridge/prior-repair/rejoin；弱故障不保证可检。
- S3/G2 弱垂直几何的实际功能候选/数值求解拒绝：计划300，原生成功0；不能将其解读为已证明物理可行集合为空。
- L1 第10000历元原生 factor slots `10034`；主 raw bias 某轴区间宽度约 `128.30 m/s²`；主 bias 仅 RW 外扩，没有可发表的全故障观测收缩证明。
- SC6 新运动学与测距收缩有原始共同事件下的包含论证；700/700同一输出中心 H/V 严格收紧；P口径缓存优化在配对前缀状态/行动/PL/风险/ledger 一致。

引用事实以当前仓库 `docs/fde_next/SC6_RESULTS.md` 为准。**SC6 曾以全部帧40/50ms严格满足为目标而留下 A18 OPEN；本轮提出独立的新 P95 验收合同，不回写或改写原来实验的历史结论。**

## 1. 研究范围、边界及不可互换的结论

**一个系统：**仍使用当前增量 GTSAM FGO，UWB/IMU 真正的传感器数据（在仿真中生成）、raw 物理故障池、联合 FDE、受污染历史 prior 的实际恢复、单次原生 commit、与 committed tag 绑定的滚动全状态集合及时间一致风险。不能为了比较 ON/OFF 在 FDE OFF 运行一条与 ON 完全不同的估计器或缺失 PL 的简化程序。

研究合同必须显式标记 `D0/G0`、`D1/G1`、退化 `G2`；风险仍为预注册 `epsilon=1e-6` 的条件性时间一致 source 事件，ADC 共享出生 ID、bias RW 和故障残留不可清零。允许每时刻至多一个 UWB 单anchor episode 和一个 IMU 单轴 episode；UWB/IMU 可以同时活动，类别内部仍受原并发合同限制。改动故障覆盖或运动域需要另立版本，不能暗中替换旧结论。

**互相独立的五项验收：**

1. `FDE_ACCURACY_BENEFIT_EVALUATED`：配对实验完整且计算正确；**不要求 ON 在每种指标/场景都优于 OFF**。
2. `STANFORD_INTEGRITY_EVALUATED`：真实逐历元 H/V 图、MI/候选HMI、条件性可用率与拒绝分母一致。
3. `REALTIME_P95_TARGET_MET_FOR_TESTED_PROFILES`：全部预注册的**正例必验** paced 场景两个 P95 均达标，同时缺失/拒绝与及时有效比例另行合格；不要求零超时。
4. `RECOVERY_OBSERVABILITY_RESOLVED_FOR_SCOPE`：D1/晚期/姿态缺口有机制诊断；确有必要信息时实现并验证恢复；不可辨识时给出合法反例/范围限制，**不冒充 FDE 成功**。
5. `RESOURCE_BEHAVIOR_EVALUATED`：slot、ledger、告警、RSS、bias 区间、P95 的长流依赖与处置有实测和边界；只有经真实安全映射证明才可授予有界资源资格。

全部文件、实验、分析均应服务于以上能力；不额外开展无关的全架构重写。

## 2. W0 — 冻结配对评估协议、重新定义 P95、生成 Stanford 工具

### 2.1 FDE ON/OFF 必须是“同一份原始输入”

至少将健康 S0、联合 accel S1、联合 gyro S2 及一条 D1（含强/弱或晚期）纳入配对实验。S0/S1/S2 主比较预注册 **5 个种子**：`20261010,20261011,20261012,20261013,20261014`；如果旧仿真器对任何种子确实无法构造对应场景，记录实际阻断并在观察新指标前冻结替代方案，禁止挑选有利种子。其他六轴/窗口/长流采取小而足的互补矩阵，避免全因素笛卡尔积。

ON、OFF 必须固定完全相同的：测量序列与时间戳，真实初始化观测，IMU原始ADC/Bias-RW随机源，UWB每条 range，故障注入起点/持续时间/幅值、anchor几何、运动域、seed、FGO lag、raw PL算法/风险事件及AL。对原始输入（至少逐测量值、ID、source 出生块、stamp、注入清单）计算加密摘要且 bit-equal；仅允许预注册的 `fde_enabled` 策略差异。**ON/OFF 都以同一共享 `Sc6Service` 和同一 paced 200Hz/20Hz外壳运行**；FDE OFF 只关闭功能排除/bridge/prior-repair/rejoin行动，保留实际FD诊断、名义优化、raw PL、时效门控和风险资格，并说明现有代码的任何限制。不要用 ON paced vs OFF offline 的延迟作对照。

原始 truth/injection labels 仅供独立评分/审计，不得反馈到 FGO、监测器、动作选择或 PL 收缩。改变未来的 truth、故障标签、源数据，不得改变当前输出。若某一分支拒绝、崩溃或因域外无法得到合法 PL，保留计划帧和 paired-missing，不从配对表删除；速度/姿态等评分仍从实际 committed 末态读取，禁止用回溯平滑状态替代。

**定义**：位置 RMSE `sqrt(mean_k ||p_tag_hat(k)-p_tag_truth(k)||²)`；水平/垂直 RMSE 同理；姿态误差 `||Log(R_true^T R_hat)||`（SO(3) 0到π，检查坐标系与四元数符号）；速度和各轴 accel/gyro bias 原单位逐轴与向量分别报告。按相同有效配对索引、相同坐标系与时间戳评分；同时分别输出 ON/OFF 全部分母的独立结果。

每一对运行至少报告：

- 绝对 RMSE ON/OFF；差值 `Δ=RMSE_OFF−RMSE_ON`（m）；相对改善 `% =100 Δ/RMSE_OFF`（若 OFF=0，填 NA 而不是无穷或0）。
- APE/RPE1/RPE20（含有效pair分母）、H/V、速度、姿态、accel/gyro bias 的 RMSE、P95、max；FDE 带来的**副作用和退化必须显示**。
- 故障阶段：`healthy_before`、`fault_active`、`detected_not_isolated`、`quarantine/bridge`、`post_rejoin`、`late_unrecovered`；分段按真实测量/receipt与故障manifest对齐，允许某些阶段无样本并填 NA；重叠时定义固定唯一优先级或并列标签并分别说明，不重复计入整体 RMSE。
- 每一物理故障 episode 的 onset、第一报警、第一合法隔离、真正 prior 移除、实际 bridge、IMU rejoin，以及 TTD/TTI/TTR。每个分母区分：故障计划数、可检测资格数、告警数、真正恢复数和拒绝数；不得把未检测登记为恢复成功。
- 既有 SC6 S0/S1/S2 数字作为**历史参考**；新五种子结果才是 Closure 的成对证据，不要拿历史不同计时或版本当新配对。

**跨种子统计：**按场景先计算每个种子成对的 RMSE 与绝对/相对差，再以“独立种子/场景”为统计单位汇总。给出中位数、均值、范围；样本量允许时使用按种子重采样的描述性95%区间，显式 N=5 与不确定性；不得将一条10000帧时间序列当10000个独立实验。任何跨场景汇总须提前注册权重（默认**各场景等权**），不能只保留改善场景。报告包含完整负收益案例，禁止选择性展示。

### 2.2 Stanford Diagram — 两张正式图＋不可作假的分母

参考 ESA Navipedia 的 *The Stanford–ESA Integrity Diagram: Focusing on SBAS Integrity*：`https://gssc.esa.int/navipedia/index.php/The_Stanford_%E2%80%93_ESA_Integrity_Diagram:_Focusing_on_SBAS_Integrity`。**本轮实现经典单实际输出 Stanford Plot，而非声称实现 Stanford–ESA 的 all-geometries 枚举方法。**

每个时间/状态绑定的观测点使用**实际原生提交 UWB tag**和独立真值，且空间集合保护的就是该点：

`HPE_k = hypot(tag_x_hat-tag_x_true,tag_y_hat-tag_y_true)`；`VPE_k = abs(tag_z_hat-tag_z_true)`。

水平图 x=`HPE`、y=`HPL`；垂直图 x=`VPE`、y=`VPL`。分别给出 FDE ON/OFF **同尺度分面**（每张图两子面），1:1线 `PL=PE`、x=AL、y=AL；故障/健康阶段与种子有可辨识符号或图例。短流展示逐点散点；长流密集可用2D直方图、对数计数热图，并在正文说明bin、颜色与溢出/裁剪计数。图须脚本从不可变逐历元trace生成，禁止用RMSE/平均PL合成散点，也禁止删除超界点/超范围点使图更好看。图示 `AL` 为研究诊断阈值，不是实际应用规范。

研究AL敏感性分别对 `(HAL,VAL)=(2,3),(5,5),(10,10)m` 统计；正式图默认(5,5)m，保留其它阈值表。**阈值变更仅为输出后评分**，不得影响在线决策、故障选择、状态、PL、源风险；有 noninterference 对照。

**单轴诊断分区（明确边界等号规则）**：对 `e=PE≥0, p=PL≥0, a=AL>0`，只有真实 `spatial_qualified=true`、未过期、同stamp/receipt、合法 finite 的点才进入 Stanford 图，另报门控：

| 诊断集合 | 规则 | 解释 |
|---|---|---|
| `covered` | `e≤p` | 保护界在该次实测中覆盖误差，不代表概率已被实测证明 |
| `underbound` | `e>p` | 误差超过PL，必须统计，包括 `p>a` 时的不可用区域 |
| `AL_admissible` | `p≤a` | **仅假设应用采用a时的数值准入**，不代表生产已发布 |
| `nominal_admissible` | `p≤a && e≤p` | 空间覆盖且低于AL；还需时间/来源门控才能称诊断可用 |
| `MI_candidate` | `p≤a && p<e≤a` | 在“按该AL宣告可用”反事实假设下，非危险的误导信息区域 |
| `HMI_candidate` | `p≤a && e>a` | 在同一反事实可用假设下的危险误导区域 |
| `AL_unavailable` | `p>a` | 即使位置误差很小，也不得按该AL宣称可用；其中仍单独统计 `e>p` |
| `not_current` | 未资格/拒绝/过期/超时/丢失 | **不作为可用样本**，空间有效但超时可另绘灰色参考层、不能混进及时可用率 |

> `MI_candidate` / `HMI_candidate` 是**Stanford 几何诊断分区**。本项目当前 `publication_protected=0`、`APPLICATION_AL_UNQUALIFIED`，不能把它们写成运营上实际发生的 MI/HMI 事故或已获得的正式安全风险资格。`operational HMI` 在未具备发布/应用合同前填 **NOT_QUALIFIED**，而不是通过展示零次证明安全。统计上 `underbound` 与 `AL_unavailable` 可交叉，不能把有重叠的集合相加凑总分母；互斥分区则 `nominal_admissible, MI_candidate, HMI_candidate, AL_unavailable`。对每轴、每seed、每阶段分别列出。

每张图必须附与实验manifest对账的表：`scheduled, received, attempted, committed, spatial_qualified, time_current, refused, expired, delayed, plot_points, excluded_from_plot, covered, underbound, AL_admissible, timely_AL_admissible, MI_candidate, HMI_candidate, AL_unavailable`。检查互斥分区总和、两轴同源的receipt/source/epoch；不同 AL 都使用同一数值轨迹。**零观测 underbound/HMI 不能代替原来 `epsilon=1e-6` 的数学定理。**

**下沉生成物（必需）**：
`docs/fde_next/sc6_closure_horizontal_stanford.png`、`docs/fde_next/sc6_closure_vertical_stanford.png`、`docs/fde_next/sc6_closure_fde_accuracy.png`（分辨率适于导师汇报/论文；可选矢量 SVG）。图中文字包括单位、图例、风险/来源限定、场景/种子、N和AL；如有未完成配对必须醒目显示而不是画正常图。

### 2.3 实时标准变更（必须写成新合同，不改旧 A18）

将本轮**正例必验 paced**场景固定为：原 S0/S1/S2 ON/OFF（同线程/同平台/同输入），D0 六轴代表与D1晚期代表，20/20、30/10、40/40，以及最终 L0 5000/L1 10000。禁止事后剔除慢轨迹或切到 offline 通过；排队测试按正例和受控负例分别报告。

`core_ms` 指一次在线已接纳服务从其声明起始至核心处理/必要源证据/真实 FGO+FDE+raw proof+唯一commit及清理完成；`measurement_to_sink_ms` 从与传感器stamp映射的释放参考时刻到真正结果接收端交付（含排队）。非在线GT/scorer/I/O 独立报告；本应在线做的数值认证不可挪到服务前隐藏。确保 ON/OFF 使用同一归一化计时边界和硬件配置，记录冷启动、CPU 线程、构建flags、WSL2/负载以及来源复制代价。

采用**nearest-rank P95**：将非负有限时延升序，`P95 = sorted[ceil(.95*N)-1]`；N是该测试中所有**实际开始且具有可测时延的已准入服务**，必须包括慢帧与超时帧；不能只对 `timely=1` 或 `PL_valid=1` 的行算。对于拒绝/未尝试/无sink样本另设分母和拒绝原因，不能把它们变成0ms或者直接删除使 P95 通过。报告 P50/P95/P99/max、`core>40`及`sink>50`计数/率、最大连续积压、队列峰值、gap/cancel/overload、按时资格比。

**双门槛**：

1. 全部注册正例profile分别满足 `P95(core)≤40ms` 和 `P95(measurement_to_sink)≤50ms`；
2. 每个注册正例profile满足 `timely_spatial_qualified / scheduled ≥ 0.95`，且不允许把源合同被篡改、实际后端未提交或缺失数据的失败误归为合法保护。剩余≤5%的拒绝/超时也必须全量报告。特定预设负例（cancel、overload、退化几何）不强求此正例比率，而以正确拒绝、无陈旧输出为正例行为。

此次阈值是**研发验收合同**，并不等价于应用要求、安全认证或WCET；禁止将旧 SC6 `REALTIME_TARGET_MET_FOR_TESTED_PROFILES=OPEN` 改写为历史 PASS。新能力仅叫 `REALTIME_P95_TARGET_MET_FOR_TESTED_PROFILES`。

## 3. W1 — D1/晚期故障、姿态与 bias 的可观测性及原生恢复

本轮优先回答 **为什么早期强故障可以被真实隔离，而D1 gyro/相同幅值晚期故障失去功能可检测性**；不能仅靠更低的报警阈值、改变强故障幅值或借 GT 选择候选获得成功。

### 3.1 固定正例与反例

用相同 source law/运动域/配置，至少包括：D0 六轴既有早期强故障与不同种子复验；D1 加速度和陀螺**同幅值早/晚**配对（含 L1 的 121、1121、3121、5121、7121、9121 注册时间）；弱50ms与500ms、交错/重复、UWB+IMU联合、超 replay 的真实恢复拒绝。故障强度沿用SC6预注册manifest；报告每次实际 onset、是否数学scope内、首FD、隔离、bridge、prior替换、rejoin和已声明迟滞窗口。若数据生成器针对某场景隐含前瞻认证或使用其他幅值，先记录并修复协议，不偷换被测系统。

### 3.2 因果诊断矩阵（必要，不能只写“未检出”）

对每次漏检，保存可诊断的最小证据（不需要全部raw长流备份）：

- 当前 IMU 6轴实测 vs 在全局birth-q、偏置RW、预注册 A/Omega 下的健康必要区间；旧/新 q、区间宽度、离检测阈值的margin；
- 物理故障自动机的可行模式、`no_active`、`active`/结束残留、潜在幅值区间与排除原因；
- 同期 UWB 几何、测距壳约束、联合位置/速度/姿态/bias集合，FGO实际状态及其版本；
- UWB功能FD与IMU功能FD分别的首报警时间及关联偏差；
- 是否有足够回放材料、本次是否保留/替换受污染 prior、后端为何选择 KEEP/bridge/rejoin。

分别判定 `DETECTABLE_AND_MISSED_BY_IMPLEMENTATION`、`DETECTABLE_BY_NEW_CERTIFIED_RELATION`、`NOT_IDENTIFIABLE_UNDER_DECLARED_LAW`、`NUMERICAL_INDETERMINATE`、`OUTSIDE_SCOPE`。不能仅以未检测到故障就证明不可辨识；若声称不可辨识，给出**两种合法真值/故障历史产生同一或重叠可观察测量集合**的构造或可验证必要关系可行解，且保留噪声、初始、物理源、共享bias与故障合同。

### 3.3 数学和恢复开发

必须实际调查/原型验证跨历元 UWB 约束对 `p,v,R,b_acc,b_gyro` 的信息是否足够，检查健康 IMU 与含故障IMU两种分支：

- 可认证的收缩器 `C(X)` 要求 `F∩X⊆C(X)⊆X`（包含所有合法fault branches，外包络舍入，风险不重新抽签），且对滚动separator可归纳传递；
- 健康分支能收紧、含任意幅值故障分支不能收紧时，可以继续保持分支或取其保守并集；不能以“FDE选了健康”直接从最终raw PL中删除故障分支；
- 仿真/独立 oracle 用小规模线性可观测性、SO(3) 姿态/杆臂多分支、弱UWB垂直/镜像几何、共享bias漂移做正负对照；禁止将小模型可观测性直接泛化为整系统资格；
- 如果成熟，进入主raw边界并给必要条件/相关性证明，记录 bias/姿态/velocity收缩与新的PL；否则保留原安全fallback和 `MAIN_BIAS_OBSERVATION_CONTRACTION_OPEN`，但必须交付实际反例与原型，不准无操作地跳过。

**姿态专项**：针对 S2 ON≈2.215rad、OFF≈1.741rad，以及机动长流旋转偏差，核对 quaternion sign、body/world frame、`SO(3)` log误差、可观测rank、FGO初始化、IMU bridge、prior替换优化metric如何影响实际姿态轨迹。需要区分“真实可观测性不足”“功能恢复数值副作用”“评分坐标错误”。有可安全优化之处做 ON/OFF A 类闭环配对；不得牺牲位置保护或把全状态协方差当raw证据。

**G2专项**：调查首个原生候选数值拒绝与多分支、初值、秩、镜像，设定独立几何oracle。能安全求出并保护才授予有限可用性；没有合法唯一解或数值无法保证时明确 fail closed，不用GT选择镜像。原计划300/0不得从SC6历史记录抹去。

**真正 FDE 成功** 必须具有后端真实唯一 commit、factor ledger、删去受污染prior（有必要时）、bridge和IMU重新接入的原始receipt。单独 PL 覆盖错动作不是 FDE 恢复成功。若同一幅值在晚期物理上不可辨识，本轮允许科学上的 `DETECTABILITY_LIMIT_PROVED`，但相关场景 `RECOVERY_VERIFIED` 仍为否，不能把拒绝等同功能修复。

## 4. W2 — 资源收敛与 P95 防退化

优先解决长期增长而非追求严格零超时：

- 已知 GTSAM `findUnusedFactorSlots=true`，但 `marginalizeLeaves` 新插入的边界prior仍可能增长空slot。调查真正源头；若实现压缩/安全回收，要逐项核对 `factorLedger`、`group_to_slots`、`removed_factor_slots`、boundary prior来源、候选数值与真实 commit；不可擅自重新编号旧slot又让后续误删。
- 与固定时间风险和故障历史相关的 alert records/因果来源身份不能按内存方便随意清零。研究完整性保存身份压缩或容量拒绝；如暂不解决 O(T)，必须报告速度、容量触发条件与保持保护的退路，不能声称恒定内存。
- bias集合可能随 RW 累积增长，本身有物理原因；不要靠截断区间/虚构独立先验换取长期稳定。其增长应与故障检测能力、P95、PL和潜在可观测性记录关联。
- 保留 SC6 `P` 型性能对照：固定数学/功能策略时，对健康和联合故障的全部 state/action/receipt/PL/source/risk/ledger 做语义相等或明确数值容差校验。若算法确实改变 FGO 状态，使用 `A` 配对而非声称纯计算等价。
- 所有新 profiler 在paced线程/冻结平台上测量40/40慢路径、故障高峰、prior替换与长流后期；禁止为通过 P95 移走必要在线工作、加大窗口资源上限却不计成本、取消原 fault bank 类别。

**W2完成不强制证明全时刻 slot O(1)**；但要明确已证的有限测试范围及未证的长期资格。任何 code-level整理必须运行真正的历史故障prior替换及超replay拒绝，不可仅看健康场景。

## 5. W3 — 最终冻结、复验和论文级结果交付

冻结：工作树、提交、源码和二进制SHA、配置、随机种子、运动/几何/故障manifest、在线证据模型、构建flags、平台、时钟及数学版本。W0–W2 中影响上游 SC1–SC6 核心证明的改动须跑直接相关原回归，再在最终候选冻结后完成最后一轮面向论文的配对与 paced 结果。长流只在冻结后执行；不要尝试多个版本挑最快一次。老报告、失败尝试、晚期漏检和超时都保留完整分母。

**最终最小矩阵（不做笛卡尔积）：**

| 类别 | 预注册必验 | 目的 |
|---|---|---|
| P0 | `S0/S1/S2 × 5 seeds × FDE ON/OFF`，相同paced实现与原始输入 | 主要 FDE 收益、姿态副作用、Stanford成对图 |
| P1 | D0 六轴每轴早期强故障（至少主种子）＋一个重复混合故障正例 | 原生恢复与跨类别故障覆盖 |
| P2 | D1/G1 gyro早期＋同强度晚期、accel早期＋晚期；弱/短各一，超replay一个 | 可检测范围与晚期恢复/不可辨识 |
| P3 | 20/20、30/10、40/40（主要有故障正例）＋S3/G2诚实负例 | 窗口与几何可用性；P95 |
| P4 | 5000健康 paced L0＋10000机动重复故障 paced L1，**两条长流均运行FDE ON/OFF同源同pace配对**（OFF失败也保留全部分母） | 长时间风险、精度、PL、时效与资源增长 |
| N | wrong-FDE、候选丢失、资源、slot引用篡改、取消/过载、后提交失败、来源未来变更 | fail closed/单commit/证书绑定 |

对 P0 与 P2 必须给 S0/S1/S2及 D1 真实场景的 ON/OFF pair manifest。只要代码后续有影响该场景数学、功能或计时的更改，就不能把原先的实验结果冒充最终冻结候选；只重跑必要受到影响的场景并说明覆盖论证。对不受影响的历史SC6长流允许复用前提是固定同一二进制/数学/输入身份，且不能伪称重跑。本轮的 `P95` 结论必须来自本轮定义的同一计时口径的 paced 行；不得以旧 offline 结果替代。

**最终输出必须能回答导师的三个问题：**

1. `FDE ON` 相比 `OFF`，在何种故障/时间阶段/运动/种子下让 tag位置 RMSE 降低多少 **米和百分比**？速度、姿态、bias 是否改善或恶化？整体改善的场景权重是什么？
2. 水平/垂直 Stanford 图如何体现 `PE`、`PL`、`AL`，同一故障下 ON/OFF 点云怎样变化，哪些历元受时间/来源门控拒绝？各 AL 数值诊断可用率、underbound、MI/HMI 候选各多少？
3. D1/晚期IMU故障为何漏检；有无合法跨时间证据可以修复？实际有多少次修复/失败？当前窗口40/50ms P95能否保持、slot/alert/bias是否增长？

## 6. 验收清单（SC6-C01—C20）

> 所有条款都要有 *actual evidence + passing/failing denominator + pointer to code/trace/test*。`PASS`是证据通过，`OPEN`是缺口；“测试已运行”不等于“能力通过”。

| ID | 必需验收 |
|---|---|
| C01 | 核对启动时实际本地HEAD、SC6报告、配置/平台和原回归；保留SC6历史结论，不自动push |
| C02 | 预注册P0–P4/N最小矩阵、5种子、AL敏感性、FDE策略、P95口径；测试前冻结，不按结果挑seed/故障强度 |
| C03 | 同一真实FGO/原生commit/rolling source与风险下，ON/OFF逐原始测量及注入清单 bit-equal/digest相同，区别仅为FDE行动策略 |
| C04 | ON/OFF同一 paced计时、同一时间/世界坐标/lever/GT评分；配对失败、拒绝、未尝试均计入计划分母 |
| C05 | 报告各run与配对位置APE/RMSE、HPE/VPE、RPE1/20、速度/姿态/六轴bias统计；ON/OFF绝对差、百分比及退化，不用PL代表精度 |
| C06 | 逐episode健康/故障/隔离/恢复/晚期不可恢复分阶段统计、TTD/TTI/TTR、真实receipt动作及原生数值变化；正确标注NA和分母 |
| C07 | 按种子而非按时间帧做 paired 汇总及不确定性描述；场景等权或预注册权重，保留所有负收益/失败 |
| C08 | 水平 Stanford 真实逐历元 HPE/HPL，与实际stamp/tag/receipt一致，ON/OFF同尺度，1:1线和AL、来源/分母注释 |
| C09 | 垂直 Stanford 真实逐历元 VPE/VPL，同上；2/3、5/5、10/10阈值表，未资格/晚交付不可伪装及时可用 |
| C10 | 独立单元测试MI候选/HMI候选、边界等号、PL<PE但不可用、超时但空间有效、无输出、单位转换及分母互斥；生产资格明确关闭 |
| C11 | D1 gyro、晚期强故障、弱故障逐个诊断真实物理区间、RW、事件q、模式/候选/FD动作/日志；无证据不能把漏检说成不可辨识 |
| C12 | 有必要信息时真实检测并执行合法bridge/prior-repair/rejoin；不可辨识时给相容观测反例并保留恢复未成功；错误动作仍有raw覆盖 |
| C13 | bias/姿态联合约束正负原型、SO(3)/lever/秩/镜像，主raw采用新约束时必须具备全故障集合包含与时间递推证明；否则保持合法fallback与OPEN |
| C14 | S2姿态退化、D1长流姿态误差与恢复metric因果分析；真实数值ON/OFF姿态及速度/bias副作用不隐瞒 |
| C15 | G2数值拒绝独立分析，若不具合法候选则按函数拒绝保留，不能直接声称物理解集为空或GT选镜像 |
| C16 | 合同内所有注册paced正例双P95≤40/50ms、timely_spatial_qualified/scheduled≥.95；精确nearest-rank分母、P99/max/超时与队列记录齐全 |
| C17 | 有限长流slot/alert/bias/RSS/证书资源趋势，若压缩slot则真实ledger/恢复/单commit语义对齐；无O(1)证明不授O(1)资格 |
| C18 | 保留既有SC1–SC6相关回归，wrong FDE/tamper/overflow/post-commit failure/source gap/真实迟故障等负例，风险与AL/生产发布开关无改动 |
| C19 | 冻结最终必要短/长流、逐run及汇总分母、三张真实论文级图、独立脚本复现；不从聚合统计伪造Stanford点 |
| C20 | `docs/fde_next/SC6_CLOSURE_RESULTS.md`唯一新报告；逐C01–C20结论、代码SHA、反例、限制、复现命令、本地阶段commit与自有缓存清理；不自动push |

**命名与完成规则**：三项能够分别正常签发的结果是 `FDE_ACCURACY_BENEFIT_EVALUATED`、`STANFORD_INTEGRITY_EVALUATED`、`REALTIME_P95_TARGET_MET_FOR_TESTED_PROFILES`。若 W1 仍证明晚期不可辨识，报告 `DETECTABILITY_LIMIT_PROVED_FOR_SCOPE`（须有有效反例），**不能**授予 `LATE_FDE_RECOVERY_VERIFIED`。如果没实现安全资源压缩，`RESOURCE_BEHAVIOR_EVALUATED` 可按有限测试验收，而 `RESOURCE_OPTIMIZATION_OPEN` 继续保留。结论可为 `SC6_CLOSURE_EVALUATED_WITH_GAPS`；只有所有预注册必需功能范围真正达到正例与证明时才可用 `SC6_CLOSURE_COMPLETE_FOR_REGISTERED_SCOPE`。不能靠改小范围、吞掉失败或把有限条件数学界说成硬件资格获得完成标签。

## 7. 代码落点、最小交付文件和复现

**优先复用现有代码**：`research/sc6_system.cpp` 的独立 producer/worker/真实sink、`research/sc6_service.cpp` 的唯一实际GTSAM commit/证书签发、`research/sc6_provider.cpp` 的外部privileged source/GT、`research/rolling_recovery.*`、`research/open_fault_pool.*`、`research/rolling_raw.*`、`research/rolling_boundary.*`、`research/time_uniform_event.hpp`、`tools/sc6_system_research.py` 与 `tools/sc6_evaluate.py`。

建议新增一个薄的 `tools/sc6_closure.py` 或对现有脚本做精简子命令，用于配对运行/评分/Stanford制作。图表代码独立、无反向导入数值在线模块。新CLI建议最终有 `checks, paired, stanford, final` 几种工作模式，**这些是待实现接口，不要当作现有命令已经存在**。如与现有实际入口冲突，选择更小改动并在报告给出确切可运行命令。

源码/最小交付：

```text
docs/requirements/sc6_closure/SC6_CLOSURE_REQUIREMENTS.md
docs/requirements/sc6_closure/GOAL_SC6_CLOSURE.txt
docs/fde_next/SC6_CLOSURE_RESULTS.md
docs/fde_next/sc6_closure_fde_accuracy.png
docs/fde_next/sc6_closure_horizontal_stanford.png
docs/fde_next/sc6_closure_vertical_stanford.png
# 必要研究源代码、测试和最少量配置；避免复制/重写旧SC6
```

**保存与复现**：W0→W3每个通过阶段都先本地git commit后继续，最后一次冻结结果和报告再commit；工作树需干净。无需请求批准继续下个包，不自动push。只清理本轮自建临时缓存、CSV大文件、临时编译日志；保留自有代码/需求/测试/最终报告/必需图片，不删除旧证据。跨会话中断先检查HEAD与本地实际进度，仅续未通过条款。禁止盲目循环几十种种子“找到好看的图”，禁止伪造文档、性能指标或结构化实验结果。

## 8. 来源边界

**工程事实**来自 SC6 现存实际报告与上列GitHub源码；**数学证明责任**来自本需求要求，不因这里的文字自动成立。Stanford Plot 的定义/AL分区参考 ESA Navipedia（见§2.2）；具体的“诊断MI/HMI”与研究应用门控是本项目保守区分设计。上述seed/门槛/矩阵属于**下一轮新预注册计划**，不是已有SC6运行证据。
