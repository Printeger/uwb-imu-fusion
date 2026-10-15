# SC8 实验手册
## 正式仿真、代表性实测数据、算法对照、消融与论文结果

**版本：SC8-EXP-1.0 / 2026-10-15**  
**目标仓库：** `Printeger/uwb-imu-fusion`，`feature/realtime-uwb-imu-pl`。  
**设计时实际读取的远端：** `fa5b3dc9ffe3c88492352491d79ba022f373bfb5`；父提交为 SC7 交付 `ef2595f48fb664ae2905f2fa6850f743c9ce3473`。启动时以真实本地 HEAD 为准，不回退用户新代码。  
**执行：** 一个连续主 Goal，W0–W4；工作包验证后本地提交并继续，不自动 push。  
**本文件是实验设计，不包含已经运行的 SC8 结果。**

> 本轮不以“我们的方法必须胜过 SFUISE”“全部 PL 必须小于某个 AL”作为成功条件。成功是：在明确的数据、方法、时间、故障和资格边界下，实际完成可复现的配对实验，解释收益、代价和失败，交付科学有效的实验报告。不能把旧版本失败当作当前不支持，也不能把新版本未知当成已经通过。

## 0. 需要回答的九个问题

| 实验 | 研究问题 | 核心证据 |
|---|---|---|
| E00 | 数据接口和评价器是否正确？各路径是否真的是当前系统？ | 新版输入审计、因果性、实际后端/receipt、独立指标核对 |
| E01 | 健康仿真中，在线融合精度和不同方法的代价如何？ | 原生末端 APE/ARE/RPE、coverage、在线/历史结果分列 |
| E02 | 少量代表性实测数据上能否运行，精度如何？ | 六条候选序列、最新接口、同源方法比较、真实失败 |
| E03 | UWB、IMU、联合故障下，FDE 到底改善多少？ | 注入前后×FDE ON/OFF配对，检测、隔离、恢复与全状态误差 |
| E04 | PL 是否覆盖实际误差、是否紧致、何时不可用？ | H/V Stanford、超界、AL敏感性、可用性与资格分层 |
| E05 | 100–1000历元窗口值得吗？收益来自FGO还是完整性？ | 两类窗口实验、5–50s真实时域、精度–PL–代价–资源前沿 |
| E06 | 我们的完整流程各模块花了多少时间？ | 同边界全服务/核心/交付、exclusive breakdown、P95/P99 |
| E07 | SC7的各项改动分别带来什么？ | 故障FDE、G2数值求解、tag关系、分支支持、联合IMU约束消融 |
| E08 | 长期运行是否退化？失败是否可预测？ | 冻结长流、晚期故障、bias/slot/RSS/队列、窗口敏感性 |

这些实验共享相同运行结果。禁止将“数据×算法×窗口×故障×seed×计时重复”全排列。以一个 run registry 合并重叠任务，同一仿真或bag只运行必要方法；SFUISE在线与最终历史通常从同一次执行导出，不算两个求解进程。

## 1. 依据及当前必须正视的限制

### 1.1 旧多数据报告只能说明什么

用户提供的 `uwb_imu_accuracy_timing_report.md` 是**历史接口、库存和故障线索**，不是 SC7 当前精度或运行能力。它记录：ISAS-Walk1–3、三条 own_vicon、STAR-Loc、MILUV、HUEC，以及旧仿真；GT覆盖、标定方式、不同采样率；旧 SFUISE 保留官方测距剔除。[D1 §数据接口审计/逐序列结果]

特别需要复查而不能照搬：

- 旧 SFUISE VIVE→anchor 变换用了全部三条 Walk，own_vicon 外参/杆臂/时延用了全部三条录制。即使未用GT优化估计器参数，也属于**GT辅助接口标定**，不能冒称测试集完全未参与。正式主表需要分离标定与评价；无法分离时设独立“GT-assisted, test-involved”表。
- 旧 own_vicon 的大量固定偏移及对齐前后巨大差别、旧姿态100°以上、部分方法公里级发散都是需要检查坐标、单位、时间及数值流程的线索；不据此证明数据差或某方法本质不行。
- 旧 direct nominal vs ROS fde=off 等价性为FAIL，位置最大差约0.293m、姿态约180°。SC8必须在当前路径上复核；不能将旧nominal runner的1ms称为SC7整系统开销。
- 旧 SFUISE 总wall包含2×bag回放和等待；旧current主要是离线吞吐。二者不能直接作“快几十倍”的结论。[D1 L426–428, L596–630, L647–655]

### 1.2 实际源码中的窗口和源接口限制

设计时读取的 `research/rolling_policy.hpp` 要求：

- `2 ≤ fixed_lag_epochs ≤ 1000`；
- `follow_fgo` 时 `L_I=L_F`；`explicit` 时 `2≤L_I≤L_F`；
- `runtime_resize=disabled`；恢复回放长度从 `L_F + recovery_extra_epochs` 派生；
- 当前边界捕获还检查每个50ms历元11个含共享端点IMU样本等来源关系。[C2]

因此 **(L_F,L_I)=(30,1000)当前不是合法配置**。不能仅改YAML、解除guard就声称实现了独立1000窗口或任意采样率证明。配置接受1000也不代表曾验证1000稳定或实时。

SC7的条件性raw保证来自注册仿真law、初始观察、物理运动界、共享原始噪声和故障范围；sample0 bias 收益依赖原0ms来源及首故障最早5ms的约定。真实bag没有这些自动资格。SC8必须把真实源接入和其数学资格分开，而不是把所有实测标记为同一raw定理下合格。[D2 §W2/Final limits]

## 2. 数据选择：三类核心实测＋仿真，不再先跑40条

### 2.1 推荐候选及预先用途

以下名称来自旧库存，**需由本地最新实际文件确认存在、校验hash和metadata**。本手册按上传报告中 SFUISE 对应的 ISAS-Walk1/2/3 三条序列设计；若用户本地的“三个数据集”还包含 UTIL 等其他数据族，由E00记录实际库存，不把TDoA当作这里的ToA录制。选择依据是代表性、GT与接口信息、传感器差异，不是历史APE谁更好。

| ID | 候选 | 本轮用途及理由 | 标定/风险边界 |
|---|---|---|---|
| SIM | 当前SC7仿真，不是旧figure-eight runner | 唯一能够严格控制源law、故障真值与raw包含论证的主实验；D0/D1、G0/G1/G2 | 条件仿真保证单列 |
| R1 | SFUISE `ISAS-Walk2` | 官方基线同域、步行轨迹，正式在线比较 | Walk1优先用于接口标定，不在其上挑参数后称盲测 |
| R2 | SFUISE `ISAS-Walk3` | 同硬件独立录制，检查重复性与轨迹变化 | 与Walk2同一已冻结坐标/标定 |
| R3 | own_vicon `2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle` | 自采较长无障碍录制，未来采集流程的基准 | 原Livox加速度单位及IMU→tag变换重新确认 |
| R4 | own_vicon `2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle` | 与自采基准配套的障碍环境录制 | “obstacle”不是逐量测NLOS真值；短流不用于长期稳定性结论 |
| R5 | STAR-Loc `loop-3d-z_s3` | 不同硬件、3D/垂直激励；检验VPL/姿态相关性 | 官方rig→IMU及tag坐标、range_calib、anchor ID |
| R6 | STAR-Loc `loop-2d-fast_s3` | 相同setup的快运动，减少硬件混杂，补动态压力 | 不是因旧SFUISE崩溃而选，旧失败需重新排查 |
| CAL-W | `ISAS-Walk1` | 主校准/接口开发，附录给结果不混入未参与标定主表 | 官方ToA偏置优先于重新GT拟合 |
| CAL-O | own_vicon `2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle` | 仅为候选校准/静态检查录制，先查运动激励 | 旧aligned RMSE小**不能证明它静态或适合完整6D外参标定** |
| CAL-S | STAR-Loc官方标定；若需开发片段，用独立 `loop-2d_s3` | 不用R5/R6测试GT反向调映射/噪声 | 已公布数据提供者标定与本工程GT拟合分别披露 |

旧报告列出三条自采录制，用户说“两个自采数据”：不能擅自推断是两个文件还是两类场景。先读本地manifest解析为R3/R4两类；如实际新版文件名不同，按无障碍/障碍、独立GT及运动覆盖映射，并在预注册文件记录一次，不反复询问已有文件能解答的问题。

### 2.2 备用与本轮不优先项目

**首备选：MILUV `default_1_circular3D_0`**（或预登记替代 `default_1_random3_0`），适合飞行与另一硬件；只用一个机器人、一个指定tag、固定anchor测距，主比较不添加视觉、测高、磁力计、移动机器人间range或第二tag。官方数据本身含这些模态，公平比较只取共同UWB+IMU输入。[S3]

**HUEC**保留一对官方命名LOS/NLOS序列作为后续外部验证；本轮先不扩至8条。理由是需要优先花时间统一前三个数据族和低冗余条件，不是“旧版崩溃所以不可用”。四anchor不等于不能融合定位，但对三维纯UWB单故障排除后的冗余极少，不能套八anchor假设池/纯单帧PL资格。

备用替换仅因当前文件确实不可用、GT/时间/关键标定缺失或数据接口经定向复核仍不成立；**方法精度差、我们输给SFUISE、算法拒绝不能作为删除序列依据**。已经看见正式算法结果的序列不得悄悄替换。每个数据族最多补选一条备用；失败留在筛选表和结果分母。

### 2.3 新版筛选程序：不凭旧输赢判数据好坏

1. metadata-only：hash、duration、单位、每条量测time、GT覆盖与最大gap、独立anchor数、tag/IMU frame、lever、曝光/采样时间、NLOS标签来源；确认range不是TDoA。
2. 量测审计：无GT的时间/重复ID/字段合法性；允许评估器用GT独立检查坐标与测距一致性，但不按测试残差清洗数据或调sigma。
3. 最新接口短冒烟：每条候选一个预固定15–30s前缀或文件全长较短者，全部主方法相同；检查构建、崩溃、因果、实际输出及covariance合法性。冒烟是接口诊断，**不按APE排名入选**。
4. 固定六条正式候选、CAL/TEST边界与输入schema。GT coverage主统计目标≥90%；低于者保留coverage不充分结论，不能只删算法大误差区。每种方法自身输出coverage单列，不能复制GT覆盖。

不能凭这些资料现在就担保哪条“最新系统一定跑得好”；本轮选择是有依据的候选，E00负责在当前版本上验证可用性。

## 3. 三条证据通道：不同资格，不能混算

| Track | 内容 | 允许结论 | 禁止结论 |
|---|---|---|---|
| T-SIM | 原law仿真，模型内噪声/运动/故障 | 条件raw包含保证＋实现实验；仿真GT可精确评分 | 零超界样本独自证明10^-6或硬件资格 |
| T-REAL | 原始实测，无新增注入 | 定位/时效/经验鲁棒性；记录natural residual与告警 | 隐含所有传感器健康、把未知真故障当误警标签 |
| T-SEMI | 在实测raw流加入已知fault overlay | 对“额外注入”的鲁棒性、恢复和精度变化 | 原真实噪声及天然故障自动满足仿真law；未命中注入之外的告警全部叫误报 |

真实输入有80Hz、200Hz、250Hz IMU和非同步测距，不能复制/插值成11个“独立200Hz”样本，不能复制4个anchor构造8个。需要重用通用名义FGO和现有数据适配器。有限工程改造可接入**无统计资格的研究FD/FDE诊断路径**，但不得解除source guard并把`raw_qualified`改true。

每次运行记录 `requested_method`、`effective_method`、`fde_executed`、`pl_method`、`scope_id`、`statistical_qualification`。O-FULL要求完整实际FDE及可用的对应证据生产者/消费者；若只接通nominal，标为O-NOM，不能以同名冒充O-FULL。如果实测只能工程FDE，命名O-FDE-ENG且PL=NA/UNQUALIFIED；这仍可用于明确标注的精度比较，不能顶替完整系统数学验收。

有可计算但未证明的PL只能放在单独“empirical diagnostic / unqualified”图表；无合法来源就NA/拒绝，不合成一个数。正式研究发布/硬件旗标保持OFF。本轮不强行完成真实传感器概率标定。

## 4. 方法清单和公平比较

| ID | 算法/输出 | 在哪比较 | 注意 |
|---|---|---|---|
| O-FULL | SC7融合＋联合FDE＋滚动raw PL | 主精度/完整性/系统时延 | 默认L_F=L_I=30 |
| O-NOFDE | 同一算法与PL，但不执行测量排除、bridge、污染prior替换/恢复动作 | 主FDE因果对照 | 保留FD诊断、原始数据、PL与时效；说明实际关闭范围 |
| O-NOM | 同一FGO，不运行FD/FDE/PL | E06纯估计开销，少量健康精度一致性 | 不把它的低开销叫完整系统低开销 |
| S-ONLINE | 固定官方SFUISE策略，逐次记录首次因果输出 | 在线精度、鲁棒性、计算与输出延迟 | 保留官方测距剔除；不是“无FDE基线” |
| S-FINAL | 同一次SFUISE运行在结束后保存的最后历史修正值 | 独立“最终历史精度”栏 | 不叫全局batch最优、不叫实时结果；不是额外求解进程 |
| O-W… | 预注册窗口配置 | 仅E05 | 不是各数据主表都跑一遍 |
| O-ABL… | SC7组件消融/原Closure参考 | 仅E07 | 条件/风险相同才比较PL |
| A-UWB（可选） | 已有纯UWB RAIM/PL | 单帧几何和传感器融合增益附录 | 保护纯UWB tag，不是融合PL，范围独立 |

不新增大批ESKF/Huber/其他SLAM基线来挤占本轮。SFUISE＋ours ON/OFF＋必要组件消融已经能回答主要问题；额外算法需真正不同研究问题才加入。

### 4.1 SFUISE版本和历史输出必须先查

当前 `tools/run_sfuise_benchmark.py` 固定SHA `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`，隔离workspace、官方测距剔除、Release-equivalent编译，脚本将多个range合并为RTLSStick。[C3] 官方README说明维护仓库从KIT-ISAS迁到ASIG-X。[S1]

W0读取本地remote、HEAD和官方配置，默认保留已固定且可复现的官方基线，不因看到更新就更换实验版本。必须纠错或采用新上游时，在正式跑数前冻结新SHA与差异；“旧版复现”和“维护版”不得混在一列。可添加**只读观测/计时导出补丁**，验证不改变残差、优化状态、剔除和更新顺序；不能在benchmark中偷偷改SFUISE数值算法以帮它成功或故意禁用鲁棒策略。

每条输出记录：`estimate_stamp`、`published_wall_time`、`max_sensor_stamp_used`、`revision_index`、`mode=first_causal/final_history`。连续时间样条可在任意时刻求值，但若使用了晚于该时刻的控制点/测量，就不是该时刻的零前瞻输出。允许固定滞后输出，必须报告state age/lookahead和可用时刻，不伪称窗口末端实时。最终历史值可能是每个状态退出窗前最后版本，不自动等于用整条轨迹重新求解。

GT仅评估，不供S-ONLINE选择历史版本。实时在线分母按实际发行值；不能事后用最终CSV替换。TDoA数据不能用ToA适配器只改字段名。

### 4.2 时间同步、批处理和输入一致性

原测距保留真实t与ID。官方SFUISE将同一RTLSStick内range视为同一时间时，必须核查实际batch跨度，明确原子同步消息和人为聚合之间的区别。只可按预注册同一批策略让所有方法得到相同消息；记录`batch_start/end`、等待、实际测量time及等效模型误差。不能无限等到凑够anchor而不计延迟；不能把不同时间的range拼为“同步真测距”并继承原raw定理。

建议同时保留 native原消息主比较与公共20Hz窗口的工程补充（仅真正需要时），两者不混合排名。无论选择哪个，GT和true state不用于batch组成，重复测距不是独立冗余。修改SFUISE消息表示造成的源契约变化必须披露。

原输入的共同部分、单位转换前后和校准/注入步骤均有摘要。ours与SFUISE可能因消息布局不同而不是文件byte相同，但**归一化后的物理观测多重集、原时间及注入ID必须逐项相等**。ON/OFF的共同接口文件则应逐字节一致。

## 5. 标定与评分：正式实验开始前先把尺子校准

### 5.1 三种坐标处理不能混为一谈

1. **传感器/地图接口标定**：IMU单位、extrinsic、lever、时延、UWB固定偏置、GT→anchor world。优先官方外部标定；本工程GT辅助估计只用CAL资料，冻结后用于TEST。
2. **绝对位置主指标**：已校准anchor/world下实际在线tag位置，**无按方法、按轨迹的后验SE(3)/Sim(3)对齐**。评价固定偏移、初始化和真实全局误差。
3. **形状精度补充**：允许独立列SE(3)-aligned APE，标记evaluation-only；度量轨迹形状，不能替代主APE、不能把对齐后的误差和原未变换PL混在Stanford中。UWB/IMU有米制尺度，不默认Sim(3)缩放。

如果只剩两条自采且没有独立标定，用无障碍序列**预固定前段**作CAL（例如前20%，最低需满足独立运动激励检验），其余连续后段作测试；中间固定隔离区段不计主评价且不重置风险。禁止按误差大小选择切点。校准不足以识别全部旋转/杆臂/时延时使用官方/硬件测量值或明确`CALIBRATION_OPEN`，不能凭一个小拟合RMSE宣布全标定可靠。独立测量不存在时，这两条仍可进带披露的工程附表，不充当盲测证据。

所有方法共用同一标定、不分别用GT替它们寻找最佳时间偏移。以前全三Walk拟合结果可复现为历史附表，但新正式列优先用Walk1单独拟合并检查可辨识性；不得沿用测试参与的拟合却写“完全无GT参与”。

### 5.2 指标名称与定义

用户提出ARE/RPE：本文保留 **ARE=Absolute Rotation Error（绝对姿态角误差）**；位置使用SLAM常见APE/ATE。evo的APE用于绝对对应位姿，RPE用于相对运动；把“ARE”当位置误差会混淆。[S4]

令 `p_tag = p_body + R_body*lever`，真值和估计均转到相同物理点：

- `APE_tag,k = ||p_hat_tag,k − p_true_tag,k||`；主统计RMSE、median、P95、max，单位m。
- `HPE = hypot(e_x,e_y)`；`VPE=abs(e_z)`，与HPL/VPL的保护轴一致。
- `ARE_k = ||Log(R_true,k^T R_hat,k)||`，单位rad并给deg。检查SO(3)、四元数q/−q同义，不能把body和tag不同frame直接比较。
- **tag相对平移误差**：`[(p_hat_j−p_hat_i)−(p_true_j−p_true_i)]`的范数，公共时间间隔0.5s和1s；0.05s仅作为固定20Hz仿真原短RPE补充，不能跨不同采样率用“一帧”比较。
- **body SE(3) RPE**：`E_ij = [(T_true,i)^−1 T_true,j]^−1 [(T_hat,i)^−1 T_hat,j]`，平移和旋转分开。它可能受姿态影响，不和tag平移RPE同名混列。
- 速度/accel bias/gyro bias误差：仿真可评分；真实bias没有独立真值填NA，不能把FGO估计bias或静态fit当其真值。真实GT速度若差分得到，固定平滑/导数方法并注明派生GT。

RPE匹配采用同一时间对列表，不以输出频率/样本索引决定不同方法的时间跨度；遇GT/输出gap不跨gap插值造点。报告每个指标N（APE点数、RPE时间对数不同）。所有方法的共同时间交集及各自全coverage两表并列；不得只报共同成功易帧而隐藏缺帧。

**评分器最低测试**：零误差、固定刚体偏移、固定姿态偏差、q与−q、正确/错误lever、dt偏移、SE3变换、输出缺失、重复/错序、已知直线/旋转轨迹。tag差分逐对检查 `RPE_ij≤APE_i+APE_j`；另与冻结evo版本核对相同位姿关系和时间对。这个不等式不直接用于混有姿态的SE3平移RPE。[S4]

## 6. 故障注入设计：覆盖主问题，不做全排列

### 6.1 在算法看到数据之前注入

生成一个 immutable overlay manifest：物理sensor/source、axis/anchor、onset原时间、duration、amplitude及单位、shape、affected raw IDs、基准与overlay摘要、是否声明域内。所有方法使用**同一注入后观测**，GT不变。IMU在raw ADC层修改，预积分和共享端点自动消费；不能给每个PIM独立加不同fault，也不能仅在PL函数中修改残差。

对输入同raw值与ID：ours/SFUISE offsets只应用一次；确认ToA的`range_raw`/校准`range`两字段都映射到同一物理注入，不能一方法注入了另一方法绕过。原bag只读，临时overlay可以流式生成，无需复制整个bag。

### 6.2 模型内主故障表

下表F0/FU/FA/FG/FJ是**手册的逻辑故障类别**，不是现有runner case名称。现有F0–F5曾用于六轴测试，绝不能直接把现有F0当作健康。实际生成唯一`SC8-H/SC8-U/SC8-A/SC8-G/SC8-J`注册ID并核对其注入清单。

| ID | 类型 | 主设置 | 比较用途 |
|---|---|---|---|
| F0 | 无新增故障 | 相同noise seed，保留自然噪声 | baseline；实测不叫ground-truth healthy |
| FU | UWB单anchor episode | 原SC7冻结幅值/持续（如2.25m，1/3epoch）及相同onset | 纯测距异常识别与排除 |
| FA | accel单轴 | 原注册强故障（如20m/s²，500ms），独立安排 | IMU隔离、速度与bias及恢复 |
| FG | gyro单轴 | 原注册强故障（如1rad/s，500ms），独立安排 | 姿态副作用与恢复 |
| FJ | UWB+IMU | 允许范围内并发/错峰；同原始幅值、单轴、共享参数 | 统一故障池与联合恢复 |
| FL | 多次相同幅值晚期episode | 继承L1事件表 | 长期检测/恢复，不逐渐加大幅值 |

上表数值用于声明的仿真强正例示例；W0读取实际manifest核对准确值。真实数据的相同幅值是**stress overlay**，不能宣称代表实际设备故障发生分布。

主统计用D1同一M3/G1轨迹：F0/FU/FA/FG/FJ，各3个SC8预注册新噪声seed `20261020/20261021/20261022`；它们不是日历时刻。固定相同3个seed，主故障起点在准备充分之后、与边缘化边界存在已登记关系（例如epoch121）；不能看到误差后选注入点。

**边界补充，不全排列**：D0六轴单强故障各一例；UWB弱幅、IMU弱50ms、负幅与错峰各少量定向例。弱/强具体数值在算法结果前按原sigma、域边界和任务表冻结；只做2–3个幅值×2种duration的小矩阵，不50套阈值扫描。无统计功效证据时只称“样本报警比例”，不称校准Pd曲线。弱故障未报警但仍被raw集合覆盖，是应记录的边界，不预设为失败或全部恢复。

### 6.3 实测叠加与域外压力

只选R2 Walk3和R3无障碍自采，分别一次FU、FG、FJ（三份overlay）作T-SEMI配对；其余实测只做原始数据，避免故障重复应用到全部六条。ON/OFF及SFUISE都接收相同原始＋overlay。原始实测可能本就有NLOS/漂移，不能用“未注入=无故障”。对新增注入的收益可计算差分中的差分：
`(RMSE_inj,OFF−RMSE_clean,OFF) − (RMSE_inj,ON−RMSE_clean,ON)`，同时保留各四格原值；正值才支持ON减少了额外注入损害，且“clean”仅是未新增注入。

慢ramp、同时两IMU轴、两个anchor共故障、未知杆臂/时延漂移等如果不在原scope，最多设置1–2个独立压力负例，标`OUTSIDE_SCOPE`。不能给其原条件保证、不能期望检测器必然自主知道其域外。输出拒绝只在实际有证据时才计为正确识别；仿真器知道标签不算在线能力。

### 6.4 FDE评价不能只看RMSE

统计单位为物理episode，记录：计划、实际注入、属于scope、首报警、正确axis/source、过排除、误排除、实际因子删除、污染prior修复、bridge、rejoin后至少4epoch、恢复失败/回放外、TTD/TTI/TTR。分级成功：`FD`、`ISOLATION_COMMITTED`、`REJOIN_EXECUTED`、`ACCURACY_RECOVERED`不能互相代替。

未知真实自然故障没有可靠标签时，不计算其“误警率”；可报告alerts/hour、exclusions/hour、动作持续时间与人工核查。仿真健康F0可报告误警/健康误排除；实测overlay可评价注入命中，但不得将其它告警全判错。

平均时延之外保留故障开始相位。每帧core50ms通过不是TTD≤50ms；传感器采样、凑批、检测、候选commit、bridge结束均占时间。分阶段APE/ARE/RPE保留隔离瞬间的位置跳变与后续姿态偏差，不仅报告整个序列RMSE。

## 7. E00–E08 实际最小矩阵与预算

### E00 — 新版预检与基线观测

**输入**：六实测候选、独立校准、当前仿真。**方法**：O-NOM、O-FULL/ENG、S-ONLINE实际短前缀。**输出**：数据资格表、当前适配错误修复、输出语义及评分单元测试。

必须复查：旧direct/ROS不同、official SFUISE姿态语义、80/200/250Hz、4/8anchor、tag数、时间偏差/逐range time、初始化法、GT-only访问白名单。允许少量有验证的接口/导出/评价器/构建修正，禁止借E00做大规模调参或修改PL数学框架。一次明确的已证实现bug可修并运行确定性小测试；不能一条数据失败就终止所有其他实验。

### E01 — 健康精度和在线/历史差异

D1/M3/G1 F0×3seed，另D0/S0、G2/S3各1seed。O-FULL、O-NOFDE、S-ONLINE（同run导出S-FINAL）；S0若FULL与NOFDE路径完全同semantic，仍留一次对照。O-NOM只在一个健康场景用来核对纯估计，不需要全量重复。

结果：APE/ARE/RPE、coverage、姿态/bias（仿真）、S-ONLINE→S-FINAL变化及修正年龄。不要求最终历史一定优于在线；从数据判断。没有GT后验对齐的world主表与对齐形状附表分开。

### E02 — 六条实测主表

每条测试序列 O-FULL（或明确标记的O-FDE-ENG）、O-NOFDE/ENG-NOFDE、SFUISE一次；在线和历史两种SFUISE输出分列，**三次求解而非四次**。原始录制不是Monte Carlo，重复运行同一bag不能冒充多个独立数据集。主指标以序列等权汇总、同时列每序列值/coverage/失败。不得把不同方法的“各自成功序列”族均值直接排名。

R6若超出既有物理运动证明，只在T-REAL经验表，保留所有输入；不能用真实GT最大加速度调小/扩大在线证明以冒充合格。旧40条仅库存，不重复全跑。

### E03 — FDE定量收益

复用E01 F0。D1/M3/G1：FU/FA/FG/FJ×3seed×{FULL,NOFDE,SFUISE}，600epoch每条；保存独立故障事件表、首发与历史输出。同一方法的F0与故障流使用同标准随机draw，避免改变随机调用次数改变非故障部分。另6轴/弱/错峰定向例只跑FULL（必要时NOFDE），不扩到所有基线。

实测R2/R3各FU/FG/FJ×三求解方法，共18条附加运行；未注入结果复用E02。S-ONLINE原测距剔除作为官方算法一部分，不称SFUISE是FDE OFF。

**预期结论格式**：“在指定故障集、3个seed与该轨迹中RMSE下降X%；健康代价Y；某类姿态恶化Z”。不能验收要求X必须>0或超过旧67%。不把某一强fault成功率推广到全部允许幅值。

### E04 — 完整性与可用性

不额外重放，读取E01/E03/E05/E08的同stamp/tag/receipt输出。比较旧raw参考、SC7完整界及其合法消融；SFUISE没有对应PL填NA，不给其生成协方差×K后命名同风险PL。纯UWB A图另列不同保护对象。

主图：T-SIM H/V Stanford全尺度＋小误差局部图（不藏溢出/超界）；输出完整可见点和图外计数。AL=(2,3),(5,5),(10,10)m只后处理，研究AL没有应用资格。AL sweep 0.1–10m（含原值）只读已有输出，报告水平、垂直及joint可用率。

T-REAL/T-SEMI图必须独立标注未认证统计范围。无有限合法来源PL时不给伪散点，使用coverage/refusal图。参考真值非零误差时，经验超界是相对GT参考；若有GT独立误差界u，则可报告保守上下包络：`max(0,||p_hat−p_ref||−u)`至`||p_hat−p_ref||+u`。GT误差证据不足不声称实际HMI精确标签。

### E05 — 窗口100–1000研究（成本边界，不要求全部实时）

**统一定义**：L_F=FGO活动历元数，L_I=完整性显式历元数，N=整条测试长度；它们不是同一个参数。每epoch仍50ms时，100/300/600/1000对应5/15/30/50s；不能把1000个IMU样本或1000条单anchor range当1000epoch。

**A组（联动）：** `(L_F,L_I)=(30,30),(100,100),(300,300),(600,600),(1000,1000)`。它同时改变FGO优化和完整性，称“joint window scaling”，不把APE变化只归因于PL。

**B组（隔离）：** 固定`L_F=300`，对比`L_I=30,100,300`；(300,300)复用A组。这是在现有合法`L_I≤L_F`约束内分离完整性窗口代价。若后续需要固定FGO1000再扫完整性到1000，可单列下一轮，不在本轮偷偷新建另一套历史窗口。

所有7个唯一配置共用两份**2100epoch**数据（健康＋重复故障，一seed）；公共主评价区间epoch1101–2100，确保最大1000已真实边缘化且具有共同风险birth时刻。故障在1201、1601按固定强度/不同传感器安排，初始化与前缀全部真实处理、计入总CPU，稳态主表另列。不能从成熟快照装入但不计其构造，再叫长窗在线结论。

不得运行中resize；每个配置独立启动、记录实际active keys/retired IDs/explicit proof/replay length与source身份。联动组恢复缓冲随L_F变化是**混杂因素**，报告实际值及恢复收益不能只归因于信息；B组固定L_F因而保持replay一致。

先对每一大窗口执行到首次退出后一个定向canary（只取必要流）；保存可复用checkpoint仅当确有匹配版本/风险/输入证明，不另建复杂数据库。默认资源门限于W0锁定：RSS不超过min(8GiB,物理可用内存50%)，单测试墙钟上限20min，测量服务连续20次>2s则stop/censored。超限保留完成到的epoch、原因、已测时延和未尝试分母；不要修改scope或删掉1000行。不存在可用资源时明确`RESOURCE_LIMIT`，不是“未测但推算出性能”。

**只对默认/已准入实时profile要求40/50ms P95。100–1000为scale frontier，允许显示非实时**。最终选择默认窗口依据预注册精度、PL、及时率和资源多目标，不按验证集找最小APE后在同一集宣称最优。保留30窗口基线，不因较大更慢就删它，也不保证窗口越大PL必然更小。

### E06 — 时间、模块breakdown与系统开销

直接复用同冻结run中的计时。额外仅选一健康、一报警恢复，必要时各2次串行确认（不把3次wall run当数千独立样本）。普通全速吞吐与paced1×来源延迟分列。S-FINAL只是输出版本，不分配虚构额外离线求解耗时；有额外batch重算才单列它的真实成本。

ours建议exclusive阶段：输入/验证、IMU处理与prepare、FD、候选数值求解/FDE、原生commit/恢复、联合状态与边界、H/V支持、证据校验/封装/sink、维护清理；嵌套inclusive事件仅作钻取不能相加。堆叠柱用**exclusive均值**与总均值闭合；各阶段P95另画点/线，不能将阶段P95相加标为total P95。源q/source身份构造、候选失败和fallback的成本不得省略。

同类方法报告measurement-to-sink、state age、core、CPU/epoch、CPU/传感器秒、峰值RSS和RTF（固定 `wall_processing / sensor_duration` 的定义）。SFUISE原生window时间可能覆盖不同数量观测，必须同时给processed measurement count和update rate；不能仅以current1ms/epoch对其窗口均值30ms排名。

### E07 — 有研究意义的消融

只用S0/D0、D1/FJ各一个seed，G2用于数值消融；最多三个合法删组件变体，不做2^n：

- **A-FDE**：FULL vs NOFDE，复用E03；回答动作对精度的作用。
- **A-SUPPORT**：joint/state相同、terminal branch/tag支持改为已有保守box参考；回答末端H/V支持收益。同center只读oracle在离线诊断计时，主服务不得偷偷排除其成本。
- **A-JOINT**：禁用SC7 raw IMU/sample0收缩，使用原物理传播＋同一有效H/V外包络；检查raw论证和tag关系可组合性，不把无效混搭当合法消融。
- **A-G2**：旧数值候选 vs新六初值求解，G2一短流即可；旧失败是有意义结果，不加GT救援。

若没有可独立启停而保留包含证明的组件，使用原Closure完整参考作合并消融并如实标注“联合改动”，不能拆出虚假的独立贡献。额外 tag-retention拆分只在实现已有安全开关且不超过预算时开展。bias图必须注明S0收益来自sample0，S1/S2无持续观测收紧也要画；不能以VPL下降证明全15D可观测。

### E08 — 长流与长期边界

最终代码：L0健康5000 FULL一次；L1混合10000 FULL/NOFDE各一次。同输入、相同完整过去，不仅重复一个snapshot。S-FINAL/S-ONLINE长流只在短流稳定且总预算允许时加，不是本轮必须无条件再跑。

记录早/晚同幅值故障、原生prior修复、IMU再接入、APE/ARE/状态误差、HPL/VPL/bias宽度、slot/ledger/source/provider/RSS、尾时延与queue。5000/10000分别约250/500s；时间一致数学事件不代表无限内存或无限期紧致性。不得通过重启/重置风险/重新健康初始化掩盖漂移。

### 总运行预算

默认冻结矩阵目标 **约120–145次真正求解进程，上限160**（包括短配对、实测与大窗，排除确定性单元测试、只读评分和从同run导出的SFUISE历史）。数字以E00去重展开manifest为准，不能用未运行组合填表。大部分是≤600epoch或短真实序列，不重复SC1–SC7 campaign；性能确认追加不超过4次。若预算中断，保存未完成run ID继续同Goal，不随机缩减困难场景。160是防止全排列的研究预算，不是数学验收阈值。

## 8. 完整性、可用性与统计口径

### 8.1 Stanford分区与分母

每个点必须绑定实际输出的物理tag、stamp、frame、源模型和receipt。横轴PE、纵轴PL，保留`PE=PL`、`PE=AL`、`PL=AL`线。[S5] 本轮不是Stanford–ESA all-geometries枚举，不把所有fault分支点当实际输出样本增加N。

对有限非负 `e=PE,p=PL,a=AL`，四个互斥空间诊断类：

- covered-admissible：`e≤p≤a`；
- MI候选：`p<e≤a` 且 `p≤a`；
- HMI候选：`p≤a<e`；
- AL-unavailable：`p>a`，其中还需另报`e>p`。

另有交叉诊断 `underbound=e>p`，不能因为PL本就超过AL而漏算。规则全部按原double/inward-outward边界约定执行；边界等号在测试中固定，不看图形落在哪个bin判断。

**资格和时间另设维度**：空间有效但迟到者可画灰色层，但不能计为及时服务；未资格、缺失/拒绝、过期、非有限，不能删分母或强置PL=0。分别给 `planned/received/attempted/committed/spatial_valid/timely/within_AL/timely_and_within_AL`，两轴joint合取不能将H和V通过率相乘。

未具备应用/正式发布合同时，HMI只叫`counterfactual HMI candidate under research AL`；不能把`publication_protected=0`导致的零“真正发布HMI”当安全性成果。真实未认证PL图即使全部在对角线上方也不授予𝜀资格。

### 8.2 除Stanford以外的完整性指标

| 指标 | 含义与分母 | 适用限制 |
|---|---|---|
| PE/PL安全指数 | `SI_H=HPE/HPL`、`SI_V=VPE/VPL`；>1是实测低估 | PL=0/∞/NA单列；不取平均掩盖最大值 |
| margin | `HPL−HPE`、`VPL−VPE`分布 | 小margin不等于风险更大，小PL必须有同scope证明 |
| HPL/VPL紧致性 | median/P95/max、同center old−new与ratio | 不同风险/不同保护对象不能同列“更好” |
| numerical availability | 数值PL≤诊断AL的比例 | 不代表实际发布资格 |
| timely conditional availability | 空间资格、按时、两轴满足AL / planned | 全分母，AL来源明确 |
| empirical continuity | 本来可用后发生中断的次数、持续时间、最长缺口 | 有限运行统计，不声称已校准连续性风险 |
| misleading bound count | 所有spatial样本中PE>PL | 含迟到层另报；不是故障漏检概率 |
| FD/FDE分项 | 每episode检测/正确隔离/重接/精度恢复率及TTD等 | 原始真实数据无fault GT则不作误警标签 |
| false alerts | 仿真F0的报警/健康排除数，按epoch及episode规则 | 实测用alert/exclusion rate而非无依据P_FA |

不同𝜀水平/未认证Gaussian covariance不在主实验自动扫。当前风险公式与来源先验保持固定，数学覆盖与功能FD概率不同；SFUISE官方residual reject也不能不经推导声称同一𝜀。少量seed与长序列零HMI不证明10^-6；统计图只检验实施一致性和效果。

### 8.3 配对、统计与失败

对每条独立仿真seed/真实录制分别计算 `ΔRMSE=RMSE_OFF−RMSE_ON` 与 `%gain=100Δ/RMSE_OFF`，OFF=0填NA。报告健康、fault_active、isolation/bridge、恢复后和晚期窗口，受阶段优先级影响的样本数透明。

以**seed或独立录制为单位**汇总均值/median/范围。3seed只支持描述性差异；可提供按seed配对bootstrap的探索区间但标N=3，不声称高置信普遍优势；10k相关epoch不是10k独立样本。真实序列等权，数据族等权的总体分数另列；不得把长轨迹pooled RMSE当唯一结论。

失败有两层：数据/接口资格失败与算法运行失败。崩溃、数值拒绝、无有限输出、资源截断均在manifest中；有限但显著发散的轨迹仍保留实际数值，不能只因APE大而删除。报告完整共同比较集与失败覆盖表，不使用双方不同成功子集的平均值证明排名。图上log轴或inset可以改善显示，但注明范围及溢出，不截掉大误差。

## 9. 时间与模块开销合同

### 9.1 三条时间轴

`measurement_time`、`arrival_wall_time`、`output_stamp`、`max_data_time_used`、`start/commit/proof/sink_wall`必须分开。模拟用独立绝对200Hz/20Hz释放，实测按原始时间戳1×回放；不能等上帧完成后才释放下一帧。同一数据不同算法使用相同发布节奏，rosbag全速/2×吞吐另列。

报告：传输/凑批等待、队列、core、实际measurement-to-sink、state age/估计时间滞后、CPU利用率、冷启动、RTF。一个SFUISE历史点误差低但晚几百毫秒产生，不算与零前瞻输出等价。初始source审计/全文件预载成本不可遗失，启动计时单列；若某证明工作必须用当前数据则计入core。

### 9.2 实时通过定义

默认30/30及既有20/20、40/40、30/10正例：`P95(core)≤40ms`，`P95(measurement_to_sink)≤50ms`，且`timely_spatial_valid/planned≥.95`，保留max/所有miss。nearest-rank `sorted[ceil(.95*N)-1]`，N为所有可测已启动服务，含慢帧/abort；未产生sink的样本不能记0ms，也不能为了P95PASS删除。

真实输入若方法只能未认证工程输出，用`timely_estimate/planned`衡量功能时效，**不套空间保护资格**；实时目标与记录原生更新周期并列。100–1000窗口为压力/性能前沿，达到或未达到都作为实验结果；不要求它们通过默认deadline才准结项。

相同CPU线程/affinity策略、构建O3、实际DSO、SFUISEDebug覆盖、后台负载固定；不以绑核只能ours却不给SFUISE资源的方式造优势。使用单线程受控主表，官方默认多线程可附表。不要将内核CPU时间与wall相加，也不能把所有进程wall的P95当每epochP95。

P95低于阈值不等于最坏时延或TTA保证。40/40在SC7接近40ms只说明余量小；正式新平台需复测，不能承诺无论机器都实时。

## 10. 逐实验图表设计：有视觉层次，但不靠装饰夸大

本节是**待数据生成的图表规格**，不能在SC8运行前生成看似真实的数值曲线。所有图由最终metrics/trace自动产生，报告里标run ID、source/代码身份、样本数与缺失数。

| 图号/实验 | 推荐图表与布局 | 读者应一眼看到什么 |
|---|---|---|
| F00 / E00 | 数据–方法准入矩阵：行=六序列，列=raw完整性/工程FDE/ours nominal/SFUISE；色块＋文字状态；旁边简洁时间/GT覆盖条 | 能运行和统计资格是两回事；旧失败重新核查 |
| F01 / E01–02 | 配对点线/dumbbell：每条序列APE，ours ON/OFF、SF在线/历史；另给ARE/RPE小表；健康/自然数据分组 | 改善多少、哪里退化；最终历史是虚线且不和在线同排名 |
| F02 / E02 | 三个预选代表的XY＋z(t)或3D轨迹，颜色为实际误差；相同尺度、GT黑线；保留长偏差标注 | 轨迹形状与全局偏移、垂直误差，不用最漂亮数据替换 |
| F03 / E03 | 故障恢复事件带：同时间轴APE/ARE、raw range/IMU诊断、FD/排除/prior/bridge/rejoin；故障区间半透明 | FDE何时发生、误差尖峰与恢复是否同步 |
| F04 / E03 | 分故障类paired-gain森林图或点区间，显示每seed，绝对m与%分列；旁列episode成功/失败计数 | 不是只报一个67%；强/弱与姿态副作用保留 |
| F05-H / E04 | 水平Stanford：PE横、PL纵，ON/OFF同坐标；full AL视图＋小误差放大，log计数色条；qualified/late/unqualified分层 | 左移=精度，下降=界紧；灰色迟到不算可用 |
| F05-V / E04 | 垂直Stanford同规格 | 健康垂直保守性、G2多解和AL局限 |
| F06 / E04 | AL–可用率曲线（H/V/joint分线）＋SI经验CDF与>1尾部计数 | 应用要求变化时的代价，不调整在线算法 |
| F07 / E05 | 窗口–APE/HPL/VPL/P95/RSS折线；旁图PL–latency Pareto，点大小=RSS，标每个(LF,LI) | 1000是不是值得、哪里资源截断；联动/隔离两组不能混线 |
| F08 / E06 | 各方法latency ECDF/箱线＋exclusive均值堆叠breakdown，健康/恢复分组；40/50线和failure率 | 时间花在哪、FDE/PL真实开销；均值堆叠不伪装分位数 |
| F09 / E07 | 消融组件×指标热图（相对基线百分比附原值）或小型waterfall：H/V、状态精度、P95 | 哪个组件改善界、哪个改中心、哪个只省算力 |
| F10 / E08 | 长流四条对齐时间轴：APE/ARE，H/V或bias宽度，P95滚动诊断，slot/RSS；故障竖线 | 短时正例是否延续、晚期故障和资源线性增长 |

主报告最多12个figure family（F05的H/V两图）；不每run保存20张图。每图独立PNG与可编辑SVG可保留，重复格式是论文交付不是过程备份。可有2×2小面板，但避免12个挤在一起的微小坐标。统一180mm双栏或90mm单栏、300dpi、正文8–10pt以上；色盲友好，ours蓝、NOFDE灰、SF在线橙、SF历史紫虚线，GT黑、危险区域少量红。图中额外用线型/符号，打印灰度仍可读。

不用3D柱状图、面积/双轴混合造成尺度错觉、不同run自动轴限掩盖差别、雷达图混合m/ms/%。标统计单位N与CI来源；没有有效数据时画NA/拒绝面板并解释，而不是空白成功图。

**“炫酷”的实现方式是清楚的分组、时间联动、可见的权衡和高质量排版，而不是多加装饰。** 图上任何伪造曲线、GT辅助选择的示例轨迹、删除异常点均禁止。

## 11. 代码落点与最少实现

复用以下真实文件，不新写估计器：

| 现有路径 | SC8工作 |
|---|---|
| `tools/sc7_research.py` | 仿真method开关、窗口配置、input digests、原生run/冻结/评分复用 |
| `research/sc6_system.cpp` / `sc6_service.cpp` / `sc6_provider.cpp` | 同一实际consumer、独立producer、sink计时、truth隔离；外部source适配前检查契约 |
| `research/rolling_policy.hpp` | 正确解析LF/LI，验证100–1000，不通过放开LI>LF假装支持 |
| `tools/run_dataset_benchmark.py` / `benchmark_common.py` | 数据库存、接口审计、CAL/TEST分离、当前runner，而非沿用旧CSV结论 |
| `tools/run_sfuise_benchmark.py` / `benchmark/sfuise_adapter_ros` | 固定版本，首发/历史捕捉、消息/时间/offset一致、完整服务计时 |
| `tools/sc6_closure.py` / `sc6_evaluate.py` | 评分与Stanford复用；不足之处加单元测试后修复 |
| `research/joint_raw.cpp`及tag/support相关实现 | 消融合法组合与观测输出，原则上不在SC8重做数学 |

建议只增一个调度/绘图入口 `tools/sc8_experiments.py`、一个机器可读的最终注册文件`config/sc8_experiments.yaml`（若已有类似就扩展），少量observer与评分测试。本开发包里的`SC8_PLAN.template.yaml`是设计模板，**不是当前仓库已经实现的配置API**。

目标CLI（需实现并在最终报告写实际可用命令）：

```bash
python3 tools/sc8_experiments.py preflight --build
python3 tools/sc8_experiments.py register
python3 tools/sc8_experiments.py run --group E01,E02,E03
python3 tools/sc8_experiments.py run --group E05,E07,E08
python3 tools/sc8_experiments.py evaluate
python3 tools/sc8_experiments.py figures
python3 tools/sc8_experiments.py report
```

E04/E06应消费已有run，不能在`evaluate`暗中重新跑大实验。每个run有唯一ID，由输入、method、window、fault、seed、pace、source/binary身份决定；重复调用只续未完成项或明确重新执行并保留此前失败。复用缓存需全部身份匹配，不能因文件同名就复用。旧运行只列historical，不填新实验单元格。

## 12. 一个连续Goal的开发顺序

| 工作包 | 工作与退出条件 | 保存/继续 |
|---|---|---|
| W0 可比性与登记 | E00；读取新代码/数据，独立标定，确认方法/时间/GT/窗口scope；修必需接口/观测器 | 最小单元测试＋代表短流；commit后自动继续 |
| W1 主对照 | 冻结候选，执行E01/E02/E03；导出SF首次/最终；输入配对与完整分母 | 验证匹配、状态与故障表，commit后继续 |
| W2 深度实验 | E05大窗、E07消融、E08长流；E04/E06收集已有证据 | 资源/保护失效按预注册处理，不以OPEN终止其它独立实验 |
| W3 统计与论文图 | 独立评分核对、E04/E06/F00–F10、报告及一次直接相关核心回归 | 不为追求漂亮结果再调算法 |
| W4 最终审计 | run registry与分母/图表对账、结论资格、冻结SHA、清理与本地Git | 实际交付报告，不仅实施计划 |

本轮允许安全的adapter、时间戳/校准读取、benchmark构建、观测导出、评分器和轻量计数修复。**不允许**为了让某方法赢而改变噪声/风险/AL/故障强度、换不同GT初值、关闭对手官方剔除、混入额外传感器或重写PL定理。不可避免的数学/核心估计修改先独立测试、冻结为新方法版本，并重跑直接受影响的对照；不要边改边把前后数据装进同一最终列。

阻塞一个真实数据适配不阻塞已成立仿真；一个大窗非实时不阻塞正式论文实验。有限原因报告是合法结果，但**仅打印NOT_SUPPORTED、没有读当前接口和尝试适当修正不构成完成**。缺文件/权限先用已存在入口检索，只有不可自动解决的输入才询问。

## 13. 验收 SC8-A01—A24

| 条款 | 验收证据 |
|---|---|
| A01 | 实际入口HEAD/ELF/DSO/平台、旧报告身份、无覆盖用户文件；明确SC8非SC7复跑 |
| A02 | 六条候选及CAL/TEST库存，选择依据不含算法输赢，备用和未选理由透明 |
| A03 | 最新单位、frame、lever、range偏置、时间/批次/GT覆盖；旧失败重新验证，direct/ROS关键链路对照 |
| A04 | GT-assisted calibration与test-involved标记正确；无按测试/方法最优对齐或标定泄漏 |
| A05 | T-SIM/T-REAL/T-SEMI分层，当前固定采样源的guard不伪造放行，requested/effective方法一致 |
| A06 | SFUISE固定官方版本/剔除/编译；首发与最终历史独立，lookahead/state age和算法补丁可审计 |
| A07 | 物理观测多重集、注入和source ID配对，原数据不改；ToA/TDoA、range_raw/range转换无错置 |
| A08 | APE/ARE/tag-RPE/body-RPE、GT转换和缺帧评分独立单元测试，至少一次evo对应口径核对 |
| A09 | E01健康/因果/历史结果实际完成，不用历史结果替在线 |
| A10 | E02最新实测主表与失败/资格表；不能将O-NOM伪称O-FULL，部分未获资格如实记录 |
| A11 | E03 UWB/accel/gyro/联合故障与未注入配对、额外真实注入、全阶段误差/episode恢复 |
| A12 | 按seed/录制统计，负收益保留，场景权重预定；不把相关epoch当独立样本 |
| A13 | E04两张真数据Stanford、AL/SI/完整分母/时间和资格；无PL的方法NA，零超界不当风险证明 |
| A14 | E05 30/100/300/600/1000联动＋固定FGO300隔离实验，实际历元/时域/回放对账 |
| A15 | 大窗边缘化、common prefix/event/评价区间与资源截断记录真实；不要求所有大窗40/50ms |
| A16 | E06统一core/sink/state age/CPU/RTF，exclusive阶段均值与总值闭合，P95不相加 |
| A17 | 默认准入profile双P95及95%及时有效分母，真实经验和数学资格分开；超时不被删 |
| A18 | E07实际合法组件消融，有原生/同center区分及原始风险不变，不用不合格Gaussian收紧 |
| A19 | E08冻结长流ON/OFF、晚期故障、x0退出、原生恢复和资源趋势；无重启清零 |
| A20 | F00–F10真实图表，原值/单位/N/overflow/NA/研究scope清楚，缺数据不伪造 |
| A21 | 一次必要相关核心回归，观测器无副作用、错误FDE/来源/receipt/finite-mode/风险等负例 |
| A22 | 所有run计划=完成+失败+取消+未尝试，保留版本与失败原因，不追求只输出PASS |
| A23 | 唯一主报告、结论JSON、冻结manifest与可复现脚本，出处和限制明确 |
| A24 | 每包本地commit后继续、最终diff审计，清理仅本轮过程文件，不自动push |

最终状态以“实验完成”和“被测系统能力”分开：

- `SC8_EVALUATION_COMPLETE_FOR_REGISTERED_MATRIX`：已按合同完成实际测试和结论；允许结果显示精度退化、某窗非实时、已验证的功能拒绝。
- `SC8_EVALUATED_WITH_EXECUTION_GAPS`：关键方法/数据接口或必需实际运行因无法闭合尚未完成，列出缺失单元格；不能把打印占位符等同完成。
- 系统能力另列：仿真raw scope、真实FDE功能、实测PL资格、P95 profile、1000窗可运行性、恢复成功域、姿态和资源限制。不以全部算法赢或全部HMI=0为结项门槛。

## 14. 报告、文件与运行管理

**唯一中文主报告：** `docs/sc8/SC8_RESULTS.md`，含简短英文Results摘要（真实数值、同样范围）。结构：研究问题与预注册；数据/CAL/TEST/scope；方法和在线/历史；总体精度表；FDE与注入；完整性/Stanford；窗口；耗时breakdown；消融；长时资源；失败/限制；复现与版本。

必要永久交付：本需求/Goal、实际实现与测试、冻结配置/manifest、报告、F00–F10结论图（PNG/SVG）、一个精简`SC8_METRICS.json`。JSON只保留每run汇总、独立统计单元、绘图必要直方图/选定时间曲线与对应数据身份；不是全部矩阵/证书备份。时间曲线展示若抽样需保留峰值/事件，统计仍对全样本计算，规则透明。

完整临时CSV、输入audit、bag overlays、每候选矩阵、编译log放本轮`.cache/sc8/`，当前结果尚未核对前不得删除唯一数据逼迫重跑；复核后清理本轮自有材料。不得复制整个仓库/原始数据集、不改旧报告、不删除用户或旧实验文件。需要外部数据时优先本地，不默认下载百GB。断点恢复用已冻结run ID与原始生成配方，预算耗尽不是Goal完成。

至少每W包有一次有实质内容的Git保存；正式最终运行前冻结代码版本；最终文档SHA与计算SHA分开，禁止虚构自引用hash。任何修复后只重跑受影响组，旧失败和原版本表保留，不能将旧binary曲线标为新binary。

## 15. 引用与责任边界

D1为用户上传历史报告；D2为SC7结果。本文的候选选择、窗口组、种子、资源门限、矩阵与图表是**新的设计建议**，不是来源已经证实的结果。

- [D1] `uwb_imu_accuracy_timing_report.md`：接口/库存L12–74，逐序列L92–188，计时L426–645，入口差异L647–655。文件由用户提供，执行时从项目/本地实际找到，不假设其容器路径等于开发机路径。
- [D2] `docs/fde_next/SC7_RESULTS.md`：C13 sample0范围、C15多根、P95、源law和长期资源限制。
- [C1] [设计时分支HEAD](https://github.com/Printeger/uwb-imu-fusion/commit/fa5b3dc9ffe3c88492352491d79ba022f373bfb5)。
- [C2] [rolling_policy.hpp固定版本](https://github.com/Printeger/uwb-imu-fusion/blob/fa5b3dc9ffe3c88492352491d79ba022f373bfb5/research/rolling_policy.hpp)。
- [C3] [run_sfuise_benchmark.py固定版本](https://github.com/Printeger/uwb-imu-fusion/blob/fa5b3dc9ffe3c88492352491d79ba022f373bfb5/tools/run_sfuise_benchmark.py)。
- [C4] [sc7_research.py固定版本](https://github.com/Printeger/uwb-imu-fusion/blob/fa5b3dc9ffe3c88492352491d79ba022f373bfb5/tools/sc7_research.py)。
- [S1] [SFUISE作者维护库README](https://github.com/ASIG-X/SFUISE)；Li, Cao, Hanebeck, *Continuous-Time Ultra-Wideband-Inertial Fusion*, IEEE RA-L 2023, DOI 10.1109/LRA.2023.3281932。
- [S2] [STAR-loc作者数据说明](https://arxiv.org/abs/2309.05518)；[官方仓库](https://github.com/utiasASRL/starloc)。
- [S3] [MILUV作者项目](https://www.decar.ca/miluv/)；[作者论文](https://arxiv.org/abs/2504.14376)。
- [S4] [evo官方指标文档](https://github.com/MichaelGrupp/evo/wiki/Metrics)。
- [S5] [ESA Stanford–ESA介绍](https://gssc.esa.int/navipedia/index.php/The_Stanford_%E2%80%93_ESA_Integrity_Diagram%3A_Focusing_on_SBAS_Integrity)：传统PE/PL图与all-geometries扩展不同，本轮只实施前者。
- [S6] [OpenAI官方Goals说明](https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex)：目标状态、暂停、恢复及预算边界，不替SC8科学结论背书。

**最终交付应从“实现了哪些模块”转换成“哪些证据支持哪些论文结论”。有利结果与不利结果都需要同样精确地说明。**
