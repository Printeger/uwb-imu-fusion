# SC7 任务需求书
## 联合状态完整性收紧、水平／垂直 PL 归因与困难几何恢复

**英文：** Joint-State Integrity Tightening and Degenerate-Geometry Recovery  
**版本：** SC7-1.0 / 2026-10-14  
**执行：** 一个连续主 Goal，W0→W1→W2→W3；工作包完成后本地 Git 保存并继续。  
**编写时实际核对的远端：** `Printeger/uwb-imu-fusion`，`feature/realtime-uwb-imu-pl`，`43abd3fcb35568259e6b2e4131dfdc909e216c3a`。  
**唯一新结论文档：** `docs/fde_next/SC7_RESULTS.md`。

> 本文是下一轮研发合同，不是新算法已经完成的声明。文中明确区分：用户讨论确定的目标、已审阅源码的事实、以及本轮新增的数学推导与待验证方案。没有重新编译或复跑用户工作站上的实验。启动时以实际本地 HEAD 为准，不回退较新成果。

## 0. 任务边界与成功目标

### 0.1 这次一起开发，但分别验收

| 方向 | 本轮实际目标 | 不能用什么代替 |
|---|---|---|
| HPL/VPL 归因 | 对同一输出中心量化几何、传播、故障解释、外包络与投影的影响；优先解释健康 S0/L0 的较大 VPL | 看 Stanford 散点猜主因，或把诊断反事实当受保护输出 |
| C15 / G2 | 修复平方测距初始化与局部迭代导致的原生数值失败；接入真实 FGO、唯一 commit 和 raw PL | 只跑独立 WLS；通过 GT 选镜像；把所有局部解一律称为有统计资格 |
| C13 / raw bias | 实现带故障解释的多时刻 UWB–IMU 联合约束，安全收缩可辨的非位置方向，争取在主 raw 并集上实际收缩 bias | 只改善 FGO bias 点估计；只证明已知姿态/健康 IMU 小模型；只收缩某个分支而忽略其余分支 |
| 联合 H/V 收紧 | 在不减故障范围、不改风险和噪声的前提下，改善两方向保护界，VPL 优先；保留跨时间／分支／坐标关系 | 只让 HPL 通过5m AL就停止研究；给最终PL乘一个缩小系数 |
| 集成验证 | 保留 FDE ON/OFF、Stanford、实际边缘化／恢复和 P95 时效，必要短流后完成受影响长流 | 只测孤立核；重新开始全项目大规模 campaign |

**一套离散 FGO不变。** 新集合算子是其完整性侧的证明计算，不改成连续时间估计器，不新增传感器，不重建第二套 FDE／数据库／事件总线。连续动力学关系可用于证明，并不改变离散估计架构。

### 0.2 冻结合同

保留现有初始15D观测、ADC、共享端点、bias随机游走、UWB噪声、出生 source ID/q、时间一致总风险和故障自动机。保留每通道同时最多一个合法 episode、跨通道并发及重复故障的原定义；不得擅自要求“两个故障之间必有健康间隔”。原D0/D1运动域、G0/G1/G2坐标、lever、50/500ms与1/3epoch故障族的**真实配置**优先于聊天中的示意数字。

允许 research-only 实现新初始化/求解器、联合收缩、证明接口、来源绑定、原生研究模式和必要兼容修复。生产 prior/risk/AL/正式发布规则不改。未知硬件或应用证据继续未知。

不以 `HPL≤2m / VPL≤3m` 为数学通过门槛；5/5与10/10也只用于后处理。不得降低 sigma、q、故障幅值或缩减覆盖制造收益。空间有效、按时有效、AL数值通过、硬件资格必须分列。

### 0.3 本轮完成状态

分别授予：

- `HV_ATTRIBUTION_COMPLETED_FOR_TESTED_SCOPE`：归因有量化、分组和反事实边界。
- `G2_NATIVE_RECOVERY_VERIFIED_FOR_SCOPE`：原先失败的G2输入形成实际原生闭环；不承诺唯一高度或小VPL。
- `RAW_JOINT_CONTRACTION_IMPLEMENTED_FOR_SCOPE`：新多时刻／多状态约束已进主 raw 链路。
- `MAIN_RAW_BIAS_CONTRACTION_VERIFIED_FOR_SCOPE`：至少一个注册原生场景中，**合并全部允许故障解释后的 bias 集合**出现有证明的严格收缩；不是只缩小一个健康分支。
- `JOINT_HV_TIGHTENING_VERIFIED_FOR_SCOPE`：H/V分别取得合法实测收益，并通过非退化检查。
- `REALTIME_P95_TARGET_MET_FOR_TESTED_PROFILES`：按第11节的真实paced口径通过。

全部必需条款成立才标 `SC7_COMPLETE_FOR_REGISTERED_SCOPE`。缺项用 `SC7_IMPLEMENTED_WITH_OPEN_CAPABILITIES`，具体写 `C13_OPEN`、`G2_NATIVE_OPEN`、`H_TIGHTENING_OPEN`、`V_TIGHTENING_OPEN`、`P95_OPEN_<profile>`，不以“工作包已执行”掩盖能力缺口。

证明某方向确实不可辨识是有价值的研究成果，但不等于这个方向已经恢复／收缩。本文不承诺所有15维、任意故障和任意几何都可收缩。

## 1. 基线核对：需要复用的真实代码

| 文件／函数 | 本次读到的现状 | SC7接入点 |
|---|---|---|
| `research/epoch_raim.hpp::uwbCandidate` | 平方测距差QR产生唯一初值；GN原步长大于10即在line-search前拒绝；局部Gaussian参考随后构造 | G2初始化、尺度/信赖域、分支候选；数值解与Gaussian资格分离 |
| 同文件 `spatialAnchors/epochFde` | anchor差分SVD相对门槛、至少5个range；N+1候选与局部参考有既有证明标签 | 不通过简单调小SVD门槛或增加初值后直接沿用旧统计标签 |
| `research/rolling_raw.cpp` | p/v用物理加速度界；R用角速界；bias单独RW外扩；现有p-v耦合及lever球约束已存在 | 新raw IMU测量关系、联合参数／状态收缩，不重新命名旧功能 |
| `research/rolling_boundary.cpp` | 每批range取9个mask，先做坐标外包络；两轮时间传播；最终围绕actual tag计算PL | 保留局部／跨时间关系，增加相应证书绑定与最终支持函数 |
| `research/raw_range_contraction.cpp` | 上下距离壳、平方差slab、预算frontier，分支box合成全局low/high后计算HPL | 新分支/叶盒表示、二维支持；保持未决frontier覆盖 |
| `research/sc6_service.cpp` | 时序FD与原fault pool共存；恢复／proof／唯一commit；主proof仍不借时序报警认定IMU健康 | 保留现有强故障恢复；新约束先proof-only，改变功能时单独对照 |
| `research/rolling_recovery.*` | 原生prior修复、capsule数值初值/权重、实际重积分与再接入 | bias集合或初值改变可能影响FGO轨迹，要做A类闭环比较 |
| `tools/sc6_closure_observability.py` | 静态tag-only小模型rank 11/15、额外姿态观测正对照；弱故障重叠；G2多初值诊断 | 保留独立oracle与限制，不用未装载传感器的正对照关闭C13 |
| `tools/sc6_closure.py`、`sc6_system_research.py` | 配对、Stanford、P95/分母、源hash、长期评分已有实现 | 扩展薄的SC7入口，不复制全套框架 |
| `config/sc6.yaml`、`sc6_closure.yaml` | G1高度0/4m、G2高度0/.001m；D0/D1及已有种子／故障脚本 | 真实读配置，不根据场景名偷偷改变在线假设 |

上游报告中的 G2 初值约−325.126m、首步约319.827m、独立36/36局部WLS正例，是**诊断事实**；不是36条原生合格轨迹。SC7应重新生成首个冻结输入做最小复现，不要求恢复已清理的CSV备份。

## 2. 本轮新增的数学澄清：先避免错误归因

本节是根据源代码和集合公式进行的**设计推导**。它细化此前“故障并集可能撑大PL”的讨论，不把尚未测出的主因写成既定结论。

### 2.1 当前bias与姿态的直接作用边界

当前raw p/v传播不使用实际IMU加速度减bias；bias范围变大**不会直接通过本公式积分成米级位置界**。bias能经检测、恢复metric和FGO实际输出中心间接影响PL；引入新IMU关系后依赖图会变化，须重做归因。

lever为 `(0.2,-0.1,0.08)m` 时长度约0.2375m。给定body位置，纯旋转对tag单轴偏移范围不超过 `[−||l||,+||l||]`；两个姿态造成的直接tag差异最多 `2||l||`。这不是对整个闭环姿态误差影响的全局上界：反复投影、状态关系或恢复中心改变仍可能间接增宽。不能把数米VPL全部归给lever，也不能完全排除姿态的间接作用。

### 2.2 垂直集合半宽与中心偏移

对固定输出中心 `z_hat` 和区间 `[z_min,z_max]`：

`VPL = r_z + |z_hat - c_z|`，其中 `r_z=(z_max-z_min)/2`、`c_z=(z_max+z_min)/2`。

这是精确代数分解；实现仍外向舍入。其他“传播／故障／投影贡献”一般不是独立可加项，不能按某次删约束的差值拼出100%饼图。

### 2.3 **HPL与VPL对立即取hull的反应不同**

给定同一个中心c、同一批最终分支集合Q_j：

`sup_(x∈union Q_j) |x_z-c_z| = max_j sup_(x∈Q_j) |x_z-c_z|`。

仅把这些Q_j的z投影取最小包围区间，**不会改变当前精确的最大垂直误差**。即使中间有空洞，最远端点仍相同。因此“把每分支VPL分别算完再取max”本身不能改善当前VPL。

但水平欧氏范数不同。设某分支坐标最大偏差为d_xj,d_yj，则

`HPL_branch = max_j hypot(d_xj,d_yj) ≤ hypot(max_j d_xj,max_j d_yj) = HPL_global_box`。

后者可能拼接互不共存的x/y极值。对每个分支已经是矩形的情况，前者是这些矩形并集的精确最大水平范数；它仍可能大于真正非线性可行集的最大误差。

**所以：**HPL可以先尝试更紧的分支／叶盒输出支持；VPL要真正减少合法外包络的z极值，例如更强测距约束、跨时间相容性和更少投影损失。取hull造成的时间／状态关系丢失，可在**以后**的传播和再次收缩中增宽VPL；必须用至少两次更新的对照检验，而不是声称当前取max就有效。

### 2.4 G2要区分三件事

anchor平方差矩阵的病态、当前位置测距Jacobian的局部秩、全局镜像可辨识性不是同一概念。近共面并不意味着所有off-plane位置的range Jacobian秩亏；满局部秩也不保证上下两支全局唯一。

G2保持既有0/.001m几何。不能把“分层仅1mm、range sigma .1m”直接当成所有垂直信息为零的证明。需要同时计算差分设计矩阵与局部真实range Jacobian，验证噪声壳内是否容许多个位置分支。

## 3. W0 — H/V归因与冻结实验协议

### 3.1 零侵入基线

启用只读诊断时，同一输入的FGO state、FD/FDE动作、raw证书、风险和发布语义应保持不变；新增计时单列。首先只用S0、S1/S2及G2首失败epoch，随后两个后期快照，不再复跑全SC6 Closure。

在W0结束前冻结：开发seed `20261010`、确认seed `20261011`；快照集合S0/31与250、S1/61与300、S2/61与300、G2/1。必要时复用原场景实际长度，不改变已有故障幅值。确认seed不用于挑方法／阈值。

### 3.2 同中心过程记录

每个快照保持原始measurements、source q、父证书、fault域和最终actual tag相同，记录：

| 节点 | 记录及解释 |
|---|---|
| 上一separator与传播后 | p/v/R/bias、tag范围、H/V支持，识别传播增长 |
| 每个UWB mask／每个叶盒收缩后 | H/V、z低/高端点、x/y极值、导致端点的branch/source ID |
| 取global hull前后 | 当前HPL人工角点差；当前VPL不变恒等式；不把必需fault覆盖当算法错误 |
| p-v／lever／前后时间收缩后 | 各算子在固定次序下的变化；记录重叠作用，避免重复归因 |
| separator投影 | 当前坐标端点是否变；再传播／再次观测后是否因丢相关而增宽 |
| actual tag绑定 | VPL半宽/中心项；HPL记录box大小和中心偏移，但不虚构唯一可加分解 |
| 数值资源 | passes、visited boxes、frontier、fallback、精确数值身份和exclusive耗时 |

被识别为决定性端点的分支，应给出“可行见证／仅外包络极点／无法判断”标签。外包络端点不一定是真实可达点；不能说它必定是物理下限。

### 3.3 独立反事实，禁止进入发布

可以分别诊断：所有range健康、固定姿态、缩小bias支持、去掉某些相关约束、使用不同q、强几何。每个改变均须标记 `DIAGNOSTIC_ONLY / NOT_SAME_CONTRACT`；对真实输入直接减sigma、q或使用GT不得签发新资格。仅全局合法放松的R1仍可作保守参考。

尤其需要一个保持同父p/v/R、同中心而只改变bias区间的核测试，核对当前raw p/VPL不存在直接bias依赖；它不是允许生产忽略bias或证明FGO中的bias无关。

### 3.4 运动×几何的控制实验

执行 `M0/G0、M0/G1、M3/G0、M3/G1` 四个短前缀，默认300epoch、一个开发seed。**几何主对照使用共同的已声明且能包住M0/M3的D1运动域**；另保留原M0/D0/G0作为legacy参考。当前domain可能由recipe.kind决定，需要隔离研究override并同时传给所有消费者，不能只改名称。

同一运动跨geometry时使用相同标准化噪声draw、初始观测与故障recipe；range数值必然因anchor改变，不能声称raw bytes完全相同。G0/G1高度差都为4m，实际geometry文件/代码坐标才是依据。分别报告geometry、motion与motion-envelope变化，不能把L0与L1直接作为单因素因果比较。

### 3.5 W0出口

一张H/V归因表：有证据的主因、交互项、尚未分离项、预期有效算子，以及固定的后续评价时刻。必须允许结果否定“bias是当前VPL直接主因”等猜测。W0完成后继续W1/W2，不以写出诊断表结项。

## 4. W1 — C15：几何退化感知的数值恢复

### 4.1 只修求解机制，不偷偷改变观测

保留原始range残差与其正确保留子集协方差，原默认算法作reference。新研究路径可加入尺度归一化、QR/SVD、LM／trust-region或阻尼GN；不更改sigma、不加假高度／姿态测量、不删除难处理anchor，不简单把10m步长门槛改成几百米。

`step>10`在当前line-search之前触发，是需处理的机制。使用信赖域求解一个可接受的小步与接受准则，而不是先产生巨大不可信GN步后放宽资格。最终在最终点重新计算residual/Jacobian/所需参考，不能因循环提前结束保留上一点的统计量。

### 4.2 确定性多初值与边界

最低实现：anchor几何SVD诊断＋确定性有界候选池。可包括旧平方差seed（适用时）、anchor质心/尺度得到的几何seed、冻结预定义的平面两侧seed，以及来自**已验证**预测集合的代表点。必须标明来源与稳定tie规则；seed数量、迭代/分解/成本上限在长流前冻结。

纯UWB A路径若使用融合先验seed，须单独标为混合候选器；不可继续称“A完全纯UWB且保留旧先验独立性”。推荐A用observation-only多初值，联合候选再使用合法边界辅助。预测mean可用作数值起点，不自动充当健康观测或证明剪枝依据。

多初值不必枚举所有真实解才能得到一个合法估计中心；**但未被数值求解发现的可行分支不能从raw保护域消失**。最低目标是找到可执行候选，最终raw集合独立覆盖全部允许状态。不得把“某seed未收敛”写成该物理分支无解。

### 4.3 数值候选与局部Gaussian资格分开

新增多个data-dependent初值、根选择、局部限制或融合先验，会改变局部统计选择事件。原`FROZEN_MODEL_ONLY`、旧N+1计费、检测alpha不能因接口字段相同自动继承。

为每个结果分开记录 `NUMERIC_CONVERGED`、`FUNCTIONAL_POST_TEST`、`RAW_SCOPE_QUALIFIED`、`GAUSSIAN_REFERENCE_QUALIFIED`。新多根选择可先作为功能FDE诊断；raw保证由对所有真状态/实际center成立的集合链承担。若要沿用某Gaussian检验的概率解释，必须补其固定模型/多根选择条件证明；不能由WLS objective过门槛推断global raw正确。

### 4.4 镜像与原生闭环

对上下镜像均与已知raw模型相容的情况，raw保护必须保留两支或其有证明外包络；不可使用“机器人应在地面上方”作为未声明新信息。已注册初始law若确实排除一支，需独立证明，而不是按GT选择。

处理流程：原始batch→全部合法fault候选与数值seed→候选收敛/完整支持核查→同一FGO实际行动→至多一次原生commit→完整raw集合绑定actual tag。所有新数值成本计入P95。

W1正常目标：旧G2首失败输入上不再因单一坏seed失败，完成原S3/G2 300epoch的原生连续正例，含x0退出、完整raw资格/时效分母。无需VPL<AL；若真实信息仍不允许有效服务，必须分别给出数值已修复和物理/证明仍阻塞的结果，C15不能因“拒绝得更好”被标完全完成。

## 5. W2 — C13：带故障分支的多时刻raw联合约束

### 5.1 主估计与主证明是不同计算

FGO bias估计/协方差不得直接作为raw真值集合。新proof算子先不反馈估计器，不重写现有时序差分FD；由此避免把已经恢复的48/48强episode能力重新混入未知数学修改。

先构造5–10epoch独立局部子问题验证符号、source、fault参数和可观性，再接入现有30epoch窗口。局部成功不等于主流水线C13关闭；仅10epoch窗口也不一定含健康片段，不能因此假设某段IMU无故障。

### 5.2 需要保留的关系

在统一body/world和gravity符号下，核对真实模型：

`y_a = R^T(a_w-g_w)+b_a+f_a+n_a`  
`y_g = vee(R^T dot R)+b_g+f_g+n_g`  
`z_u = ||p+R*lever-anchor||+f_u+n_u`  
`b_next = b + w`，并包含位置/速度积分关系。

离散实现必须与点采样ADC、RW跳变、跨PIM共享端点和预积分定义一致；非线性/连续时间/浮点误差有可检查的外包络。当前bias只是RW扩张，新约束不能把其更新偷换为独立高斯后验。

对同一物理episode保留未知共同theta，跨相邻样本/预积分/窗口共享。当前允许任意幅值时，不可偷偷把theta限制到注入幅值附近；无法处理无界参数时合法消元、保守放松或拒绝，不得硬截断。

### 5.3 分支安全组合与递推

每一故障解释h内，联合约束形成`F_h`，新算子满足 `F_h ∩ X_h ⊆ C_h(X_h) ⊆ X_h`。最终需要覆盖 `union_h C_h(X_h)`。若资源不足，保留未处理解释的旧合法外包络；预算超限不能只保留处理过的“好分支”。

局部健康分支可以使用f=0；最终不能因为FD PASS、FDE选择、局部拟合更好而令其它分支为零。已报警的完整支持也不是知道真实轴/起止的oracle。未观測/当前末端可能有fault的轴，要保留bias/fault可互换解释。

复用完整性rolling separator，将相关state/theta/shared source跨真实FGO边缘化传递。保守降阶允许丢紧致性，不允许丢真轨迹。若只收紧当前窗口而没有同步传播到新separator，下个窗口重置bias范围，不能授予主C13通过。

### 5.4 两条可实行研究路线

**J1：联合必要条件收缩。** 少量状态/故障块上，用区间积分、p-v关系、raw IMU方程和UWB壳做前后传播；保留R/lever和共享theta，不先造很大的15D一般全局求解器。

**J2：有限相关表示。** 若独立box损失主导，用带来源生成元、受约束仿射集、低维多面体或分支box保留必要相关性。根据W0选需要保留的量，提供投影/降阶外包证明，不为追求优雅重写整个估计器。

至少实际实现一种新joint关系并接入主proof。若第一条未收紧，定位是模型不可辨、外包络过松还是编码问题，必要时尝试另一条有证据的路线；不是无目的调参数或盲增迭代。

### 5.5 可观性、实际收益与C13判定

独立测试至少包括：健康/已有信息正例、仅tag静态yaw gauge、bias与fault重叠、弱故障ADC重叠、旋转与重力耦合、原始初始law不同但同观测的合法解释。不能把添加独立姿态传感器的小模型结果直接用于主系统。

仅局部Jacobian rank不足是局部证据；声称全局“不可能辨识”须构造两种满足全部公开观测/运动/初始化/source/fault范围的不同状态历史。噪声支持重叠否定的是零错误的确定分离，不自动证明任何概率检测都无功效。

记录收缩发生在bias哪个轴、哪个时间、哪个分支以及**全故障并集合并后的宽度**。主C13正常成功至少要有原生完整故障域中某bias方向的严格收缩、跨窗口延续、同事件证明及actual输出绑定。只减小某个分支、仅收紧速度或更换FGO bias mean时，分别记录其成果，但 `MAIN_RAW_BIAS_CONTRACTION_OPEN` 不得清除。

如果完整合同下某方向确实不可辨识，保留它并继续其它可辨方向和H/V方法；不要加健康间隔、无故障prior或GT reset来制造成功。

### 5.6 时序FD与恢复的兼容

Closure已有相邻样本必要条件，避免初始bias累计对强阶跃FD的退化。本轮不要将其重新当作未开发，也不要未经证明用当前样本更新的bias集合再次“独立检验”同一样本。

若新集合用于功能检测，需使用上一已提交证书或有整体联合包含证明的更新；若新capsule影响prior metric和恢复center，标为算法闭环变化。必须保留旧早/晚、重复、非x轴、超replay与真实prior修复的必要回归。

## 6. W3 — 共同的H/V收紧方法与真实输出绑定

### 6.1 优先路线H：保留当前分支／叶盒的二维支持

不先把所有分支的x/y极值拼成一个矩形；在相同actual center下对每个**覆盖所有未决点的**分支／叶盒计算最大水平范数，然后取max。对box是四个水平角点的最大距离，等价于`hypot(max_abs_x,max_abs_y)`；不需要另写全局优化器。

最低独立测试：`Q1={(3,0)}, Q2={(0,4)}, c=(0,0)`，全局box给5，真实并集给4。此例验证算法关系，不代表真实轨迹必有同样降幅。

当前range kernel在若干位置使用零向量临时计算距离，最终PL又以actual committed tag重算。新HPL必须保存可迁移到最终center的集合/支持描述，**不能在原点算出半径后直接贴给另一个center**。同分支所有保留盒和资源耗尽frontier都要计入；仅汇总求解成功的盒不合格。

若进一步采用支持函数／多边形上界，必须证明多边形外包真实xy投影；有限方向采样的最大值通常不是所求最大范数的上界。禁止把采样结果或局部最优化的最大值当作认证HPL。

### 6.2 优先路线V：真正缩小z可行极值

按W0证据选择：更紧range shell联合约束、保留部分几何分支、跨历元一致episode及theta、联合状态约束、较少投影损失。允许有预算的区间分支求界；处理不完的frontier必须仍覆盖，不能据未搜到可行点删除区域。

VPL不能靠“先算每分支再max”单独获得收益。应证明`z_max`降低或`z_min`提高，或明确区分真实估计中心改变引起的变化。若极值有合法真状态／fault/source见证，保留它；若只有松外包络则研究更强必要约束。

### 6.3 跨时间fault相关性

当前每批9个mask是原物理episode范围的逐帧外放松。更紧表示可维护有限时间段内的source、开始/结束、remaining duration和共享theta；不同epoch之间只能组合相容的分支。

不能把全部30epoch强行假设为同一个故障anchor，也不能在一次episode结束后忘记其状态污染；允许的重复／紧邻episode必须保留。未来新故障、未报警故障、跨separator未结束故障及其未知幅值均有对应状态。若旧父证书已丢相关，新的细分只能保守覆盖其所有可能历史，不能凭空恢复一个“真实故障历史”。

实现可以采用符号自动机、分段共享参数或保守合并；不要求存全前缀或枚举指数数量的完整轨迹。合并可以变松，但必须标明是否丢失区分／单独恢复能力。不按内存容量无证明地top-K删分支、不给未知概率置零。

### 6.4 证书、集合与标量界一并升级

新增`bound_method`及其版本化支持证据，复用既有source/transaction/receipt。至少绑定：父证书、完整原始batch/IMU支持、fault scope、event/q、几何/frame、center、集合/分支外包络、roundoff与预算策略。可用旧字段作兼容表示，但不得由旧box seal认证未包含在seal内的新HPL。

区分“用于下一窗口的state外包络”和“当前tag更紧的输出支持”；它们可以不同，只要均覆盖对应真状态并一致绑定。若下游假设`HPL=hypot(PLx,PLy)`，必须改为可验证的method dispatch或继续保守输出旧式HPL；不能只改打印数字。

保留旧安全raw方法为oracle/fallback。只有新旧界各自在同一事件/故障范围/时刻/frame及actual center上成立时，才能使用其交集或逐指标较小有效界；要记录实际回退/组合方法和全部成本。未raw合格的Gaussian数值PL禁止参与此合并。

### 6.5 不同结果分开比较

- **M类（数学）**：相同父证书、全部输入、事件、故障域与actual center，比较新旧H/V、集合和bias支持。
- **A类（闭环）**：新边界/求解器改变恢复或实际轨迹后，比较原生state、FDE、PL、风险和精度；不能用M类非加宽结论要求两条不同轨迹所有PL严格排序。
- **P类（性能）**：固定数学、候选与scope，只改工程执行；逐语义比较state/action/PL/source/risk/receipt，物理slot若不同需完整稳定ID映射。

W0→W3沿用这些分类。几何/运动/q诊断反事实另列，不混入同模型百分比。数学新界在同输入同center上两方向均不加宽（数值外舍入界单列），并分别至少取得一个注册实场景HPL、一个VPL严格改进。不能把小fixture收益当主系统收益。

## 7. 统一数学保证与独立验证

### 7.1 保留原时间一致source事件

继承当前全局出生block分配与`epsilon=1e-6`，不重新设计风险合同。相同原始source跨窗口重复使用同一出生q；引入新constraint不再假设独立，也不重复收费。使用新原始关系若需要前所未计入的随机源/模型误差，必须补充其合法处理或维持资格拒绝，不能忽略。

目标形式保持：在声明的全部噪声／初始化／运动／fault范围内，所有被签发实际tag的位置超界事件均包含于原事件失败集合。数学收缩须在该共同事件上对全部合法历史和数据驱动选择同时有效。

重复使用一条确定性集合约束可以是幂等操作，不自动增加独立信息；错误是把同一个样本复制成多个独立noise likelihood，或据其重复消费减小置信范围。

### 7.2 三项核心算子义务

`Reach(X,U) ⊆ PropagateOuter(X,U)`；  
`X∩F ⊆ ContractOuter(X) ⊆ X`；  
`Project(X) ⊆ ReducedOuter(X)`。

全局`R∈SO(3)`、动力学符号、连续运动余项、姿态chart转换、坐标投影、共享故障参数、浮点外舍入及fault union都包含在义务中。近秩阈值、不确定PSD、overflow、空集、边界舍入要有可重复负例。使用区间库时检查实际编译flags和线程舍入环境；不为速度开启破坏外包络的fast-math。

### 7.3 最小独立测试族

| 族 | 必测内容 |
|---|---|
| K1 PL恒等式 | VPL半宽+中心偏移；同集合union→hull的VPL不变；HPL人工角点3/4/5反例；最终center更换 |
| K2 跨时相关 | hull先做后约束与分支先约束再合并的差异；至少两时刻例验证投影损失而非伪造当前VPL收益 |
| K3 G2求解 | 原首失败输入、全部9mask、固定多seed、QR/SVD尺度、LM接受/停止、near-range-zero、共面与近共面区别 |
| K4 镜像 | 两支合法noise实现；局部rank与全局歧义分开；只找到一根不能删另一根；GT标签非干扰 |
| K5 联合state/bias | 5–10epoch原型、姿态/比力/RW/共享端点、健康和fault重叠、无界theta与重复episode |
| K6 主C13 | 不用truth/FGO covariance收紧；全fault并集bias确实收缩或合法反例；下一窗口仍消费该约束 |
| K7 安全资源 | 任意时刻触发预算，未处理frontier仍在；全分支矛盾、失效父证书和scope/receipt篡改正确拒绝 |
| K8 原生 | x0退出、真实prior修复/接回、错误FDE、缓冲外拒绝；每epoch至多一次commit |
| K9 统计/输出 | A/B/数值根选择不偷继承Gaussian资格；Stanford分区、AL后处理、P95分母 |

独立oracle优先解析小问题、高精度端点、不同算法的区间/稠密参照。两个都松的外包络不必互相包含，不能只看rolling是否包含另一个松盒就替代算子证明。局部WLS、稀疏采样、数值Jacobian秩都不单独证明完整raw风险。

## 8. 实验安排：少量互补验证，不重启大型campaign

### 8.1 冻结顺序

W0冻结输入、假设、诊断快照与评价尺度；W1/W2开发时只运行相关K族和1–2个快照。方法/配置冻结后才进行以下最终矩阵。旧Closure结果仅作为历史参考；新方法受影响的结果必须在新版本实际运行。

| 组 | 最小运行 | 目的 |
|---|---|---|
| A 归因 | §3四个运动×几何短前缀，各300、一个seed；关键快照细分 | 消除geometry/motion/domain混淆；同中心H/V归因 |
| B G2 | S3原300输入，开发seed+确认seed各一次新方法；原首epoch失败参考即可 | 真实首步修复、300历元、边缘化后闭环；镜像负例另作fixture |
| C 数学与bias | 注册5–10epoch正负例；S0/S1/S2两个seed各一条400以内新旧方法对照 | 主C13与H/V；不重跑五种子全矩阵 |
| D FDE/窗口 | 至少S1/S2两个seed，最终ON/OFF同源paired；20/20、40/40、30/10各250一次 | 保留FDE收益/姿态退化，恢复和P95；与C可合并同一次输出 |
| E 连续 | L0健康5000、L1重复UA/UG10000，最终方法各一次paced | 继承过去source、原生恢复、PL/bias趋势、资源/P95 |
| N 负例 | 现有弱/错FDE/超replay/取消/过载与K族受影响子集 | 不用所有旧测试笛卡尔积 |

长流若方法改变恢复中心／FDE策略，L1补一个同源OFF作为新长期精度声明的依据；仅当前tag支持改变而功能轨迹严格相同可用短流ON/OFF＋长流新界，不伪称重新测了长期FDE提升。可复用的结果必须对应同源码功能身份、输入/策略和相同计时边界；旧总体平均数不能生成新散点。

每日不跑5000/10000；最终冻版才做必要长流。一份trace同时出精度、PL、恢复、P95和资源；不为每指标另跑一次。没有新的纯性能候选不强制35/140次配对。受影响bug修复只补对应行，保留旧失败与未尝试分母。

### 8.2 成功不靠挑场景

S0/L0健康的VPL短板必须保留；不得只展示S1/S2中的有利改进。预定义G2和联合fault仍是目标，不改几何来让G2“通过”。确认seed失败要保留。构造有激励的独立小例可帮助证明方法，但须和主S0/S1/S2的实际效果分开。

如果某个预注册边界确有合法极值见证，不能要求算法消除它。报告“此条件下有真状态达到该界”与“当前算法尚未消除虚假极值”，后者不得自动叫信息论下界。

## 9. 评分与图表：同时看H、V和全状态

保留Closure的世界tag末端APE/RPE，不用事后轨迹对齐、GT选根或最终历史平滑替换在线实际输出。FDE ON/OFF保持同seed、初始观察、raw字节／来源、运动/geometry和paced策略，区别仅是合法行动开关；如果新算法同时改变proof方法，比较器应明确固定它。

报告位置/HPE/VPE、速度、SO(3)误差、各轴bias的RMSE/P95/max、恢复前后阶段、paired缺失分母。PL报告同中心新旧值、原生新旧值、bias全并集与分支值、q、区间大小、极值分支、fallback和资源。

继续水平/垂直Stanford图：x为实际PE，y为同stamp/receipt的PL；同尺度新旧或ON/OFF，原全范围图+可选局部放大，不裁掉超界点。AL2/3、5/5、10/10仅后处理。空间有效但超时点可作灰色参考，未资格/缺失另计，不冒充正常可用。MI/HMI仅研究反事实分区，不称运营事故或硬件资格；零underbound样本不能证明10⁻⁶风险。

最多保留4张新图：H归因/新旧Stanford、V归因/新旧Stanford、bias/非位置收缩、G2/恢复或P95最有信息的一张。表格必须独立给出结论和所有失败分母，不能只有图片。

## 10. 集成接口与防止证明绑定倒退

优先扩展已有结构为少量read-only/版本化字段，不创建第二套结果体系。最低需要：

- `AttributionSnapshot`：父集合、stage、branch/source、极值、method、q及固定center；只用于诊断。
- `NumericalCandidateResult`：初值来源、几何诊断、最终根、收敛/post状态、局部Gaussian资格独立字段；不是raw证书。
- `JointConstraintEvidence`：状态／theta／源关系、投影与预算、支持范围、与父证书的包含关系。
- `TagSupportEvidence`：box/branch-union/support表示、H/V上界、actual tag/receipt绑定及可靠fallback。

这些是拟新增语义，不是当前已存在类型名。字段密封/hash不能替代数字关系验证；源风险/center/分支改变必须失效。若旧验证器要求HPL等于坐标box半径，新增紧界需更新方法校验，不能靠关闭检查通过。

当前`Sc6Service::step`中恢复行动在proof之后／之前的真实依赖必须核对：新proof改变recovery seed/metric则A类比较，不能声称nominal完全不动。后端已成功commit但新证据失效时，禁止二次update补证；保留未受保护实际估计及明确失配状态。

工具入口建议扩展薄的`tools/sc7_research.py`或现有脚本子命令为`check / attribute / geometry / joint / final / report`。它们是**待实现接口**，结果里写真实命令，不预先称可执行。新默认research profile opt-in，旧Closure入口继续工作。

## 11. P95与资源合同

沿用用户确认的验收：`P95(core)≤40ms`、`P95(measurement-to-sink)≤50ms`；nearest-rank定义 `sorted[ceil(.95*N)-1]`。包含所有可测已接纳服务的慢帧和超时，不只对合格/按时行算。正例必验profile的`timely_spatial_qualified/scheduled≥95%`；故意负例单独评价正确拒绝。

保持真实200Hz IMU/20Hz UWB独立绝对节拍，计入候选多seed、raw constraint、来源验证、证书、单commit、清理、排队与sink实际交付；GT评分和研究文件I/O单列。不能把慢证明放后台先发布，不能降低输入频率、扩大期限或切窗口掩盖失败。

G2新正例同样记录P95：若数值恢复成立但P95不达标，分别报告，不取消G2分母。短fixture样本P95不是WCET；全部实测最大延迟、miss、gap、abort与功能拒绝均保留。

资源至少报告active states、slot/live/ledger、raw replay、capsules、fault解释/叶盒/生成元、source身份、RSS、累计维护费用。窗口默认30/30，20/20、40/40、30/10仍应正确工作。禁止为了C13保留无限raw前缀或给每个历史fault做完整优化；超限保守合并/fallback/拒绝，不概率剪枝。

本轮不要求顺便完全解决所有slot长期增长；保持其OPEN边界并防止新方法引入未声明爆炸。只有实际证明和长流实测支持，才改长期资源状态。

## 12. SC7-A01—A20 验收

| 条款 | 必须交付／正常通过条件 |
|---|---|
| A01 | 实际HEAD、AGENTS/构建/旧配置核对；原噪声/risk/fault/AL/发布保持；新scope opt-in |
| A02 | 同中心stage归因、极值来源和VPL半宽/偏移；互相作用项不虚构独立百分比 |
| A03 | 2×2 motion/geometry共同域对照；G0/G1/G2及局部rank/全局歧义不混淆 |
| A04 | 当前hull的VPL恒等式、HPL跨支角点反例和跨时间损失独立测试通过 |
| A05 | G2稳健初始化/尺度/信赖域实际实现，不以放宽旧步长门槛代替 |
| A06 | G2完整9mask原生候选、第一epoch及300历元闭环、x0退出、实际一次commit与有效raw界；不再只是36个离线WLS |
| A07 | 镜像/未知根不丢、数值解与Gaussian/raw资格分离、seed多重选择影响有处理 |
| A08 | 新raw多时刻state/IMU/故障关系和连续/数值包含证明，已进主滚动链路 |
| A09 | 至少一个主注册场景bias全故障并集严格收缩且跨窗传递；若只其它状态/健康分支则C13保持OPEN |
| A10 | yaw/bias/fault/weak overlap独立反例，scope内不可辨识不靠虚假健康或GT解决 |
| A11 | 一套共同集合支持H/V，HPL分支／叶盒上界和VPL极值均正确，实际center绑定 |
| A12 | 注册原生场景HPL、VPL各有严格合法改进；同center双方向不加宽（已认证舍入差异单列）；所有反事实与退化分开 |
| A13 | episode重复／跨边界/theta/source q继承正确；旧父集合不凭空恢复历史，预算frontier不丢 |
| A14 | 新证据/表示/center/receipt完整绑定；错误FDE、故障后commit失败、回退均安全 |
| A15 | M/A/P对照及同源FDE ON/OFF、全状态精度、阶段恢复分母；不把PL收益冒充APE改善 |
| A16 | 新水平/垂直Stanford及AL敏感性、未资格/超时分母正确，无聚合指标合成散点 |
| A17 | 注册正例paced双P95达标及及时空间有效率≥95%；窗口变体、所有max/miss明确 |
| A18 | 必要长流、真实边缘化/重复恢复/历史prior修复、资源增长与无全前缀依赖核验 |
| A19 | 相关独立K族及一次必要核心回归；只对受影响行补测，旧失败/未尝试不删 |
| A20 | 唯一SC7_RESULTS含每条PASS/OPEN、证明范围、真实版本/复现；必要代码测试配置、少量图及本地commit/清理 |

A09未通过不以“A08已有非位置约束”替代C13；A06未通过不以“A05求解器写完”替代C15；A12仅HPL改善不等于VPL也完成。严谨反例可以完成相关研究条款，但未获实际能力仍OPEN。

## 13. 执行与停止规则

一个Goal按W0→W1→W2→W3连续进行；W1/W2可在已稳定接口上交错推进，不同时让多个代理破坏同一工作树。阶段commit不是终止信号，不需要用户重复授权普通研究修改。数学／函数失败要诊断、调试或尝试有依据的替代，不能第一次无收缩就收尾。

不因硬件AL、真实数据、原Gaussian raw定理未完成而阻止本轮条件仿真。也不靠凑时长、重复回放或无限调参制造进展。只有全部要求实际完成，或真实预算/权限中断，或有新的明确反例显示当前剩余路线受约束且其它可安全工作已处理时交接。中断不是成功。

在`SC7_RESULTS.md`只保留当前结论、准确OPEN义务、最新实现commit、冻结矩阵和下一条可执行命令；恢复继续同一Goal，不重写整套规划或回到SC1。可证明不可辨识但C13未闭合时，保留研究结论与尚未取得的能力，不向用户声称“全部解决”。

## 14. 产物、Git和资料使用

长期只保存必要源码/参数化测试/最小配置，本需求书、主Prompt和**一个**`docs/fde_next/SC7_RESULTS.md`，以及最多4张新增结果图。旧Closure报告只加入口，不复制或覆盖其事实。临时raw audit、矩阵、box、CSV、日志置于本轮独占`.cache/sc7/`；冻结配对还要用时保留，摘要/分母核对完成后清理，不为干净树删除未完成代码。

每个有实际增量和直接验证的工作包显式stage相关文件并本地commit，自动继续；不自动push、不盲目`git add .`、不破坏性reset/clean、不重写用户历史、不新增数据／二进制／仓库备份。启动时用户有未提交内容要保护，不强制clean。

复现命令必须对应真实CLI和真实完成的构建；检查ELF/DSO身份，不让旧可执行文件冒充新方法。结果报告分别记录被测源码与文档SHA，不伪造自引用。

### 14.1 来源事实与本轮方法建议

固定源码前缀：`https://github.com/Printeger/uwb-imu-fusion/blob/43abd3fcb35568259e6b2e4131dfdc909e216c3a/`。

主要已读取来源：

- `research/epoch_raim.hpp`：平方差seed、`spatialAnchors`、GN步长/收敛与局部Gaussian处理。
- `research/rolling_raw.cpp`：物理传播、RW、p-v/lever必要约束；无主bias观测收缩。
- `research/raw_range_contraction.cpp`：9mask、box/叶盒、预算与坐标hull计算。
- `research/sc6_service.cpp`：Closure时序FD、proof、恢复与原生commit消费者。
- `tools/sc6_closure_observability.py`：小模型rank、弱故障支持重叠、G2多初值诊断；其结论范围不得放大。
- 上游`SC6_CLOSURE_RESULTS.md`及用户提供的结果/图：C13/C15 OPEN与P95基线属于报告级证据，本规范不是独立长流复验。

本轮提出的§2数学澄清、joint约束与候选策略需由实际实现与K族验证，不能视为已发布算法已认证。

### 14.2 文献只借工具，不借资格

用户提供的 *Brunovsky Decomposition for Dynamic Interval Localization*（Rohou与Jaulin）将动态关系分解为非线性代数约束和积分链，并通过contractor缩小可行轨迹集合；可以借鉴时间耦合方式，**不意味着本工程的SO(3)、反复fault、source事件与滚动投影已自动成立**。

用户提供的 *Safety-critical LiDAR-inertial odometry with on-manifold deterministic protection level*采用UBB及流形椭球集合方法；可借鉴表示和流形处理，**不能把它的噪声、LiDAR量测或故障范围替换成本工程既有合同**。本规范只依据提供文稿的摘要/方法范围，不移植未逐项复核的定理。

GTSAM官方LM文档是求解器方法参考，不保证全局根完整：
https://borglab.github.io/gtsam/levenbergmarquardtoptimizer/

IBEX官方contractor文档用于核对收缩/并集原则，不强制引入新依赖：
https://ibex-team.github.io/ibex-lib/contractor.html

OpenAI官方Goal使用说明只支持启动/暂停/恢复语义，不为数学目标背书：
https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex

**最终要回答：当前H/V为什么宽、哪些解释可合法排除；G2能否真实运行；主bias是否真正收缩；这些改动是否保持故障覆盖、实际精度与P95。不是再增加几个结构体或新的完成标签。**
