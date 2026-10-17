# SC9 — SC0–SC8 成果的统一真实传感器系统集成
## Unified Rolling UWB–IMU Integrity System for Real Sensor Data

**正式开发合同：SC9-R1 · 2026-10-17**  
**仓库／分支：** `Printeger/uwb-imu-fusion` / `feature/realtime-uwb-imu-pl`  
**编写时实际核对的远端：** `979242d22d4d5fb84bb883976cc1c63d5860e2b0`。Walk Follow-up 的较新本地实现以用户结果报告和启动时本地 Git 为准，不回退到远端。  
**执行：** 一个连续主 Goal，W0→W5；阶段验证后本地提交并继续，不自动 push。  
**唯一新结论文档：** `docs/sc9/SC9_RESULTS.md`。

> **主目标：把已经开发的增量 FGO、当前历元检测、UWB/IMU 物理故障池、实际 FDE 恢复、跨边缘化滚动集合、时间一致风险与 HPL/VPL，变成由仿真、bag/CSV 和 ROS 数据共同调用的一套核心。真实数据不能再被默认改道至旧通用重型 FDE。**
>
> 本文的“现状”来自已查阅代码和 Walk Follow-up 报告；其余为本轮设计要求、待实现算法和待验证证明义务，不表示它们已经完成。运行时锚点数量不再限定8个；但“支持任意有限数量输入”不等于“任意数量、任意几何都可定位、可隔离、可实时”。

---

## 0. 为什么这轮必须改变目标

现有 `Sc6Service` 已组织了拟发表的滚动算法，但与仿真源的固定形状耦合：每50ms接收10个新IMU样本，边界含11样本，8条测距、9种当前UWB分支、18种功能动作，原始噪声源和故障起点使用固定索引。真实 `nominal_dataset_runner` 调用的是 `RealtimeIntegrityPipeline`；后者不是上述滚动完整性服务。[C1–C5]

Walk Follow-up 将真实录制全部处理完，但 ON 全为 `UNPROTECTED_BEST_EFFORT`，0次真实恢复，七组 ON/OFF 状态逐字节一致；Walk2 生成437096个动作、selected=0，core P95约1109ms。该结果证明了旧工程路径的局限，**不能代替新滚动系统的真实数据实验**。[R1]

本轮不再以“优化旧工程动作枚举”“更多best-effort提交”“所有表格填完”为主要成功标准。遇到固定锚点、固定dt、来源资格、初始化相关性等障碍，应改造对应核心和模型；不能只加一条 `REAL_SOURCE_UNQUALIFIED` 然后结束整个任务。

### 0.1 正常完成必须出现的五件事

1. 仿真、Walk2/3及ROS回放调用同一个滚动核心与实际增量后端；新模式中旧通用FDE构造／调用次数均为0。
2. 核心支持运行时锚点集合、每批动态有效测距、真实单条时间戳、变长IMU和共享边界来源，而非为Walk复制一个“5-anchor专用版”。
3. 保持原注册仿真数学行为；新非均匀时间／非8锚点仿真有明确的集合包含和风险依据，而非仅观测到PE≤PL。
4. Walk真实输入上出现实际UWB排除、IMU剔除／bridge／rejoin和联合故障恢复，保留全部失败及资格边界；不能只有普通输出。
5. Walk2/3原始及注册注入全录制完成同源ON/OFF、功能、P95与资源评价；物理精度在参考足够时给CAL限定评价，不将缺少硬件安全认证当成所有科研评分永远NA的理由。

### 0.2 明确保留的边界

- 真实硬件概率、应用AL和正式受保护发布，本轮不通过改变标志授予资格。没有外部证据时继续关闭，不阻塞可执行的工程研究。
- 单anchor UWB episode、单轴IMU episode及二者联合／重复的原故障范围继续保留；锚点数量泛化不自动扩大为任意多同时故障。
- 不更换为连续时间估计器，不新增视觉、测高或GT观测，不以Huber替代FDE，不丢弃过去已验证的滚动、prior修复和集合模块。
- 不规定一定胜过SFUISE，不以HPL<2m或VPL<3m验收正确性。不强求每一帧零超时。
- 任意有限锚点接口允许明确的内存／计算准入上限，但不能重新硬编码5、8或固定bitmask容量；几何／冗余不足必须输出具体原因。

## 1. 系统架构：换输入适配器，不换算法

```text
原注册仿真 / 泛化仿真 / CSV-bag回放 / ROS真实消息
                │  各自只负责解码、单位、已验证校准和来源元数据
                ▼
     CanonicalSensorEvent + AnchorRegistry + ModelEvidence
                │ 真实时间、原始ID、有效位、异步range、变长IMU
                ▼
       因果组帧／边界来源构造／时间与资源准入
                ▼
       一个 UnifiedRollingIntegrityCore（复用Sc6Service）
       ├─ 既有增量FGO与当前预测
       ├─ 当前UWB检测 + IMU/temporal检测 + 物理故障池
       ├─ 有依据的功能候选、G2稳健求解、实际隔离/bridge
       ├─ rolling raw联合状态集合、terminal support、HPL/VPL
       ├─ 受污染prior处理、共享来源与历史风险传递
       └─ prepare → 唯一原生commit → 绑定actual tag → 更新边界
                ▼
  不可变估计/动作回执 + 明确分级的界/资格 + 实际sink时效门控
                │
                ▼
  独立评分器：GT、故障注入标签、APE/ARE/RPE、Stanford、耗时
```

**只有一个运行时核心，不是两个数值实现。** `legacy_sim`、`generalized_sim`、`real_empirical`是模型／证据提供者和运行配置，不是三套FDE。旧研究runner可以保留为薄包装；旧通用工程FDE只作显式命名的历史参考，不能被注册为本轮 `Ours-Full`。

### 1.1 既有成果迁移对照表（启动时填入实际符号）

| 资产 | 必须复用／推广的内容 | 禁止替代 |
|---|---|---|
| 离散增量FGO | 预积分、UWB因子、fixed-lag、稳定ID、原生事务 | 重建另一套优化器；每帧整图重启 |
| 当前RAIM/数值候选 | 当前真实测距、功能post-test、稳健多初值、G2镜像 | 标签挑根；固定8条完整测距 |
| 联合物理故障池 | UWB/IMU源、episode相位、历史影响、重复故障 | 只做UWB单帧后称联合系统完成 |
| 实际恢复 | 精确因子支持、prior修复、bridge、健康IMU接回 | 仅打印计划；所有拒绝后nominal冒充成功 |
| 滚动完整性 | p/v/R/bias边界、共享源、跨实际marginal keys传递 | FGO协方差直接充当raw保护证书 |
| SC6/SC7收紧 | 联合raw约束、H/V terminal support、故障并集、数值fallback | 为接真实输入默认关闭所有新模块 |
| 时间一致事件 | source birth身份、预算不重置、旧证书消费 | 每换窗口/掉线/重接重新分配同一风险 |
| SC8评价 | 同源配对、first-online/final-history区分、P95、分母 | 旧图表当新实测；短前缀充整段 |

## 2. 源码落点和已确认的固定假设

下列为固定远端版本的现状。迁移时扫描其全部调用者，不能只删报错条件。

| 已有文件 | 已确认／需核对的耦合 | 改造责任 |
|---|---|---|
| `research/sc6_service.{hpp,cpp}` | `imu.size()==10`、回放`10*replay+1`、epoch/source ID绑定 | 提炼共同核心；数据合法、模型假设、风险资格分层 |
| `research/sc6_provider.{hpp,cpp}` | 8维range数组；truth/draw参与仿真源认证 | 旧仿真适配器保留；禁止成为真实数据消费者的依赖 |
| `research/time_uniform_event.hpp` | birth按10样本分组；每块128源，初始21源；时间固定5/50ms | 保留V1兼容；实现真实source账本及新的可变布局事件证明 |
| `research/rolling_policy.hpp` | `raw_imu_slice.size()==11`、epoch-based边缘化 | 按实际区间验证共享支撑；FGO窗口与完整性窗口仍同一策略 |
| `research/joint_raw.cpp` | 10次积分、dt=1/20、ds=1/200、首样本ID=1/time=0 | 真实dt积分和余项，泛化初始化/局部故障相位 |
| `research/rolling_recovery.cpp` | 9项缓存、18动作、排除后恰好7条；fault→epoch算术 | 动态源子集、跨历史相同anchor身份、区间与ledger定位 |
| `raw_range_contraction.*` / `rolling_boundary.*` / `numerical_candidate.*` | 枚举mask、每range时间、合并/投影与PL绑定 | 动态几何、异步约束、保留全部合法分支与frontier |
| `open_fault_pool.*` / `sc6_temporal_monitor.hpp` | 样本索引、相邻时差、报警支持、duration解释 | 实际时间段与可变噪声/运动差分界 |
| `apps/nominal_dataset_runner.cpp` | `RealtimeIntegrityPipeline`；`raw_pl_constructed=false` | 新模式显式构造共同滚动核心；旧模式不静默兜底 |
| `src/uwb_imu_pl/estimation/*` / ledger / 实际ROS节点 | 因子组、异步range、预积分、ID及slot | 真实输入支持与事务一致，建立统一库调用 |
| `tools/sc8_*` / Walk follow-up工具 | 老路径选择、best-effort、标定/评分 | 路由审计、原始/监测关闭/动作关闭/完整模式分开 |

本轮不把 `raw_source_law.hpp` 中旧finite50检查误说成当前开放式系统只能运行50帧；它属于另一个旧模型。所有硬编码须按**实际调用图和语义**分类，不能全仓库把8/10/50盲替换。

## 3. 资格分层：能研究运行，不伪造保证

### 3.1 三种模型证据状态，共用同一运算核

| 证据状态 | 允许执行 | 允许声明 |
|---|---|---|
| `VERIFIED_FOR_DECLARED_MODEL` | 既有/泛化仿真中实际满足的源与初始law，完整算子 | 对该声明范围的条件位置误差界；非硬件保证 |
| `DECLARED_EMPIRICAL` | CAL支持、已登记但未获极小尾概率资格的初始/噪声/运动模型；允许功能FDE和数值集合 | 工程恢复、经验误差与诊断界；不得称真实raw统计合格 |
| `MISSING_OR_INCONSISTENT` | 原始消息可合法处理时普通估计；可独立成立的监测继续 | 缺少哪些资格明确拒绝；无根据的有限PL不得输出为有效 |

区分 `input_well_formed`、`model_assumptions_present`、`set_numerically_valid`、`bound_valid_under_assumptions`、`statistically_qualified`、`functionally_recovered`、`time_current`、`application_available`、`publication_protected`。可以通过结构化枚举/原因列表实现，不必逐个做布尔字段。

**错误输入与未认证证据不是一回事。** ID冲突、未来输入、NaN、不可解释source gap、过期receipt必须拒绝；仅缺硬件尾概率证据时，可在明确的经验模型下运行同一FDE/集合核心。禁止把旧 `source_valid=false` 改成true；也禁止让 `source_valid=false` 默认调回旧重型FDE。

如果恢复prior依赖集合，经验模式中只能把“在声明但未认证假设下的有限集合”当作数值优化参考并披露；不能将它称为独立健康先验或认证证据。连数值集合／结构一致性都没有时，不能强制恢复。保留有界普通降级，但**其数量不能冲抵实际FDE恢复指标**。

### 3.2 证据生产与真值隔离

旧仿真provider可以使用真实轨迹/draw独立验证生成law；这些特权材料只能在适配器/评分器，不能进入共同核心的候选决策。新泛化仿真也须公开这种资格来源。

真实adapter只读原始sensor及固定CAL配置，不读TEST GT、真实注入标签、未来记录或事后最大残差。相同到达输入下，改变未来数据或评分GT不得改变当前状态/动作/界。允许在独立CAL使用参考设备估计参数；禁止把TEST误差最小的变换选作标定。

## 4. 任意锚点数量：动态数据，不是改常数

### 4.1 三种数量必须分清

`N_registered(t)`是登记的anchor个数；`N_visible(k)`是当前批中不同有效anchor的个数；`M(k)`是当前有效测距条数。一个anchor可有多条异步range，不能把M当独立几何冗余。

- 核心和数值算子使用运行时容器；稳定`AnchorId`不等于数组下标，不要求连续ID、不要求小于256，不按某个anchor排序硬编码故障。
- 当前测距可为0、1、2、…；支持任意数量的**输入处理**，几何不足时允许受控IMU传播或明确功能/PL不可用，不越界、不补虚假anchor、不把无测量当健康。
- 对单anchor故障，排除作用于本候选中该source的所有相关观测，而不是只删数组中的一条；同一source在恢复缓存/历史中的影响按真实episode区间处理。
- 固定位置的anchor可出现/消失/重新出现；再次出现保留原source与过去故障影响，不能自动洗白。新anchor登记可在线进行但坐标/标定须有版本；移动anchor未建模时拒绝对应测量，不混用坐标版本。
- 几何冗余按实际Jacobian、有效prior、故障支持、秩/自由度及全局镜像判断。不能写成“N≥某常数即自动满足FDE/PL”。

### 4.2 动态候选与证明分支

单当前anchor故障的功能候选通常为 KEEP 加每个当前可见source的排除方案；证明还要覆盖carry-in历史、缺失source、IMU活动/结束相位。**功能候选集与保护假设集合不是同一个集合**，不能把从功能候选中筛掉的情况从保护范围删掉。

稀疏source映射、版本化共享子集/因子分解、先验必要条件筛选可以复用既有经验；禁止恢复为每个epoch全历史笛卡尔积。运行时资源预算满时保留未处理分支的保守外包络或明确不可用，不截取前8条测距或前若干故障假设后声称覆盖完整。

本轮程序性测试至少覆盖 `N={1,2,3,4,5,6,8,12,32,65}`，其中1–3等为稀疏/拒绝/受控预测测试，32/65为结构与预算压力，不要求所有大N实时。**这些是测试点，不是把支持范围固定成这个列表。** 正例性能至少覆盖实际5-anchor Walk与5/8/12-anchor泛化仿真。

## 5. 真实时间和变长IMU：推广每一个下游算子

### 5.1 规范输入契约

拟新增/提炼 `SensorEvent`、`ProcessingEpoch`、`SourceLayout`、`AnchorRegistry`、`ModelEvidence`；复用既有类型，不再造第二套ledger。每条输入带稳定sensor/source/measurement ID、原始采样时间、接收时间、单位、frame、valid与校准版本。逻辑epoch只表示第几次处理；不得由 `batch_id` 或 `timestamp/50ms`推断。

对于区间 `[t_{k-1},t_k]`，保留真实新IMU样本及左右边界引用，数量可变。0个新IMU不等于0时长，也不自动允许重复使用上一次样本；只有在预注册hold与误差模型覆盖时才可传播。初始化时间可为任意纳秒时间，不要求ID=1、t=0。

- 以实际`Δt_i`积分、传播RW和运动包络；所有gap门槛预注册，不从TEST最大缺口自动放大。
- 分别处理重复消息、同time不同source、重复source、乱序、迟到和丢样；定义有限reorder/watermark策略，额外等待计入交付。
- 不把82Hz插值为200Hz后当独立ADC；不伪造8条range，不修改时间戳让它们符合旧guard。
- 如果插值/hold用于因子或边界，保留父raw来源、系数、时差、相关性与余项；同一端点只承担一次源噪声。

### 5.2 异步UWB测量必须正确建模

每条range保持自己的采样时刻。不得将跨多个时间的读数统一标成批末时刻后仍声称是精确同期几何。

可以实现以下任一有依据方案，并在W1固定：

A. 使用离散FGO已有时间状态/局部运动映射，在各自采样时刻计算range因子，父状态与IMU依赖完整；不要求改成连续时间后端。

B. 对组帧时刻使用已证明的时间运输误差集合，把时间差、速度/加速度/姿态及杆臂影响进入残差/集合；相应统计相关性与名义近似单列。

若B需速度域，本轮必须显式建立其来源，不能仅用加速度上界推得任意绝对速度界。选择的时间近似影响功能阈值和PL，不能只修GT评分。批内span、实际传输误差、零span退化一致性都要验证。

### 5.3 fault支持、历史与重积分按时间定位

IMU episode记录真实半开时间段、轴、共享幅值与父来源；起止允许落在采样/处理边界之间。由raw样本→插值/hold段→PIM→boundary prior的依赖图确定污染支持，不用旧 `(timestamp−5ms)/50ms` 算术。

一个epoch可包含故障发生前、活动中、结束后三段，不能给整个epoch一律相同故障幅值。区间内故障起止未知时保留允许的支撑分支/外包络；下一次故障结束后共享尾端也不能自动当健康。未直接采样的运动段仍需物理余项。

同一因子使用两个样本平均、线性插值或hold时，污染判定须根据全部父source及其系数/区间，而不只是检查因子末端时间是否落入episode。

恢复窗口按实际retained状态和时间区间保存；`fixed_lag_epochs`、`window_epochs`继续统一解析，默认follow-FGO，显式窗口模式继续可用。参数变化不修改过去source ID或风险；本轮不要求在线热改窗口，启动时不同大小必须正确工作。

## 6. 数学推广：不删guard而丢掉它保护的前提

以下为本轮必须完成的推导和实现义务，不是已经由旧证明覆盖的事实。

### 6.1 初始状态来自真实测量，不再假设独立Gaussian prior

真实因果bootstrap使用启动段UWB/IMU；其状态与随后仍被使用的相同原始样本相关。不能把它的估计协方差当作另一次独立初始观测，再同时消费这些样本。

分离 `NominalInitialization` 与 `InitialSetEvidence`：前者为FGO的数值seed/优化metric，后者为完整性使用的集合与来源依据。推广路径必须实际实现：

- 原独立初始观测law仍可用于旧仿真对照。
- 新仿真建立由原始启动观测＋事先声明物理域构造的初始可行集合，使用同一source事件，允许集合多解或较宽；bootstrap输出只是被保护的估计中心。
- 真实CAL下可给经验初始域/不确定度和敏感性分析，标记`DECLARED_EMPIRICAL`；证据不足则原始统计资格拒绝，但名义及有模型依据的工程FDE继续。
- 第一个样本不因位于“新任务开始”就必然健康。保留旧模拟首故障最早5ms的旧合同；真实数据初始化可存在carry-in故障，其范围必须显式表示，不借用旧sample0健康收缩。

**本轮正常完成需要一个测量bootstrap的新仿真包含正例**，不能所有新证明都继续依赖原来的人工独立15D初始观测。

### 6.2 可变source布局的时间一致事件

现有V1每块128=10×6 ADC+10×6 RW+8 UWB、初始21的布局不能直接用于任意锚点和变长区间。[C2] 保留旧V1作为兼容模型，新增source-layout-aware版本。

建议复用总预算结构：

\[
\delta_0=\varepsilon/2,\qquad
\delta_k=\varepsilon/[2k(k+1)],\qquad
\sum_{k\ge0}\delta_k=\varepsilon.
\]

每个出生块有实际原始随机坐标数 `d_k`；对合格Gaussian坐标或有依据的尾界，选择定向上界使

\[
\sum_{j\in B_k}\Pr(|\xi_j|>q_{k,j})\le\delta_k.
\]

这只是设计形式；需给出具体source分区、数值tail上界与实现证明。没有新源的块不复制旧源、不回收过去已花风险。共享/派生值引用原birth q，同一raw样本跨PIM、初始化、重试、历史重建只归属一次。

源数/身份变化必须在看测量幅值或选择候选前确定，或给出有效的条件尾界／全潜在源构造。不能只对通过valid/残差筛选的样本重新编号后假定分布仍独立Gaussian。丢包、到达延迟与噪声相关性未知时不能臆造条件独立；保留证据不足或使用有依据的保守源宇宙。不同轴/设备相关噪声按有来源的联合模型或边缘尾界处理，union bound本身不要求彼此独立，但具体tail资格必须成立。

时间一致风险与接口数量分离：logical epoch、原始sample序号、纳秒时间、bit表示上限各自校验。不得通过时间乘法溢出/重新起点重置历史风险。新旧事件ID不能相互冒认；同输入同source布局的旧合同应复现旧q，新的layout须独立证明而不是要求所有q不变。

### 6.3 非均匀时间的集合传播与联合收缩

状态仍为 `p,v,R,b_a,b_g`，主保护对象仍是**actual committed tag位置**。真实区间采用 `h=t_k−t_{k-1}`，按同一物理模型使用 `A h`、`A h²/2`、姿态角增量及 `sigma_RW sqrt(Δt_i)`。不能仅将原循环10改为sample_count、仍保留旧总时长。

联合raw测量关系继续为：

\[
f^m=R^T(a-g)+b_a+f_a+n_a,\quad
\omega^m=\omega+b_g+f_g+n_g,\quad
r_j=\|p(t_j)+R(t_j)\ell-a_j\|+f_j+n_j.
\]

对非均匀积分重新推导quadrature／hold／姿态余项；例如有jerk界J的右端点加速度积分误差可用 `J/2 Σ Δt_i²` 的保守速度界，位置加权误差另行推导。RW是sample-grid跳变，不当成可微bias。旧固定`2e-10`等数值界不得在更大dt、量级和输入数量下无证明照搬。

对合法分支h及旧外包络X，收缩需满足 `F_h∩X ⊆ C_h(X) ⊆ X`；全故障输出保留 `⋃_h C_h(X)` 的保守外包络。重复／自适应使用观测需在共同源事件上保真，不把当前测量先生成“独立健康先验”后再次检验自身。

新泛化仿真至少包括：5-anchor、非均匀近82Hz IMU、异步有效/缺失range、非零运动/杆臂、真实初始化；12-anchor、另一采样配置；跨边缘化故障及重接。小型解析／高精度oracle和故障相容反例与真实原生短流均需有证据。

### 6.4 不因本轮泛化丢掉H/V收益

保留SC7联合raw算子、tag关系与terminal support，支持动态source和真实时间。同中心比较旧合法外包络和新约束；HPL/ VPL分别验收。不同故障分支取并集、当前/历史相关性、镜像、未处理frontier和定向舍入不变。

不要求新初始law和新风险布局下的界一定比旧仿真小；不同条件不能作等价“提速/收紧”宣传。不以任意比例缩放PL，不把未合格Gaussian界与raw界取min。

## 7. 功能FDE、数值恢复与资格不能再相互卡死

### 7.1 一个功能通路，不把缺硬件风险资格变成“永不排除”

统一核心应明确分开：`DetectionEvidence → FunctionalCandidate → NativeCommitPlan` 与 `ConditionalBoundEvidence → StatisticalQualification → PublicationGate`。两者共享输入/状态/版本/故障支持，但工程FDE不以“拥有已认证的硬件故障prior”作为永远必要的行动条件。

真实经验模式允许在事先声明且经CAL检查的检测/运动/噪声模型下选择一个数值可执行、支持明确的经验动作，实际改图并评价效果；这不表示该动作是安全认证的唯一真故障解释。最终raw统计资格和完整故障并集仍由独立条件决定，绝不能把功能选中的分支作为真值删掉其他分支。

新的经验FDE策略必须有明确的候选规则、数据依据、数值后检、最大修改范围、拒绝条件和rollback/唯一commit验证。这里rollback只允许撤销尚未提交的proposal；原生commit后发生证明或deadline失败，应保留真实状态变更并降级该输出，不能假装未发生、重做第二次commit或无证明恢复旧状态。不能简单把旧cover/risk检查注释掉，也不能用“研究模式”绕过事务、健康状态机或数值结构检查。

### 7.2 原有创新能力都要在真实路径实际消费

UWB：支持当前故障source、短持续episode、历史污染；动态排除后的因子组要按实际成员构造并保持来源。

IMU：故障支持映射到全部受影响PIM/插值边界；隔离后有合法bridge和bias continuity；受污染marginal prior确实替换；接回IMU使用正确原始样本、bias线性化与共享端点，不只返回KEEP。

历史：从前一边界继承误差/来源/故障状态；证书失效不能自动变健康。超恢复buffer时拒绝恢复或显式受控重建，不能从真值重启、重新领风险。

数值：复用G2稳健求解，按动态几何判断rank；找到局部根不等于找到所有根。功能post-test阈值按实际有效行与自由度处理；若采用新多根选择，不自动授予原Gaussian显著性保证。

### 7.3 报警常驻的排查与修复

在Walk1 CAL及Walk2已登记片段分离：预UWB innovation、更新后残差、各anchor均值/相关、IMU预测/时间误差、启动prior、有效range数量、故障模式可辨性。新算法应回答“仿真和真实差在哪里”，而不是为所有真实输出回落nominal。

允许用CAL有依据地重新估计真实传感器的工程噪声/偏差/时间模型，冻结新模型ID并在Walk3确认；这不是改变旧仿真sigma。禁止用TEST误差或故障标签选择更宽阈值，只为降低报警率。未注入的原始录制也不能自动叫健康真值，评价异常检测须区分自然未知与额外注入。

**迁移完成的关键证据不是ON/OFF完全相同，而是实际UWB、IMU、联合强故障发生时，ON有正确源支持的原生修改、接回及后续状态；无故障时能够继续普通完整处理。**

## 8. 标定和精度：继续推进，不再把所有结果绑在硬件认证上

Walk1作为CAL；Walk2/3保持TEST。已有旋转/时移和rank9加速度关系只属于诊断，不能自动当完整安装标定。[R1] 但**科研经验评价不要求先获得硬件级极小尾概率证明**。

本轮必须进一步执行：核对 `T_WI=T_WT*T_TI`、lever、gravity、时钟、单位和滤波延迟；CAL分段／留出验证、激励与秩分析、合理扰动下参数稳定性；将可辨识的共同外参／时间候选冻结为 `CAL_ONLY_EMPIRICAL_REFERENCE`，充分验证后可用于限定的物理目标误差评价，并附标定敏感性。不能仅因不存在厂家极小误差界就永远不尝试CAL经验参考。

若仍存在不可辨识自由度或安装定义无法确定，继续保留相关APE/ARE NA，给出最小缺失观测；允许对可确认的量分别评分，不伪造完整参考。官方资料／独立测量优先，TEST不参与标定。缺GT外参主要影响评分，不应成为实时sensor FDE架构迁移的整体阻塞。

实际传感器内部坐标、IMU→tag和时间关系影响估计，应与仅影响评分的tracker外参分开。没有true bias的真实数据不计算bias真值误差；仿真继续全15D评分。

## 9. 执行工作包与硬性阶段门

| 工作包 | 实际工作 | 阶段门：完成后继续 |
|---|---|---|
| W0 路由与基线 | 本地HEAD/diff、旧raw路径与旧工程路径对账；SC0–SC8资产对应表；冻结输入/阶段矩阵 | 查明真实调用图，选择统一核心，最小旧仿真基线可复现 |
| W1 共同核心与通用输入 | 动态anchor/IDs、实际时间、变长IMU、异步range、bag/CSV/ROS适配；资格分层；保留旧薄包装 | 旧8-anchor同源语义回归；非8/非固定dt输入能进入同一核心而非旧路径 |
| W2 数学与初始化推广 | source布局事件、数据初始化依赖、变dt包含/余项、故障相位、真实marginal传递；泛化仿真 | 5/12-anchor泛化条件仿真有原生raw有效正例、镜像/弱源负例正确 |
| W3 Walk功能打通 | CAL/实际模型诊断；先UWB再IMU/联合，但包结束前必须都有实际尝试和正例目标；同核ON/OFF | Walk有效注入有实际隔离/bridge/prior/rejoin；不以best-effort替代 |
| W4 ROS与运行成本 | 同核节点、无损规范消息、input parity、队列/交付、热点复用、资源 | 真正1×时间回放、正确未来/缺口/过载、P95与实际行动保持 |
| W5 冻结系统评价 | 原始/注入Walk2/3、必要泛化与旧sim回归、SFUISE公平对照、精度/PL/FDE/P95/资源图 | 按RI01–RI24逐项验收，不转移到下一Goal |

W1/W2需交错处理真实时间对数学的影响；W3标定可并行，但不得发生多人同时修改相同核心文件。每个工作包本地Git保存后直接继续，不等待逐阶段批准。禁止将“证明尚未完成”自动变为“此后所有工作不做”；也不能在证明未闭合时伪装已合格。

## 10. 正式最小验证矩阵（轻量但必须是真系统）

### 10.1 开发与oracle

每项数学/数据结构改动用一个参数化确定性测试族＋1–2个定向原生快照：

- 动态N/ID重排/加入未观测anchor、重复source、有效位、0range、坐标版本；65锚点暴露固定位mask。
- 非均匀dt积分、共享端点、半区间fault、跨窗sample/birth q、不同行为的起始时间、初始样本可能污染。
- source数量／风险变动、缺失／筛选相关性、零新源、重试、不重置预算、overflow与有界资源。
- 同source同center的独立集合oracle、SO(3)/镜像、不可辨识bias-fault对、空集合／数值未决／frontier。
- 真正prior替换、ledger稳定ID、插值父来源、一次commit、迟到后禁止当前保护、错误FDE下仍保留适用证明分支。
- 仿真真值和未来输入不干扰在线核心；同线值ROS/direct状态、动作、source、风险、边界语义比较。

不重复几十次相同小测试凑数量；未改动的原理论复用旧证据与少量防回归。最终核心安全回归一次，之后补丁只补测直接受影响部分。

### 10.2 场景层次

| 组 | 默认范围 | 必须回答的问题 |
|---|---|---|
| B0 旧兼容 | 原8-anchor、200/20Hz：健康、UA、UG、G2及一个跨窗片段 | 重构没有丢掉旧功能/数学；旧law下数值与分支语义可对照 |
| B1 任意数量 | N=4/5/8/12的代表非退化仿真，另1/2/3/32/65小例；相同库与构建 | 输入动态、冗余/拒绝有依据；不是5-anchor特例 |
| B2 Walk-like | 5-anchor、约82Hz不均匀时间、真实bag时间/valid mask作为外生时间表；合成非零噪声与D1运动 | 时间、丢测、来源、bootstrap、联合FDE和条件raw证明能同时成立 |
| B3 通用动态 | 不同采样率，如100/400Hz或可变频；12-anchor；当前可见集合变化 | 避免只为Walk写特殊逻辑；至少一次故障跨源变化/边缘化 |
| R0 实测原始 | Walk2/Walk3各完整 `ROLLING_OFF` / `ROLLING_ON` | 相同实际输入，全部到达材料进入同核，持久在线与拒绝分母 |
| R1 实测故障 | Walk2 U/A/G/J、Walk3 J确认，ON/OFF同源成对；保留原弱/无效注入反例 | 强有效输入真排除/接回，弱或未进入因子的注入不冒充漏检/成功 |
| R2 ROS | Walk2/3实际1×时间，至少原始及一个联合注入；消息→同核 | 与direct相同线值，实际FDE/receipt/时间与来源全链路 |
| L 连续 | 最终默认窗口的一个5000健康＋10000重复故障泛化仿真；Walk使用整段原录制 | 新事件/来源不断续接、较晚故障、资源/队列与P95不因滚动失效 |

不做 N×rate×seed×fault×window 全组合；B1/B2/B3覆盖互补风险。窗口默认30/30，另20/20、40/40、30/10各一个短正例；测试对任意配置的接口行为，不重跑100–1000窗口大规模实验。

### 10.3 实测注入必须先核对实际输入有效性

保留旧20/40s脚本作为历史回归，但上轮弱UWB为0有效样本不能当功能正例。[R1] 在看算法结果前，只基于原始time/valid/source元数据选定新增有效注入manifest，优先真实anchor ID；记录选择规则而不是按GT/检测结果挑片段。

强故障优先沿用 +2.25m UWB、+20m/s² accel、+1rad/s gyro和0.5s IMU；相同幅值早晚一致。若此幅值在合法模型/几何中无法分辨，先给出独立相容性证据，保留原例失败；可以增加事先推导的功效例，但不能把它冒称修好了原例。

Walk2的U/A/G/J以及Walk3的J应各至少出现一次真实正确支持的恢复正例，且恢复后≥4个实际历元继续提交。没有必要移除prior的例子不能冒称测试了prior修复；至少一个延迟发现/跨边缘化定向例子真实替换污染prior，超buffer负例拒绝。

原始背景异常未知，不能仅凭“注入期间报警”计为因果检出。记录base对overlay的新增报警、对应source、实际删除和错误额外排除；注入标签只用于评分。

### 10.4 配对定义与统计

`ROLLING_OFF`关闭动作，不关闭监测和滚动集合；`ROLLING_ON`执行动作，二者同一核心/数据/初始化/校准/风险模型。纯`NOMINAL`只作为定位成本下界，不冒充保留全部监测的OFF。

旧通用FDE最多做一个已登记短片段参考，或直接引用旧报告并说明版本；不再全量跑43万个无效动作以耗尽本轮时间。

三个比较分别命名：重构等价／数学模型推广／功能ON-OFF。新界改变恢复metric而改变轨迹时允许据实评价，不能声称纯性能等价。若外参闭合，报告tag APE、H/V、ARE、0.5/1s RPE、恢复分段；否则只报告可支持的CAL限定量/轨迹差，不伪造米级收益。

SFUISE保留官方初始化与剔除，同原始时间/数值；online/final分开。仅在参考与接口可比时做精度排名，不以“必须胜过SFUISE”作为Goal完成要求。

## 11. 实时、资源与ROS传输

### 11.1 时间与实时门槛

实际输入保持原始采样和节拍：Walk不强迫20Hz；标准仿真保持20Hz UWB基准。以独立producer的绝对时间释放，不能等待消费者处理完成后才发下一帧。构建与性能主运行不得同时抢CPU。

记录raw stamp、release target/actual、arrival、ready/start、alarm、selected、commit、bound、sink及queue。必要source校验、raw计算、FDE、历史更新、清理和输出封装均纳入在线成本；评分绘图在sink后。旧provider特权仿真认证的成本和边界单列，真实源不能借用未计时真值预计算。

本轮默认5/8/12-anchor正例、Walk原始/注入、规定窗口的目标：

- nearest-rank `P95(core)≤40ms`；
- `P95(measurement-to-sink)≤50ms`；
- 真实数据 `timely_committed/valid_planned ≥95%`；
- 合格仿真另验 `timely_spatial_qualified/valid_planned ≥95%`。

P95包括慢帧，不只统计成功/及时子集。原始全部帧、valid、启动前、attempt、commit、refused、timeout、未attempt后缀分别对账。无输出不能填0ms；all-best-effort即使按时也不抵扣FDE行动验收。大N压力例可明确超时/容量拒绝，不能外推无限算力。

### 11.2 有界在线资源

默认不重建全部raw前缀。有限IMU/range回放、boundary capsules和pending事务按实际时间/窗口及资源限制保存；保留source和故障压缩所必需的信息。长期风险索引增长不意味着必须保留所有原始样本；若某项仍O(T)，披露并防止异常退出，不宣称无限期有界内存。

动作生成成本随当前实际source/活动物理模式可解释增长。剔除资格检查前置与分解复用必须保留数学语义；不要先建立数十万不可行动对象再优化其求解。受控容量拒绝和未保护输出均要记录，不能当作系统实时成功。

### 11.3 ROS消息不能截断来源

Follow-up已发现Linktrack `uint8 id` 与 `float32 range` 不能无损承载原始大ID/float64。[R1] 本轮要提供规范化64位ID、float64距离、每条独立timestamp/valid和校准版本的ROS消息或等价lossless适配。

旧Linktrack消息兼容只通过显式双射/量化metadata，不静默截断ID；源证书始终引用原始身份，不能用重新映射的小ID重新分配故障历史。超过256 anchor时不通过uint8偷渡。相同线值direct/ROS比较FDE行动、ledger、来源及边界，不只比较OFF位置。声明协议量化误差，不放宽容限掩盖。

节点启动选项显式 `integrity_backend=unified_rolling`；注册测试要求旧 `RealtimeIntegrityPipeline` 构造次数为0、唯一增量后端实例。保留旧ROS默认以免破坏用户任务，本轮交付提供新的正式可运行入口；硬件保护开关仍关闭。

## 12. RI01–RI24 验收合同

| ID | 通过要求 |
|---|---|
| RI01 | 真实本地HEAD/构建/DSO与SC0–SC8资产映射、旧结果/用户文件保留；源/方法/模型身份分明 |
| RI02 | 三种数据入口调用同一滚动核心和唯一FGO；新模式旧工程FDE构造/调用均0，运行时证据可核验 |
| RI03 | 动态N、稀疏64位ID、N_registered/N_visible/M、有效位、重复source、N>8与N>64测试通过 |
| RI04 | anchor增减/失联返回/版本身份保留，几何不足不崩溃、不伪造独立约束、不自动洗白 |
| RI05 | 可变sample数与实际dt进入积分/RW/temporal/raw/恢复/sink；非零起始时间、共享端点和gap都有对照 |
| RI06 | 单条异步range时间模型有依据并真正进入因子、FD和PL；batching误差及父source不丢失 |
| RI07 | fault中途起止、未知支撑、共享sample、PIM和历史prior的对应正确；停止故障不等于污染消失 |
| RI08 | 旧注册law下重构语义回归，泛化仿真与真实读取相同数值时调用/动作一致；无新仿真专用主算法 |
| RI09 | 可变source事件分区/tail/计数有证明和独立测试，初始化复用/valid筛选/丢包相关性无隐含独立假设 |
| RI10 | 非独立测量bootstrap的初始集合及首样本故障范围有可运行证明正例，不借旧sample0健康资格 |
| RI11 | 非均匀raw传播/收缩、余项、镜像、全故障并集和frontier具有包含依据，至少5/12-anchor实际raw正例 |
| RI12 | 真实声明经验模式与统计资格实际分开；缺硬件prior不调回旧FDE，也不伪造source_valid/硬件保护 |
| RI13 | Walk1 CAL继续建立可检验的模型／参考，条件经验评价或具体不可辨识限制透明；TEST GT不进入优化/调参 |
| RI14 | Walk2/3完整ROLLING_ON/OFF，同源且OFF保留监测；≥95%有效计划提交，无隐瞒后缀 |
| RI15 | Walk2强UWB、accel、gyro、联合及Walk3联合有真实FDE正例，准确支持和≥4个后续历元；不能以纯nominal抵扣 |
| RI16 | 至少一个跨marginal污染prior真实修复、bridge和rejoin；buffer外正确拒绝，single commit及异常状态不吞掉 |
| RI17 | source/GT/future非干扰、wrong FDE、秩/镜像、未知模型、过期／篡改／容量等负例及相关安全回归 |
| RI18 | ROS正常和联合故障same-wire-value对应direct，全源身份/行动/receipt/边界等价，不仅OFF轨迹接近 |
| RI19 | 注册正例P95与及时提交比达标；所有miss/拒绝/队列/P99/max/处理频率披露，非零超时可接受 |
| RI20 | 5000/10000泛化流在冻结版本持续运行，较晚故障、risk/证书、资源无重置；不称无限内存/精度保证 |
| RI21 | FDE配对误差/副作用、实际动作/恢复、运行成本和source资格分层完成；无真值的bias和开放参考不造指标 |
| RI22 | 保留SC7 H/V与联合raw能力；合格仿真Stanford、真实诊断界/可用性分别展示，不靠图中零超界证明10⁻⁶ |
| RI23 | 原始真实数据、注入真实数据、泛化仿真三通道结论独立；新模型假设/未覆盖故障/外部证据清单完整 |
| RI24 | 唯一报告、最少配置/脚本、实际复现入口和阶段Git提交、清理完成；本轮未完成能力不标PASS |

### 12.1 结束状态不再混淆

`UNIFIED_CORE_INTEGRATED`、`VARIABLE_TOPOLOGY_TIME_VERIFIED`、`GENERALIZED_SIM_RAW_VERIFIED_FOR_SCOPE`、`REAL_ROLLING_FDE_VALIDATED_FOR_SCOPE`、`REALTIME_P95_MET_FOR_TESTED_PROFILES`分别签发。**主Goal正常完成需要RI01–RI24对应必需能力成立，特别是RI15，不能以“运行完矩阵”代替。**

若软件目标通过但缺真实硬件概率依据，最终可为 `SC9_UNIFIED_REALDATA_SYSTEM_VALIDATED_FOR_SCOPE`，另标 `HARDWARE_RAW_RISK_UNQUALIFIED`；这不是硬件证据不足导致所有软件任务OPEN。

若没有真实排除/bridge/接回，只能是 `SC9_PARTIAL_INTEGRATION`，即使97%输出、耗时变小也不能结为“真实FDE已完成”。硬件缺失可以给出后续采集清单；数值/模型/软件困难应继续实际改进，不作为首次停止理由。

## 13. 持续研发、测试预算和交付

### 13.1 有明确授权，不反复请示

已授权在research/显式opt-in路径完成：共同核心重构、运行时容器、真实时间和source事件的版本化推广、经验模型/CAL、功能策略分层、恢复与ROS接口、必要算法修改及验证。发现旧假设不成立必须修复或推出合法替代并验证，不能只加状态字段和文档。

不授权：打开硬件保护发布、放宽既有风险/AL来制造通过、删除原故障类别、修改TEST GT、毁坏旧证据或用户工作。新增物理假设可用于独立命名的工程实验，但不能暗中替换原任务；单UWB成功是阶段门，不是取消IMU/联合目标的授权。

至少实际尝试两类合理的解决方向后，才将某个数学方向写成明确阻塞；局部问题不阻止其他可安全继续的工作。遇到预算/工具中断，在同一报告记录SHA、最后通过项和下一条命令，恢复同一Goal。不得绕过预算、承诺无限工作或用重复实验填时长。

### 13.2 轻量运行规则

先最小局部正例，再全流。W0不重跑SC8 143个运行；W3没有任何有效动作时，不连续再跑十几条一秒/帧的失败长流。先定位首个可行动问题并修复，再启动冻结矩阵。性能先计数/短流，必要时一次35历元成对样本确认；最终真实paced长流是主证据，不额外重复140次。

W5冻结前检查真实ELF/DSO加载，不并行编译和基准测试。旧失败计数保留；修复只重跑受影响测试，不为了图漂亮反复挑最好一次。

### 13.3 文件与接口

输入需求放 `docs/requirements/sc9_realworld/`。建议只新增一个薄实验入口，如 `tools/sc9_realworld.py`，复用原读取/标定/评分器。以下是待实现目标而非现有命令：

```text
check --build       # 本轮数学/路由/输入/安全检查
legacy-parity       # 旧注册scope同源兼容
generalized         # 动态anchor与时间的新仿真
walk --mode on|off  # 同核心真实录制
ros-parity          # 实际source与行动/边界等价
validate            # 冻结后必需矩阵，支持按缺失项续跑
report              # 只读取实际结果，不能制造指标
```

最终保留必要源码/测试/配置、`docs/sc9/SC9_RESULTS.md`，以及最多4张结论图：

1. 统一路径与源布局/窗口传递的实际架构；
2. Walk原始/注入ON/OFF位置及全状态评价，未合格参考清楚分层；
3. 故障→检测→隔离→prior修复→rejoin时间轴；
4. H/V诊断、P95/queue和关键资源的精简展示，按最有信息的内容选择。

过程CSV、矩阵、日志、候选备份放本轮独占 `.cache/sc9/`，验证/配对未完成时不要删除唯一原始trace导致重跑；完成摘要与失败分母后清理。保留足以重画结论的小型汇总和生成配置，不保存大量二进制/仓库备份。显式git add，每包本地commit后继续，不自动push，不为干净工作树执行破坏性clean。

### 13.4 结果报告必须回答

我们是否真的在同一套算法上测试仿真和真实输入？多少anchor、什么sample时间和初始化已验证？真实UWB/IMU/联合FDE分别成功几次？哪些界合格、哪些只是经验假设？旧仿真保证是否保留？实际精度是否改善、姿态是否退化？P95和持续运行是否达标？仍缺什么外部数据，具体阻塞哪一项能力？

## 14. 来源与设计责任

**[R1] 用户上传《SC8 Walk Real-Data Follow-up》**：对应`docs/sc8_followup/SC8_WALK_RESULTS.md`，W0标定、W2动作/性能、W3注入、WF01–WF18及source冻结摘要。报告所述新本地代码尚不在本次远端HEAD中；执行时必须读取本地实现，不以远端覆盖。

**[R2] 前序结果**：`docs/fde_next/SC7_RESULTS.md`、`docs/sc8/SC8_RESULTS.md`及用户历史讨论。旧相同scope结论保留，新真实数据资格不得从这些报告自动继承。

**固定远端源码链接**：

- [C1 Sc6Service](https://github.com/Printeger/uwb-imu-fusion/blob/979242d22d4d5fb84bb883976cc1c63d5860e2b0/research/sc6_service.cpp)
- [C2 TimeUniformEvent](https://github.com/Printeger/uwb-imu-fusion/blob/979242d22d4d5fb84bb883976cc1c63d5860e2b0/research/time_uniform_event.hpp)
- [C3 Joint raw](https://github.com/Printeger/uwb-imu-fusion/blob/979242d22d4d5fb84bb883976cc1c63d5860e2b0/research/joint_raw.cpp)
- [C4 Rolling recovery](https://github.com/Printeger/uwb-imu-fusion/blob/979242d22d4d5fb84bb883976cc1c63d5860e2b0/research/rolling_recovery.cpp)
- [C5 Provider接口](https://github.com/Printeger/uwb-imu-fusion/blob/979242d22d4d5fb84bb883976cc1c63d5860e2b0/research/sc6_provider.hpp)

以上支持“旧实现做了什么及受何限制”，不自动证明本轮设计成立。变长source事件、数据初始化、异步range及恢复推广的正确性，由本轮推导、独立oracle和原生实验共同承担；经验CAL不冒充硬件安全认证。
