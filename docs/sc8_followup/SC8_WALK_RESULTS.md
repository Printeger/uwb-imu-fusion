# SC8 Walk Real-Data Follow-up

状态：**IN_PROGRESS，W0/W1 已执行，W2–W4 继续同一 Goal。** 本文件为本轮唯一结果报告，不改写旧 SC8。

## W0：输入、版本和参考资格

入口 HEAD `00b8486c4f45024daa046b2f4ec20a0eec98afb3`，较设计核对版本多一个演示文稿提交。原有未跟踪 `docs/sc8/SC8_RESULTS.html/.pdf` 保留。仓库及上级路径未发现 AGENTS.md。ROS Noetic、GTSAM 4、原生 runner/DSO 实际存在；入口 ELF/DSO SHA256、SFUISE pin、每个源 bag 和 CSV 哈希见 [冻结合同](../../config/sc8_walk_followup.yaml)。SFUISE 固定 `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`。生产配置、AL、risk、故障合同未改。

三条 bag 均来自 `/home/mint/ws_fusion_uwb/data/SFUISE/`，重新读取构建本轮 cache。IMU 原始 m/s²、rad/s；range 按官方 anchor ID 加一次固定 offset，`ra=0` 保留并标无效；5 anchor，tag 27956。官方 IMU→tag 偏移 `[0.1,-0.025,0] m` 保持为估计器模型参数，其独立实测误差界未知。

评分 GT 改为 bag 实际 tracker 位姿，明确 **REFERENCE_OPEN**，不沿用旧适配器将 tracker 当 IMU 加杆臂后得到的物理 tag 声称。输入变换没有使用 TEST GT。`T_AB` 将 B 坐标映到 A，`T_WI=T_WT*T_TI`，`p_Wtag=p_WI+R_WI*l_Itag`。世界变换拟合不能替代 `T_TI`。

官方 [SFUISE 论文](https://isas.iar.kit.edu/pdf/RAL23_Li.pdf)、本地固定版本 README/配置/接口均已查阅。发布配置给出 IMU-tag offset，没有给出 tracker→IMU 完整安装外参及不确定度；bag topic 清单见合同，无静态外参闭合证据。因此物理 tag APE、body ARE、0.5/1s RPE 均 **NA**，真实 raw PL/HMI 概率资格 **UNQUALIFIED**。不据此中断 FDE 工程研究。

Walk1 CAL 预先以时间前60%拟合、后40%留出。有限 `[-0.1,0.1] s` 时间搜索得到 gyro→tracker 诊断时移 -0.0383347 s；训练旋转残差 RMSE 0.0602826 rad/s，留出 0.0487206 rad/s，identity 0.485762 rad/s。旋转 det=1，正交误差 1.15e-15。三方向激励奇异值 6.768/3.501/3.327；这只支持旋转相关诊断，gyro 对平移的 Jacobian 为零，不能据此声称完整6D外参已辨识。诊断矩阵不应用到输入或 TEST GT，不提供伪精度排名。

需要外部补充：tracker→IMU 安装旋转/平移、tracker→tag 测点定义、时钟偏移/漂移与误差界、世界重力方向验证。可采三轴转动与不同姿态平移的独立 CAL，包含静止段和同步事件，测量上述杆臂并报告尺寸误差；另留独立 TEST。当前数据不能提供可认证的有限完整外参误差界。

确定性测试：左右乘/逆变换/tag 杆臂、重力与角单位、已知旋转和37 ms时移共3测试 PASS。真实 Walk1 前缀22个原生状态的未来 GT 篡改测试 PASS：两次初始化、完整 trajectory、提交/动作/源截止逐值相同。生成脚本已预登记 Walk2 定向源批次、Walk2四类及 Walk3联合确认的弱/强额外故障，时刻/幅值在 FDE 结果前冻结。

## 后续执行与验收状态

W0/W1 与 W2 局部验证已完成，Walk2 工程研究全流已运行；Walk3、注入矩阵与最终回归继续。不授予结项状态；物理外参资格保持 OPEN。

```bash
python3 tools/sc8_walk_followup.py prepare
python3 tools/test_sc8_walk_followup.py
python3 tools/sc8_walk_followup.py noninterference
python3 tools/sc8_walk_followup.py baseline
```

本轮 cache 尚需用于后续执行，未清理。禁止自动 push。资源中断时依据本轮 cache 中 execution.json 续跑 continuous/paired；脚本跳过已有完整执行记录，不重建另一个 Goal。

## CAL 加速度/杆臂补充诊断

仅 Walk1 的 tracker 平移二阶导数与角速度构造 `R_TI f_I = R_WTᵀ(a_WT-g_W) + (α×+ω×ω×)r_TI+b_T`，前60%拟合、后40%留出，固定0.62s Savitzky–Golay支撑。9参数线性设计矩阵 rank=9，条件数9.23；拟合 r_TI≈[-0.0357,0.00004,0.0380]m，训练/留出逐分量残差0.181/0.154m/s²。这是可辨识关系的诊断，不再把“gyro无法辨识平移”泛化为完整数据必然秩亏。tracker求导误差相关性、重力/偏置混淆、时钟与真实安装定义尚未独立验证，条件OLS标准误差不是硬件外参界，参考仍OPEN；未应用此拟合。复现：`python3 tools/sc8_walk_followup.py cal-accel`。

非干扰小测试v1曾误向GT增加未使用x列；已纠正为实际px字段，v2两次原生运行重新通过22状态逐值比较，以v2为验收依据。

## W1：本轮名义全录制

四个进程均完成完整录制；NOM/NOFDE 的15D状态逐值相同，源CSV与旧SC8冻结哈希一致。没有使用GT初始化。NOFDE实际代码不运行检测器，此处是off路径开销对照，不宣称保留全部监测。

|流/方法|原始批次/有效计划/可用计划|原生提交|全部有效计划覆盖|core P95/max ms|sink P95/max ms|CPU s/RSS KiB|
|---|---|---|---|---|---|---|
|ISAS-Walk2-NOM|1241/1172/1140|1140|97.270%|2.440/4.514|2.646/4.764|5.981/93768|
|ISAS-Walk2-NOFDE|1241/1172/1140|1140|97.270%|2.458/4.288|2.672/4.589|5.991/94312|
|ISAS-Walk3-NOM|1339/1253/1222|1222|97.526%|2.335/4.193|2.526/17.472|6.268/96192|
|ISAS-Walk3-NOFDE|1339/1253/1222|1222|97.526%|2.520/22.190|2.664/22.465|6.439/96768|

Walk2全无效69批、有效计划启动前32批；Walk3全无效86批、启动前31批。原始全部分母1241/1339也保留，不能将其视为1172/1253。首发/末端分别为Walk2 4.471092224/78.456339598s，Walk3 4.524949789/84.114124775s。所有可用计划均已提交，无失败后缀；95%目标仅针对有效计划分母成立。物理精度NA。原生单次backend更新及时间/事务唯一性断言通过。

SFUISE两条完整进程SUCCESS，官方剔除开启，GT topic=0；生成ROS bag逐ADC、range、ID、valid和纳秒时间与本轮原生输入全部exact相等，runtime offset=0，源偏移仅加一次。

|序列|S-ONLINE N/首末秒|S-FINAL N/支撑无效点|最后数值输入秒|最终callback上界秒|
|---|---|---|---|---|
|ISAS-Walk2|760/2.527455901–78.427455901|758/2|78.423522472|78.521199942|
|ISAS-Walk3|816/2.583362912–84.083362912|814/2|84.081641436|84.142725229|

S-FINAL按原样条支撑拒绝起始2点，未按误差筛选；历史依赖截止为整个最终导出的上界，不能解释为每点精确截止，也不参加在线延迟排名。SFUISE采用官方传感器初始化和在线重力/世界标定；ours采用相同原始观测的2秒因果初始化，两者初始化信息/时刻不同。所有物理APE/ARE/RPE仍NA。首条SF运行启动时audit-only runner编译尾部短暂重叠，运行性能需披露此环境扰动；不隐去慢值。

W1源码/输入与名义结果已保存，继续W2。

## W2：局部工程证据与实现（全录制验收进行中）

原始 Walk2 源 batch 1–120（包含启动段）实际84次尝试，提交83：57次NO_VALID_CANDIDATE、26次RISK_BUDGET_INVALID、1次AMBIGUOUS_UNAVAILABLE。前83次本来就是KEEP_ALL_BEST_EFFORT，不是真正成功隔离。首次报警source batch120、t=9.853092670s，411个假设全部保留为plausible，生成412动作；原生coverage为210个COVERAGE_GAP、202个UNRECOVERABLE。不是“5 anchor必然无秩”或“所有拒绝都是测量故障”：原生联合窗口秩/自由度、旋转步长门限、覆盖和risk资格需分别看。

该报警批原始core约12620.85ms。前缀candidate kernel总计16245ms、完整candidate阶段17383ms（后者包含前者，不能相加），core总计27599ms；原窗口构造也有实际成本。候选需要保持完整可疑集合覆盖，而当前集合不允许一个有限动作合法排除所有仍可能的故障；没有因此缩减故障集合或改概率。

实现两项独立、默认关闭的研究开关，生产ROS默认路径不变：

- `--research-eligibility-pruning`：完整生成动作ID、物理源和census。仅对冻结集合覆盖失败、mandatory health覆盖失败、history recoverability失败的动作跳过数值核与post/PL，记`SKIPPED_INELIGIBLE`，不伪记已求解。FdeManager的eligible集合要求这三项均成立，因此无论这些动作的数值、排名、tie或PL是多少，都不能进入winner排序或风险union；保留全部可能eligible动作的原始计算与严格fallback。不实现排序剪枝，不改变rank/tie阈值。研究开关强制关闭保护资格。外置object sidecar保持原pipeline对象ABI。
- `--research-best-effort`：仅在FDE原事务已安全discard、backend更新为0、无pending/poison/reinitialization、无活动bridge、无quarantined/failed/recovery-test源，且拒绝为三种允许状态时，重新prepare同一原始batch并通过普通nominalPlan提交一次。检查backend总增量=1、源时间等于该batch。discard保留ADC队列，因此不补旧状态或改时间戳。原FDE事务和候选审计单独保留，新的普通receipt标`UNPROTECTED_BEST_EFFORT`，不称prior已修复或FDE已成功；异常、源缺口和必须隔离条件不被吞掉。

9个确定性组合（不覆盖/mandatory/recoverability × favourable/tied/hostile数值）通过FdeManager对照小oracle。真实前缀剪枝与原始版：84次attempt、83次commit、495个动作身份及15D状态、保留候选数值、实际行动/拒绝、prior/bridge/ledger/health/事务逐值一致；412个动作明确未求解。core总计27.599→16.253s。这是局部调度结果，不推广成全流实时性。

加best-effort的同一片段84/84提交，**84个全部未保护，实际FDE恢复0**。普通连续输出的验证不能抵扣WF12强UWB正确排除与IMU bridge/rejoin正例。完整Walk2/3及冻结额外故障继续执行；当前尚未授予W2完成等级。

Walk2 完整研究运行：1140/1172 有效计划提交（97.270%），全1140个为未保护best-effort，0实际恢复；AMBIGUOUS_UNAVAILABLE=1057、NO_VALID_CANDIDATE=57、RISK_BUDGET_INVALID=26。core P50/P95/P99/max=988.464/1109.319/1173.916/1289.055ms，sink最大985639.262ms，CPU1058.797s、RSS1023604KiB、服务RTF13.906。完整437096动作记录中仅83个实际数值核求解，selected=0；动作集合构造平均620.749ms/attempt是主要剩余瓶颈。Walk3正在相同冻结ELF/DSO下确认。

注入源预审：Walk2弱UWB的1条源测距原本invalid，strong-U的3条中2条valid，strong-J的3条中1条valid；Walk3 strong-J的3条全部valid。保留原始有效位，不后验更换源ID；无有效注入样本不能计入可检测正例或算漏检。所有IMU episode为41个真实ADC样本，具体PIM支撑由原生ledger另行核对。

## 相关核心回归与负例（冻结算法）

测试目标重新构建后，`test_integrity_v2`选定29项、`test_realtime_incremental`的gap/未来IMU非干扰2项、`test_fde_post_selection`4项（包含9组合的研究剪枝oracle）全部通过；另补齐容量fallback、假设维数超限拒绝、deadline后真实commit降级3项，共38项通过。之前旧test_integrity_v2 ELF存在未解析旧proof符号，属于陈旧构建，已重建消除；不隐去首次预检失败。小oracle核对的是合法集合与安全拒绝不变，不是实测FDE恢复正例。

实际runner缺口负例：原输入截至6秒，删除5.2–5.3秒ADC。首次fixture沿用旧dataset_config，门限随最大输入gap被自动放大为0.108979秒，进程24次全部提交，故“应该拒绝”的测试断言失败。修正fixture为冻结原始20ms门限，未改算法；第二次进程按预期exit=1，原因`causal IMU hold exceeds imu.max_gap_s`，56原始/有效批次、32启动前、12提交、12未处理后缀，最后收据5.183996916秒。无保护输出，不将失败后缀静默补齐。普通原录制及幅值注入时间戳未变，门限均未因注入增大。

测试构建与短回归同Walk3原录制中段重叠，影响共享主机负载；独立记录开始/结束时间，核验estimator ELF/DSO前后完全相同。Walk3耗时保留实测，不作为严格独占CPU速度对照。

Walk2全流历史摘要1140次均present/valid/capacity_ok，但完整故障历史覆盖声明为0/1140，最大省略历史epoch=1120、fault_columns=160、emitted_rows=66；这不是“无数值历史摘要”，而是不能把有限窗口数值prior当完整故障保护历史。风险ledger中的bridge/history/model仍为UNKNOWN/ASSUMED_UNVALIDATED，omitted为UNKNOWN/NOT_IMPLEMENTED，不通过增大AL或置零概率掩盖。候选跳过记录的rank/dof=0表示未计算，不代表实测几何rank=0。

## W2 全录制执行完成

两条原始录制均已按冻结限制执行；下表保留实际结束状态，不把普通提交计入FDE恢复。

|序列|结束/完整源|commit/有效计划|best-effort/恢复|core P95/max ms|sink P95/max ms|CPU s/RSS KiB|
|---|---|---|---|---|---|---|
|ISAS-Walk2|FINISHED/True|1140/1172|1140/0|1109.319/1289.055|931970.742/985639.262|1058.797/1023604|
|ISAS-Walk3|FINISHED/True|1222/1253|1222/0|1137.777/1285.377|993735.706/1053675.457|1132.949/1058148|

W2全流证据已保存，继续W3；不授予真正FDE恢复、物理GT或实时性通过。
