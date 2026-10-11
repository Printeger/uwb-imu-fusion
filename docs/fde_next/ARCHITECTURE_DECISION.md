# UWB 单历元 RAIM 与融合条件 PL：架构决策

日期：2026-10-11。决策：**HYBRID**。A/B 已实际实现、运行并通过数值与
原生事务验证。采用 A 当前测距筛查、B 当前融合 tag 条件 PL、独立历史有效性
证据/风险摘要的分层接口；C 保留为历史风险研究与离线参考。

这不是生产替换批准。生产概率合同、AL、风险预算和 protected publication
逻辑均未修改。所有流 `formal_eligible=false`、`publication_protected=false`、
`raw_total_risk=UNKNOWN`。不得把 A 的 PL 贴到融合输出，也不得把 B 的历史
有效性条件解释为硬件事实。

## 1. 实现及来源审查

起点是干净的 `feature/realtime-uwb-imu-pl`，HEAD
`297d05799686fcc7d8134ca1c82656fa6e621345`，没有回退远端或改写历史。

已检查 RESULTS 最新全历史性能结论、run_fde_next.py、prepare/freeze/commit、
HypothesisGenerator/HypothesisEvidenceEvaluator、JointWindowDetector、
RankUpdateEvaluator、ProtectionLevelV2/FdeManager、risk ledger、增量统计摘要/
raw prefix oracle、U0–U3/I2 生成器及实际 tag 映射。C 的全候选行动、
独立故障身份/风险收费、原生事务和历史模型保持原样。新路径没有复制估计器。

- `research/epoch_raim.hpp`：A，纯 UWB 非线性 WLS + 冻结 slope RAIM/FDE。
- `research/fused_epoch_pl.hpp`：B，15 维预测 + 当前 range 的联合 Gaussian LS。
- `researchPreUwbPredictionV1(tx)`：已有 estimator 的只读研究接口；从 iSAM2
  已有 Bayes tree 当前联合 marginal 和 pending Combined IMU 传播预测；不做
  backend update，不读取当前 range 值或当前 UWB 因子。
- `research/epoch_raim_runner.cpp`：真实 estimator/原生 pending group commit。
- 两个 checks 可执行程序与 `tools/run_epoch_architecture.py`：参数化确定性
  校验及固定输入对照；原始 CSV/日志只存在自动清理的临时目录。

已有 nominal_predicted_state 是 IMU-only；CV frozen_values 是另一参考。
旧 pendingCurrentPrior 在 frozen CV 点传播协方差、却返回 IMU mean，故没有
直接复用。新接口清空局部 frozen_values，让 transition Jacobian 与 mean 在
同一 IMU 预测点，传播 Pose/Velocity/Bias 完整交叉项。事务 ID、base version、
批次 ID、timestamp 和 backend update count 被绑定；旧已提交批次 ID 重用拒绝。
测试将 pending range 全部增加 100 m，预测 mean/P 完全不变。

**预测模型边界：**P 是已有 backend Gaussian 权重下的条件预测协方差，
不是 raw ADC 概率证书。原始 IMU 共享端点相关性、backend 切点/重线性化的
协方差解释、初始观测 law、随机参考点及非线性余项仍未认证。这里定义一个
独立版本的冻结模型，指定这个 P 为先验 covariance；没有宣称 GTSAM 的名义
P 等于当前仿真原始误差的真实统计分布。B 的 `PredictionConditionsV1`
必须明确提供 covariance、当前噪声独立性、历史及 IMU 有效性、无当前信息
重复使用五个前提；缺一个即拒绝。runner 的正例是 **FIXED_MODEL_ASSUMED /
CONDITION_ASSUMED**，不是从 NIS 反推的证书。硬件调用者不能继承这些假设。

## 2. A：纯测距模型与 PL

版本 `epoch-uwb-slope-v1`，标签 `UWB_EPOCH_RAIM_CONDITIONAL`。
受保护量仅为同一 epoch 的世界坐标 **纯 UWB tag 位置**。
状态是 3D 位置，没有 IMU、速度、姿态、高度或钟差先验。

模型 `z_i=||p-a_i||+b_i+n_i`，名义 covariance R 必须 SPD；允许非对角项。
由 squared-range differences 构造 UWB-only 初值，再解 correlated WLS。
采用单调线搜索及收敛检查，不使用 GT 或 IMU 选择分支。
候选删除先取原始测距/协方差 principal submatrix，再白化。

在固定 p0 上，`y=H δp+B_h f+n`，白化后 `A=W H`，`W R Wᵀ=I`。
SVD 给出 gain `L=A†`、parity `Q₂ᵀ`，`ν=n_range-3`。
`T=||Q₂ᵀ W y||²`，名义为 χ²_ν；阈值 `τ=χ²_ν(1-α)`。
单 range i 的响应是 `d_i=Q₂ᵀ W e_i`，`g_i=L W e_i`，
每轴 slope `s_ji=|g_ji|/||d_i||`；漏检概率为
`F_ncx2(τ;ν, λ_i)`，`λ_i=f_i²||d_i||²`。

数值资格检查 full state rank/conditioning、残差 dof、anchor affine rank、
每候选保留几何及收敛。策略至少保留 5 个 range，故排除后仍有冗余；这是
保守的研究可用性限制，不是声称 4 条 range 无法定位。共面与近共面拒绝，
不借助 GT 解决镜像；收敛本身不证明任意噪声/大故障下全局唯一分支。
固定模型 PL 不包含全局 nonlinear branch/remainder 证明。

对任意被接受的固定候选，在 parity-noise 事件
`||η_parity|| ≤ sqrt(χ²_ν(1-tail))` 上：

`||d_i f_i|| ≤ ρ = sqrt(τ)+sqrt(χ²_ν(1-tail))`。

同时用三坐标 nominal error 的联合 tail 事件，
`k=Φ⁻¹(1-tail/6)`，得到

`PL_j = k sqrt(P_jj) + ρ max_i s_ji`，
`HPL=sqrt(PL_x²+PL_y²)`，`VPL=PL_z`。

这包含名义噪声和未检测故障；不是 RMS 或经验 sigma 倍数。
故障幅值无界。零 residual 响应不能除以 epsilon 输出有限 PL：exact state
alias 且保护非零为 DANGEROUS_NULLSPACE；只有 literal zero fault 或 exact
alias 到保护为零状态才接受 harmlessness。未解析数值秩拒绝，不以浮点接近
零证明无害。复用已有 Gaussian reference 的结构判据而不改变 C 算法。

报警后完整枚举每个 raw range 的删除候选。每个重新非线性求解、计算 post
statistic、rank 和 PL；只从合格候选中选择最小 post statistic，固定 index
打破相同 score 的顺序。选择不用注入标签或 GT；每个候选 PL 仍覆盖**所有
保留单 range 故障**，所以错误删除健康 range 不会从本候选故障集合中删去
真实故障。多候选选择的风险由下述全候选 union 支付，不假设事件独立。

A 的排除结果可以选择进入图的测距，但其 PL 仍只属于纯 UWB 输出。
A 流中的 SE3 RPE 是 A-gated backend 的功能指标，不是纯 UWB 的姿态精度。

## 3. B：当前融合输出的故障响应

版本 `epoch-fused-gaussian-v1`，标签 `FUSED_EPOCH_FROZEN_GAUSSIAN`。
受保护量是当前 committed 世界坐标 UWB tag，含非零 lever。
条件为：历史预测有效、IMU 有效，当前最多一个 UWB range 故障，固定
Gaussian 预测与本批测距噪声独立。如果真实交叉项未知，不授予独立 raw
Gaussian 资格；本轮冻结模型显式采用 block diagonal law。

切点在当前 UWB 入图前。已有 previous 15D marginal `P_prev` 和当前
Combined IMU 白化 transition `[J_prev,J_current]` 构成

`I30 = JᵀJ + diag(P_prev⁻¹,0)`，
`P⁻ = (I30⁻¹)_current,current`。

不计算全历史 fault maps。保留全 Pose/Velocity/Bias covariance，不只取
三个位置标准差。mean 是 actual previous committed state 的 IMU predict。

Pose tangent 顺序为 rotation/local translation，tag Jacobian
`C=[-R[lever]×, R, 0_velocity, 0_bias]`，range Jacobian来自真实
UwbPoseBatchFactor，在相同预测 Pose 点计算。因此包括杆臂/旋转，而不是
将 A slope 乘经验融合增益。

对固定模型 `e⁻ ~ N(0,P⁻)`、`v ~ N(0,R)`、独立：

`S=H P⁻ Hᵀ+R`，`K=P⁻ Hᵀ S⁻¹`，
`G_i=C K e_i`，`P⁺=P⁻-K H P⁻`。

B 主检测为 innovation/NIS：`T=yᵀ S⁻¹ y`，`ν=n_range`，
`τ=χ²_n(1-α)`，`λ_i=f_i² e_iᵀS⁻¹e_i`。
采用联合 LS `[I;H]`、joint covariance `diag(P⁻,R)` 计算，
parity dof 正好是 n；稠密 SVD 的 parity norm 与 NIS 相等。
单故障 slope `|G_ji|/sqrt(e_iᵀ S⁻¹ e_i)`，名义 sigma 从
`diag(C P⁺ Cᵀ)` 得到；PL 与 A 同一 noise-event 推导，dof/响应各自计算。
预测误差与 range noise 仅作为同一个 joint LS noise vector，未重复计入信息。
nominal error 与 parity noise 不需要互相独立，union bound 足够。

对照检测是仅 range parity：在相同点 `H=H_tag C`，projector 消去预测
位置不确定性，dof=n-3；它不能利用 prediction 检测落在 range state-space
内的异常。两个统计量/阈值不同，不能复用 A threshold。

实际信息流：

`committed history → ingest IMU → prepare(no history recovery material)`
`→ read-only pre-UWB prediction → B NIS / full current candidate census`
`→ native retained-range pending group → one commit → actual world tag`
`→ componentwise center transfer / research-only PL`。

候选使用当前真实 range 和 principal covariance；native group 的测距 IDs
必须等于被选择的 retained subset。报警 epoch 每次 backend update=1；没有
先 all-in update 再删除的第二次 update。

若 Gaussian tag mean 是 μ_ref，实际 mean 是 μ_commit，则输出
`PL_commit,j = PL_ref,j + |μ_commit,j-μ_ref,j|`。
该三角不等式只绑定同一保护点/时刻的中心；**不能替代 nonlinear remainder
证明**。rotation/translation correction 和中心转移超过 .25 rad/.25 m 的
数值域拒绝。标签是冻结 Gaussian，不是完整 raw HMI。

## 4. 故障保护范围与风险时域

表中 COVERED_CONDITIONAL 均仅针对声明的固定 Gaussian 模型；不意味硬件
概率资格或某个流每次可用。C 是本轮实际执行的 UWB 全历史研究 scope，
其 IMU diagnostics 另行列出，不能把“代码中有假设”当成恢复成功。

| 故障事件 | A：纯 UWB 当前输出 | B：实际融合 tag | C：全历史 UWB 参考 / IMU diagnostic |
|---|---|---|---|
| 当前单 range | COVERED_CONDITIONAL | COVERED_CONDITIONAL，需有效 prediction | COVERED_CONDITIONAL |
| 当前两个 range | NOT_COVERED | NOT_COVERED | NOT_COVERED，本次 fixed-law scope 非任意 joint 两故障 |
| 有限 3-epoch 同 anchor episode | COVERED_CONDITIONAL，各独立输出再作联合 union | COVERED_CONDITIONAL，仅此前坏 range 实际未进入历史且有效性条件成立 | COVERED_CONDITIONAL，需完整 raw/history 支持及相同 episode law |
| 任意持续故障/carry-in | NOT_COVERED，A 仅逐 epoch 单 range 条件结果 | NOT_COVERED，不能继承 clean-history | MODEL_UNQUALIFIED，有限支持/历史资格限制 |
| 已进入图的旧坏 range | NOT_COVERED 于融合；不影响 A 的纯 UWB 模型 | NOT_COVERED，已知污染拒绝 | COVERED_CONDITIONAL，已建模/有历史来源的故障 |
| 当前 raw IMU 样本故障 | NOT_COVERED，A 不使用 IMU | NOT_COVERED，已知污染拒绝 | UWB scope NOT_COVERED；IMU raw diagnostic MODEL_UNQUALIFIED |
| 历史/跨 Combined factor 的 IMU 故障 | NOT_COVERED，A 只保护 tag range 解 | NOT_COVERED，已知污染拒绝 | IMU diagnostic MODEL_UNQUALIFIED / NUMERICAL_UNKNOWN，I2 无有效 IMU 恢复 |
| 共面镜像/保护方向几何退化 | MODEL_UNQUALIFIED，拒绝 | MODEL_UNQUALIFIED，保守拒绝，即使 prediction 可给局部解 | MODEL_UNQUALIFIED，原 U3 也拒绝 |
| exact fault/state alias 且保护非零 | UNDETECTABLE | UNDETECTABLE 于扩大后的 prior+current 故障集合；SPD prediction 下 current-only 单 range 无此零响应 | UNDETECTABLE，结构判据拒绝 |
| 近秩边界/未解析 condition | NUMERICAL_UNKNOWN | NUMERICAL_UNKNOWN | NUMERICAL_UNKNOWN |
| 噪声/校准/未知相关性/非线性误差 | MODEL_UNQUALIFIED | MODEL_UNQUALIFIED | MODEL_UNQUALIFIED |

反例：在 B augmented model `[I;H]` 中，把 fault 扩大为一列 exact
`[d;H d]`，可完全模拟状态变化，parity=0 而 `C d≠0`。确定性测试用 exact
translation state alias 验证 DANGEROUS_NULLSPACE。这说明扩大故障集合必须
重新建模；一次 current-only NIS pass 不证明历史/IMU 无害。

风险：固定 `α=1e-6`、`tail=1e-9`、最多 8 range 和 9 候选，50 输出。
每候选 Gaussian coordinate event 成本 tail、parity-noise event 成本 tail。
同时覆盖所有候选和时间，不要求输出/候选独立：

`P(任一被服务固定模型界失败) ≤ 2×9×50×1e-9 = 9e-7`。

每候选所有 current single-fault leaves 共享同一个 noise event、取最大 slope；
不再逐 leaf 收费。删除故障行的 zero leaf 是结构无害。候选选择没有额外未付
事件，因全 9 候选已付；固定的候选族不能扩大成任意数据驱动 linearization
family。该 `9e-7` **不是 raw trajectory HMI**，也不继承 C 的 proof ID。

单历元 false-alarm ≤ α，50 输出 continuity union ≤ 5e-5；它与 HMI noise
union 不是同一个事件，不能为使预算闭合而混加或省略。接受事件的漏检贡献
通过 ρ 控制，幅值对应 noncentral χ² power；没有另设未限定幅值的常数 miss
probability。候选选择收费在 9e-7 中。历史污染、IMU fault、model escape、
raw nonlinear remainder、未监测两故障等遗漏事件的风险是 **UNKNOWN /
UNALLOCATED**，不是零。硬件 UWB 1e-4、IMU 1e-5 先验仍 UNQUALIFIED。

episode 联合条件还要求它未污染 prior；不能以每 epoch 单故障条件界证明
整个持续 episode 都被正确剔除。已通过 FDE 的输出可有有限当前 PL，但通过
post test **不能**自动为下个 epoch 签发 clean-history 证书。

## 5. 数值及原生恢复验证

A 的参数化检查包括独立/相关 R、稠密 normal-equation WLS oracle、covariance、
rank/dof、小扰动响应、完整单 range 候选枚举、2.25 m fault 排除、共面/近秩
拒绝、exact dangerous/harmless 单 range alias。实际非线性响应与冻结响应的
容差明确分开；没有把近零当结构证据。

B 只使用两个定向快照：健康 epoch14、真实报警 epoch15（seed20261010）。
验证 source lineage、当前 range 不影响 prediction、NIS/parity 一致、
`CK` 与独立 dense SVD 一致、30D uncondensed QR 与15D Schur mean/response、
共同固定模型 PL、frozen finite difference、非零 lever 六维 Pose 数值 Jacobian、
5 类 prediction 前提负例、SPD/batch/几何/非线性域拒绝、history alias 反例。

- epoch14 NIS=4.06176，epoch15 NIS=423.639，dof=8；同点 range parity
  为2.51950/375.163、dof5，阈值35.8882；NIS阈值42.7009。
- uncondensed QR/Schur response error 分别 5.54e-13、1.30e-12。
- 实际非线性 healthy commit range0 响应 `[.0792342,.0747232,.00678965]`；
  frozen `[.0799457,.0747815,.00512352]`，差范数 .0018126。
  frozen QR finite difference error <1e-7；lever Jacobian error <1e-8。
- 健康共同模型 PL 相差 <1e-8。这里的未压缩 QR 是与 C Gaussian oracle
  同一求解方法的**相同子问题**，不是把 C 的 raw-history covariance 换成
  B 后再声称完整 C 保护等价。
- 原生恢复 ledger 验证 measurement1501 未入图、其余7条全部存在，commit=1，
  timestamp/保护点/center transfer 绑定。U1 后4个输出均继续定位和输出条件 PL。
- 5 个既有事务/PL 回归测试通过：prepare/discard、一次原子 commit、correlated
  principal covariance、历史原子替换、research-only PL。

C 的实际流共同 timestamp/committed tag 与 B 对齐核对。C raw ADC joint
covariance、历史 fault scope、CV freeze、detector dof 与 B 不同；这些都能造成
PL/response 不同，不能强行删去信息来获得同一数字。只有上述相同固定
Gaussian 子问题支持数值等价；整体成本比较不叫等价加速比。

## 6. 固定场景与成本

固定 U0/U1 50 epoch，U2 21 epoch、U3/I2 20 epoch；非零 range/IMU 噪声、运动、
lever=[.2,-.1,.08]。同一 NoisyMotionScenario、seed20261010、原生 estimator。
A 精度是纯 tag，B/C 是实际融合 tag。所有 best-effort commit 保留在输出
分母；拒绝 PL 不删除坏输出。A-gated backend 的姿态不属于 A PL。

正例及 FDE-OFF（位置单位 m；耗时 ms）。PL 表中为各自模型的条件数值，
较小的 B PL 不能解释为比 C 更强的完整性。

| 路径/场景 | 输出/条件PL | 正确剔除/注入 | tag APE RMSE/P95/max | HPL/VPL 均值 | core median/P95/max | full median/P95/max |
|---|---|---|---|---|---|---|
| A U0 | 50/50 | 0/0 | .121246/.191871/.259343 | .930297/1.603085 | .034806/.048699/.159824 | 1.037507/2.813269/4.087150 |
| B U0 | 50/50 | 0/0 | .069253/.102512/.165751 | .422686/.496583 | .090592/.132291/.152578 | 1.274603/3.475139/4.469700 |
| A U1 ON | 50/50 | 1/1 | .121359/.191871/.259343 | .935450/1.603323 | .034883/.050823/.300784 | 1.291377/2.523742/2.968852 |
| B U1 ON | 50/50 | 1/1 | .069338/.103407/.165751 | .423877/.496506 | .090354/.143175/.516825 | 1.385582/2.967544/3.858676 |
| A U1 OFF | 50/49 | 0/1 | .141588/.203965/.540190 | .930288/1.603581（49有限） | .034846/.065043/.768071 | 1.333878/2.744276/2.967291 |
| B U1 OFF | 50/14 | 0/1 | .086662/.142114/.226145 | .549576/.723247（14有限） | .000477/.104257/.118318 | 1.246557/3.530613/3.960777 |
| A U2 ON | 21/21 | 3/3 | .111277/.170362/.173929 | .965864/1.619954 | .038784/.206731/.207475 | .929096/1.341532/1.976821 |
| B U2 ON | 21/21 | 3/3 | .066242/.111767/.165751 | .520138/.639122 | .088334/.533539/.760536 | .919561/1.979949/2.495334 |

所有行每 epoch commit=1；U0/U1/U2 无误排除，有限输出全部在原 AL
(H=2m,V=3m) 内。U1 后 epoch16–19 条件PL/commit=4/4；U2 最后故障
在epoch17，epoch18–21=4/4，不以3个 follow-up 冒充4个。
B U1 OFF：epoch15 POST_REJECTED，随后35个 HISTORY_PREDICTION_CONTAMINATED；
拒绝后的短 core 并非有效完整性服务提速。

共同输入/时间 epoch14–20，各7个实际输出：

| 路径/场景 | tag APE RMSE/P95/max | HPL/VPL 均值 | core mean/median/P95/max | full mean/median/P95/max |
|---|---|---|---|---|
| A U0 | .088300/.139016/.156490 | .929305/1.603109 | .036702/.036013/.041047/.041857 | 1.258987/.978149/2.151901/2.440091 |
| B U0 | .035178/.054212/.062694 | .443512/.484775 | .092929/.090574/.105990/.110754 | 1.397764/1.000439/2.714396/3.107636 |
| C U0 | .035178/.054212/.062694 | .819440/.977682 | 203.030448/200.149728/221.750497/225.311629 | 204.368933/201.344409/223.095285/226.637468 |
| A U1 | .089400/.142034/.160803 | .966119/1.604809 | .074265/.035573/.223588/.300784 | 1.181427/.720039/2.415493/2.727696 |
| B U1 | .032810/.053845/.062694 | .451023/.484269 | .153564/.090176/.395076/.516825 | 1.445260/1.262443/2.453028/2.811712 |
| C U1 | .032810/.053845/.062694 | .836500/.979778 | 222.733406/207.401052/282.877110/309.078665 | 224.239925/208.707673/284.548776/310.836690 |

C/B committed tag 和时间逐行相等（absolute tolerance 1e-9m）；U1 均保留
7条健康 range，排除anchor1/measurement1501。A U1 报警373.750 >35.888，
post6.554、dof5→4，HPL/VPL=1.188430/1.590707；B 报警423.639 >42.701，
post13.084、dof8→7，HPL/VPL=.480755/.510492。
报警恢复 A core/full=.300784/1.045742ms，B=.516825/1.473845ms，
C=309.078665/310.836690ms。这是**不同 fault scope 的架构成本**，不报告
“等价加速比”。A/B U0/U1 有效流 40ms core/50ms full 超时均0/50，
C共同段则7/7均超时；短样本不证明截止期或硬件准入。

B prediction U0 median/P95/max=.105851/.719076/.809944ms，U1
=.094271/.520182/.591690ms；最大 Gaussian/committed 中心差 .005844m。
U0/U1 B tag increment RMSE=.038222/.037960m，SE3 translation RPE
=.037708/.037446m，rotation RPE=.021242/.020818rad；A pure tag increment
RMSE=.170012/.170163m，A-gated backend 同样 SE3 RPE。没有拿 A 的无姿态
输出充当独立 SE3 估计器。

负例（20输出）：A/B U3 finite PL=0/20，共面拒绝；A 没有合格 tag 解，
不能用 best-effort fused tag 的误差填入 A 精度。I2 是 scenario law 明确提供
的 IMU-invalid 负例，B=0/20、IMU_PREDICTION_CONTAMINATED，仍20次
best-effort commit；没有声称 B 自动检测了 raw IMU fault。A I2=20/20
纯 UWB 条件输出，APE RMSE=.110032m，绝非融合/IMU保护。
外部 history-invalid U0：B=0/20，A仍20/20；U2 OFF+history-invalid：
B=0/20，actual fused APE RMSE=.210381m、max=.542086m。已知污染证据是
显式负例输入，不是 GT 帮助 FDE 选行。漏检的未知历史污染不能被这个接口
自动发现，因此需要分层历史证书。

所有性能数据为固定 seed；字段/分母校验后作过一次同输入配对，报告最终
配对，未作随机 seed/窗口搜索。U2 延至21仅为补齐停止后4输出验证。


C 的本轮连续流只测共同 epoch14–20（13 epoch nominal prewarm）；参考其已
记录 epoch48 结果说明长期增长，不重新开发 C。A/B 的完整服务从 IMU ingest/
prepare 起，包含 prediction、detector/FDE、单次 backend commit、tag/中心转移及
误差计算；不含 CSV 写盘和最终 estimator destruction。A core 是独立纯 UWB
RAIM；B core 是当前 Gaussian RAIM/FDE，prediction 时间单列；C core 是它的
完整历史 integrity 计算。采样耗时是开发机实测，不是 ROS delivery/WCET。

矩阵规模：A 8×3，删后7×3；B 23×15，删后22×15，prediction30×30；
最多9候选、最多8当前单 range leaves，不随任务历史枚举。已有 backend 的
marginal 查询/commit 成本仍受 Bayes tree/固定滞后影响；不宣称 estimator
整体与历史长度无关。C epoch14→20 original rows337→475，active columns252，
native hypotheses224→320，detector dof112→160，summary bytes约7.25→7.51MB。
原 RESULTS epoch48 原始检测维度384、全部768 Gaussian leaves仍存在，表明
C 的模式身份/存储/部分计算仍增长。

## 7. HYBRID 在线接口与下一步最小开发包

数学和实测回答：保护**当前**融合输出不必每次枚举全部历史故障，前提是
有有效的联合 prediction/noise模型和历史风险证据。当前 B 的范围不足以单独
签发完整导航完整性；持续故障及时排除的功能成功不能补齐统计历史前提。
因此选择 HYBRID，保持轻量在线路径，C 作历史研究/离线审计及拒绝参考。

下一阶段最小接口，而不是新的 SVD 优化：

1. `PreUwbPredictionCertificateV1`：mean、完整 P、tangent/frame/time、
   source batch/sample lineage、freeze/version、与当前 range 的交叉 covariance
   或独立性证明；认证 shared-endpoint 传播、prior law 和 marginal 重基准。
2. `HistoryValidityCertificateV1`：covered historical episodes、carry-in、IMU
   scope、累积风险上界、expires_at/version。必须由独立历史监测器或有证明
   的 sufficient statistic 签发，不能由 B NIS pass 生成。
3. `CurrentEpochIntegrityResultV1`：A/B 各自模型身份、raw retained IDs、
   current candidate census/selection charge、protected reference、单次 commit
   receipt、committed tag/center transfer、有效域和拒绝原因。

在线流先取得历史证书和 IMU预测；A 可快速筛查/提供候选，但 B 仍对全部
允许 current candidates 做资格与 PL。A/B 同源检测不能当成两个独立证据；
若选择依赖 A，B 的风险必须覆盖 A 可能触发的全部候选（本轮上界9）。
测距只在最后选定的 native group 中进入一次 backend，更新后才绑定 PL。
历史证书无效、未知相关性或 candidate 数值资格失败，融合 PL 拒绝；可以
继续 best-effort 导航或提供 A 的纯 UWB 条件 tag 输出。C 按自己的历史 scope
作补充监测，不能无证明地把 C 的历史 PL 身份转授给 B。

任务风险需分配 current conditional noise、selection、历史污染证书、IMU、
model/nonlinear escape；未知项继续阻断生产发布。先证明历史摘要在故障支持
和 noise cross-map 上充分，再验证局部 nonlinear/branch remainder。40/50ms
目标对本轮轻量数值服务已有余量，下一阶段应围绕这些接口与证明扩展，
而不是重新压缩 C 的故障 SVD。GO 生产替换尚不成立；NO-GO 数值算法也不
符合已完成的实际恢复与一致性证据。

## 8. 代码身份与复现

A 初始验证提交 `145b019`；B 数值/事务提交 `4c434d7`；测试/固定对照提交 `f1a0cca`；本文提交
可通过 `git log -4 --oneline` 查看。不 push；没有保存二进制/矩阵/CSV/备份。
不需要新增图：表格足以区分范围和成本。

环境：既有 ROS Noetic/GTSAM 工程、Release、Intel Core Ultra 5 125H；
脚本固定单线程 BLAS/OMP。从仓库根目录运行：

```bash
python3 tools/run_epoch_architecture.py check --build
python3 tools/run_epoch_architecture.py compare --historical
```

直接调用：

```bash
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
TMP=$(mktemp -d /tmp/epoch-architecture.XXXXXXXX)
"$BIN/epoch_raim_runner" config/fde_next.yaml "$TMP/A.csv" U1 uwb_epoch_raim on 50
"$BIN/epoch_raim_runner" config/fde_next.yaml "$TMP/B.csv" U1 fused_epoch_pl on 50
"$BIN/epoch_raim_runner" config/fde_next.yaml "$TMP/negative.csv" U0 fused_epoch_pl on 20 history-invalid
# 汇总后仅删除本次自己创建的 TMP。
rm -rf "$TMP"
```

`historical_reference` 通过 compare 调用原 fde_next_runner online-summary scope；
未向生产配置新增算法开关。CSV 记录模型版本、time/frame/tag、reference/
linearization tag、fault IDs、dof/threshold/post、slope、AL、资格、candidate count、
prediction/core/full timing、risk scope 和 one-update 检查。
