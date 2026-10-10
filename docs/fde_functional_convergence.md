# 离散 UWB IMU FDE 功能与适用边界

2026-10-10。本轮可实施研发已完成，原始传感器的 **UWB 条件闭环可运行，短暂 IMU 故障不能恢复**。没有满足资格的原始传感器 IMU 隔离正例；检测功效改善不作为恢复成功。硬件概率资格和受保护发布继续关闭，历史 PARTIAL_BLOCKED 结论及失败证据保留。

## 功能结果

本轮直接执行离散 GTSAM/iSAM2、Combined IMU 预积分、原 detector、候选 rank/step、post detector、FdeManager 和真实 epoch transaction。仅使用 UWB＋IMU；GT 只生成输入和评价提交位置，未提供速度、里程计或故障标签给选择器。每个流七个 epoch、每 epoch 一次 backend update。数据见 [现有数值报告](benchmark/fde_operability_20261010.json) 的 `functional_convergence`，原始 CSV/log 在 `results/fde_convergence_20261010/`。

| 范围和等级 | 实际结果 | 定位质量和可用性 |
|---|---|---|
| FUNCTIONAL：UWB raw range +2.25 m，epoch 3 | statistic 435.4703 > 72.22885；自动选择 action 2，替换 group 3002；提交配方确实去掉故障 measurement、保留七条健康 range | 隔离后四个 epoch 均继续 selected commit；七次位置误差均 <1e-12 m |
| CONDITIONAL：上述 UWB 输出保证 | 保留旧 range union/common reference 仿真证明；本次七次条件可用 | 健康 range 误差≤1e-12 m、静止、零 lever arm、每 epoch 至多一坏 range；这解释了数值零误差，不能外推真实硬件精度 |
| BLOCKED：原始 IMU accel-x +20 m/s²，十个新 raw samples，50 ms | epoch 3 statistic 0.0915054 < 72.22885；epoch 3–7 都不报警，没有剔除；KEEP 候选原步长门限失败 | 五个 epoch 未恢复/不可用，但保留 best-effort 定位；七次 RMSE 0.023094 m，最大 0.033230 m，最后 0.026271 m |
| UNQUALIFIED：上述全部输出 | `formal_eligible=false`，`publication_protected=false` | 真实硬件保护可用次数为零；未恢复保护延迟为 null / RIGHT_CENSORED |

UWB 的严格原风险选择在故障 epoch 仍拒绝，原因是完整风险超过原预算；条件 common-reference 路径才接受。原生产 epoch 7 的 4.69e-5 > 4e-5 反例复用，不重跑、不删账。不能把本次“七次条件可用”写成原生产配置恢复率。

本次 UWB 故障处理 54.661 ms，七次 core 中位数 50.106 ms、最大 109.639 ms；四个后续 commit 中只有 epoch 4 满足 40/50 ms。IMU 故障 epoch 17.332 ms、流最大 100.360 ms，但未恢复；这些耗时不代表受保护恢复延迟。计时包含 raw 输入构造、全部模型响应、检测/候选/选择、实际 commit；无实时资格或 P99 推断。

## IMU 可检测性

复用旧 raw episode、相邻支持、shared-noise、历史映射及四窗口结果；只补两个定向快照：静止和有平移加旋转激励的 epoch 12。后者由解析运动生成 raw IMU/range，估计器未收到真实轨迹或速度。六轴均验证 50/500 ms：短事件影响两个相邻 Combined 因子，长事件在观测截止前影响十个因子；未来尾部保持截断。实际有限幅值 raw 腐化及重积分与局部响应最大相对差 4.76e-4，QR 正交误差 <2e-16。健康激励流 statistic 0.000671 < 原门限，未人为制造健康报警。

下表是**冻结独立 Gaussian 模型下**的条件漏检概率，epoch 12 dof=96、门限=176.777572；加速度幅值20 m/s²，陀螺幅值0.8 rad/s。它们不是硬件 p_md，也不是隔离成功率。

| body 轴 | 静止 50 ms | 静止 500 ms | 平移旋转 50 ms | 平移旋转 500 ms |
|---|---:|---:|---:|---:|
| accel-x | .99999656 | .00023674 | .99999618 | .00002075 |
| accel-y | .99999656 | .00023674 | .99999666 | .00207766 |
| accel-z | .99999594 | .00445482 | .99999687 | .26266287 |
| gyro-x | .99999897 | .99999819 | .99999898 | .99999866 |
| gyro-y | .99999897 | .99999819 | .99999897 | .99999818 |
| gyro-z | .99999900 | .99999900 | .99999897 | .99999793 |

窗口/持续时间作用有量化反例：旧静止 epoch 3 的 50 ms accel-x 实测无报警；成熟 epoch 12 同一短事件 λ=2.95938，仍几乎必漏。旧 250 ms λ=77.1511、p_md=.579234；500 ms λ=174.6448、p_md=.00023674。延长观测能增加信息，但不保证每条确定性 realization 报警；500 ms 静止无噪声有限信号 parity=174.6448，仍略低于176.7776。

500 ms accel-x 的激励快照 λ=193.3919，有限信号 parity=193.3919，可超过门限，但这是健康冻结窗口上的真实腐化响应，并非在线 detector→isolation→commit 正例。原 all-in 状态响应范数约 20×.470492=9.410 > .25；这个 gate 是 Pose/速度/bias 混合单位的原标量门限。它只能证明该 all-in 线性修正不合格，**不能证明所有剔除候选都会失败**。未用外推所需幅值进行新注入，也未搜索幅值/参数来造恢复。

该激励也不属于原 generic bridge 的已声明恢复域：生成器初始加速度5.4 m/s²已超过原4 m/s²上界；500 ms episode又超出旧50 ms batch模式。故这一方向只获得局部检测信息，尚无匹配的恢复模型/覆盖证据，不凭stationary bridge标签执行所谓成功恢复。未放宽bridge域或把该超域场景当成合法隔离正例。

原生产快照的自由状态/固定 bias 对照直接复用：残差 .187016/.187068，固定速度后1542.9978；因此主要由速度解释短事件，非 bias 吸收，也非 raw 输入没有进入残差。静止短事件、yaw 无参考、原模型未覆盖持续/多轴/共同原因等类别必须拒绝宣称已隔离。额外里程计的旧七次 IMU 条件 commit 只作比较参照，未纳入原始传感器成功分母。

几何与激励作用不能概括成“更多运动/更低 anchor 一定更好”：复用旧单纯旋转500 ms gyro-z λ=.04310，本次平移旋转 λ=1.69920，仍不足；accel-z 本次条件漏检比静止更差。旧共面高度1.1的 accel-x λ=184.9302，比原几何高，但存在全局镜像位置歧义，不能据功效选择真实分支。

## PL 和风险保证

| 状态 | 必须满足或拒绝的条件 |
|---|---|
| 数学有限 Gaussian PL | 冻结 H、噪声和故障图正确；检测响应的零空间不能携带 protected position；余项、历史和候选上下文一致。有限数值不自动证明概率或发布资格 |
| 真正危险方向 | 若存在 f 使 Q₂ᵀDf=0 且 protected response≠0，线性 Gaussian 故障界无界。anchor 与 tag 同平面时 range Jacobian 的垂直列为零；该位置方向不能只凭 range 线性检测恢复 |
| 无害零方向 | 静止、零 lever arm 的 gyro-z：500 ms Γ≈4.0e-26，unit position shift≈4.3e-16。yaw 未检测不能误写成当前位置无界；这不保证未来运动/非零 lever arm 下无害 |
| 全局几何歧义 | 全部 anchor 在z=1.1时，静止位置z=1与z=1.2产生相同 UWB 和 IMU；40项小检查包含逐range相等和拒绝验证。bootstrap 选择分支不提供真实分支的独立证据；此歧义与局部数值 rank 失败分别记录 |
| NUMERICAL_INDETERMINATE | 旧正常/联合 research Gaussian Flat 因 hypothesis 62 rank 0/1 被拒绝。结合静止 yaw 反例，此终端拒绝本身不能证明危险位置零空间；保留 fail-closed，不调 rank cutoff 绕过证书 |
| CONDITIONAL | 旧有界 range union 是独立于 process/history/optimizer 的当前位置条件保证，故特定仿真 escape 项有结构零；带数值 guard 的 SVD 是此尺度下原型，不是任意数据的区间算术认证 |
| UNQUALIFIED | 实际 raw shared-noise 联合协方差、Gaussian tails、prior、episode 时域及 bridge/history/model 证据未获硬件资格；实际部署保持 fail-closed |

普通 nc-χ² 要求独立正确白化噪声。共享 raw sample 时应使用广义 Gaussian 二次型；旧 joint covariance / Cantelli 结果复用，不能把上表改称实测漏检界。未知的 omitted、共同原因或 hardware escape 不置零；原 prior 1e-4/1e-5、总预算4e-5、AL、step/rank和正式发布门槛均未改。

## 代码收敛和验证

| 代码 | 适用路径与处置 |
|---|---|
| 现有 `fde_manager` / `risk_budget_audit` / PL proof binding / actual-frozen IMU map | 保留已集成生产实现：完整预算、外部字段绑定、refusal语义和异常所有权；不能据本轮升级受保护资格 |
| `research/raw_imu_episode.*` | 真实 sample support、biasHat 重积分、相邻响应可用于非保护回放/诊断；完整 raw-episode 故障图进入生产完整性路径仍需合同审查。未复制替换旧生产 interval 模型 |
| `research/recovery_experiment.cpp` | 新增实际提交配方的range lineage正确排除评价、提交位置与 risk NOT_RUN 诊断；GT label只在选择后评价。可借用这些诊断方法到非保护路径，无新 best-effort 剔除规则 |
| `research/power_experiment.cpp directions` / `contract_checks.cpp` | 默认隔离、不安装：六轴方向、解析运动一致性、真实有限腐化、镜像/垂直零方向反例。检测模型保持原Gaussian门限 |
| `PriorEvidence`、range union、common-reference selector、shared-noise | research-only；只读证据接口可作常规工程审查候选，仿真 factory 和位置保证不得直接授予生产资格 |
| 无支持 raw-response 优化 | 保留默认OFF研究优化；本轮未改其算法。旧冻结35-attempt配对normal49.20→35.58 ms、joint2 173.68→161.46 ms直接复用；均Gaussian拒绝，不称保护性能 |

本轮没有新性能候选，故不再做35-attempt配对；已有四组35次和全部失败/最慢值保留。新增仅两个定向快照（24行，整进程0.12s）、40项合同小检查；最后运行一次核心安全回归9项通过（38ms），包含危险/无害零空间、风险保守分配、proof/exception/refusal。旧184个support检查、11个prior检查和历史大矩阵原样保留。

## 复现

在既有catkin workspace执行；不重跑 capture、旧大规模 replay 或配对。输出目录须存在。

```sh
mkdir -p results/fde_convergence_20261010
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_research_power fde_research_recovery fde_research_contract_checks test_integrity_v2 -j2
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
fde_bin=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
/usr/bin/time -f '%e seconds' -o results/fde_convergence_20261010/directions.time "$fde_bin/fde_research_power" config/fde_imu_order1.yaml results/fde_convergence_20261010/directions.csv directions > results/fde_convergence_20261010/directions.log 2>&1
"$fde_bin/fde_research_recovery" config/fde_imu_order1.yaml results/fde_convergence_20261010/uwb.csv uwb 0 > results/fde_convergence_20261010/uwb.log 2>&1
"$fde_bin/fde_research_recovery" config/fde_imu_order1.yaml results/fde_convergence_20261010/imu.csv imu 0 > results/fde_convergence_20261010/imu.log 2>&1
"$fde_bin/fde_research_contract_checks" config/fde_imu_order1.yaml > results/fde_convergence_20261010/contract_checks.log 2>&1
"$fde_bin/test_integrity_v2" --gtest_filter='FdeOperabilityBinding.*:P103ProofArena.NonemptyInvalidFinalBundleFailsClosed:P103ProofArena.ScopedPipelineExceptionClosesArenaAndDiscardsAttempt:FdeOperabilityStatus.*:FdeOperabilityImuReference.*:FdeOperabilityBatchProofs.*:B3ZeroSpace.*:IntegrityV2Risk.EqualAllocationNeverOvershootsAndRealExcessFails' > results/fde_convergence_20261010/core_safety.log 2>&1
python3 research/report_convergence.py
```

`research/report_convergence.py`校验本地已保留CSV/log、核心回归及旧功效/配对证据，再向原JSON追加结果；不覆盖历史段。CSV中的inf表示终端不可用，JSON规范化为null，配合具体门限/拒绝原因解读，不能当数学无界证明。

## 仍需的独立信息和外部证据

**BLOCKED只针对相应能力，不阻止本轮结束。** 短IMU事件需要足够独立运动信息；静止yaw需要方向参考或有可辨识的运动激励；共面range镜像需要被证明的物理分支域或额外几何。并非所有IMU故障必需新传感器，但本轮原始范围没有合格隔离正例；未来若用独立速度/运动传感器，应单独批准，不将旧里程计正例包装为UWB＋IMU。

**UNQUALIFIED：** 硬件需要物理故障谓词及 onset/duration/history时间映射、occupancy/共同原因和边缘概率证据、raw联合噪声/尾部证据、bridge/model域和余项证据，以及完整截止时刻测量。概率交集沿用min边缘界，不推断独立乘积。采样成功比例不能替代硬件概率承诺。上述证据与正式合同替换/受保护发布资格由另行审批处理；本轮不请求修改合同、不开启正式发布。
