# HYBRID H/P/B 实现与验证结果

2026-10-11：**HYBRID_FIXED_MODEL_INTEGRATED**。P1–P4 / A1–A10 在下述有限固定模型域内完成。nonlinear raw 回放达到功能和冻结数值验证；**RAW_UNQUALIFIED，hardware UNKNOWN**。生产 YAML、概率合同、AL、风险预算、公共 estimator ABI、formal eligibility 和 protected publication 未改变。研究目标保持 `EXCLUDE_FROM_ALL`，不安装、不接入受保护 ROS 发布。

## 版本与实现

分支 `feature/realtime-uwb-imu-pl`，起点为干净的 `cf3f4676a404259123a1d1e3c1ec670670fc9c28`。没有回退或推送。本地工作包：`c6c1e45` unified orchestration；`a5d4fe0` historical evidence/math；`7d55cf6` prediction bounds/native lifecycle；本报告所在提交保存最终分母、结果和缓存清理。

环境：Linux 6.18.40.1 WSL2 x86_64、Intel Core Ultra 5 125H、GCC/CMake Release、Eigen/GTSAM/ROS Noetic。运行 `OMP_NUM_THREADS=OPENBLAS_NUM_THREADS=MKL_NUM_THREADS=1`。场景 seed 固定20261010，没有搜索 seed 或按失败轨迹调门限；不是硬件时限认证。

| 实际代码 | 内容 |
|---|---|
| [epoch_raim.hpp](../../research/epoch_raim.hpp)、[fused_epoch_pl.hpp](../../research/fused_epoch_pl.hpp) | 复用A/旧B数值核；A报警可请求B完整候选；旧布尔条件仅在legacy入口 |
| [hybrid_evidence.hpp](../../research/hybrid_evidence.hpp) | 复用平方根摘要、21D状态/共享端点边界、历史残差可行区间并集，append/retire/rebuild |
| [hybrid_prediction.hpp](../../research/hybrid_prediction.hpp) | H完整语义复验、full P及共享端点cross、N符号/删除、history-aware detector/PL、固定模型选择 |
| [online_gaussian_history.cpp](../../research/online_gaussian_history.cpp)、[header](../../research/online_gaussian_history.hpp) | 真正排除全部当前UWB的premeasurement query，pending IMU/raw ADC共源列，full15 mean/cov/历史响应；无历史C PL |
| [hybrid_coordinator.hpp](../../research/hybrid_coordinator.hpp)、[epoch_raim_runner.cpp](../../research/epoch_raim_runner.cpp) | 一次prepare，独立A/B，唯一native commit，实际ledger/receipt/retained IDs，final中心/AL/time及receipt后H更新 |
| [hybrid_checks.cpp](../../research/hybrid_checks.cpp)、[hybrid_native_flow.cpp](../../research/hybrid_native_flow.cpp)、[online_history_checks.cpp](../../research/online_history_checks.cpp) | 固定数学/生命周期/native流、独立full-prefix/normal equations/joint QR/raw ADC oracle、原C回归 |
| [CLI](../../tools/run_epoch_architecture.py) | 保留check/compare，新增hybrid/check-hybrid/lifecycle，固定流/全分母/完整成本 |

`OnlineGaussianHistory::prepare` 缺少frozen material时曾空指针解引用：用light transaction复现，修为状态修改前明确拒绝，native fixture有直接回归。`prepare(tx,false)`仅跳过raw HYBRID不消费的current-UWB root；旧C默认行为保留。未改C SVD，不声明性能收益。最终审查修正了共用选择器在A-only报警时不可选KEEP的遗漏，定向测试要求完整9候选且可选合格KEEP；legacy不报警行为保持。

## H/P/B数学域与共同风险事件

law在观察前注册：15D顺序rotation/translation/velocity/accel bias/gyro bias，另有6D共享端点。24epoch、8range，至多一个任意幅值UWB source episode，全部source/onset枚举，持续1或3epoch；IMU无故障、初始无carry-in fault。不是任意历史、多episode或未知初始误差保证。

`F=I`加translation←0.05 velocity、rotation←−0.05 gyro bias、velocity←−0.05 accel bias；L前六维identity。`x_k=F x_(k-1)+L(u_(k-1)+u_k)+w_k`，`z_k=H x_k+f_k+v_k`。独立Gaussian primitives的sigma：初始15D 0.002、process15D 0.0002、endpoint6D 0.0001、range 0.003。C含lever的固定world tag映射；native H为固定reference tag到anchors单位方向乘C，参数化数学测试用八个固定立方体方向乘C。

native fixture实际ingest IMU、使用Combined IMU/iSAM2和UWB因子、唯一提交；range编码上述线性观测，真值为该law的15D状态。**这是线性模型与native中心集成，不是raw IMU物理运动证明。** actual tag不必等于CK参考；三轴绝对中心转移给served界，不能补齐raw nonlinear remainder。

H从实际initial-law rows初始化。receipt后追加过程、新endpoint prior及实际retained ranges，退休21columns保留故障residual sufficient statistics。对每个解释h保留完整边界响应`D_h=-gain B_h`和residual列Z。`f_hat=Z'r/(Z'Z)`，区间`[f_hat-q/||Z||,f_hat+q/||Z||]`是残差球可行集合的保守外包络。E_H是全部`D_h interval_h`并集加零故障，不根据最似然source删解释、不用PASS清零。Gaussian covariance另列，非tag-only PL。零residual而完整状态响应非零、近秩不定、资源不足拒绝；严格零完整响应才跳过。该SPD initial/process/range law使允许的非零range模式可辨，没有靠epsilon签有限无界证书。

P用`T=[F L]`：`mean=T H.mean`，`P-=T H.cov T'+Q+sigma_endpoint²LL'`，包含`F Cov(x,u)L'`及转置；遗漏cross的负例实际失败。当前range primitives独立于history/IMU，因此固定N=0；raw generator也声明range/IMU独立，hardware不能继承。通用核支持`S=HPH'+R+HN+N'H'`、`K=(PH'+N)S^-1`；augmented `[I;H]` cross为**−N**。删除先选原R主子矩阵/N对应列。独立joint QR、直接normal equations、错误符号负例通过。N=0的gain/mean/cov/NIS对齐旧B核；新共同事件门限和历史PL不同于旧current-only合同，不声称整体PL数值相等。

B计算`eta=sup||S^-1/2 H d_H||`和`history_j=sup|C_j(I-KH)d_H|`。门限`(q+eta)²`，current项`max_slope_j*(sqrt(threshold)+q+eta)`，nominal项`q sqrt((C P_post C')_jj)`，加history及final center。无报警KEEP仍覆盖全部current单fault方向；A/B任一报警检查KEEP和八个删除，A不删候选。固定模型先检查risk/AL再选post最小；raw只作numeric选择，最终资格拒绝，A PL不贴融合输出。

一维`P=R=H=C=1,d_H=10,f=-10`令innovation=0。独立oracle给eta=10/sqrt(2)、history输出5；current-only加output-history仍小于真实误差10，新eta+输出界覆盖10。历史项确实改变接受事件和PL。

原始标准Gaussian向量ξ包含initial21及每epoch process15/new endpoint6/range8，维数`21+24*(21+8)=717`。共同事件`E={||ξ||<=q}`，`q=sqrt(chi²_717 upper_tail(1e-6))=30.1928`，失败概率1e-6，小于隔离law预算2e-6。对**任何**retained历史、candidate、epoch，固定矩阵LS Gaussian error/whitened innovation是ξ的线性映射；正确covariance标准化后operator norm≤1，历史parity/profile也是正交投影/压缩。故E同时约束全部noise，QR退休不另抽噪声。路径可依赖数据，但每条路径仍受同一个确定性||ξ||界约束，无需`9*50*tail`或通过概率相乘。

在E内，真实历史fault属于H外包络，P包含其传播；当前候选接受时三角不等式界定current amplitude，合成PL覆盖reference误差，中心转移覆盖actual tag。因此对全部允许幅值/source/onset、完整adaptive选择和24输出：`P(any supported fixed-law HMI)<=1e-6`。同一episode历史/当前共享参数由更宽数值包络包含，概率仍用同一个E。单seed error<PL是实现回归，不是概率证明。

B无current fault时E保证新的接受门限，task B误报≤1e-6。固定A是已有Gaussian parity，每epoch alpha=1e-6；保守task A-or-B误报≤24*alpha+epsilon=25e-6，未假设独立，且与HMI预算是不同指标。initial/history/process/current/选择共用E，不重复收费；horizon末H STALE，不能续期清零carry-in。IMU fault、random chart、nonlinear remainder、真实初始/噪声law域外，风险不得写0。

## 实际分母与生命周期

固定数学H/P/B有效20/20，非零历史项19/20。每epoch独立未退休full-prefix normal equations核对mean、21D covariance和历史fault方向。两个native固定流各H/P/B/final receipt/center/AL有效24/24，非零历史项各23/24，native commit各24/24且一次backend update。弱历史source0 epoch3–5加0.002未排除；current source0 epoch10加0.5正确排除一次、epoch11–14四个后续有效；producer不读truth/scene有效标签。

| 固定native | max history norm(m) | max eta | max center shift(m) | max served HPL(m) | max error(m) |
|---|---:|---:|---:|---:|---:|
| weak history | 0.0206654 | 9.73409 | 0.00434215 | 0.091047 | 0.00474523 |
| current | 0.0206559 | 9.7351 | 0.00434000 | 0.091047 | 0.00474888 |

实际拒绝：12类H字段篡改、P covariance shape、generation、source/重复更新、resource cap、stale refresh/fresh initializer。核验完整law/mean/cov/segments/risk/source，不只ID。每条native epoch8撤销，P9拒绝，从initial law和已提交observation/subset recipes**新建producer重放append/retire**后恢复，epoch9继续，epoch24 H STALE。backend_updates=0 receipt及伪造observations不推进H；错误receipt、迟到、final center超AL降级不增加commit。迟到可保留numeric availability，但timelyAvailable=false。

raw提取绑定transaction/version/timestamp/excluded batch，保留原`researchPreUwbPredictionV1`只读native lineage；past retained+pending IMU摘要删全部current UWB。epoch31独立dense raw-prefix核对full15 mean/cov/shared ADC和每个历史响应；current ranges全加100不改变P。原C ν31=248/stat272.988、epoch30/31/32全字段回归通过；旧summary危险/无害nullspace、correlated deletion、dof测试保留。

最终raw族201epoch，H/pre-UWB P数值BOUNDED **201/201**，但qualified H/P **0/201**；实际commit一次 **201/201**。融合qualified/AL均 **0/201**，raw risk UNKNOWN、formal/protected=false。

| 流 | 输出 | H numeric / 非零B历史项 | A可用 | A/B报警 | 正确排除 | B有限数值 | 50ms内 |
|---|---:|---:|---:|---:|---:|---:|---:|
| U0 | 50 | 50/49 | 50 | 0/0 | 0 | 50 | 7 |
| U1 | 50 | 50/49 | 50 | 1/0 | 1 | 50 | 7 |
| U2 | 21 | 21/20 | 21 | 3/0 | 3 | 21 | 6 |
| U3 | 20 | 20/0 | 0 | 0/0 | 0 | 0 | 7 |
| H1 | 20 | 20/19 | 20 | 0/0 | 0 | 20 | 7 |
| I2无标签 | 20 | 20/19 | 20 | 0/0 | 0 | 20 | 7 |
| I2显式IMU缺失 | 20 | 20/19* | 20 | 0/0 | 0 | 0 | 7 |

*显式缺失在计算后拒绝，保留诊断历史项不算有效B界。各流健康测距误排0/全部输出。U0无故障，A/B误报各0/50。U1仅A报警（B的保守共同球门限未报警），epoch15完整9候选，排除实际measurement1501，实际retained=7，epoch16–19至少四次更新。U2 epoch15–17三次正确排除，停止后18–21历史项全部非零。H1输入epoch3/source0加0.02，无history-invalid标签；A/B不报警，数据进入图，epoch4历史项非零。U3几何拒绝20/20。I2无标签不是IMU自动检测成功；显式缺失20/20 IMU_EVIDENCE_MISSING。额外外部H缺失3epoch：B拒绝3/3、独立A可用3/3、名义一次commit3/3，实际八个源。

raw U0 mean HPL/VPL=5.8095/7.1430m，U1=5.8319/7.1460m，max均10.0431/16.8969m，明显超AL。mean历史项norm=1.4953/1.4980m、eta=22.0259/22.0983，max历史项2.7528m/eta45.4521。保守原始球和全部解释是能力缺口，不能删eta/改AL。固定正例全部AL通过，不是正确H后所有正例失败。raw law/σ不同，不能借固定风险或小PL。

## 完整成本与C调用

单位ms。固定full包含输入/prepare、H、P及重复验证、A/B、risk/AL、selection验证、commit、final、H update/rebuild及owner cleanup；H init另测。固定P95为排序索引，raw为NumPy percentile，均不是WCET。

| fixed native | full mean/median/P95/max | H init |
|---|---|---:|
| weak history | 2.86787/2.91437/3.79551/6.26776 | 0.011253 |
| current | 2.76068/2.87654/3.79403/5.94709 | 0.007017 |

| 固定阶段mean | weak | current |
|---|---:|---:|
| H query | 0.108177 | 0.107769 |
| P/native read | 0.258568 | 0.247496 |
| A | 0.0244134 | 0.023662 |
| B/H-P复验/risk/AL | 0.371458 | 0.384715 |
| selection/准备验证 | 0.00619487 | 0.00583379 |
| commit | 0.652472 | 0.571556 |
| final center/risk/AL/receipt | 0.00103996 | 0.000776958 |
| ledger/H update/rebuild/cleanup | 1.3486 | 1.33564 |

raw h_ms包括prepare/query和**完整pre-UWB mean/cov/历史集合提取**；prediction_ms是另行native读取。risk/复验在B/final内；所有H费用都在full内。full从ingest/prepare起，不含CSV/日志写盘和分析；未测函数返回后小对象析构/进程关闭。raw sampler/estimator初始化不在每epoch服务内，无protected deadline保证。

| raw阶段mean/median/P95/max | U0 | U1 |
|---|---|---|
| H/P提取 | 112.5166/130.9949/138.1253/139.5158 | 113.2364/130.3399/139.8958/150.7520 |
| native prediction | 0.3148/0.2204/0.6734/0.6986 | 0.3264/0.2289/0.6920/0.9686 |
| A | 0.0685/0.0618/0.0970/0.2154 | 0.0707/0.0624/0.1073/0.2512 |
| B | 0.2166/0.2113/0.2960/0.4490 | 0.2305/0.2058/0.4252/0.7718 |
| verification | 0.0133/0.0127/0.0204/0.0353 | 0.0131/0.0126/0.0188/0.0298 |
| commit | 1.7510/1.4519/3.3489/5.2826 | 1.8431/1.5374/3.6042/4.1645 |
| H update | 0.0213/0.0211/0.0353/0.0460 | 0.0235/0.0212/0.0413/0.0802 |
| finalization | 0.0380/0.0369/0.0521/0.0537 | 0.0382/0.0363/0.0563/0.0611 |
| cleanup | 0.0222/0.0205/0.0454/0.0719 | 0.0299/0.0268/0.0602/0.1649 |
| **full** | **115.4795/133.6891/141.2590/144.2455** | **116.3480/133.4361/143.9611/156.3298** |

U1由A报警触发搜索的epoch15 core1.023010、full约131.998，不能用core约1ms声称实时达标。其他raw full mean/median/P95/max：U2 87.3287/127.2894/132.6757/133.7740，U3 83.1826/110.4164/132.9645/134.8897，H1 83.4149/112.0904/132.6653/132.9262，无标签I2 89.1576/128.5148/136.9112/155.9024，显式I2 93.5506/112.4242/185.8863/205.6668。短prefix mean不作稳态吞吐承诺；raw整体40ms core/50ms full工程目标未达。

默认fixed/native/raw **full_C_calls=0**，H直接读residual统计，不算C历史PL/候选。固定H有O(F)模式/recipe，horizon24、16MiB numeric拒绝cap；不是无限时域O(1)。raw沿用integrity summary window10，退休保留历史影响；numeric峰值U0/U1 **3,088,408bytes**、其他短流 **2,870,856bytes**，不是进程RSS或包含所有container/allocator的内存封顶。独立C回归50epoch有800模式/numeric7,789,112bytes。

离线fde_online_checks：summary尝试114、返回56，full-prefix C oracle3、pre-UWB无fault prediction oracle1；尝试含50次提交后拒绝和tamper/resource负例，全部显式离线回归，不隐藏在HYBRID。另跑原fde_summary_checks的36参数化cases/既有growth/nullspace/dof，以及旧A/B、7个原子事务/PL核心、6个非保护发布/terminal/reference/ABI测试，全部通过。分母输出及A-only报警KEEP修正后重跑相关固定/旧A-B回归和CLI族，本表采用最后一轮；B报警由原KEEP statistic/threshold判定，CSV alarm本身表示联合搜索触发；开发流不计独立样本，无加速声明，未开展35epoch配对竞赛。

## A1–A10

| 项 | 声明域内结论 | 支持 |
|---|---|---|
| A1 | 完成 | 源码审查、旧check/summary/online C；生产合同不变 |
| A2 | 完成 | coordinator/native48/raw201，唯一prepare/commit，actual retained/receipt/final/H版本 |
| A3 | 完成 | producer固定20+48；非零历史19+46；独立prefix oracle |
| A4 | 完成 | eta/support实际消费，抵消/相关QR/H1反例 |
| A5 | 完成 | full P/shared端点/N符号/principal，epoch31raw ADC oracle，current+100不复用 |
| A6 | 完成 | tamper/version/resource/stale/horizon/未提交拒绝/final降级，两条H2真实replay |
| A7 | 完成 | U1排除+4后续、U2停止后4非零、H1无truth、I2分开 |
| A8 | 完成 | 717D共同事件完整adaptive任务risk≤1e-6；raw/hardware UNKNOWN |
| A9 | 完成 | 全部计时/init/rebuild/resource/C计数；raw实时失败如实记录 |
| A10 | 完成 | 唯一报告/入口、回归/复现、本地commit及本轮缓存清理 |

## 复现与剩余缺口

在已配置的workspace中：

```bash
python3 tools/run_epoch_architecture.py check-hybrid --build
python3 tools/run_epoch_architecture.py hybrid
python3 tools/run_epoch_architecture.py check
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib:${LD_LIBRARY_PATH:-}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_online_checks fde_summary_checks -j2
"$BIN/fde_online_checks" config/fde_next.yaml
"$BIN/fde_summary_checks"
"$BIN/test_integrity_v2" --gtest_filter='IntegrityV2Transaction.*:IntegrityV2ProtectionLevel.RecomputesPostCandidateAndStaysResearchOnly'
"$BIN/test_p0_05_publication" --gtest_filter='P005Transaction.*:P005Reference.*:P005Publication.NonAtomicFanoutCanNeverActivateProtectedReceipt:P005Abi.*'
```

`lifecycle`运行完整固定检查；`hybrid --scene U1`仅复现50epoch故障流。外部H缺失用`epoch_raim_runner CONFIG CSV U0 hybrid on 3 history-invalid`，CSV放独占cache并删除。CLI成功后自动清理CSV。永久仅源码/必要测试/此报告，无新图/数据/二进制/矩阵归档；本轮cache核对汇总后清理，旧资料/用户文件保留。

仍缺raw真实初始/噪声law、随机线性化与非线性余项、域外IMU fault风险、任务carry-in/跨horizon，以及H外包络保守性与raw提取成本。hardware/受保护发布未准入。可据当前full-state/source oracle继续局部raw/noise/linearization资格研究；本轮不批准生产替换，不用C SVD提速代替证据闭环。


## 2026-10-12：raw PL 保守性定位与最小收紧

基线 `919760e` 已包含 `c6c1e45/a5d4fe0/7d55cf6/1708c62`；数学增量提交 `f2f14b0`。复用原 A/B/H/P、事务和 Gaussian oracle，不改 C、生产配置、AL、sigma、故障域或风险金额。上面的固定 native 表修正了误填的 history/eta 两列；新回归确认其值，原计时表保留。

**结果：共享 episode 表示有效降低冻结 raw PL；raw 仍未获得统计资格。** U0/U1 各只运行31epoch前缀，定向分解15/31epoch；两个统计审计限于 U0/31、U1/31。诊断比较始终使用同一输入、旧策略实际提交轨迹及同一冻结预测，不重放另一条状态轨迹。每个快照输出 KEEP+8删除，共36候选；历史区间共12,663条。所有31epoch新旧选择一致，36候选保留原检测阈值和接受结论。另运行新研究入口 U1/19：19次唯一 commit，15epoch实际排除source0/measurement1501、保留7条，之后4epoch H仍BOUNDED，C调用0，protected输出0。

| scene/epoch | 选择 | H解释数 | eta_H | 原 HPL/VPL(m) | 联合 HPL/VPL(m) |
|---|---:|---:|---:|---|---|
| U0/15 | KEEP | 224 | 25.9225 | 6.51054 / 7.62651 | 4.79304 / 5.49138 |
| U0/31 | KEEP | 480 | 18.6061 | 4.99895 / 5.55910 | 3.85030 / 4.22948 |
| U1/15 | 删除0 | 224 | 25.9202 | 6.92490 / 7.65219 | 5.01455 / 5.49114 |
| U1/31 | KEEP | 479 | 18.6082 | 5.00356 / 5.55948 | 3.85271 / 4.22980 |

共同 q=70.2045，声明50epoch球维度4465，诊断 epsilon=1e-6；raw risk始终UNKNOWN、qualified=0。U1/15由A报警触发9候选，B KEEP未报警；其余三个快照A/B均未报警。AL仍2/3m，表中所有新旧结果均超AL。

旧 PL 每坐标分解为 `nominal + slope*sqrt(threshold) + slope*q + slope*eta_H + history_output + committed_center`；新 PL 为 `nominal + joint_fault_support + committed_center`。以下为实际提交候选的V坐标，单位m；HPL由完整x/y分量合成，不能用各项HPL直接相加。

| scene/epoch | nominal | 阈值项 | noise项 | eta项 | history_output | center | joint_fault |
|---|---:|---:|---:|---:|---:|---:|---:|
| U0/15 | 3.52457 | 1.30684 | .954427 | .352415 | 1.48662 | .00163151 | 1.96517 |
| U0/31 | 2.90939 | .816081 | .645109 | .170972 | 1.01376 | .00378878 | 1.31630 |
| U1/15 | 3.52473 | 1.31983 | .963938 | .355895 | 1.48679 | .00100760 | 1.96540 |
| U1/31 | 2.90927 | .816122 | .645127 | .170995 | 1.01377 | .00419194 | 1.31633 |

主要来源是大维度共同球乘预测/后验标准差，以及单episode被拆成独立历史与当前极值：旧VPL的nominal占46–52%，history占18–20%，current占其余约29–34%，center不到0.1%。共享表示使HPL下降23–28%、VPL下降24–28%；新VPL中nominal占64–69%。原式 `sqrt(threshold)=q+eta_H`，故current系数为 `2q+2eta_H`。这是三角不等式和独立极值的数值外包络，不是重复收取失败概率；同一Gaussian realization同时用于nominal、历史幅值拟合及innovation，不能把它们当独立噪声。联合表示仍分别最大化nominal和故障项，尚不是整个噪声—故障集合的精确联合优化。

`rawGaussianWindow`重建先去掉PIM ADC部分，再一次加入 `J Q_ADC J'`，端点source保持共享；未发现ADC covariance重复计入。4465按每factor11个IMU样本计数，50epoch实际unique501个样本对应维度4171，差294=49个共享端点×6；这是安全过计数，不是漏收风险。31epoch实际prefix维度2594，但本轮继续使用整个50epoch q，没有按访问历元缩小合同。即使仅纠正共享端点计数，q也只小幅下降，不能解释主要米级差距，且本轮未接入此调整。

### A：同一历史—当前 episode 的联合可行集合

实现见 `hybridJointEpisodeBound`：对原来的每个 source/onset/duration 解释保留同一幅值theta、历史方向D及正在持续的当前源列b；停止后的b=0。当前新onset仍覆盖全部源、duration1/3及无界幅值；没有剪去未选候选或按真实故障选择解释。

设 W白化原innovation covariance S、y为原innovation、K为原gain：

```
a = W (H D + b)
u = C (I-KH) D - C K b
I_joint = I_history ∩ {theta : ||W y - a theta|| <= q}
support_j = max_episode max_theta∈I_joint |u_j theta|
PL_j = q sqrt((C P_post C')_jj) + support_j + center_j
```

非零a时，以 `a'Wy/||a||²` 为中心、`sqrt(q²-||perp_a Wy||²)/||a||` 为半径求区间，采用向外roundoff allowance。相同共同噪声事件保证真实幅值同时满足历史区间和当前约束；输出使用组合响应u后才取union支持函数，故保留过去与当前的抵消和强化。无需独立性或新增risk。检测仍沿用原 `(q+eta_H)²`，阈值不收紧；若联合集合不一致则拒绝。危险无界literal nullspace返回UNBOUNDED_PROVEN；没有精确source-column alias见证的数值零/近零返回INDETERMINATE，不能按epsilon宣称无害。

边界快照中最大V支持解释均为anchor6/onset29/duration3：U0历史与联合幅值区间均为[-5.52680,5.45258]，支持1.31630m；U1为[-5.52689,5.45259]、支持1.31633m。幅值区间在这些最坏解释上未缩小，下降来自共享响应与避免分别选择互不相容的历史/当前极值，不能归因为削减故障幅度。15epoch对应anchor6/onset13/duration3，区间[-5.62494,5.83348]，支持约1.965m。完整CSV逐候选记录原区间、active源列、可行交集及三坐标支持；H不可能解释与零故障/当前新onset也由代码处理。

### B：完整固定策略族的低维同时Gaussian事件

固定模型另提供 `simultaneous_projections` opt-in，不改变初始/process/端点/range sigma及fault域。T=24，每epoch9种retained选择，事先计入所有历史路径而非实际访问路径：

```
M = T (4 + 16 T) 9^T = 7.42785e26
q_projection = sqrt(chi2_8 upper_quantile(epsilon/M)) = 13.2167
```

每条路径/输出至多3个标量nominal投影、1个至多8维innovation、16T个历史残差幅值投影；矩阵族在数据之前固定。白化Gaussian innovation至多8维，标量投影至多1维，均由chi8尾界支配；对全部M投影union bound后，完整自适应选择任务risk仍≤1e-6，不要求投影独立。原717维球q=30.1928；预算仍2e-6、事件金额仍1e-6。这个数值界依赖固定有限族，不是raw随机chart的资格。

独立小oracle实际枚举两epoch全部9×9=81路径、162候选，对照未退休full-prefix QR及原固定H/P/B：covariance/mean/共享fault response匹配，真实幅值在可行区间，覆盖与原检测接受均成立；old/joint/joint-low最大PL向量norm分别 .132214/.103295/.0455811m。另有强current故障实际排除source0；标量精确区间P=R=H=C=1、y=.4、q=1得到1.61421（原2.12132），以及停止episode、历史—当前完全抵消、危险/无害/近零nullspace检查。原 `check-hybrid` 20模型输出和48 native输出回归通过；这些是参数化代数/指定realization检查，非经验tail资格或seed campaign。

**raw保留原球界的具体原因：** 数据驱动方向 `a(xi)=xi/||xi||`，xi为100维标准Gaussian，使用预注册标量1e-6半径4.89164，实际投影变成||xi||，越界概率> .99。有限族也不能只为访问的1个候选收费：512个独立标量里选择最大绝对值，单个1e-6尾界的实际失败概率 `1-(1-1e-6)^512=.000511869`。两项均由oracle解析计算。raw图、Jacobian及白化矩阵随噪声连续变化，未证明可由上述固定族覆盖，因此新q只用于固定研究模型，raw仍70.2045。

### 两个raw边界快照的统计资格审查

| 核对量 | U0/31 | U1/31 |
|---|---:|---:|
| native P 与 raw frozen P 相对Frobenius差 | .0569617 | .0567531 |
| raw P 与独立full-prefix oracle相对差 | 6.52e-10 | 5.25e-10 |
| raw mean 与独立prefix差(norm) | 9.88e-12 | 6.60e-12 |
| current PIM 与 raw current covariance相对差 | 1.39923e-4 | 1.39923e-4 |
| 共享端点ADC跨factor covariance norm | 2.37440e-8 | 2.37440e-8 |
| chart相对truth旋转(rad)/位置(m) | .043416 / .096366 | .041792 / .098106 |
| native位置相对frozen chart(m) | .0002203 | .0002310 |

两者初始law_valid=0：实际初始化position确定偏置[.01,-.01,.005]m、norm .015m，prior位置sigma=.5m（归一化norm .03），并未按所用Gaussian prior独立抽样完整rotation/position/velocity/bias。实际IMU样本sigma为.02m/s²/.001rad/s；PIM配置.000707107/.0000707107具有连续时间covariance语义，不能直接与每样本sigma作同比。raw构造用实际ADC generator重建了每个样本的Jacobian与共享端点相关性，但不是backend covariance本身。native/raw约5.7%差还包含参考图、历史窗口与线性化差异，不能全部归因ADC。

当前IMU块的辅助covariance trace约.001516905，raw ADC trace仅2.85975e-7。代码核对发现未配置 `bias_integration_sigmas` 时，`biasAccOmegaInt`复用initial prior bias sigma .1/.01。**仅审计重新积分**关闭ADC并去掉这项，aux trace变为1.51758e-7，差值.001516753来自预积分bias uncertainty，约占原aux的99.99%。实际PL、风险和生产PIM均未删改此项。实际生成器bias为零且无相应bias random walk；把这类aux逐factor作为独立Gaussian源、同时又有initial bias prior和bias状态，是否对应同一个真实生成机制、是否有重复物理不确定性或跨factor相关性，现有证据不足。PIM/raw covariance很接近主要因为共享这项，不能据此证明raw ADC/noise law一致。

| 证据级别 | 本轮结论 |
|---|---|
| 已有严格证据 | 冻结线性代数层：真实retained/source；不复用当前range；ADC仅一次、共享IMU端点cross保留；raw H/P与独立未退休prefix解一致；固定有限矩阵族的完整策略风险及联合episode推导成立。 |
| 仅冻结模型成立 | Gaussian initial/aux/process假设下的S/K/P、H幅值集合及共同球支持；使用实际noisy PIM/estimated chart冻结Jacobian后成立。单坐标nominal项是该球的精确支持，不能任意缩小，但整个PL仍有三角外包络余量。 |
| 需要新理论或标定 | 初始完整15D生成law；bias/integration/process的物理source及跨factor相关性；随机线性化点和数据依赖白化/选择事件；非线性range/IMU与积分离散化余项；有效域及域外/未建模IMU故障概率、任务carry-in/跨horizon。现有快照没有这些概率上界，误差小或PL低于AL都不能补足资格。 |

**决定：值得开展聚焦的完整raw Gaussian需求书。** 本轮已定位可合法消除的episode外包络及占主导的nominal/预积分bias uncertainty，下一步应先约定真实初始/ADC/bias/process联合生成源、共享相关性、随机chart同时事件和非线性余项/域外风险，再决定如何收紧nominal。不能靠降低AL、sigma、fault scope或risk达到通过；本轮不创建新资格/证书框架，也不改变正式发布。

最少复现（CLI自动删除自身CSV；完整逐候选CSV可用最后一条直接命令查看后删除）：

```bash
python3 tools/run_epoch_architecture.py check-tightening --build
python3 tools/run_epoch_architecture.py diagnose-hybrid
python3 tools/run_epoch_architecture.py check-hybrid
# 使用上文BIN与LD_LIBRARY_PATH设置；输出只放本轮scratch
"$BIN/hybrid_tightening_checks" config/fde_next.yaml .cache/hybrid/decomposition
# 可选研究表示入口；默认hybrid仍沿用旧表示
python3 tools/run_epoch_architecture.py hybrid --scene U1 --joint-episode
```

本轮最终验证包括上述三项及直接U1/19 opt-in。没有运行新表示50epoch campaign或性能竞赛；可选CLI命令是复现入口而非本轮新增实验分母。永久保留源码/oracle与本报告，临时CSV/日志核对后清理，本地提交，不push。

Raw qualification development now lives in [HYBRID_RAW_RESULTS.md](HYBRID_RAW_RESULTS.md); legacy results above retain their original scope.
