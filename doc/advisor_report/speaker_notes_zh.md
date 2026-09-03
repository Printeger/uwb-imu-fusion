# 《Real-Time UWB–IMU Factor-Graph Fusion with Integrity Monitoring》逐页讲稿

对应文件：`main.pdf`，共 33 页。其中第 1–26 页为主讲内容，第 27–33 页为备份页。

建议主讲控制在 25–30 分钟：算法与系统部分约 10 分钟，实验设计约 4 分钟，结果与结论约 12 分钟。备份页不主动展开，只在老师追问公式、参数或验收状态时使用。

全文有三个反复出现的状态：

- `AVAILABLE`：检测通过、模型和风险条件有效，而且 HPL/VPL 没超过告警限，可以提供完整性服务。
- `ALERT`：当前 UWB batch 被检测器拒绝，不提交该批 UWB，同时不输出有限 HPL/VPL。
- `UNAVAILABLE`：模型、风险预算或几何条件不足，或者 PL 超过告警限；此时系统不能给出有效的完整性承诺。

---

## 第 1 页：标题页

### 这页要讲什么

先用一句话定义工作：这是一个实时 UWB–IMU 因子图定位系统，不仅估计位置，还要在使用 UWB 之前判断它是否可信，并给出位置误差保护界。同时比较保留全部历史和固定时间窗两种增量后端。

### 名词解释

- **UWB**：超宽带测距。标签到多个 anchor 的距离可以帮助估计三维位置。
- **IMU**：惯性测量单元，提供加速度和角速度，频率高，但单独积分会漂移。
- **Factor graph**：把待估状态看成节点，把 IMU、UWB 等约束看成连接节点的因子，然后联合优化。
- **Integrity monitoring**：完整性监测。它回答的不是“估计值是多少”，而是“这个估计现在能不能信、风险是否受控”。

### 建议讲稿

“今天汇报的是实时 UWB–IMU 因子图融合和完整性监测。核心有两部分：第一，比较 full-history 和 fixed-lag 两种增量后端；第二，在单历元和持续故障下检查 detector、保护级以及故障后的服务撤销行为。页脚的 Git 和 protocol hash 用于说明结果对应哪一版代码和实验协议。”

---

## 第 2 页：Problem statement — accuracy is not enough

### 这页要讲什么

强调“定位准”不等于“定位可信”。系统既要输出状态，也要在坏测量进入图之前进行检测。

### 名词解释

- **Pose**：位置加姿态。
- **Inertial bias**：IMU 的零偏。即使设备不动，读数也可能不是零，而且会随时间慢慢变化。
- **Accuracy**：估计值离真值有多近。
- **Consistency**：系统给出的协方差是否和实际误差水平相符。
- **Integrity**：系统能否在有危险误差时及时告警，以及给出的误差上界是否可靠。
- **Commit / reject**：commit 是把当前 UWB batch 加入因子图；reject 是整批不加入。当前实现不是只剔除某一个 anchor 的 FDE。

### 建议讲稿

“导航目标是用 200 Hz IMU 和 20 Hz 分组 UWB，实时估计位姿、速度和 IMU 零偏。完整性目标更严格：当前这组 UWB 要先经过统计检测，统计量 (T) 不超过门限 \(\gamma\) 才能提交。超过门限就整批拒绝。因此我们关注三个不同问题：结果准不准、协方差诚不诚实、系统能不能在危险时拒绝服务。”

---

## 第 3 页：Current development status

### 这页要讲什么

给老师一个诚实的项目边界：哪些已经实现，哪些有组件级证据，哪些还没有闭环。

### 名词解释

- **Incremental iSAM2**：新测量到来时只更新受影响的部分，而不是每次从头优化整张图。
- **Fixed-lag marginalization**：只保留最近一段时间的显式状态，把更老状态的影响压缩成边界先验。
- **Monte Carlo（MC）**：用大量随机重复实验检查统计性质。
- **FDE**：Fault Detection and Exclusion，故障检测与剔除。除了发现异常，还要定位坏 anchor，并用剩余测量重新求解。当前没有实现。
- **Rare-event certification**：针对极低危险概率做统计认证，需要比普通仿真大得多的样本量或重要性采样。

### 建议讲稿

“左边是已经写进系统的能力，包括增量估计、固定窗边缘化、detect-before-commit 和当前 batch 的单 anchor 故障保护级。中间是已有组件验证。右边是没有关闭的部分，特别是实测数据、持续故障保护级、FDE、多故障和稀有事件认证。最重要的是，Week-4 的冻结验收仍然是 INVALID。本次导师汇报实验是独立证据，不会反向把旧 gate 改成 PASS。”

---

## 第 4 页：End-to-end architecture

### 这页要讲什么

从传感器输入一直讲到 commit/reject，说明实时线程和完整性判断在什么位置。

### 名词解释

- **Ordered events**：把不同线程、不同频率到来的 IMU/UWB 按时间顺序整理后再处理。
- **Preintegration**：把两个 UWB 历元之间的大量 IMU 样本压缩成一个因子。
- **Callback enqueue-only**：ROS 回调只负责把数据放进队列，不直接修改估计器状态。
- **Single owner**：只有一个有序 worker 能修改状态和写日志，避免并发导致结果顺序不确定。

### 建议讲稿

“数据先进入有序事件队列，IMU 在相邻 UWB 历元之间做预积分，然后由 iSAM2 或 fixed-lag 后端进行预测。完整性模块在 UWB 提交前读取快照、计算 detector 和 PL，最后决定 commit 还是 reject。这里故意让回调只入队，由一个 worker 串行修改状态，这样同一输入可以得到确定、可复现的结果。”

---

## 第 5 页：State and factor graph

### 这页要讲什么

解释每个历元估计哪些量，以及 IMU 因子和 UWB 因子怎样连接状态。

### 名词解释

- \(R_k\)：第 \(k\) 个历元的姿态。
- \(p_k\)、\(v_k\)：位置和速度。
- \(b_k^a\)、\(b_k^g\)：加速度计和陀螺仪零偏。
- **Prior**：初始先验，告诉优化器起始状态大概在哪里，以及有多大不确定性。
- **Grouped UWB factor**：同一时刻多个 anchor 的测距作为一组处理，可以保留测距间的协方差关系。
- **Bayes tree / clique**：iSAM2 内部组织稀疏问题的数据结构。可以理解为只重算受新测量影响的局部子树。

### 建议讲稿

“每个历元的状态不只是三维位置，还包括姿态、速度和两类 IMU 零偏。IMU 因子把前后两个状态连起来，当前 UWB group 约束当前位姿。iSAM2 用 Bayes tree 保存求解结构，因此增量更新不需要每次重算全部历史。右图只是最小的一段：上一状态经过 IMU 因子到当前状态，再由多个 UWB range 形成当前测量因子。”

---

## 第 6 页：IMU preintegration

### 这页要讲什么

不要逐项推导公式，重点解释为什么需要预积分，以及它怎样同时传播运动和不确定性。

### 名词解释

- \(\Delta\tilde R,\Delta\tilde v,\Delta\tilde p\)：一段时间内由 IMU 累积出来的相对旋转、速度变化和位置变化。
- **Exp**：把角速度积分得到旋转的李群指数映射。
- **Bias correction**：预积分时用了某个零偏估计；零偏稍微更新时，用雅可比做一阶修正，不必重新积分所有 IMU。
- **Jacobian \(J\)**：描述输入发生小变化时，输出大约怎样变化。
- **Random walk**：零偏不是固定常数，而是随时间缓慢随机变化的模型。

### 建议讲稿

“UWB 是 20 Hz，但 IMU 是 200 Hz，所以两个 UWB 历元之间有十个 IMU 样本。预积分把这些样本压缩成一个从上一状态到当前状态的运动约束。三行公式分别是旋转、速度和位置的累计。下方说明零偏小幅变化时可以用一阶雅可比修正，同时按配置的噪声和零偏随机游走传播协方差。”

---

## 第 7 页：UWB measurement model and provenance

### 这页要讲什么

说明 UWB 残差怎样计算，以及为什么完整性算法必须保留每一行测量的来源。

### 名词解释

- \(p_k+R_k\ell\)：标签天线的世界坐标；\(\ell\) 是 IMU 到 UWB 天线的杆臂。
- \(a_i\)：第 \(i\) 个 anchor 的世界坐标。
- \(\rho_{k,i}\)：实际测得的距离。
- **Residual**：模型预测距离减实际测量距离。
- **Provenance**：测量溯源信息，包括 anchor ID、地图版本、measurement ID 和 factor ID。
- **Whitening**：按测量协方差对残差和雅可比归一化，使正常噪声在变换后近似单位协方差。
- **Cholesky / SPD**：Cholesky 是正定矩阵分解；SPD 指对称正定。协方差不是 SPD 时说明模型无效，系统应 fail closed。

### 建议讲稿

“UWB 残差就是估计位置到 anchor 的预测距离减实测距离。因为保护级要对每个物理 anchor 建立故障假设，不能只保留匿名矩阵行，所以整个流程保留 anchor 和 factor 的身份信息。对于相关测距噪声，我们用 Cholesky 分解做白化。如果协方差不合法、白化检查失败，系统不会勉强输出一个数字，而是返回 UNAVAILABLE。”

---

## 第 8 页：Two incremental backend modes

### 这页要讲什么

这是后端对比的核心页。说明 fixed-lag 不是简单删除旧数据，而是把旧信息压缩进 Schur 边界先验。

### 名词解释

- **Full-history**：从第 0 个历元开始，所有状态和因子都显式保留。
- **Fixed-lag**：这里只显式保留最近 200 个历元，即约 10 秒。
- **Marginalization**：消掉不再需要显式保留的旧变量，同时保留它们对剩余变量的统计影响。
- **Schur boundary prior**：边缘化后落在窗口边界状态上的压缩先验。可以把它理解为“旧历史留下的一份信息摘要”。
- **Relinearization**：当前估计变化后，重新计算非线性因子的局部线性近似。
- **Dense boundary factor**：压缩后的边界先验可能比原来的局部因子更稠密，这是 fixed-lag 的额外代价。

### 建议讲稿

“蓝色 full-history 把所有历史都留在图中，适合作为短序列参考，但图和内存会持续增长。橙色 fixed-lag 只显式保留最近 200 个历元；更老的状态不是直接扔掉，而是通过 Schur 补压缩成边界先验。因此当前状态仍能继承旧数据的信息。代价是每次窗口滑动要做边缘化并更新较稠密的边界因子。两者都是增量后端，区别主要是怎样表示历史，而不是当前 UWB 模型不同。”

---

## 第 9 页：Detect before commit

### 这页要讲什么

解释完整性流程最重要的因果顺序：先用 IMU 和历史形成先验，再检测尚未提交的当前 UWB。

### 名词解释

- \(P^-\)：使用当前 UWB 之前的状态协方差，也叫 pre-measurement prior covariance。
- **Read-only snapshot**：完整性模块只读取一个固定版本的线性化快照，不在检测过程中修改图。
- **Invariant**：实现必须始终满足的条件，而不是“通常成立”的经验规则。
- **Ordering/version audit**：检查协方差、雅可比、残差和当前 batch 确实属于同一个图版本和线性化点。

### 建议讲稿

“流程是 predict、提取 UWB 之前的先验、做完整性检测，最后才决定提交。关键不变量是当前待测 UWB 绝不能已经进入 \(P^-\)，否则就相当于让被告参与制定自己的判定标准，会造成信息泄漏。通过时提交整个 batch；失败时拒绝整个当前 UWB batch，保留 IMU 预测。当前没有识别坏 anchor 后用剩余 anchor 重算的 FDE。”

---

## 第 10 页：Conditional current-group innovation detector

### 这页要讲什么

把公式翻译成“残差相对其预期波动有多异常”，并说明门限怎样控制误警率。

### 名词解释

- \(e_w\)：白化后的当前 UWB 残差。
- \(H_w\)：白化后的测量雅可比，描述状态误差怎样投影到测距残差。
- \(S_w\)：innovation covariance，当前残差在无故障条件下应有的协方差。
- \(T\)：把残差按 \(S_w\) 归一化后的总异常程度。
- \(\chi^2_m\)：自由度为 \(m\) 的卡方分布。这里通常有 8 条 range，所以 \(m=8\)。
- \(P_{FA}\)：False Alarm Probability，无故障时被误判为故障的目标概率。
- **Post-fit residual**：测量已经参与拟合后的残差，容易因为估计器吸收故障而变小，所以这里只作为诊断。

### 建议讲稿

“可以把 \(T\) 理解为当前八条测距整体离正常范围有多远，而且已经考虑了状态本身的不确定性。无故障且模型正确时，它近似服从卡方分布。门限 \(\gamma\) 由目标误警率决定：门限越高越不容易误报，但也更容易漏掉小故障。主 detector 用的是 conditional innovation，因为它的先验明确排除了当前 UWB；全图残差和 post-fit 残差只做辅助诊断，需要单独校准。”

---

## 第 11 页：Protection level and declared scope

### 这页要讲什么

说明保护级不是普通的“三倍标准差”，而是名义不确定性加可漏检故障带来的最坏偏移，并明确适用范围。

### 名词解释

- \(P^+\)：当前 UWB 通过检测并用于更新后的协方差。
- **Nominal component**：没有故障时，由协方差和尾部概率得到的误差界。
- **Fault component**：对声明的故障假设，考虑检测器仍可能漏检时造成的最大位置偏移。
- **HPL / VPL**：Horizontal/Vertical Protection Level，水平和垂直保护级。
- **HAL / VAL**：水平和垂直告警限。PL 超过告警限，即使数字有限，也不能提供 AVAILABLE 服务。
- **Noncentral chi-square**：故障使卡方统计量偏离中心分布后的模型，用于连接故障大小、漏检概率和位置影响。

### 建议讲稿

“保护级由两部分组成：正常噪声下的位置不确定性，以及单 anchor 故障在漏检边界上可能造成的最坏偏移。三轴保护级再合成为 HPL 和 VPL。要输出 AVAILABLE，不仅 detector 要通过，还要求 anchor 几何、风险预算和数值条件都有效，并且 HPL/VPL 不超过告警限。这里的形式范围只覆盖当前 group 的单 anchor 偏置，不覆盖已经进入历史的持续故障。”

---

## 第 12 页：Research questions

### 这页要讲什么

把后面的图表组织成四个明确问题，让老师知道每张图在回答什么。

### 名词解释

- **Tail latency**：不是平均耗时，而是较慢的尾部，例如 P95、P99。
- **PFA**：实际无故障样本中误报警的比例。
- **Power**：有故障样本中成功检测出来的比例。
- **TTD**：Time To Detect，从故障出现到首次越过门限的时间。

### 建议讲稿

“实验围绕四个问题：第一，固定 10 秒窗口是否保持当前状态精度、协方差和 PL；第二，它是否真的把图规模和资源占用限制住；第三，两种后端的误警、检测能力和检测延迟是否相当；第四，哪些结论只是仿真观察，哪些已经达到组件验证或形式声明。后面每一组结果都会对应这些问题。”

---

## 第 13 页：Simulation setup

### 这页要讲什么

交代数据怎样生成、anchor 怎样布置，以及为什么两后端比较是配对的。

### 名词解释

- **Figure-eight trajectory**：8 字轨迹，同时包含左右转向，比单直线更能激励二维运动。
- **Seed**：随机数种子。固定 seed 可以重复得到完全相同的噪声和故障实现。
- **Paired realization**：两种后端使用同一份 IMU/UWB 样本，而不是分别随机生成。
- **Input digest**：对输入内容计算的摘要；摘要一致说明比较双方真正看到了相同数据。
- **Umeyama alignment**：常见的轨迹刚体或相似变换对齐方法。本实验不做对齐，避免用后处理掩盖世界坐标误差。

### 建议讲稿

“仿真包含直线、圆和 8 字三种轨迹，八个 anchor 放在三维空间的立方体角点。每个 seed 先生成一份完整的 IMU/UWB 序列，然后原样回放给两个后端，输入 digest 必须一致。误差直接在共同世界坐标系中计算，不做 Umeyama 对齐。需要强调，这仍然是高保真确定性仿真，不是实测数据验证。”

---

## 第 14 页：Focused-PL experiment matrix

### 这页要讲什么

解释本次资源集中在哪里：PL 故障实验跑完整矩阵，基础精度、性能和 ROC 只跑 smoke 对比。

### 名词解释

- **Suite**：一类实验集合。
- **Smoke experiment**：缩小规模的链路检查，能说明代码和分析流程跑通，但不能替代完整统计样本。
- **Focused-PL**：把计算预算集中到 PL 相关问题的 profile。
- **Cell**：由 backend、轨迹、anchor、故障幅值共同定义的一格实验条件。
- **Resume/checkpoint**：每个序列完成后记录 checkpoint，中断后可继续而不会重复行。

### 建议讲稿

“这次不是所有实验都跑到最大规模。名义精度、窗口消融、ROC 和性能仍是 smoke；完整规模用于两个 PL 问题。单历元故障使用 640 个独立 seed，对应 8 anchors、8 个故障幅值、每格 10 seeds，并在三条轨迹和两个后端上配对回放。持续 1 米故障使用 8 anchors、每个 10 seeds、三条轨迹。故障从 epoch 300 开始；文件只保存分析需要的区间，但估计器从 epoch 0 完整执行。”

---

## 第 15 页：Representative trajectories and anchor layout

### 这页要讲什么

展示独立的 1000-epoch 8 字轨迹，并说明这张图已经修复了过去跨 trajectory 连线的问题。

### 名词解释

- **GT**：Ground Truth，仿真真值。
- **Top view**：俯视的 x–y 平面，更容易看清 8 字的两个叶片。
- **Anchor layout inset**：右上角三维小图，说明 anchor 在高度方向也有分布。

### 建议讲稿

“这张代表轨迹只选 figure-eight，按 backend 和 epoch 排序，绝不把直线、圆和 8 字的数据串起来。1000 个历元约 50 秒，覆盖一个完整双叶。黑色虚线是真值，蓝色是 full-history，橙色是 lag 200；三条线基本重合。右上角给出八个 anchor 的三维布局，说明垂直方向也有几何约束。”

---

## 第 16 页：Nominal position accuracy

### 这页要讲什么

说明 smoke 数据里两后端当前状态几乎数值一致，并解释为什么不是“删掉历史却没有损失”的矛盾。

### 名词解释

- **ATE RMSE**：所有位置误差平方平均后开方，是总体位置精度指标。
- **P99**：99% 样本不超过的误差值，用来观察尾部。
- **Paired block**：同一 seed 和轨迹组成一个配对统计单元。
- **CI**：Confidence Interval，置信区间。本页比较 lag 相对 full 的 ATE 变化。

### 建议讲稿

“在当前 smoke 样本中，两种模式的 ATE RMSE 都是 0.088 米，P99 都是 0.184 米；配对差异接近机器精度。原因是当前图主要由相邻 IMU 因子和当前 UWB 因子组成，旧状态边缘化后，其信息通过 Schur 边界先验传给保留窗口，所以当前状态可以数值一致。这里的 PASS 只针对这 6 个配对 block 的 smoke 结果，不能扩展成所有数据和所有图结构都无损。”

---

## 第 17 页：Consistency and integrity containment

### 这页要讲什么

区分位置精度、协方差一致性和 PL 包络，同时诚实呈现姿态不可观的负面结果。

### 名词解释

- **NEES**：Normalized Estimation Error Squared，把实际状态误差按估计协方差归一化。三维位置理想均值大约接近 3。
- **95% covariance coverage**：实际误差有多少比例落在协方差给出的 95% 椭球内。
- **Containment**：实际误差是否小于相应保护级。
- **Weakly observable yaw**：仅靠当前运动和 range 很难约束绕竖直轴的航向角，因此 yaw 可能漂。

### 建议讲稿

“两种后端的位置 NEES 都约为 2.842，接近三维位置的理想量级；协方差覆盖和 HPL 包络也相同。但这里有一个必须保留的负面结果：姿态 RMSE 达到 1.71 rad，约 98 度。原因是当前仿真姿态恒定，而且 range-only 对 yaw 约束很弱。所以本页只能说明位置部分表现一致，不能把位置 ATE 好直接说成完整导航状态都准确。”

---

## 第 18 页：Window-length ablation

### 这页要讲什么

展示窗口长度在精度、延迟和图规模之间的关系，但避免从单 seed smoke 得出过强结论。

### 名词解释

- **Ablation**：只改变一个关键设计参数，观察结果怎样变化。
- **History length**：显式保留的历元数；0 表示 full-history。
- **Active factors**：当前图中仍参与求解的因子数量；图中点越大，因子越多。

### 建议讲稿

“这里比较 lag 20、50、100、200 和 full-history。当前 smoke 中，各窗口的 ATE 都约 0.0895 米，差别很小；因子数则从 lag 20 的 40 个，到 lag 200 的 401 个，再到 full 的 503 个。P99 延迟没有随窗口长度单调下降，因为边缘化本身有成本，比如 lag 100 在这次运行中约 11.2 ms。由于这里只有一个 seed，这页主要展示分析方法和趋势，不能据此选择最终最优窗口。”

---

## 第 19 页：Latency comparison

### 这页要讲什么

强调“资源有界”和“短序列一定更快”是两回事。

### 名词解释

- **Empirical CDF**：横轴是耗时，纵轴是耗时不超过该值的样本比例。
- **P99 latency**：99% 的历元耗时不超过该值。
- **Platform gate**：对目标硬件必须满足的正式性能门槛。

### 建议讲稿

“左图看耗时随 epoch 的变化，右图看耗时分布。500-epoch smoke 中，full-history 平均约 1.33 ms、P99 约 1.87 ms；lag 200 平均约 4.62 ms、P99 约 14.77 ms，反而更慢。这不是 fixed-lag 设计失败，而是序列还短，full 图没有大到拖慢求解，而 fixed-lag 从 epoch 200 后已经持续支付边缘化和边界先验提取成本。正确结论是 fixed-lag 保证规模有界，不是保证任何长度下都更快。”

---

## 第 20 页：Non-overlapping timing breakdown

### 这页要讲什么

解释耗时差异具体来自哪里，并说明计时分类没有重复相加。

### 名词解释

- **Estimator time**：IMU 状态更新和 UWB commit 的时间。
- **Prior extraction**：提取当前状态先验协方差和完整性快照的时间。
- **Integrity/PL time**：计算 detector、故障灵敏度和保护级的时间。
- **Non-overlapping**：各类别互不重叠，总和可以解释整体 core wall time。

### 建议讲稿

“这张堆叠图解释上一页为什么 lag 更慢。full 的 estimator、prior extraction、integrity 平均约是 0.44、0.12、0.75 ms；lag 200 对应约 2.89、0.95、0.77 ms。完整性计算本身几乎相同，额外开销主要来自 fixed-lag 的估计器更新和边界先验提取。这也支持前面的解释：当前瓶颈不是 detector，而是窗口滑动时的边缘化。”

---

## 第 21 页：Resource growth and marginalization

### 这页要讲什么

用 active values 和 active factors 证明 fixed-lag 的核心价值是长期资源有界。

### 名词解释

- **Active values**：当前图里显式保存的状态变量数量。
- **Active factors**：当前图里显式保存的约束数量。
- **Bounded graph**：运行时间继续增长时，图规模不再随时间线性增长，而是在窗口附近稳定。

### 建议讲稿

“full-history 的 active values 和 factors 会随 epoch 持续上升。lag 200 在第一次边缘化后趋于稳定：当前名义实验中最大 active factors 是 401，而 full 已到 503；在 500-epoch 性能序列中，lag 仍是 401，full 已到 1003。也就是说 fixed-lag 的优势会在更长的持续运行中体现，代价是刚才看到的边缘化开销。”

---

## 第 22 页：Detector ROC and AUC

### 这页要讲什么

比较两后端的 detector 行为，解释 ROC、AUC、PFA 校准和负面 gate 结果。

### 名词解释

- **ROC**：改变检测门限后，把误警率和检测率画在一起的曲线。
- **AUC**：ROC 曲线下面积；0.5 接近随机，1 表示完全可分。
- **Calibration set**：专门用来选门限的数据，不能再拿来评价同一门限。
- **Holm correction**：做多项统计检验时控制整体误报概率的方法。
- **Non-inferiority（NI）**：不是要求 lag 更好，而是检查它是否没有比 full 差超过预设容忍量。

### 建议讲稿

“两种后端的 conditional AUC 都是 0.846，ROC 曲线基本重合。在目标 PFA \(10^{-3}\) 处，当前 smoke 的经验 PFA 是 0.002，检测 power 约 0.491。fixed-lag 相对 full 的 power 非劣效检查通过，但 Holm PFA 检查失败，所以不能说误警指标已闭环。另一个边界是 \(10^{-5}\) 只是一条理论配置；几千个无故障样本远远不够认证这么低的概率。”

---

## 第 23 页：Single-epoch fault — empirical PL challenge

### 这页要讲什么

这是单历元故障的核心结果页。先教听众读三幅图，再报告分母、包络和 HMI。

### 名词解释

- \(T/\gamma\)：检测统计量除以门限。大于 1 表示拒绝，小于等于 1 表示 detector 通过。
- **Finite-PL denominator**：只有 detector 通过且实际输出有限 HPL/VPL 的样本，才进入 containment 分母。
- **Observed HMI**：系统显示 AVAILABLE，但实际水平误差超过 HPL或垂直误差超过 VPL。
- **Near boundary**：\(0.8\le T/\gamma\le1.2\)，即靠近检出/漏检分界的样本。
- **Clopper–Pearson upper bound**：二项分布的精确置信上界。即使观察到 0 次 HMI，也不能把真实风险直接说成 0。

### 建议讲稿

“左图横轴是单 anchor 故障幅值，纵轴是 \(T/\gamma\)，黑色虚线 1 是检测边界。中图比较实际水平误差和 HPL，右图比较垂直误差和 VPL；点在对角线下方表示被保护级包住。总共有 3840 个故障时刻样本，其中 2252 个 detector 通过并给出有限 PL，这 2252 个全部包络。observed HMI 是 0，但按全部 3840 次挑战计算，单侧 95% 精确上界仍是 0.078%，所以不能说风险为零。近边界有 584 个样本，其中有限 PL 分母为 360，也没有 observed HMI。这个实验是经验挑战，不是对 \(4\times10^{-5}\) 的稀有事件认证。”

---

## 第 24 页：Persistent 1 m fault — conservative service withdrawal

### 这页要讲什么

说明持续故障下系统采用保守策略：检测到就撤销当前 UWB 和有限 PL；同时如实报告少量漏检和严重 IMU 漂移。

### 名词解释

- **Persistent fault**：从 epoch 300 开始，同一个 anchor 的测距持续增加 1 米，直到序列结束。
- **Availability band**：绿色 AVAILABLE、红色 ALERT、灰色 UNAVAILABLE 的时间色带。
- **IMU-only drift**：UWB 连续被拒绝后，只靠 IMU 推算导致的累计位置漂移。
- **Service withdrawal**：系统明确说“现在不给完整性保证”，而不是把空白 PL 当成 0。

### 建议讲稿

“时间线依次画 detector 统计量和门限、commit/reject 与 availability 色带、水平误差和 HPL、垂直误差和 VPL。红色 ALERT 区域的 PL 空白是有意撤销，不是漏画，更不是 PL 等于零。完整矩阵有 8 个 anchors、每个 10 seeds、三条轨迹和两种后端。故障历元的拒绝率是 98.94%，每条序列至少检测到一次，最长连续拒绝 300 个历元；ALERT 占 70.72%，UNAVAILABLE 占 28.22%，拒绝时输出有限 PL 的违规数为 0。但最大 IMU-only 漂移达到 2036.528 米，而且仍有约 1.06% 故障历元通过并提交。因此这里只能说‘检测后保守撤销服务’，不能说持续故障下已经有认证 PL，也不能说已经实现故障 anchor 的 FDE。”

---

## 第 25 页：Validation dashboard — frozen evidence, not upgraded

### 这页要讲什么

把新实验和冻结的 Week-4 正式验收分开，避免导师误以为新图自动关闭了旧 gate。

### 名词解释

- **Frozen evidence**：已经冻结、带 checksum 的历史验收产物。
- **Gate**：达到预先规定条件才可标记 PASS 的验收项。
- **Checksum re-verification**：重新计算文件 SHA-256，确认复用的证据没有被修改。
- **INVALID**：证据缺失、协议不完整或产物不满足结构要求，不等同于某个科学指标单纯 FAIL。

### 建议讲稿

“绿色部分是冻结 Week-4 里已经 PASS 的组件：非中心卡方计算、IMU Monte Carlo、Method A/B shadow、ROS topic 场景和 fixed-lag 性能。Snapshot、ROC 和 History 当时没有完成，所以仍是 INVALID。我们重新验证了旧产物 checksum，但没有修改旧 acceptance summary。本次 focused-PL 数据用于导师汇报和发现问题，不升级 Week-4 gate。”

---

## 第 26 页：Conclusions and next steps

### 这页要讲什么

用“现在能支持什么、还不能支持什么、下一步是什么”结束主讲。

### 名词解释

- **Paired input identity**：两个后端的输入 digest 完全相同，差异只能来自后端而不是随机数据。
- **Outage/IMU-drift integrity model**：在 UWB 不可用期间，显式给出随 IMU 漂移增长的风险或保护界模型。
- **Persistent-fault provenance**：边界先验要能说明历史中是否可能含有故障信息。
- **Importance sampling**：为了估计极低概率事件，主动多采样危险区域并用权重还原真实概率的方法。

### 建议讲稿

“目前可以支持的结论是：两种后端都是增量 iSAM2；fixed-lag 用 Schur 边界先验压缩历史；detect-before-commit 和配对输入已经落实；完整单历元挑战中所有 observed HMI 都被显式统计；被拒绝的 batch 不输出有限 PL。不能支持的结论包括实测泛化、持续故障认证 PL、FDE、多故障和稀有事件概率。下一步优先加入 outage/IMU 漂移模型或 FDE，并让 fixed-lag 边界先验保留故障来源，再用实测数据和重要性采样验证。”

---

# 备份页讲稿

## 第 27 页：Backup — conditional innovation derivation

### 什么时候使用

当老师追问“为什么统计量服从卡方分布”时使用，不建议主讲主动展开全部推导。

### 名词解释

- \(\delta x\)：当前先验状态误差，假设服从零均值高斯分布，协方差为 \(P^-\)。
- \(\nu_w\)：白化后的测量噪声，协方差为单位阵。
- \(z\)：把 innovation 再按 \(S_w\) 标准化后的向量。
- **Noncentrality \(\eta_i\)**：有故障时统计量偏离中心卡方分布的程度。

### 建议讲稿

“无故障时，白化残差由状态不确定性和单位测量噪声组成，因此协方差是 \(S_w\)。再用 \(S_w^{-1/2}\) 标准化后，各维近似独立标准高斯，平方和自然服从自由度为 \(m\) 的卡方分布。若第 \(i\) 个 anchor 有偏置 \(f_i\)，统计量变成非中心卡方，非中心参数 \(\eta_i\) 描述这个故障在 detector 空间里有多显著。”

---

## 第 28 页：Backup — PL ingredients and risk allocation

### 什么时候使用

当老师追问保护级如何把漏检风险转换成位置误差界时使用。

### 名词解释

- \(K\)：类似卡尔曼增益，描述测量 residual 会怎样修正状态。
- \(g_{i,j}\)：第 \(i\) 个 anchor 故障对位置第 \(j\) 个方向的 failure slope。
- \(P(H_i)\)：第 \(i\) 类故障发生的先验概率。
- \(P_{MD,i}\)：该故障发生时 detector 漏检的条件概率。
- \(P_{NM}\)：没有被当前故障模型覆盖的风险预算。
- \(P(HMI)\)：危险误导信息概率。

### 建议讲稿

“保护级先用增益和后验协方差描述正常更新，再计算每个 anchor 故障从测量空间传播到位置空间的斜率。总 HMI 风险由未建模风险和各故障模式的发生概率乘漏检概率组成。实现还会检查 anchor map 是否完整、所有假设是否可监测、灵敏度是否有限、风险是否超预算以及版本是否一致。任何前提失败都返回 UNAVAILABLE，而不是输出看起来正常的有限 PL。”

---

## 第 29 页：Backup — resolved research parameters

### 什么时候使用

当老师询问采样频率、噪声、风险指标或 fixed-lag 参数时使用。

### 名词解释

- \(\sigma_a,\sigma_\omega\)：加速度计和陀螺仪白噪声强度。
- \(\sigma_{b_a},\sigma_{b_g}\)：两类零偏随机游走强度。
- **HAL / VAL**：水平/垂直告警限，分别为 2 米和 3 米。
- **Relinearization skip**：iSAM2 隔多少次 update 才按配置触发常规重线性化检查。

### 建议讲稿

“这里列的是实际解析后的研究配置：IMU 200 Hz、UWB 20 Hz、range sigma 0.1 米、重力 9.80665。形式配置的 PFA 是 \(10^{-5}\)，总 HMI 要求是 \(4\times10^{-5}\)，单 anchor 的目标漏检概率是 \(10^{-3}\)。fixed-lag 是 200 历元，也就是约 10 秒。需要说明的是，这些物理和风险参数来自统一 YAML；实验 protocol 只规定矩阵、seed、bootstrap 和展示标准，避免多处配置不一致。”

---

## 第 30 页：Backup — statistical protocol

### 什么时候使用

当老师质疑统计是否跨 seed 混算、是否挑选结果或怎样处理缺失样本时使用。

### 名词解释

- **Bootstrap unit**：重采样时不可拆分的基本块，这里是完整的 `(seed, trajectory)` 序列。
- **FWER**：Family-Wise Error Rate，多项检验中至少出现一次假阳性的总体概率。
- **Censored TTD**：序列结束仍未检测到时，不虚构检测时间，而是标记为右删失。
- **FAIL 与 INVALID**：FAIL 表示数据完整但科学标准没通过；INVALID 表示数据或协议本身不满足分析条件。

### 建议讲稿

“时间戳严格匹配，不做空间或时间对齐。Bootstrap 以整条 seed/trajectory 序列为单位，避免把相关历元假装成独立样本。校准、名义评估和故障评估 seed 互斥。多个 PFA 检验用 Holm 控制整体 5% 误报。TTD 只在单条序列内计算；没检出的序列保持删失。结果不好就报 FAIL，文件缺失、重复、非有限或不完整则报 INVALID。”

---

## 第 31 页：Backup — reused component evidence

### 什么时候使用

当老师询问哪些结果来自本次实验、哪些来自历史 Week-4 产物时使用。

### 名词解释

- **Noncentral MC**：检查非中心卡方边界和统计实现的 Monte Carlo。
- **Method A/B shadow**：两种独立实现或路径在同一候选数据上并行计算，用于检查一致性。
- **ROS topic scenarios**：通过实际 ROS 消息链路覆盖的确定性场景。
- **Frozen result**：保留原 Git、协议和 checksum 的历史结果，本次只读复用。

### 建议讲稿

“本页列的是旧 Week-4 已经 PASS 的组件证据，不是本次重新包装出来的结果。非中心计算、IMU MC、Method A/B shadow、八个 ROS topic 场景和三次 12000-epoch fixed-lag 性能测试都有冻结产物。我们核验了它们的 checksum，但这些组件 PASS 不能代替当时没完成的 Snapshot、ROC 和 History gate。”

---

## 第 32 页：Backup — Week-4 acceptance boundary

### 什么时候使用

当老师问“既然新实验已经跑了，为什么总状态仍是 INVALID”时使用。

### 名词解释

- **Acceptance summary**：正式验收状态的权威汇总文件。
- **Success token**：只有所有规定 gate 都满足时才生成的成功标记。
- `week4_acceptance_modified=false`：manifest 明确记录本次流程没有修改冻结验收。

### 建议讲稿

“Week-4 的规则是预先冻结的。左边五项当时 PASS，右边 Snapshot、ROC 和 History 不完整，因此整体必须保持 INVALID，也没有 success token。本次 focused-PL 虽然产生了新的经验数据，但它属于另一个 artifact namespace，不能事后改变旧协议或把旧验收补成 PASS。这是为了防止看到结果后移动门槛。”

---

## 第 33 页：Backup — selected references

### 什么时候使用

答辩最后或老师询问算法依据时简要说明，不需要逐条朗读。

### 名词解释

- **Forster et al.**：流形上 IMU 预积分的经典工作，是第 6 页公式的主要理论来源。
- **Kaess et al. / iSAM2**：增量平滑和 Bayes tree 的经典工作。
- **GTSAM fixed-lag smoother**：当前 fixed-lag 实现和边缘化行为所依赖的库与文档。
- **Brown and Hwang**：状态估计一致性、统计检测与完整性基础参考。

### 建议讲稿

“预积分部分主要依据 Forster 的工作，增量图优化依据 iSAM2，fixed-lag 的工程实现基于 GTSAM，对一致性和完整性概念则参考经典随机信号与卡尔曼滤波教材。最后再强调一次：汇报中的数值都由同一个 `summary.json` 生成 LaTeX 宏，没有手工从图上抄数，因此 PDF、summary 和 raw 数据可以相互追溯。”

---

## 建议的结束语

“这次工作的价值不只是得到几张结果图，而是把当前能力边界明确了：单历元 current-group、single-anchor 故障下，detector 与有限 PL 的经验表现可复现；持续故障下，系统能够保守撤销当前 UWB 和完整性服务，但代价是严重的 IMU-only 漂移，而且还没有 FDE 或持续故障认证 PL。下一阶段会优先补这两个缺口。”
