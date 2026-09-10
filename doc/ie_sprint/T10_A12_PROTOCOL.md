# T10-A12 单 checkpoint 运行前协议

登记时间 UTC: 2026-09-09T13:19:30.403122+00:00

指挥接受 `REVIEW_ACCEPTED_A11_BOUNDED_FIRST_BLOCK_DIAGNOSTIC_SCOPE`，仅 A11 限定诊断接受，不是 estimator 成功、validation 准入或外部 REVIEW.md。T10 IN_PROGRESS，A08 历史图级 FD 15/18 和 UNKNOWN 限制保留。

范围严格为 A10 P1 step seed10101 原 raw/config 与 A11 封存 call50。只恢复首次 conditional navigation graph；禁止新数据、truth 读取、其它场景/参数、chain、Stage2、gate、held-out 和 scheduler。不修改生产源码、配置、依赖或 /usr/local。新增独立 C++ 诊断程序链接现有 core/GTSAM，Python 仅处理文件和静态线性代数。

## 阶段一：恢复及静态审计（零 iterate）

只用既有 loader/initializer/GraphBuilder 和 paper Pose3 prior；initializer 仅从相同 raw 重建固定先验和 IMU preintegration。outer1 的 c=0 是原 estimator 起始状态，不是对未知 truth 的断言。按原 automatic discovery 的 MakeUwbFactor 顺序及 beta+0 恢复条件图；全部 checkpoint key/类型/坐标必须完整，无缺项初始化补洞。

硬门：共同准备初始 Values/hash 与 graph-linearization/hash 精确匹配 A11；checkpoint 123 keys、615 tangent dimensions、371 factors，类型/keys/order 逐 factor 一致。call50 目标、五类梯度、全部 factor 终态误差与 A11 比较，数值容差 abs 2e-12 + rel 2e-12（小梯度 abs 2e-10 用于累加次序舍入）。以捕获 call50 实际接受615维delta及其原 base 做 native retract，与终态 max local 差 <=1e-12；重算该方向三步长 {1e-4,1e-5,1e-6} 的图/factor FD，与封存证据按 abs 2e-9 + rel 2e-9 比较，仍保留原导数判据 abs差 <=5e-9+0.005*abs(analytic)。不得换求解方向。任何身份硬门失败则停止，保留失败，不执行 A/B。

静态输出：白化 J、rhs、列/梯度/实际 delta、factor 证据、native damping 对角数值；不求新导航方向。物理尺度明确固定 S=(1 rad,1 m,1 m/s,1 m/s²,1 rad/s)，与原驻点审计相同，J_q=J S、q无量纲。另报告仅作为解释的固定先验1σ尺度 S_prior=(0.01 rad,0.05 m,0.1 m/s,0.01 m/s²,0.01 rad/s)，由原先验而来，不更改 estimator 或判据。分别报告谱、rank/condition、最弱5个向量的五类组成与主要坐标；诊断 rank 阈值 max(1e-12,1e-10*sigma_max)，并附1e-11/1e-9相对阈值敏感性，非 gate rank 锁定。报告 X/V/B 及细分列范数。物理信息严格只含原 graph 的 J'J，damping 另表。

静态支持阻尼失衡假设的预登记必要证据：物理尺度列范数 max/min >=100；native isotropic lambda/列曲率比跨列 >=1e4；在至少一个最弱模式上 isotropic damping/物理曲率 >=0.1，且最强模式该比 <=1e-4；弱5模式上非零梯度投影可识别（绝对投影>1e-8）。全部满足且恢复硬门通过，才称支持开展一次对照；不据此宣称根因。任一未满足，明确否定本轮这项具体对照前提，A/B NOT_RUN。

每次静态程序外部上限120s、零 iterate；只做一次 checkpoint 数值采集，编译错误可修，恢复失败不得用近似图绕过。静态 SVD 处理上限60s。归档实际打开文件/proc maps/动态库内容身份、命令、exit、成本及所有失败。

## 阶段二：条件式预算（尚未授权执行门通过）

若静态支持，先另写 PHASE2_PREREGISTRATION.md（含实际证据、唯一变化和规则）。A/B 从同一恢复 graph/Values 与封存 lambda=0.10000000000000006 开始；新 optimizer 只恢复必要状态，固定 lambdaFactor 10 的 currentFactor 无历史记忆，accepted/inner counters 从零记录并说明对应 +50/+104。

A isotropic；B 仅 diagonalDamping=true；两臂各最多150 iterate、外部30秒，无retry、无第二候选。保留 V2 内部 relTol=0，外部 generic rel1e-6/abs1e-8 AND 原 navigation stationarity tol1e-6、安全因子8、原物理尺度。原maxIterations参数50保持不变（直接iterate的诊断循环上限150），SEQUENTIAL_CHOLESKY/order规则、lambda范围、接受规则及全部先验保持相同。trace verbosity TRYDELTA 与 A11 一致。

A先执行并与封存 call51..200逐次比较：目标 abs2e-10+rel2e-12；五类梯度和步长 abs2e-9+rel2e-8；lambda abs0+rel1e-14；接受/拒绝计数精确相同。若A未复现或异常/超时，B NOT_RUN，不改容差、不重试。B仅在A复现后执行；按原 generic AND stationarity 报告是否收敛。若均未收敛，只有B终态E不高于A（上面E数值容差）且最大尺度梯度 <=0.5*A，才标记本checkpoint固定预算局部改善；其余为无此证据/负结果。保留末20次梯度范围、中位数、接受/拒绝、lambda和耗时，不据 walltime差异宣称速度普遍改进。不得扩大预算或集成策略。
