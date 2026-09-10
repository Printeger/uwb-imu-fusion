# T10 完整交付方案与持续执行任务

依据：用户在R08复审后明确要求结束“每轮一个小修改、再报错、再补救”的推进方式，改为T10完整方案。
本文件替代尚未执行的R09最小任务建议，保留R08独立复审发现和所有历史证据。
本次只制定方案，没有启动实现、validation或test；以下是下一次完整工作包的执行范围。

## 1. T10究竟交付什么

T10不是“求解器在某个seed跑通”，也不是“找到一个让eta显得有效的阈值”。
按冻结roadmap第569–606行，它必须完成：工程正确的评分与最终推断；有限validation选择；
参数与指标锁定；未见test上的风险/覆盖和下游比较；有证据的门控/论文主张取舍。

完整交付由五件东西组成：

1. 同一生产入口通过全部适用端到端工程验收，已知LOS、metadata、失败/中断与输出语义问题闭合或准确分类。
2. 按基础轨迹划分、完整失败记账的validation结果，以及可重放的参数选择记录。
3. locked_gate.yaml与locked_metrics.yaml，或明确的NO_ADMISSIBLE_GATE决定，不能强造零接受赢家。
4. 符合准入条件时，锁定后一次性held-out RQ3结果：两条隔离路径、风险覆盖、实际工作点轨迹、代价与失败。
5. T10结论书：保留full gate、简化为简单gate、降为探索性诊断，或明确尚无法科学验收；给出论文改动草案。

**工程完成、研究得出负结论和T10原验收完成是不同状态。**
若始终无法得到正确管线/可用参数，工作包可以以明确失败报告停止，但T10仍为BLOCKED/INCOMPLETE，
不能因预算到期标DONE。若正确且有限的研究已完成、结果不支持eta，则可关闭该研究问题并收窄主张；
这不等于整篇论文完成或保证可发表。

## 2. 改变工作方式，避免再回到碎片循环

采用一个T10工作包和一个主执行manifest，各阶段是内部检查点，不是每阶段回来索要一张新prompt。
路径、编译、ABI、API、导出、身份和已证明的correctness修复在本包内自主完成；修复后重跑受影响检查。
通过阶段门就自动继续下一阶段，不能停在“ready”“preflight通过”或“建议再开下一轮”。

历史ticket全部不可覆写，但新correctness修复有新的run ID即可在本包内执行受影响重跑。
“失败结果必须保留”不等于“第一次接口错误后必须结束对话”。
普通数值失败保持为统计中的失败，继续其他预登记输入；不能见一次失败就另开求解器研究。

仅以下情况暂停依赖工作并向用户报告具体裁决点：
- 必须改变冻结目标/状态/评分定义或实验问题，无法在现有合同与本计划内解决；
- 关键输入/权限确实缺失，无法完成对应范围；
- 触及本计划总预算或有限修复边界；
- 最终论文主张需要用户确认。其余独立且已授权工作继续。

原roadmap第815–821行已规定：编译、路径、API适配由Codex解决；审查主要介入数学歧义、
实现错误与假设错误难以区分、claim取舍。本计划恢复这一边界。

## 3. 阶段A：集中修复和完整工程验收

先保存源码快照/dirty内容hash、二进制/库、有效配置和当前所有已知失败。不得用空git diff替代untracked源码记录。
复用现有C++/GTSAM、生成器、runner和evaluator，不复制完整estimator或再造通用任务平台。

集中处理R08已暴露的问题：

| 问题 | 必须完成的处理 | 验收依据 |
|---|---|---|
| 空support误用handoff request | 无C严格策略分流；禁止handoff | 同真实入口零候选链 |
| 空support进入非空评分器 | 验证空结果身份后输出NOT_APPLICABLE_NO_CANDIDATES，继续final | 不删除非空评分器输入校验；完整最终输出 |
| 零eligible时跳过final | 保留候选、原因、Suppress参考与失败状态 | 非空候选/零eligible实际fixture |
| 陡ramp metadata失配 | 找出实际obs/factor/失败谓词/二进制数值，区分语义错误和求值舍入 | 从冻结现场恢复；不足时本包内至多一次带完整现场输出的开发诊断复现 |
| 缓ramp无C lambda耗尽 | 审计最后接受状态、outer2、P/D、generic与驻点，判断实现缺陷还是合法非收敛 | 不凭gradient单项通过宣布四项AND收敛 |
| 中断被汇总为0 | 无记录/损坏与观测到零分开；可恢复只依赖完整终态证据 | 杀进程/残缺文件fixture；UNKNOWN/NOT_EXPORTED |
| bias列阶段含混 | decision与final指标分列，失败与fallback分列 | C++真实导出→独立评价→聚合往返 |
| validation与旧development标签并存 | 新产物语义明确、role贯穿内容身份，旧包不重标 | 正确role握手及跨role拒绝 |
| all_range被错误代用 | 原始全部valid ranges、同raw初始化与数值策略独立求解 | 图/Values/mask/命令证据；禁止给Suppress结果改名 |

不预设metadata根因。不以删除检查、随意加容差、加先验或改precision让测试变绿。
数值专项诊断最多占阶段A四小时活动工作；只有证明实现违反既有数学/停止语义才做修复。
若只能改变求解器接受/停止规则或模型，写出证据与amendment裁决点；不能静默改变，亦不能无上限续试。
尚无法解决的合法非收敛以失败进入矩阵；若它阻止基本LOS或任何参数准入，最终如实判定未完成。

工程验收必须覆盖**同一个实际生产入口**，不能用局部单测替代：

| 路径 | 必须走到的终点 |
|---|---|
| raw LOS→自动零候选 | Stage2无C→score不适用→final→独立all_range→evaluator |
| 非空候选但零eligible | Suppress最终轨迹或准确失败，分母完整 |
| 单组Use、单组Suppress、多组mixed | raw factor唯一、accepted live C、最终身份与两套分数 |
| ramp多段/短段/边界 | 不丢候选、不非法合并，不因不适用伪造有效分数 |
| recovery失败→fallback成功/失败 | 至多一次、记录原失败、目录互不覆盖 |
| score/identity/metadata拒绝 | 错误定位到字段，拒绝不能被记成零候选 |
| 跨role、错scenario truth、错parent、变库 | 消费前拒绝；test标签隔离 |
| 中断/空文件/截断输出/恢复 | 不重复消费完成run、不把目录存在当运行通过 |

工程真值只使用开发fixture。必须以失败前版本复现已知接口问题、修复后验证完整路径；
构建和适用U01–U14检查通过后，独立审计门输出证据，而不是仅自填PASS。
不必重跑无关全仓测试。正式实验开始前集中解决一批问题，不能把12输入矩阵当接口测试集。

## 4. 阶段B：一次预登记全部数据与矩阵

dev10101/10102与其基础轨迹永久development；R08两条base永久validation。
保留旧split中val turn01/20101、turn02/20102，以及尚未使用的test turn01/30101、turn02/30102、turn03/30103。
先核验实际使用历史。不同scenario、prefix和同base派生噪声保持同role；3条test仍仅支持有限同运动族验证。
synthetic geometry/beta/noise是明确生成假设，不冒充实测独立标定。

每条base固定九场景，均在新实验前写入机器可读manifest：

| 场景 | 定义 |
|---|---|
| LOS、step1、step2、step3、ramp_gentle、ramp_steep | 复用旧split精确场景，不根据结果重新设计幅值 |
| step2 + 较高noise | 实际range sigma从.05变.10m，名义sigma同步正确声明；不能只修改评分权重 |
| step2 + 较少观测 | UWB从5Hz变2.5Hz，实际移除相应消息、重新初始化/input plan；IMU保留 |
| step2 + 较弱几何 | synthetic八anchor的x坐标乘.1，数量、其余坐标/噪声不变；生成与estimator使用同一已声明几何 |

后三项只各改一个因素，不互相组合。历史评价仍[3,6]s；同输入不同方法严格相同时间关联，
跨采样场景报告不同matched count，不装作相同采样实验。弱几何检查主要改变R，noise/count用于实际改变N。
这一设计源于R08八组N=6400I导致eta与s确定映射，不是为了构造eta获胜的数据。

validation为2×9=18输入；test为3×9=27输入。均保留失败/空候选/空接受/不可评分。
同base的九场景是配对条件，不算九个独立样本。

两条RQ3路径都保留：
- AUTO：完整自动发现后冻结候选，所有gate在同输入的同一Stage2候选上比较。
- FIXED_PARTITION_DEBUG：只对step2、两种ramp和上述三种单因素变体（每base六项），
  用运行前冻结scripted support，固定常值段隔离model mismatch；不输入幅值/GT初始化。
  两条路径分别建namespace、表与图；DEBUG不能修补或冒充AUTO成功。

## 5. 阶段C：有限validation参数选择

支撑配置只有旧提案P1/P2/P3三套，不展开它们内部参数笛卡尔积。每套在18个validation输入各运行一次，
最多54次AUTO producer；基于物理模型的数值精度/容差/预算一视同仁，不能按标签分别调solver。
选择规则在本批开始前固定：先比较完整失败率，再在相同成功域比较逐base的完整obs bias-field误差，
然后成本和固定配置ID打破平局；不按单个gate分别选择支撑配置。
主六场景必须有有效Stage1/2、每base至少有eligible组，才有正式工作点准入；stress失败另报但不删行。
若没有配置合格，输出NO_ADMISSIBLE_SUPPORT，停止正式gate准入，不继续搜P4/P5。

使用选定的共同支撑与冻结候选做缓存比较。每种策略等量、最多24个预声明点，
阈值变化不重跑Stage1/2；以下默认表是受限搜索族，不是全参数最优证明：

| 策略 | 固定候选点（均24个） |
|---|---|
| fit_only | gamma=.25,.50,…,6.00 |
| s_fit | s={.0125,.025,.05,.10,.20,+inf}m × gamma={.5,1,2,4} |
| full_gate | s={.0125,.025,.05}m × gamma={.5,1,2,4} × eta={0,.5} |
| nominal_curvature | lambda_min(N)门槛={0,25,50,100,200,400,600,800,1000,1200,1600,2000,2400,2800,3200,4000,4800,5600,6400,8000,10000,12000,16000,25600}m^-2 |
| eta_only（simulation诊断） | eta={0,1/24,…,23/24}，不加入gamma/s |

full的eta=0表示允许选择不使用eta筛选的退化政策；如果选中，必须明确写“eta未被选用”，不能称eta验证有效。
nominal-curvature/eta-only主要作缓存诊断，不声称已有它们的下游轨迹比较。
相同(s,gamma)的eta=0/.5子集也单列，用于区分“加eta的改变”和“不同搜索族范围”。
参数范围在validation结果前冻结；若目标覆盖不可达就报告不可达，不临时增加阈值或插值出一个赢家。

沿用三个candidate coverage目标{.25,.50,.75}和偏差<=.10；依次按覆盖差、accepted squared bias error、
bad correction、固定ID选择；两条base均须非空接受，epsilon_bad=.20m，两base各bad rate<=.10。
候选域含所有ineligible观测，非零接受但靠丢弃不可评分失败行不能合格。
.50作为预选工作点；其他点锁定为risk–coverage曲线点。选择只使用AUTO validation，DEBUG只解释。
无合格点则NO_ADMISSIBLE_GATE，不用all-suppress的UNDEFINED risk伪造安全赢家。

对预选工作点在18个AUTO输入运行suppress_all、structured_debias、fit_only、s_fit、full_gate五个final，
LOS另独立all_range；至少得到三gate的实际下游比较。非选中的24个阈值点不逐个跑final。
检查LOS新失败及RMSE+.05m/P95+.10m容忍，并同时公布实际差值；不因容忍宽松隐藏退化。
若预选点不满足准入，输出NO_ADMISSIBLE_GATE，不继续按轨迹误差循环选下一点。

## 6. 阶段D：锁定与held-out，一次完成

生成完整锁定包：selected support、各策略阈值、coverage目标、排序与tie规则、epsilon_bad、
LOS容忍/trajectory failure、时间关联/指标、实际源码/二进制/库/生成器、split、cache规则、预算及场景hash。
准入审计须核实适用工程验收、真实LOS结果、完整失败记账、未使用test标签和同图输出身份。
审计通过后在本工作包中自动继续，不再为了执行已登记test索要下一张prompt。
此处执行权限以用户采用本完整任务prompt为准，替代R08/R09旧的“不运行test”局部边界。

锁定后27个AUTO test输入各运行一次，只用选定支撑；18个FIXED DEBUG test各运行一次。
每个AUTO输入运行五种final、LOS加all_range；DEBUG默认只评分/缓存诊断，不冒充final效果。
每条test的全部预声明决策/final及估计产物冻结后，整批统一开放evaluation-only truth给独立evaluator。
展示锁定工作点和曲线点的实际test coverage，不根据test重新对齐覆盖或重选阈值。
matched coverage只有实际差<=.10且风险非空可比时才称匹配；不满足则直接报告不能匹配。

若test发现可复现correctness bug：隔离受影响结果、保留失败，用开发fixture证明与修复，
不改变方法/threshold/selection；显式失效并完整重跑受影响项，标为test-exposed correctness rerun，
不能把它称为从未见过的确认性重复。若修复本质改变方法或受到test指标驱动，不再调参救结果：
当前test降为探索性，确认性证据缺失必须记入最终结论；新的未见test需要另行决策，不能偷偷换seed。

## 7. 研究判定与最终审查

同时回答：
- score与实际bias误差/坏修正/好修正拒绝关系如何？
- 采用修正相对Suppress的轨迹收益、failure/fallback和成本如何？
- full相对fit_only/s_fit是否在可比较coverage下有实际增量？不同N/模型失配条件下如何？
- LOS是否退化？自动ramp分段是否改变了固定模型诊断结论？

默认实用差异报告尺度预登记为bias RMSE 5mm、trajectory RMSE 1mm、bad rate 2个百分点；
它们是本任务的判断尺度建议，不是安全界或统计显著性阈值；在test前锁定且不因test结果更改。
按base汇总配对差，报告每条base以及全部场景；仅两val/三test base，不宣称充分统计显著性。
只有在至少两条test base上有达到预登记尺度的同方向收益、总体配对方向不恶化，且其余base、LOS、
失败和覆盖代价没有推翻该用途时，才提出有限范围的实用增量主张；若证据冲突，给出INCONCLUSIVE。
不能用bad correction减少自动推导trajectory更好。对风险贡献和轨迹贡献分别给结论。

| 最终证据 | 工作包结论与论文建议 |
|---|---|
| full相对简单gate有重复实用增量 | 保留限定适用范围的政策主张；展示代价/失效例，不作保证 |
| full与s_fit相当，eta未提供清晰增量 | 建议简化为s_fit；eta保留为诊断，登记方法简化草案 |
| gamma等简单gate已足够 | 采用有证据的简单版本，收窄方法贡献 |
| 门控和修正没有实用收益 | 降为探索性模块；C2收益主张撤回/未支持，不能靠系统描述自动兜底 |
| 实验没有区分政策/覆盖不可比/独立base过少 | 明确证据不足，不把“未检出”写成“等效” |
| 工程不正确或无可准入配置 | 不锁gate，不伪造test；T10保持未完成，交付失败裁决而非下一张零碎修复建议 |

贡献升降的最终措辞由用户审阅完整证据后确认。Codex先提交具体保留/删除/改写段落草案，
不静默改冻结结构或将PARTIAL改SUPPORTED。

## 8. 总预算和有终点的修复规则

建议工程活动工作上限16小时，其中数值专项至多4小时；不是保证日历交付时间。
无人值守实验/生成/评价总wall预算12小时：预排程9小时、correctness/系统恢复reserve3小时。
该预算是本完整方案建议，执行前写入主manifest，不能把它说成已消耗或已有用户日历承诺。
同一工作包内reserve可按预登记理由自动使用：只用于证明的correctness或基础设施恢复，
不用于增加阈值、选seed、改精度或让普通数值失败变成功；每次留记录，不要求逐次用户审批。

计划最多111个AUTO/FIXED producer（54+12 validation、27+18 test），最多230个AUTO工作点final
（validation 18×5+2 LOS reference，test 27×5+3 LOS reference），等价final可在完整身份验证后复用。
这不是111×230个组合。阈值计算只读缓存。单producer/单final仍以900s硬限、fallback计入final；
外层总预算优先，不承诺所有单项达到上限时仍能完成整个矩阵。
R08单输入约几十秒至90秒只是成本估计依据；启动前按阶段实测分布估计scheduled成本，
若超过9小时，按预登记规则先省等价final执行和可选附图计算，不删失败输入、主场景或两条RQ3路径。
必要核心预算不足则明确报告，不擅自改变科学矩阵。

同一个确定性输入/实现身份不允许“再试一次碰碰运气”；有证明的修复或基础设施恢复才有新attempt。
完整工程回归后的大批次correctness返工最多两轮，集中处理已知问题并重跑受影响项，
不按每个小错误分轮。达到返工/时间上限且核心仍不正确时提交完整blocker dossier并停止依赖实验，
T10不能标DONE；已经正确且独立的工作仍交付。不无限制造R10/R11式ticket链。

## 9. 必交付产物和沟通

实际文件路径可复用现有目录，至少提供：
- T10_MASTER_PROTOCOL.md / 单一运行manifest、split、预算和attempt ledger；
- 工程完整链路验收矩阵、源码快照、失败分类和修复失效范围；
- AUTO/FIXED分离的validation/test score、decision、final、metrics及全失败行；
- validation选择记录、locked_gate.yaml/locked_metrics.yaml或NO_ADMISSIBLE决定；
- 按base显示的risk–coverage、配对trajectory/bias、LOS、failure/fallback和成本图表；
- 独立复算、实际命令/exit/日志/hash/重放方法；
- T10_FINAL_REPORT.md：支持/不支持/不足的结论、论文改动草案，以及T11/T12接入说明；
- STATUS与CLAIM_EVIDENCE同步，但不把T10结果冒充T11 prefix、T12真实主实验或T13论文完成。

工作中定期汇报实际完成的阶段、已知问题、预算和下一门；不要每修一个文件请求新prompt。
最后只做一次完整科学审查，除非中途遇到第2节真正需要用户裁决的事项。

## 10. 可直接发送的总任务prompt

请执行 doc/ie_sprint/T10_COMPLETE_DELIVERY_PLAN.md 定义的整个T10工作包。
本任务替代尚未执行的R09最小方案及R08旧的单轮停止/不运行test边界，但保留全部历史失败、
数据角色、冻结数学和不使用test调参的约束。

先完整读取STATUS、冻结论文结构/roadmap、方法/实验合同、R08独立复审与本计划。
将本计划的矩阵、参数选择、预算、修复/重跑规则及自动test准入门登记为T10_MASTER_PROTOCOL和机器可读manifest。
持续完成集中工程修复→同生产入口完整验收→有限validation选择→锁定→准入审计→未见test→
独立评价与最终结论报告。常规代码/接口/构建/输出/身份correctness修复在本包内自主处理，
不因为一个报错结束任务或要求用户再给下一轮prompt；科学失败如实记账，不调参救结果。

工程门通过后自动进入已登记实验，不能停在ready或只交付文档。达到有限预算/返工边界时如实交付
已完成与未完成以及证据，不假称T10完成。若需改变冻结数学或遇到真实外部阻塞，提出具体裁决点，
同时继续不依赖该裁决的已授权工作。

不预设eta、复杂gate或修正必须有效。最后提交一份完整可审查T10结果及保留/简化/撤回主张草案，
而不是另一个只修单个报错的最小任务。
