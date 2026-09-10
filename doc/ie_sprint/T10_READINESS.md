# T10 准入与 Stage 1 诊断

状态：`IN_PROGRESS / A04_REVIEW_ACCEPTED / A05_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE / A06_REVIEW_ACCEPTED_BOUNDED_SCOPE / A07_REVIEW_ACCEPTED_BOUNDED_SCOPE / A08_REVIEW_ACCEPTED_LIMITED_ENGINEERING_SCOPE / A09_REVIEW_ACCEPTED_BOUNDED_SCOPE / A10_REVIEW_ACCEPTED_BOUNDED_SCOPE / A11_REVIEW_ACCEPTED_BOUNDED_SCOPE / A12_REVIEW_ACCEPTED_BOUNDED_SCOPE / A13_REVIEW_ACCEPTED_BOUNDED_SCOPE / A14_REVIEW_ACCEPTED_BOUNDED_SCOPE / A15_REVIEW_ACCEPTED_BOUNDED_SCOPE / A16_REVIEW_ACCEPTED_BOUNDED_SCOPE / A17_REVIEW_ACCEPTED_BOUNDED_SCOPE / A18_REVIEW_ACCEPTED_BOUNDED_SCOPE / A19_REVIEW_ACCEPTED_FAILED_RUN_DIAGNOSIS_SCOPE / A19_R01_ENGINEERING_FIXED_FRESH_RUN_FAILED_BEFORE_STAGE1 / A19_R02_STOPPED_AT_FIRST_ALGORITHMIC_FAILURE / A19_R03_ENGINEERING_PASSED_FRESH_RUN_SIGSEGV_BEFORE_STAGE1_TRACE / A19_R04_CRASH_ROOT_FIXED_ENGINEERING_PASSED_FIRST_STAGE1_IDENTITY_FAILURE / A19_R05_IDENTITY_FIXED_STAGE2_CONVERGED_EXPORT_FAILED_BEFORE_SCORE / A19_R06_AUTOMATIC_SCORE_COMPLETE_PAIRED_REFERENCE_FAILED / A19_R07_UNBLINDED_DEVELOPMENT_PAIRED_REFERENCE_COMPLETE / A19_R08_LIMITED_VALIDATION_MATRIX_8_OF_12_SCORED_NO_GATE_LOCK`

## A19-R08 当前交付与准入判定

R08完成绑定split的有限synthetic validation入口和预登记的12输入固定矩阵。CMake ABI与受影响工程门通过；
真实role/split/parent/content身份贯穿producer、Stage2、decision和final，test reservation未接触。12条均至多
一次：8条完整Stage1/Stage2/score/五final，3条首次算法失败，1条系统中断不重试。8条均为1/1 eligible；
所有三gate的Use/Suppress向量相同。turn01 step1/2/3使用候选，其余已评分输入抑制。

冻结1320项后8个独立evaluator exit0。7条structured可配对结果的历史RMSE仅1条改善、6条恶化，P95全部
7条恶化；gate使用的turn01三个step相对全抑制RMSE分别恶化0.453/0.746/0.553 mm。两LOS均无有效final，
LOS相对all_range代价UNAVAILABLE；turn01/ramp_gentle的四个抑制final在唯一fallback后失败。

本轮只支持两base的有限validation characterization，不锁gate、不作统计/泛化推断、不运行held-out/test、
不升级C1--C3。Stage2 export保留legacy `UNBLINDED_DEVELOPMENT`字段但实际role身份为validation且
`consumable=false`；这一标签限制已披露。唯一主阻塞是validation零候选LOS support/refit/score/all_range
路径闭合，然后才能进行有限cache选择与held-out。[完整证据](evidence/t10_a19_r08_validation_compare_20260910T115109Z/VERIFICATION.md)。

## A19-R07 当前交付与准入判定

R07 的唯一实现扩展是让 development final 的 accepted-empty、candidate-all-suppressed 无C recovery和
fallback使用既有333-bit `PAPER_CERTIFIED_PAIR_REDUCTION_V1`。无C明确禁止inexact handoff；原四项AND、
lambda/容差/200 outer、非空C Stage2/handoff、score/gate、payload schema和legacy/default均不变。真实小图
normal recovery与受控fallback均实际调用certified callback，错误identity在callback前拒绝；8项refit/export、
15项inference/policy及完整raw prepare通过。

唯一fresh P1 step seed10101从raw重新初始化并exit0：Stage1 90 outer、Stage2 24 outer，1 group/1 eligible/
0 unavailable；eta `0.58397847413582771`、s `0.016357299043480211 m`，两段gamma
`0.1431107780213654/1.4164987078762303`。五个final各实际运行一次且全部有效、fallback 0。共同参考为
339 factors/123 X-V-B keys/0 C；structured_debias final为371 factors/125 keys/2 C，使用32/32 candidate。

冻结387项decision/final/代码/配置/身份后，独立truth评价得到：共同参考全段ATE RMSE/P95为
`0.0234272037/0.0391565429 m`（41点），历史`[3,6]s`为`0.0151169771/0.0210468736 m`（16点）；
structured分别为`0.0240652206/0.0394080403 m`和`0.0158797893/0.0229025775 m`。因此structured相对
参考在历史区间RMSE/P95为`+0.0007628122/+0.0018557039 m`，在这个已见seed上略差。其accepted bias
RMSE仍为`0.005767514545 m`且bad correction 0/32，不能把bias-good当作轨迹改善。

fit_only、s_fit、full_gate都通过eta/s并因max gamma `1.4164987078762303 > 1`抑制整组，与suppress_all的
终态identity和trajectory相同；本输入没有区分三种gate，也不能评价eta增量。空接受risk为UNDEFINED。
本条仍是带R05永久truth事件的`UNBLINDED_DEVELOPMENT`，不准入正式validation/test、不锁gate、不升级
C1--C3。R07结束该seed的工程修复循环；唯一主阻塞转为预登记有限validation和held-out未见数据证据。
[完整证据](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/VERIFICATION.md)。

## A19-R06 当前交付与准入判定

R06 只修生产输出与审计：同一 Stage2 joint graph/Values 的 X/Pose3、V/Vector3、B/ConstantBias、C/double
完整原子导出和 production reader 逐位回读；严格 JSON 区分 inner convergence、guard evaluation 与
qualified handoff。真实含非零/零边界 C 的 production writer→reader、失败不发布、同图数值复核、
score→decision→final→独立 fixture evaluator、完整 raw prepare 和 14/14 受影响测试均通过。原共享 schema、
solver、333-bit、handoff guard 和工作点保持冻结。

fresh attempt1 是明确的预初始化父目录错误；保留失败并用新 ticket 重试后，attempt2 从 P1 step seed10101
原 raw 完成 Stage1 90 outer、Stage2 24 outer 和完整评分。Stage2 导出 125 keys/1027 scalars 并逐位回读；
1 group、1 eligible、0 unavailable，eta `0.58397847413582771`、s `0.016357299043480211 m`，两段 gamma
`0.1431107780213654/1.4164987078762303`。estimator truth/GT open 0，external wall 103.136841s。

五策略各尝试一次。structured_debias 使用全部 2 段/32 候选并成功，`[3,6]s` ATE RMSE/P95
`0.0158797893/0.0229025775 m`；suppress_all、fit_only、s_fit、full_gate 全部抑制，两次（recovery与合同
fallback）都在冻结 200 outer 报 `MAX_REFIT_ITERATIONS`。因此三个 gate 的决定完全相同，且共同参考没有
有效 trajectory；relative benefit 为 UNAVAILABLE，不能从 structured_debias 的绝对误差推出“优于抑制”。
零接受 accepted risk 为 UNDEFINED。

R05 truth 暴露记录永久保留；方法/decision/final hash 冻结后才由 evaluator 读取 truth。首次 evaluator
只在 truth hash 阶段因 `sha256:` 前缀处理 exit1，登记 amendment 后仅修前缀规范化，第二次 exit0。本结果
统一标 `UNBLINDED_DEVELOPMENT`，不准入 validation/held-out，不锁 gate，不升级 C1--C3。唯一主阻塞为
冻结 suppress-all recovery/fallback 不收敛导致的共同参考缺失。[完整证据](evidence/t10_a19_r06_live_c_score_compare_20260910T092530Z/VERIFICATION.md)。

## A19-R05 当前交付与准入判定

R05 用共享 factory/validator 修复 Stage1 identity defect，保留原精确 schema，不改变 solver。完整 raw
prepare 在真实 371-factor/123-value 图后校验生产请求，contract accepted、0 optimizer call；精确正反例和
小型真实 C++/GTSAM Stage1→partition→Stage2→score→decision/final→独立 evaluator 均通过。

唯一 fresh P1 step ticket 从原 raw 初始化。Stage1 90 outer、327 calls/468 trials收敛，冻结两段
`[3,6]s`候选、共32条；Stage2 24 outer、96 calls/189 trials、1次合同handoff后满足原四项AND，首次越过
历史outer15。评分尚未调用时，runner持久化同一Stage2 Values，复用的A17 serializer只接受X/V/B，遇到
live scalar C抛`NUMERIC_REFERENCE_UNSUPPORTED:unsupported value`。exit2、82.672844s、RSS 36708 KiB；
按首个实际运行失败边界无修复重跑。

group/eligible/score/eta/s/gamma、五种decision/final/fallback、ATE/bias/risk/coverage和相对suppress-all
收益均`NOT_RUN/UNAVAILABLE`；32条只完成candidate partition，没有实际Use/Suppress决定。agent另在decision
冻结前查看evaluation manifest和部分bias truth，已封存为协议事件；estimator自身truth打开0，但本run不得
产生独立收益评价。T10不准入validation，C1--C3不升级。
[证据](evidence/t10_a19_r05_identity_score_compare_20260910T083223Z/VERIFICATION.md)。

## A19-R04 当前交付与准入判定

R04 用保存的原 R03 runner/库取得了真实 SIGSEGV 栈：`free(0x41)` 来自 factor 0 的
`NoiseModelFactor::error`，调用方是 `GraphLinearizationContentHash`。手工 runner 的 Eigen
16-byte/SSE 与工程 core 的 32-byte/AVX 对齐 ABI 不一致；改用普通 CMake target 同构构建后，同一完整
raw prepare 路径在 2.3502 秒内成功，生成 371-factor/123-value 内容身份并明确 0 optimizer call，故原
软件崩溃根因已经修复。真实 mixed score→decision→final 及独立 evaluator 消费门也通过，工程输入上产生
一组 Use 和一组 Suppress。

11/11 startup gate 后签发的唯一 fresh science ticket 完成 raw 初始化/建图，却在 Stage1 outer1 前因
runner 的 `A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY` schema 不在 Stage1 producer allowlist 中而
`INVALID_INPUT`。exit1/2.31691s，无 timeout，0 outer/0 conditional call。按首次算法失败边界无 retry；
Stage2、真实 score、五策略 final、truth 评价、ATE/bias/risk/coverage 与收益比较全部 `NOT_RUN/UNAVAILABLE`。
T10 仍不准入 validation，C1--C3 不升级。唯一下一阻塞是统一并测试该 Stage1 schema 握手，然后另行签发
fresh ticket。[证据](evidence/t10_a19_r04_crash_score_compare_20260910T072815Z/VERIFICATION.md)。

## A19-R03 当前交付与准入判定

R03 的严格 Stage2 非空 live-`c_s` inexact handoff、12 个封存 outer15 试步逐位状态检查、12 类反例、
真实 mixed graph 联合审计、五策略 development adapter 与独立 evaluator 工程门均通过；legacy/default
语义和最终四项 AND 未放宽。最终启动预检 8/8 通过。

attempt1 是明确的预初始化启动缺陷（参数 `char*` 地址比较），exit2/0.084951s 且没有输出叶；修正和身份
更新后 attempt2 打开冻结 config/完整 raw input，写入 nonconsumable manifest 后在 Stage1 trace 前
SIGSEGV exit139/2.096557s。由于不能确认崩溃仍在算法开始前，按停止边界无第三次运行。Stage1
`UNKNOWN_NOT_EXPORTED`，Stage2/score/eta/s/gamma/五策略 final/truth evaluator 均 `NOT_RUN`，不能报告
零 eligible 或零风险。T10 仍不准入 validation，C1--C3 不升级。唯一阻塞是另行授权后对该 fresh runner
初始化/建图区间 SIGSEGV 作带栈定位。[证据](evidence/t10_a19_r03_inexact_handoff_20260910T061142Z/VERIFICATION.md)。

## A19-R02 当前交付与准入判定

依照[预登记协议](T10_A19_R02_PROTOCOL.md)，第一轮 estimator-free 启动预检只因 `ldd` symlink 文本路径
比较缺陷 exit1；全部实际启动探针已通过且 estimator `NOT_RUN`。改用 realpath 比较后第二轮 9/9 exit0，
才签发并消费唯一 fresh ticket。

P1 step seed10101 从原 raw 初始化；Stage1 在90 outer、327 calls/468 trials后收敛并冻结2段/32候选，
两段均位于 `[3,6] s`、各16条、short=false。Stage2 完成14个 outer 后在 outer15 第3 conditional call
耗尽原 lambda 搜索上界，累计83 calls/170 trials、82 accepted/88 rejected，返回
`CONDITIONAL_LM_LAMBDA_SEARCH_EXHAUSTED`。进程 exit1，external `79.63632955400004 s`、RSS
`35828 KiB`，无 timeout；truth/GT/oracle/checkpoint 未读取。

该失败发生于初始化和优化之后，故不得作为启动错误使用第二次 attempt。Stage2 未收敛，score、完整
group/segment 评分、五种处理 decision/final/fallback 与 evaluation-only truth 均 `NOT_RUN`；ATE、bias risk、
coverage 及相对 suppress-all 的结论均 `UNAVAILABLE`，不能解释为零。T10 保持 `IN_PROGRESS`，不进入有限
validation，C1--C3 不升级。唯一主阻塞为冻结设置下 Stage2 outer15 的 certified conditional LM lambda
search exhaustion。[完整证据](evidence/t10_a19_r02_auto_score_compare_20260910T041247Z/VERIFICATION.md)。

## A19-R01 当前交付与准入判定

依照[预登记协议](T10_A19_R01_PROTOCOL.md)，R01 修复了 common-reference/candidate 全量 certificate metadata、
真实 mixed CertifiedLm 集成门及 Stage1 检查点/异常状态。17/17 工程门通过；真实小图 2 candidate +
2 reference 在非零 fixed beta 与 live `c_s` 下完成 7 outer、14 calls/22 trials，并由现有 scoring 产出
有限 eta/s/gamma。该结果仅为 engineering fixture，不能支持正式政策比较。

工程门后唯一 fresh P1 step ticket 已消费，但 launcher 因缺少 `pilot/` 父目录在 Stage1 前 exit2，
external 0.108370 s；Stage1/Stage2/scoring 全部 `NOT_RUN`，callback/trial 精确为零，truth/GT 打开 0，
无 retry。launcher 已在失败后修正，当前 hash 与已消费运行身份明确分开，修正后 estimator `NOT_RUN`。
因此 A19-R01 没有新增自动闭环 development score，更未提供正式 validation/cache/final/locked gate。
T10 保持 `IN_PROGRESS`，C1--C3 不升级。[完整证据](evidence/t10_a19_r01_stage2_integration_20260910T024847Z/VERIFICATION.md)。

## A19 当前交付与准入判定

A19在实现前登记[限定amendment、工程门和唯一运行预算](T10_A19_AMENDMENT_PROTOCOL.md)。工程门通过43-pair证书相容性（5.015875s）、A18 C++104项、discovery39项、refit/scoring24项和相关121项回归；默认关闭、A19不可消费schema及错误role/身份拒绝通过，三项已修复工程失败保留。

唯一fresh P1 step seed10101运行的Stage1为CONVERGED：90outer/327calls/468trials、327接受141拒绝、四项AND通过、最终尺度梯度9.520208e-7。Stage2 outer1在创建certified optimizer前因common-reference raw range缺证书metadata返回 `NUMERIC_REFERENCE_UNSUPPORTED:UNKNOWN_FACTOR`；零Stage2 optimizer call/trial/P-D证书。score、eta/s/gamma均NOT_RUN；Stage2 amplitude/boundary/group和eligible/unavailable为NA。无retry，truth/GT打开0，external62.0414s。

因此A19没有提供门控比较所需的真实下游评分证据，T10仍不能进入正式validation。唯一下一步是补齐common-reference fixed-beta metadata并以真实mixed-reference/candidate certified Stage2 fixture封门；本轮已按首次失败停止。[完整证据](evidence/t10_a19_stage2_score_20260910T014316Z/VERIFICATION.md)。

## A18 当前交付与准入判定

A18限定交付完成待review：登记 REVIEW_ACCEPTED_A17_CERTIFIED_FIRST_BLOCK_PROTOTYPE_SCOPE，来源本指挥会话及独立复审 `/tmp/t10-a17-commander-review-j5heubp8/REVIEW.md`。默认关闭C++进程内333bit证书保留A17数值语义；真实AutomaticSupportProvider全Stage1接线覆盖实际beta+非零u，开发输出schema隔离且旧cache reader拒绝。43-pair全证书/有理P/保守fidelity通过，完整批次4.783429s；C++104项/原discovery39项通过，全部工程失败保留。
唯一fresh P1 step seed10101运行exit0、60.5083s：90outer/327calls/468trials，327接受141拒绝0unresolved；证书累计31.1697s。首block完整复现A17数值；Stage1最终原objective/step/KKT/navigation四项AND通过，chain后尺度梯度9.520208e-7。实际2段/32candidate/short0（每段16条、3–6s）；boundary/group/eligible/unavailable为NA，Stage2/score/cache/final/gate/validation/test/T11/scheduler NOT_RUN。
仅本development输入Stage1观察，非端到端/held-out证据；T10 IN_PROGRESS，A14失败/A15全部FD失败/A12负结果/A08历史15/18与C1–C3限制保留，不锁gate、不升级claim。完成归档后停止等待review。

[完整A18交付](evidence/t10_a18_certified_stage1_20260910T010000Z/VERIFICATION.md)。

## A17 历史交付（本轮指挥接受）

登记指挥接受 `REVIEW_ACCEPTED_A16_FIXED_ENDPOINT_PRECISION_AUDIT_SCOPE`；本指挥会话与独立复审 `/tmp/t10-a16-commander-review-7d4bmy57/REVIEW.md` 为来源，仅接受A16限定审计。
[A17限定amendment/冻结工程门与预算](T10_A17_AMENDMENT_PROTOCOL.md)先于实现/测试；默认关闭的独立C++ PAPER_CERTIFIED_PAIR_REDUCTION_V1原型复用原GTSAM solve/native retract，333bit P/D与fidelity比较/转换均保守舍入。模型/初始化/priors/原generic AND stationarity、lambda预算保持，core/GTSAM/legacy未改。
工程门通过：A16固定端点/615维方向严格一致，D证书复现，独立精确有理P包络通过；23个区间/失败/转换反例、15个原策略相关GTest、原型小图2calls计数/原驻点通过；实际旧Stage2 reader拒绝原型schema。首次Python nextafter缺失、空分支CSV工程失败及实施前lambda勘误全部保留。
唯一fresh P1 step seed10101原raw初始化pilot exit0、external89.2440s/RSS35880KiB：20calls/43solves/43trials，20accepted/23rejected/0unresolved；最终尺度梯度2.2196421412e-7，原阈值1e-6/roundoff5.0389874292e-9，lambda0.010000000000000005，满足原generic AND stationarity。
这只是首conditional block合格，不是Stage1收敛；程序立即停止，无chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler，未评candidate/segment/group/eligible/unavailable为NA。证书累计求值81.2431s，尚无全链路成本/收益证据。
[完整交付](evidence/t10_a17_certified_prototype_20260910T002211Z/VERIFICATION.md)含全部P/D区间、逐factor/原始端点/实际命令/失败/库身份。原型独立策略身份与consumable=false，禁止旧缓存消费；不铺开Stage2/cache/final集成，不升级claim/准入。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。A17本地限定交付完成待review，完成后停止等待review，不自行扩展下一轮。

## A16 历史交付与准入判定

登记指挥接受 `REVIEW_ACCEPTED_A15_TERMINAL_NUMERIC_DIAGNOSTIC_SCOPE`，来源为本指挥会话，非外部REVIEW.md、非estimator成功或validation准入。
[A16预登记协议](T10_A16_PROTOCOL.md)先于实现/计算；唯一call17 trial1恢复初始/末态graph、Values、371factor/123keys/615delta及两端1886native残差严格一致，原生retract一次后封存binary64端点。
零optimizer iterate/新estimator进程；固定50/100位参考与有向区间表明该端点对下降 `+2.59734275148281994e-13`；两档差`3.20911e-48`，独立区间半宽`9.16821e-46 / 7.22877e-96`满足预登记门，分支核对通过。
原生残差求值/白化将下降高估`4.00953e-13`，解释原白化恒等式与A15稳定GN差异的约79.26%；仍剩`D_ref-GN=+1.04901e-13`，未进一步区分端点舍入/非正交、J/r误差与GN余项。不宣称全域导数正确或solver会收敛。
[完整证据](evidence/t10_a16_endpoint_precision_20260909T160252Z/VERIFICATION.md)含公式/独立误差依据、逐factor分解、原始端点、命令/失败及实际库身份；core/GTSAM/模型/生产策略/合同不改，无truth/GT读取。
唯一[solver amendment草案](T10_A16_SOLVER_AMENDMENT_DRAFT.md)为paired reduction与误差证书/resolution判据，明确接受/失败/身份失效/回归；PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED，本轮不实施或追加实验。
T10 IN_PROGRESS；A14失败、A15全部124个factor FD失败、A12负结果、A08历史15/18及C1–C3限制保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评计数NA。
A16本地限定交付完成待审，完成本次审计后停止。

## A15 历史交付与准入判定

指挥接受 `REVIEW_ACCEPTED_A14_OPT_IN_IMU_MODEL_AND_BOUNDED_PILOT_SCOPE`，来源为本指挥会话，非外部 REVIEW.md，非 estimator 成功或 validation 准入。
[A15预登记协议](T10_A15_PROTOCOL.md)先于实现/捕获。默认关闭被动TRYDELTA模式无额外solve/InspectFirstLinkedLmTry/extension；只捕获首block并阻止chain。
唯一P1 step seed10101新模型捕获exit1，external1.75785s/RSS31160KiB；初始graph/Values/common严格复现A14，末态17calls/16accepted/42trials及E/五类梯度/lambda复现，仍不驻点。
全部42个trial 615维完整；静态零optimizer恢复371factor、16accepted native retract及内容身份通过。call17的19trials为18次线性差负/1次零，均未计算model fidelity。
指定首/末拒绝方向的稳定线性下降为+1.54833e-13/+5.80613e-15，而linked总目标相减为-1.36424e-12/-3.18323e-12；两正值仍低于原5.16324e-13 resolution门。
三条实际完整方向及末态最大梯度坐标在预登记连续步长判据下支持导数一致性；13356个factor残差FD中124失败全部保留，不能写全步长通过，A08历史15/18限制不消除。
结论支持线性化差值数值分辨力与linked分支限制；非线性真实下降/残差求值误差与非线性余项分离仍证据不足，不宣称改算术即可收敛。
唯一[最小后续提案](T10_A15_NEXT_ACTION_PROPOSAL.md)为同call17 trial1零iterate非线性残差精度审计，本轮NOT_RUN，无solver修复或数值语义amendment实施。
[A15完整证据](evidence/t10_a15_terminal_numeric_20260909T152633Z/VERIFICATION.md)保留编译失败、中止重叠构建、唯一pilot失败、完整静态结果和source/raw/config/actual runtime身份；truth/GT未读，GTSAM不变。
T10 IN_PROGRESS；A14失败、A12负结果、A08历史限制及C1–C3原状态保留。chain/Stage2/score/cache/gate/final/validation/test/T11/scheduler NOT_RUN；未评candidate/segment/group/eligible/unavailable计数NA。
A15限定诊断本地完成待审；完成后停止，不自行执行下一轮。

## A14 历史交付与准入判定

指挥接受 `REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE`；A13唯一方案已技术接受，仅显式opt-in development。
[A14 amendment/冻结判据](T10_A14_AMENDMENT_PROTOCOL.md)先于实施。新PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1显式K0，live bias、其他噪声/RW/Qi、交叉项、初始化/priors/solver保持；缺省legacy I6。
实际模型身份接入common、discovery/support、Stage2 producer/cache/request、final及直接core消费门，跨模型拒绝。
A12四SHA精确复现；1680静态检查、127 C++工程回归及Python mock-runner合同通过，实际库身份闭合。
唯一P1 step seed10101 pilot为exit1：outer1第17调用lambda搜索耗尽；16接受/26拒绝trial，Gmax=1.20549e-5未达原驻点。
chain/Stage2/score/cache均未到；candidate/segment/group/eligible/unavailable计数NA，不能把失败空partition解释成零候选或零eligible。
external wall0.688s/RSS27396KiB；无retry，truth/GT未读，raw/config/GTSAM不变。
[A14完整交付](evidence/t10_a14_conditional_imu_20260909T142807Z/VERIFICATION.md)保留源码/配置/输入/库/命令/负结果；T10 IN_PROGRESS、A14本地限定交付完成待审。
A12负结果、A08历史15/18/UNKNOWN及C1–C3限制保留；validation/test/T11/scheduler/gate/final NOT_RUN；不默认迁移或升级claim。
本轮停止，不自行执行后续末态诊断。

## A13 历史交付与准入判定

以下为A13当时状态；其唯一草案在A14获技术接受并实施，见上文。

指挥接受 `REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE`；限定A12诊断，不是成功或准入。
[A13协议](T10_A13_PROTOCOL.md)先登记，只恢复同P1 step call50。原graph/Values身份门通过，零optimizer构造/iterate。
实际40/40 PIM的biasAccOmegaInt=I6，与测量密度逐sample叠加；不是初始bias prior或已声明RW。
A10模型声明未覆盖该额外项及物理来源，结论不一致；不是通用GTSAM bug或非收敛唯一根因的证明。
1600步native重放mean/cov精确一致，完整cov来源总和误差9.44e-16；白化/交叉/曲率814/814检查通过。
同冻结方向总κ=0.244713，其中IMU0.198580；原完整precision下K的方向敏感性归因为99.9997925%，不是来源信息简单相加。
唯一[amendment/最小修复草案](T10_A13_AMENDMENT_DRAFT.md)为显式paper conditional-live-bias模型，legacy不变，需新身份/回归；
PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED。本轮不实施修复或其回归，不继续实验。
[A13完整证据](evidence/t10_a13_imu_covariance_20260909T135328Z/VERIFICATION.md)：一次静态进程1.52s、离线1.65s；
保留诊断漏include及SyntaxError，生产source/raw/config/库不变，truth/GT未读未hash。
A12两臂不驻点/B负结果、A08历史15/18及全部历史限制保留；Stage2/validation/test/T11/gate/scheduler NOT_RUN。
T10 IN_PROGRESS、准入PROPOSED_NOT_ADMITTED、C1–C3不升级。完成本次审计后停止。

## A12 历史交付与准入判定

指挥接受 `REVIEW_ACCEPTED_A11_BOUNDED_FIRST_BLOCK_DIAGNOSTIC_SCOPE`，仅A11限定诊断接受，不是estimator成功或validation准入。
[A12协议](T10_A12_PROTOCOL.md)先登记，且只用P1 step seed10101 call50。图/Values身份、371factor、五类梯度及call50实际方向证据全部复现。
声明物理尺度后白化J943×615、rank615；列跨度68712、cond(J)=451687。原阻尼/曲率对弱模式0.408642、强模式2e-12，支持单次对照但不证明根因。
A150次精确复现A11每条数值；B仅diagonalDamping=true。A/B最终梯度0.073059237/0.048386890，均不满足原驻点；
B没有达到预登记梯度减半的改善判据，不集成策略。各150次、外部7.15/6.31s，clipping均0、无retry，预算关闭。
保留静态接口必需context漏传exit1、修正后的唯一checkpoint采集及两臂cap未收敛。
生产core/GTSAM和原raw/config身份未变；本轮未读/hash truth，实际file-open和/proc maps齐全。
chain/Stage2/gate/held-out/scheduler NOT_RUN，段/组/eligible/unavailable NOT_EVALUATED，无新评分或cache。
最小后续提案：同checkpoint最弱旋转方向的逐factor白化曲率及IMU预积分协方差贡献审计，零iterate，不改参数；本轮NOT_RUN。
[A12完整证据](evidence/t10_a12_checkpoint_scale_20260909T131349Z/VERIFICATION.md)。A08历史15/18限制保留；准入仍PROPOSED_NOT_ADMITTED，C1–C3不升级。

## A11 历史交付与准入判定

指挥接受 `REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE`，仅A10限定交付验收，
不是estimator成功、validation准入或外部REVIEW.md。A08历史图级15/18和三个最小步长UNKNOWN不变。

- [协议](T10_A11_PROTOCOL.md)先于实施/运行登记：只用冻结P1 LOS/step/ramp seed10101，最多3进程/120s；
  第49/50实际接受的完整delta按原3步长、原FD判据全部通过才可同optimizer续至总200次。
- 实际3/3进程，exit1，无timeout/retry；50次已保存数值、计数、common init/graph身份全部复现A10。
  A10没有保存完整50次Values，本轮已单独保存，不冒称存在历史逐状态对照。
- 原生retract匹配的六个615维实际方向：图级18/18、factor6678/6678通过；call50最大梯度坐标9/9通过。
  三条均条件式继续同一optimizer到200次，无lambda reset/初始化/容差/默认迁移；仍未满足原驻点。
  最终梯度LOS/step/ramp为0.000301066/0.073059237/0.144923784，roundoff远不能豁免。
- 600次均接受更新且记录目标下降；梯度振荡下降，支持缓慢接近驻点，未见无更新停滞。
  检查仅覆盖call49/50六方向，不升级为通用导数正确性证明，不消除A08历史限制。
- 全部在首块后停下：completed outer=0、chain=0，Stage2/gate/final NOT_RUN；段/组/eligible/unavailable
  NOT_EVALUATED，无新Stage2 cache或有效eta/s/gamma。真实额外wall38.025269s、最大RSS80908KiB，含诊断/strace成本。
- runner/core因默认关闭诊断重编；GTSAM与A10一致，/proc maps、ldd、SHA/buildID前后一致。
  inputs/configs/truth/生成器/合同/冻结材料hash未变；estimator file-open trace无truth/GT读取。
  保留首轮诊断JSON raw-string编译失败；修正后build通过，92/92 focused GTests通过。
- 最小后续动作仅提案：对封存P1 step call50作一次C++静态Jacobian列尺度/条件数及实际lambda阻尼比例审计，
  先校验graph/目标身份，不调用iterate。本轮NOT_RUN，不继续加cap、不猜测性修solver/Jacobian。

完整证据：[A11 VERIFICATION](evidence/t10_a11_first_block_20260909T122715Z/VERIFICATION.md)、
[逐run汇总](evidence/t10_a11_first_block_20260909T122715Z/SUMMARY.csv)、
[驻点进展](evidence/t10_a11_first_block_20260909T122715Z/FIRST_BLOCK_PROGRESS.pdf)。
T10仍IN_PROGRESS；A10 validation方案仍PROPOSED_NOT_ADMITTED；held-out/scheduler/gate lock/claim升级NOT_RUN。

## A10 历史交付与准入判定

本轮用户/指挥会话已接受 `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`；这不是外部 REVIEW.md。
A08 图级 FD `15/18` 及三个最小步长限制不变。A10 数据仅 development，不能升为 held-out。

- **已实现/验证：** [生成协议](T10_A10_PROTOCOL.md)、自包含 Python 生成器（无 estimator）、
  同运动 UWB/IMU、实际 PCG64 seed、稳定 obs_id、独立 raw/truth、总动态 latent bias、完整合成参数来源。
  合成假设不伪称真实独立标定。数值一致性、同 seed byte复现、不同 seed 实际随机通道变化和5个腐败/边界
  测试通过；首轮 C2 join checker 失败原样保留。旧 bag base error 仍 UNKNOWN。
- **实际 pilot：** 3 套候选 × LOS/双链路 step/ramp，各 seed10101，共9/9次，每次120s硬限。
  全部在首次 conditional LM 的50-call cap报 `CONDITIONAL_LM_STATIONARITY_NOT_REACHED`，
  0 completed outer、0 chain update；Stage2/score/final NOT_RUN，无 Stage2 cache或有效 eta/s/gamma。
  输出0段/0组是失败的空 artifact，实际 eligible/segment/group count=NOT_EVALUATED，不是零风险。
  三套support参数不可比较，不选参数、不扩展预算。总外部wall9.750853s，最大RSS27356KiB。
- **实际隔离/身份：** 9/9 raw ledger 全328条及nominal sigma=.05逐条核对；共同init/graph身份跨候选一致；
  truth路径运行期不可达、file-open trace无truth/GT读取；真实/proc maps与前后ELF/lib哈希一致。
  执行配置中orientation字段拼写被忽略但实际默认false且raw has_orientation=0，未读GT；
  交付driver已改为正确initialization字段，未重跑estimator，原配置/driver已封存。
- **具体待审方案：** [validation admission](T10_A10_VALIDATION_ADMISSION.md)、
  [split proposal](T10_A10_SPLIT_PROPOSAL.json)：2个独立validation base/seed、3个future test base/seed，
  68个潜在estimator slot，四gate各12缓存点，选择/metric规则、B_total=14400s及25%reserve。
  split角色/数据spec已列明，尚未生成的recording/cache内容ID为null；scheduler改造仅方案，未冒称实现。
- **准入仍拒绝：** 需要已接受的development AUTO收敛/评分证据、数值支持域审查与role/provenance/split
  接线实现。A10没有执行held-out validation/test、gate sweep/lock、正式指标或claim升级。

完整证据：[A10 VERIFICATION](evidence/t10_a10_synthetic_pilot_20260909T115705Z/VERIFICATION.md)，逐次统计/成本/实际cache身份见
[PILOT_SUMMARY.csv](evidence/t10_a10_synthetic_pilot_20260909T115705Z/PILOT_SUMMARY.csv)。以下早期段落是历史事实；A10增量优先。

前置检查和 Stage 1 只读诊断已经由指挥官接受。A02 子阶段实现了默认关闭、版本化的 conditional-navigation
stationarity qualification，并使用原 step/ramp 输入和除 A02 policy 外完全相同的有效参数各运行一次。
T10-A02-R01 已最小修复内外层 stationarity 参数可冲突的问题；A02-R01/A03 已由独立复审接受。A03 按预登记只把两份隔离配置的 outer cap
从 50 改为 500，各运行一次。两条均在到达 500 前因原 inner cap 失败，未进入 Stage 2。A04 已独立复审
接受；A05 随后按运行前登记集成默认关闭、版本化的 V2，并只把现有 A03 step/ramp 的 policy 从 V1 改成
V2 各运行一次。V2 恢复旧失败 checkpoint，但两条稍后均因原 lambda 上界耗尽而 Stage 1 失败。以上只属
`DEVELOPMENT_IMPLEMENTATION_AND_DIAGNOSTIC_RUNS_ONLY`，不执行正式 validation、阈值 sweep、gate lock、
test 或 RQ，也不改变冻结 objective、外层停止条件、默认 cap 或默认行为。T07、T09 保持 `DONE`。

A05 独立复审以 `CHANGES_REQUESTED` 返回；原报告 F1/F2 稳定登记为 `T10-A05-RF1`、`T10-A05-RF2`。本轮 review-fix
仅纠正异常诊断计数并补本轮动态库身份：catch 不再虚构一次 trial/rejection，只累计 linked GTSAM state
已经提交的 transition；异常期间无法从 state counter 判断是否开始过未提交 trial 时，计数为 confirmed lower
bound，状态为 `INCOMPLETE_EXCEPTION_DURING_ITERATE`。诊断 schema v2 同时区分 `NOT_EXECUTED`、`COMPLETE`
和该不完整状态。runner、全部实际 focused test executable、core 和 GTSAM 的 realpath/SHA-256/build ID 与
raw/normalized `ldd` 已绑定，测试前后关键身份一致。历史 A05 step/ramp 当时未绑定共享库的缺口不能事后补证。
完整本地证据见
[`evidence/t10_a05_review_fix_20260909T040832Z/`](evidence/t10_a05_review_fix_20260909T040832Z/VERIFICATION.md)。

本轮指挥会话随后实际核对三个 manifest、三个当前源码快照、最终运行身份、discovery 34/34、双策略 post
probe 与 final audit，并明确接受
`REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE`。这是当前会话的限定工程接受，不是外部
`REVIEW.md`；指挥没有重建或重跑 step/ramp。RF1/RF2/RF1-N01 在该范围收口，历史 A05 加载库身份缺口与
V2 最优点 exhaustion 边界仍保留。

A06 已按预登记对 step outer 123/ramp outer 168 各执行一次 checkpoint observer，并由本轮指挥会话接受为
`REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE`。该接受不是外部 REVIEW；指挥实际核验 manifest
`247/247`、源码 `4/4`、当前身份 `9/9`、discovery `34/34`、历史 `122/167 x 86` 和终止拒绝 `11/10`，
未重建、重跑 estimator 或修改仓库。`64*epsilon` 不是 factor/retract/累加的完整浮点误差上界；A06 没有
新增该类 solve，但既有 `InspectFirstLinkedLmTry` 本身包含 diagnostic solve。A06 证据见
[`evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/`](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。

A07 已以 actual linked `TRYDELTA` 补齐 22/22 个 trial 的 24 key/120 维方向，并只对预登记 6 个方向做同一
live graph 的事后求值。linked 与事后 linear predicted decrease 一致，但 6 个方向的 18 个 nonlinear FD
均不满足 `g^T u`；高 lambda 方向出现解析下降/实际一阶上升的符号反转。逐 factor long-double 差值累计与
direct graph change 同为目标上升，排除仅有总目标直接相减的符号错误；UWB expression 的绝对变化占主导，
与 pose prior/IMU 呈 `5.28e4--6.85e6` 倍对消。A06 的单一主导坐标在相同步长通过，因此仍需唯一的逐 factor
方向导数证据才能指定错误 factor。见
[`evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/`](evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)。

本轮指挥会话已接受 `REVIEW_ACCEPTED_A07_BOUNDED_EXACT_DIRECTION_DIAGNOSTIC_SCOPE`；这是会话验收，不是
外部 `REVIEW.md`，指挥未重建、重跑 estimator 或修改仓库。A08 在其冻结六方向与原三个步长上保留
1314 个逐 factor 点，证实 linked `PriorFactor<Pose3>` 对 `-Local(x,prior)` 的 identity Jacobian 与 Pose3
retract 不一致；同一 residual/objective 的 paper-only factor 使用 `-D_x Local(x,prior)` 后全部 factor 点通过。
UWB、Combined IMU、velocity/bias prior 均已检查，未见同量级不一致。修复后两个原失败 checkpoint 都接受
下降步，step/ramp 分别在 outer 178/173 满足原四项 AND；随后 Stage 2 都在原 50 轮上限失败，无有效估计或
cache。图级冻结方向仍保留三个 `h=1e-6` 判据失败，精确浮点来源为 `UNKNOWN`，没有放宽判据或把
`64*epsilon` 当完整误差上界。完整证据见
[`evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/`](evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/VERIFICATION.md)。

本轮指挥会话随后接受 `REVIEW_ACCEPTED_A08_PAPER_POSE_PRIOR_JACOBIAN_FIX_SCOPE`；这是会话验收，不是
外部 `REVIEW.md`，不虚构指挥重建、重跑或修改仓库。图级 `15/18` 与三个最小步长失败限制继续保留。
A09 在运行前登记并只将 A08 两份隔离 development 配置的 `max_refit_iterations` 从 50 改为 200；结构化
preflight 证明无其他有效数值参数变化。step/ramp 各唯一一次且无 timeout，Stage 1 的 `178/173 x 87`
公共非计时字段、Stage 2 前 50 轮的 `50 x 22` 字段均与 A08 精确一致。Stage 2 分别在 outer 66/124
首次四项 AND 全通过并导出 refit estimate；两个规范封存 cache 均为
`CONVERGED/COMPLETE_WITH_SCORE_UNAVAILABLE`。step 的 1 个 group、ramp 的 3 个 group 全部不 eligible，
所以有效 score 仍为 0，两进程以 `ONE_OR_MORE_GROUP_SCORES_UNAVAILABLE` exit 1。这是 development
budget 充分性证据，不是正式 scoring/validation。

前置检查证据：[`evidence/t10_precheck_20260908T073803Z/`](evidence/t10_precheck_20260908T073803Z/VERIFICATION.md)。
本阶段证据：[`evidence/t10_stage1_diagnostics_20260908T081328Z/`](evidence/t10_stage1_diagnostics_20260908T081328Z/VERIFICATION.md)。
A02 实现与对照证据：
[`evidence/t10_conditional_stationarity_policy_20260908T091614Z/`](evidence/t10_conditional_stationarity_policy_20260908T091614Z/VERIFICATION.md)。
A02-R01 修复与 A03 限定预算诊断：
[`evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/`](evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/VERIFICATION.md)。
outer-121 停滞定位：
[`evidence/t10_conditional_lm_stall_20260908T130400Z/`](evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md)。
A04 固定检查点恢复对照：
[`evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/`](evidence/t10_fixed_checkpoint_lm_recovery_20260908T142233Z/VERIFICATION.md)。
A05 V2 集成与定向运行：
[`evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/`](evidence/t10_v2_conditional_lm_recovery_20260908T151942Z/VERIFICATION.md)。
A06 新失败 checkpoint 定向诊断：
[`evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/`](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。
A09 Stage 2 有界预算诊断：
[`evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/`](evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/VERIFICATION.md)。

## A09 时点准入结论（A10 更新见页首）

T10 的工程骨架已经能表达同一 Stage-2 cache 的 `fit_only`、`s_fit`、`full_gate` 和
`nominal_curvature` diagnostic，并能把 decision 与 final replay 接到同一 parent cache；但当前不能开始正式
validation。A08 后真实 T07 step/ramp development AUTO 已越过 Stage 1，A09 又证明 Stage 2 在 66/124 轮
达到原四项 AND，并形成两个内容绑定的 partial-score cache；旧“均未越过 Stage 1”表述已失效。当前直接
阻塞改为：A09 的 1/3 个 overlap group 全部不 eligible、没有有效 score；当前 scheduler 明确只接收
`development`；split、
独立数据身份、调参预算与待锁数值尚未由审查确定。真实数据的 fixed `beta`、噪声、几何/时间及参考 provenance
也未闭合。

这些真实数据缺口不一概阻塞合成 validation。可以先提供具有完整生成 provenance、独立基础轨迹和真实随机
seed 的合成 calibration/validation/test 单元，并显式固定 `beta`、`sigma`、锚点、杆臂、时钟/时间偏移和
其他生成假设。当前 T07 sidecar 仅标注 `INJECTED_COMPONENT_ONLY`，基础记录的总潜在 bias 为 `UNKNOWN`，
所以不能直接承担 total synthetic latent-bias validation。

## A09 时点逐项准入清单（A10 更新见页首）

| 项目 | 当前事实 | 证据路径 | 缺口 | 最小补齐动作 |
|---|---|---|---|---|
| AUTO discovery → Stage 2 | T06 真实 development 短输入曾完成 Stage 1/2。A08 paper-only prior fix 后 step/ramp 在 outer 178/173 完成 Stage 1；本轮指挥会话已接受 A08。A09 仅把 refit outer cap 50→200，Stage 1 与 A08 精确一致，Stage 2 分别在 outer 66/124 首次四项 AND 全通过并导出 estimate | [T06 最终复审](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md)、[A08](evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/VERIFICATION.md)、[A09](evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/VERIFICATION.md) | A09 已由本轮指挥会话限定接受，非外部 REVIEW；默认 cap 未迁移，正式 validation identity/split/provenance 仍未闭合 | A09 复审接受已登记；审查 A10 新合成输入与首次 conditional LM 失败，不追加旧输入 budget |
| Stage 2 → 有效评分 | A09 step/ramp 已有内容绑定的 AUTO Stage-2 cache；分别为 24/27 段、17/21 short、0/1 boundary、1/3 groups，eligible group 均为 0，全部 group score 为 `NOT_APPLICABLE_SHORT_OR_BOUNDARY`。有效评分仍只由 engineering GTSAM fixture 支持 | [A09](evidence/t10_a09_stage2_budget_diagnostic_20260909T111641Z/VERIFICATION.md)、[T09 证据边界](../../paper/CLAIM_EVIDENCE.md#t09-batchcacheevaluator-证据边界) | 缺 provenance 完整的 AUTO eligible candidate 与有效评分覆盖 | 用独立、完整 truth/provenance 的 validation 单元预声明验证至少一个 eligible，并保留 zero-eligible/failure 终态；不把 A09 development cache 或 fixture 提升为科学证据 |
| decision/final | T09 已证明两个 `fit_only` final consumer 可从同一真实 AUTO partial cache replay，request 不同、parent 相同，Stage 1/2 为 `NOT_RUN_CACHE_REPLAY`，final export `VERIFIED`；由于候选全不 eligible，状态为 `NO_ELIGIBLE_CANDIDATES`，不是 accepted-candidate 闭环 | [T09 最终收口](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md) | 缺带有效评分、accepted/suppressed 决策的 AUTO final | 先取得合格 AUTO validation cache，再从同一 cache 运行预声明 operating points；保留零接受、fallback 和失败分母 |
| engineering fixture | automatic scoring/GTSAM fixture 可制造 nonempty eligible candidate 并验证数值/接口；它是工程 fixture | [T06 最终复审](evidence/t06_final_review_20260907T043727Z/VERIFICATION.md) | 不是隔离 runner 的科学输入，也没有 held-out 身份 | 仅继续作为针对性回归；不得进入正式 risk/coverage 分母 |
| fixed-partition DEBUG | 已有完整 Stage 2 cache、diagnostic 与 final replay，namespace 为 `FIXED_PARTITION_DEBUG` | [T09 最终收口](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md) | 预声明分段绕过 discovery，不能代表 AUTO | 保持独立 DEBUG namespace，只用于 mismatch/接口诊断；正式 AUTO 统计不混入该缓存 |
| bias truth 语义 | T07 truth 只等于人为注入分量；基础记录总潜在 bias 为 `UNKNOWN`；seed 7001/7002 只进入上下文身份，没有随机生成作用 | [T07 实现说明](T07_IMPLEMENTATION.md)、[T07 review-fix](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md) | 缺 `TOTAL_SYNTHETIC_LATENT` 或独立 measured reference | 合成数据导出完整生成参数和 total latent truth；真实数据另给独立测量参考。始终把 post-fit residual 单列，绝不作为 truth |
| calibration/validation/test split | 当前 step/ramp 共用同一个基础记录、基础轨迹和内容；两个 seed 不是独立随机重复，而且数据已经用于 development | [身份记录](evidence/t10_precheck_20260908T073803Z/IDENTITY.md) | 没有按记录/基础轨迹/真实 seed 分组的未见 validation/test；当前 development 输入不能静默改标签 | 提供 split manifest，按 base trajectory/recording/生成 seed 分组；封存 test，先只授权 calibration/validation；同一 base 的扰动只能表述为该运动下扰动泛化 |
| 合成 calibration | 当前基础记录并非自包含生成数据，base component 只达到 `EMPIRICAL_CONSISTENCY_NOT_GENERATION_PROVENANCE` | [T07 review-fix](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md) | 固定 `beta`、nominal `sigma`、anchor/lever/time、基础 bias 与随机源未形成完整生成 provenance | 新建或补齐自包含合成 manifest，明确所有固定值、禁用项和 RNG；这些明确假设足以支持合成 validation，不必等待真实标定 |
| 真实 calibration | 独立 LOS fixed `beta` 缺失；GT 刚体点、杆臂、时钟/时间标定和 nominal sigma 来源未闭合 | [STATUS 未决项](STATUS.md#已知未决项与阻塞范围) | 不能生成可审查的真实 bias truth/轨迹指标和 fixed-calibration 正式配置 | 分别补 LOS calibration artifact、sensor-specific sigma、anchor/lever/time transform 与 GT/reference provenance；不以 residual 代替任一项 |
| scheduler role/provenance | `load_manifest` 硬拒绝非 `development`；带数值 policy 的 cell 只允许 `TEST_ONLY` 或 `PENDING_VALIDATION` provenance；现有保护有效 | [`run_experiments.py`](../../tools/paper/run_experiments.py)、[身份记录](evidence/t10_precheck_20260908T073803Z/IDENTITY.md) | 还没有 T10 validation admission、split binding 或 validation/test 防串用身份 | 保留现有入口；经审查后增加版本化 validation schema/入口，把 role、split hash、calibration/noise hash、budget ID、metric version、code/binary/cache ID 绑定到 run/cell/request identity。test 入口继续拒绝 |
| 四个 operating points | scheduler 已识别 `fit_only`、`s_fit`、`full_gate`、`nominal_curvature`，可共享 cache；curvature 读取 `lambda_min_N_m2_inv` | [`run_experiments.py`](../../tools/paper/run_experiments.py)、[T09 最终收口](evidence/t09_final_rereview_closeout_20260908T065646Z/VERIFICATION.md) | 待锁阈值、预算、排序规则及 curvature 输出的显式量纲/通过字段未完成 | 预声明每个策略相同有限评估数；decision 记录直接写 `lambda_min_N_m2_inv`、`tau_N_m2_inv`、单位和 pass；所有策略共享候选 cache |
| 评价定义与预算 | 合同要求 test 不参与选择，失败/零覆盖完整计入；计划预算最多用总预算 75%，至少保留 25% | [实验合同](EXPERIMENT_CONTRACT.md)、[roadmap T10](../v2/v2_roadmap.md#t10验证集锁定门控评估-rq3不为保住-η-改测试标准) | 支撑阈值、`tau/gamma/s/eta/tau_N`、bad correction、LOS 容忍度、配对规则、operating points 数量、总计算预算均待审 | 由负责人提交数值表、排序函数、每策略相同的 evaluation 数和 `B_total`；review 后才运行 validation。不得用 test label 回填 |

## AUTO、fixture 与 DEBUG 的实际边界

| 路径 | Stage 1 | Stage 2 | 有效评分 | decision/final | 可用于 T10 科学评价 |
|---|---:|---:|---:|---:|---|
| T06 真实 AUTO development 短输入 | 已通过 | 已通过 | 0/3 group eligible | partial cache 可 replay；`NO_ELIGIBLE_CANDIDATES` | 否；只有零 eligible/final 工程路径 |
| T07 step AUTO | A08 paper prior fix 后 outer 123 接受 3 步，outer 178 原四项 AND 全通过 | A09 cap=200 时 outer 66 四项 AND 全通过 | 0/1 group eligible；score unavailable | 内容绑定 cache 已封存；无 decision/final | 否；development-only、A09 已限定接受但零 eligible |
| T07 ramp AUTO | A08 paper prior fix 后 outer 168 接受 1 步，outer 173 原四项 AND 全通过 | A09 cap=200 时 outer 124 四项 AND 全通过 | 0/3 group eligible；score unavailable | 内容绑定 cache 已封存；无 decision/final | 否；development-only、A09 已限定接受但零 eligible |
| automatic engineering GTSAM fixture | 工程构造 | 工程构造 | 至少一个有效 score | 接口/engine 测试可达 | 否；仅 fixture |
| fixed-partition DEBUG | 绕过 | 可运行 | 可运行 | 可 replay final | 否；仅 `FIXED_PARTITION_DEBUG` |

因此“automatic 管线已能写出最终结果”和“存在 automatic accepted-candidate 科学闭环”是两个不同事实。
前者在 partial-cache replay 范围成立，后者仍缺证据。

## T07 step/ramp Stage 1 定向诊断

只读诊断实现记录实际 linked-GTSAM convergence 输入/有效容差/全部谓词、既有 inner iteration/lambda，
并在每个完成 outer iteration 以同一份 `lm.values` 分别审计旧 bias conditional graph 和更新 bias 后 graph。
旧 trace 18 列保持原顺序，新字段追加；诊断不参与 solver 更新、停止、接受或 fallback。实现、测试和两条
运行的完整身份见 [本阶段证据](evidence/t10_stage1_diagnostics_20260908T081328Z/VERIFICATION.md)。

实际链接的 GTSAM 4.2 先判断 `current_error <= error_tolerance`；未触发才计算下降量，并以相对或绝对下降
条件的 `<=` 判断，relative tolerance 为精确零时禁用该条件。本阶段以官方 4.2 源码和加载库探针共同核对；
原 `gtsam::checkConvergence` 返回值仍是 conditional LM 唯一停止依据。

原 step/ramp 输入与配置各运行一次，runner 均按预期 exit 1，在第 50 轮保持
`FAILED/AUTOMATIC_DISCOVERY/MAX_OUTER_ITERATIONS`，Stage 2 与 final 未运行。解析后输入、calibration、solver
hash 与历史一致；effective scientific config 逐行保持历史前缀，只追加后续版本中不适用的 final/gate 字段。
新 trace 的全部 18 个旧字段逐行比较最大浮点差为 `0`，终态/轮数/布尔字段完全相同。

### 停止条件和 LM 触发

| 条件 | 阈值 | step：50 轮 / 末轮 | ramp：50 轮 / 末轮 | 结论 |
|---|---:|---:|---:|---|
| relative objective | `1e-8` | 0/50；`1.050703e-6`（105.07 倍） | 0/50；`2.458630e-6`（245.86 倍） | 未通过 |
| combined scaled step | `1e-6` | 0/50；`3.030450e-4`（303.04 倍） | 0/50；`3.979003e-4`（397.90 倍） | 未通过 |
| chain ADMM/KKT | chain-specific primal/dual 与 KKT `1e-8 objective/m` | 50/50；末轮 KKT `9.946135e-9` | 50/50；末轮 `9.795496e-9` | 已通过，不是直接 blocker |
| pre-chain navigation stationarity（旧 bias） | `1e-6 objective` | 0/50；`2.684819e-4`（268.48 倍） | 0/50；`1.882850e-4`（188.28 倍） | LM 停止时尚未满足 |
| post-chain navigation stationarity（更新 bias） | `1e-6 objective` | 0/50；`6.406500e-3`（6406.50 倍） | 0/50；`8.732409e-3`（8732.41 倍） | chain 后进一步偏离 |

两条运行共 100 次 conditional LM check 全部可复算且只由 relative-decrease 谓词触发，error/absolute 谓词
均未触发。step 末轮实际 previous/current error 为 `4.8420450895904015/4.8420419861117985`，relative decrease
`6.409437635487609e-7 <= 1e-6`，LM/inner iterations 为 `1/1`，lambda `1e-6`。ramp 末轮为
`3.3150964454486624/3.3150964328953707`，relative decrease `3.786704835291057e-9 <= 1e-6`，LM/inner
iterations `2/2`，lambda `1e-7`；其 absolute decrease `1.2553291739436645e-8` 仍大于 `1e-8`。

chain 前后严格使用相同 navigation Values。更新 bias 后梯度在 step/ramp 各 50/50 轮均大于更新前；末轮
post/pre 分别为 `23.86x` 和 `46.38x`。这支持 block coupling 放大驻点偏离，但不能把全部差异归因于 chain，
因为 chain 前已经未通过。

末 10 轮 relative objective、combined step、post-chain gradient 的首末比分别为 step
`0.2482/0.3529/0.5743`、ramp `0.2482/0.4810/0.5337`，每项均 9/9 次下降。观察窗口内没有明确平台；
它只说明 50 轮处仍在变小，不能推出增加 outer 上限就会成功。

### 归因强度

- **已经证实：** generic LM 的 relative-objective 停止不足以满足旧 bias 条件图的导航驻点要求；chain 更新
  又在同 Values 上显著放大梯度。三个 outer 指标仍未通过，chain audit 全轮通过。
- **实现判断：** 没有证据表明 linked GTSAM 的公式、调用或记录实现错误。是否将 conditional stationarity
  加入 inner solve 资格属于下一轮需 review 的 solver correctness/design 选择；当前合同只以 post-chain
  四项 AND 决定 outer 成功，所以本轮不静默改行为。
- **速度/预算：** 末段仍下降，支持“尚在缓慢收敛”，不支持“已经平台”。仅凭这两条轨迹无法证明更多 outer
  budget 足够，也不能据此修改上限。
- **数据/模型：** `UNKNOWN`。基础 total latent bias、独立 calibration 与模型误差仍不能由本诊断分离。

## A02 条件驻点策略开发对照

A02 的实现、配置身份、strict JSON、默认 discovery/refit 回归和 failure artifact 均通过定向验收。generic
convergence 已通过但驻点未通过的小图会继续同一 optimizer；同步满足的小图与旧策略同轮且 Values 在预声明
容差内；invalid/nonfinite/exception/cap 均不返回成功。cap 路径保留最后一次有效 audit，不导出轨迹。实际
step/ramp 的 input plan 与诊断基线相同，solver identity 因 policy 分离。

两条 A02 运行仍在 outer 50 轮终止并未进入 Stage 2。conditional LM iteration 总数由 step `115` 增至
`538`、ramp `118` 增至 `513`，Stage 1 墙钟由 `0.137763/0.140818 s` 增至
`0.533520/0.553288 s`。末轮 conditional 驻点已通过，但 chain 后 gradient 分别为 `6.317840e-3` 与
`8.691964e-3`；relative objective 和 combined step 也仍分别超阈值 `103.74/302.23` 倍与
`243.35/396.44` 倍。末 10 轮三项仍单调下降，故不称平台，也不保证增加 outer cap 会成功。

这支持 conditional accuracy 问题已在 A02 checkpoint 局部消除，而外层 block coupling/fixed-point 推进仍是
直接阻塞。相对基线的小幅末轮下降不足以证明整体收敛得到实质改善；数据/模型/calibration 的贡献继续为
`UNKNOWN`。完整数值及复算见
[`COMPARISON.json`](evidence/t10_conditional_stationarity_policy_20260908T091614Z/COMPARISON.json)。

### A02-R01 与 A03 限定预算结论

独立复审指出 `DiscoveryOptions` 的 outer 与 `conditional_lm` 各自持有 tolerance、roundoff 和五类 navigation
scale，实际 inner solve 使用后者而 support canonical 使用前者。当前 provider 与独立 partition 入口已经用
同一检查强制 A02 两份参数精确一致；七类冲突逐项在两入口拒绝。一致输入可用，默认策略身份/行为和
standalone `RunCheckedConditionalLm` 保持兼容。focused build、config 11/11、discovery 26/26、refit/scoring
23/23 与 runner contract 通过，状态为 `LOCAL_FIX_PASS_AWAITING_INDEPENDENT_REVIEW`。

A03 在运行前登记，step/ramp 配置语义各只改 outer cap `50→500`，每条只运行一次且整进程小于 120 秒。
两条前 50 轮 84 列 trace 中全部非计时字段与 cap=50 精确一致，最大差 0；config/solver/support 与潜在
Stage-2 cache producer config identity 分离。step/ramp 分别完成 120/126 轮，在 outer 121/127 的
conditional solve 中用尽原 50-check inner budget，最终有效 audit 最大缩放梯度为 `2.29030094e-6` 与
`1.01200920e-6`，均未达到 `1e-6 + roundoff`。最后完成轮的 objective/chain 通过，但 combined step
`2.59280873e-6/1.50678469e-6` 与 post-chain navigation gradient
`5.48215815e-5/3.20539280e-5` 未通过；不存在四项 AND 同时通过的轮次。

因此 Stage 1 均失败，Stage 2 均未运行，eligible candidate 未评价；development truth 仍只覆盖 injected
component，base total latent bias、fixed beta 和科学 provenance 未闭合。A03 说明 cap=50 不是唯一最终阻塞，
但不支持增加 inner/outer budget 或修改 tolerance。完整复算见
[`COMPARISON.json`](evidence/t10_a02_r01_budget_diagnostic_20260908T120339Z/COMPARISON.json)。

## step outer 121 conditional LM 停滞定位

按独立复审接受意见，本轮只用当前 A03 step 配置复现一次，整进程 `2.506007 s`、未 timeout；ramp 未运行。
observer 仅在 outer 121 启用，记录 linked GTSAM `TRYLAMBDA`、每次调用前后 optimizer state、起止 Values、
factor error audit 和预声明有限差分，不改变 config、identity、接受或停止判断。前 120 轮 79 个非计时 trace
字段与 A03 baseline 逐字符串完全相同。

50 次 wrapper `iterate()`/check 中，第 1 次候选 model fidelity `0.9236501`，被接受并令 optimizer/inner
iteration `0→1`、lambda `1e-5→1e-6`。第 2--50 次均为同一未接受候选：linearized cost change
`+5.42677e-13`，nonlinear tentative cost change `-9.76996e-15`，model fidelity
`-0.0180033 < 0.001`；同时该差绝对值小于 `relativeErrorTol*error = 4.84087e-6`，linked `tryLambda`
据此直接返回 true，不写回 Values/state，也不 increase lambda。实际计数为 1 次 accepted update、49 次
small-cost 无更新返回、0 次 lambda-search rejection。A02 wrapper 的 linked convergence 因 error 不变而通过，
但 stationarity `2.29030094e-6 > 1e-6 + roundoff`，遂对同一 state 重复调用直到 50-check budget。

最大解析梯度坐标 `x1` translation local coordinate 4 为 `-2.29030094e-6 objective/m`。运行前冻结的
13 个中央差分步长中，`1e-4..3e-8` 连续 8 点满足预声明误差界，故不支持该坐标存在 Jacobian/objective
graph 不一致。tentative 目标差处于总目标相减敏感尺度，但 Values 不变的已证直接原因是候选未接受和
linked small-change 返回，不是 accepted update 在记录时丢失。完整分支、Values/factor capture、有限差分及
linked library/source 身份见
[`VERIFICATION.md`](evidence/t10_conditional_lm_stall_20260908T130400Z/VERIFICATION.md)。

最小修复建议是仅在 A02 qualified policy 下把首次“linked iterate 返回但无 authoritative state update，且
stationarity 未通过”改为明确 fail-fast 原因，避免重复消耗 check budget；不把失败变成功、不重置 optimizer、
不私增 lambda。该行为尚未实施，须审查并版本化进入 solver/support/cache identity。若要覆盖 linked GTSAM
的小差停止并继续 lambda 搜索，则属于另一个 solver/method amendment。

## A04 固定检查点 LM 恢复可行性

A04 已按预登记使用唯一一次 A03 step replay 重建不可移植的 live graph；整进程 `0.957253 s`，ramp 未运行。
两臂从 outer-121 authoritative 50-call 后同一 73-factor graph、24-key Values、objective
`4.8408701715814653` 和 `lambda=1e-6` 启动。前 120 outer 的 79 个非计时字段及 authoritative 50-call
CSV 与上一轮精确相同，shadow 不回写原估计器；原终态仍为 Stage 1 failure，Stage 2 未运行。

A 保持 linked 内部 `relativeErrorTol=1e-6`，50 calls/50 trials 全部由 small-change 返回，0 accepted，
gradient 保持 `2.2903009375e-6`，不驻点。B 只把内部该值改为零，外部 generic check 仍用 relative
`1e-6`、absolute `1e-8`。首 call 在 `lambda=1e-6` 拒绝负 model-fidelity 候选，增加到 `1e-5` 后以
model fidelity `0.0571895` 接受；第二 call 在 `1e-6` 以 `9.30233` 接受。两步 objective 均严格下降，
总下降 `3.8635761e-13`，最终 gradient `3.4517447e-7`，以原 tolerance/roundoff 通过。A/B 墙钟分别
`0.054561/0.002497 s`，均无 timeout，lambda upper `1e5` 与 50-call cap 未变。

因此 fixed checkpoint 的停滞可由继续原上界内 lambda search 恢复；不能再把它描述为上界内没有下降步。
但 B 明确改变内部数值语义，且只在 shadow 小问题验证，不是 A02 V1 或默认策略修复，更不是 Stage-1 成功。
后续最小集成候选应是另行登记、默认关闭的 V2 policy：optimizer 内部 search 用 rel=0，wrapper external
check 保留原 rel/abs，acceptance、lambda upper/cap、stationarity 和失败出口不变，并版本化隔离 identity/cache。

## A05 默认关闭 V2 集成与定向验证

A05 在实现及运行前登记为 `DEVELOPMENT_CONDITIONAL_LM_RECOVERY_V2_INTEGRATION`。新增的 V2 policy
只令 optimizer 内部 `relativeErrorTol=0`，外部 generic check 保持 relative `1e-6`/absolute `1e-8`；原
stationarity、roundoff、五类尺度、GTSAM strict-decrease/model-fidelity 接受、lambda upper `1e5`、50-call
inner cap 与外层四项 AND 均不变。默认策略仍为 V1。V2 的 policy/内部与外部 tolerance 已进入 effective
config 和 solver/support/snapshot/scheduler producer identity；V1/V2 身份逐项不同。没有实际 Stage-2 cache，
所以这里只证明 cache producer identity 分离，不能说已产生或复用 cache。

focused checks 最终为 discovery 30/30、config 11/11、refit 23/23、inference 18/18、paper methods 5/5、
Stage-2 cache 7/7，以及 runner/scheduler identity checks 通过。config 新 fixture 首次因缺 automatic 必填字段
失败，修正 fixture 后 11/11；该中间失败保留于 evidence。公开 C++ diagnostic struct 已扩展，旧 overload
调用形式保留，但受影响二进制已全部重新编译/链接，不声称 binary ABI 兼容。

两份运行配置经机器比较确认相对 A03 V1 基线只有 policy 一项变化，并严格各执行一次：

| 场景 | 旧 checkpoint 恢复 | 新失败 | conditional 总计（含失败 outer） | 外层终态 |
|---|---|---|---|---|
| step | outer 121：3 calls / 4 trials / 1 rejected / 3 accepted；gradient `3.4517447e-7`，驻点通过 | outer 123：先接受 1 个严格下降步，随后 11 trials 拒绝；lambda `1e5`，gradient `1.9922351e-6` 未通过 | 921 calls / 935 trials / 15 rejected / 920 accepted / 1 no-update | 完成 122 轮；objective 40/122、step 0/122、chain 122/122、navigation 0/122；Stage 1 failed |
| ramp | outer 127：2 calls / 5 trials / 3 rejected / 2 accepted；gradient `6.2729090e-7`，驻点通过 | outer 168：10 trials 全拒、0 accepted；lambda `1e5`，未进入 stationarity qualification | 907 calls / 919 trials / 13 rejected / 906 accepted / 1 no-update | 完成 167 轮；objective 81/167、step 36/167、chain 167/167、navigation 0/167；Stage 1 failed |

step/ramp 子进程分别 exit 1，runner 墙钟 `0.820071/0.868272 s`，均未 timeout。两条没有首次满足全部四项
停止条件；Stage 2 均未运行，eligible candidate 均未评价，fixed beta 状态仍为
`MISSING_CALIBRATION_DEVELOPMENT_ONLY`，truth/provenance 不足以科学评价。V2 因而是“可恢复旧停滞点但不足以
通过真实定向 Stage 1”的 development 策略；不能迁移默认或进入 validation。

### A05 review-fix 诊断与 provenance 边界

- `T10-A05-RF1`（映射独立复审 F1）：原 catch 对 linearize 异常虚增 1 次 trial/rejection。修复后 V1/V2
  均为 wrapper calls=1、confirmed trials/rejections=0、accepted=0、normal no-update returns=0，generic 与
  stationarity 均未执行、失败且无 Values。fixture 的 factor 调用计数独立证明该 pre-trial 情况实际 trial=0。
- linked GTSAM 在 tentative `graph.error(newValues)` 之后才通过 `increaseLambda/decreaseLambda` 更新 inner
  counter。可控 trial 内异常证明 candidate 已执行但 counter 仍为 0，故 wrapper 无法可靠给出总 trial 数；
  该情况明确为 `INCOMPLETE_EXCEPTION_DURING_ITERATE`，现有数值只表示 confirmed completed lower bound。
- `T10-A05-RF2`（映射独立复审 F2）：本轮测试身份已完整绑定并在测试前后复核；A05 历史 step/ramp 仍只保留
  当时 executable/source 身份，当前 core/GTSAM 哈希或事后 `ldd` 不作为历史运行时加载证明。
- 普通成功、V1 small-change/no-update、V2 recovery、call cap、lambda exhaustion、失败无 Values、CSV/JSON
  schema 和 policy/producer identity 回归均通过。公开 diagnostic struct 改变后所有依赖目标已重链接。
- 已知边界保持：合法图初值已达最优、非零 residual 且 `J^T r=0` 时，V1 可 generic+stationarity 成功；V2
  仍先在 lambda 上界耗尽后失败，不运行 external check/audit、不导出 Values。这是 A05 已登记语义，不是本轮
  correctness fix，也未经证实为历史 outer 123/168 的根因。

本轮指挥会话已将上述 review-fix 及 RF1-N01 以
`REVIEW_ACCEPTED_A05_DEVELOPMENT_INTEGRATION_AND_COUNTER_FIX_SCOPE` 限定接受。该接受没有外部复审文件，且不把
本轮身份追溯为历史 A05 运行证明。

### A06 新失败 checkpoint 定向诊断

- A06 在实现与运行前登记；两份配置是 A05 归档的逐字节副本，仅输出/run ID 和 checkpoint observer 不同。
  step outer 123/ramp outer 168 各一个 estimator 进程，实际 exit 1、`2.418821287/2.476906258 s`、无 timeout，
  不 retry，A04 shadow recovery 关闭。
- step 的 authoritative final stationarity 为 `1.9922351334744626e-6 > 1e-6 + 2.834613252956474e-12`；ramp
  为 `1.512759625631882e-6 > 1e-6 + 2.267357040736464e-12`。两点均为“已评价且未通过”，不是从 no-update
  反推。主导坐标固定 FD 分别通过 7/13 与 8/13，支持该坐标解析梯度与 objective 一致，但不证明完整方向。
- step 先接受 `lambda=1e-5` 的下降，随后 11 个 trial 从 `1e-6..1e4` 均预测下降却实际升高，fidelity 全负；
  ramp 的 10 个 trial 从 `1e-5..1e4` 同样预测下降却实际升高，前 9 个 fidelity 负，最后一个 predicted
  decrease 低于 linked 可分辨条件而不可用。两点方向范数均随 lambda 单调缩小，最终按未改规则到 `1e5` 失败。
- 预声明 `64*epsilon` 总 objective 判据只覆盖 step 2/11、ramp 7/10 个拒绝 trial；消减敏感重要但不能解释
  全部。缺 exact full delta 和逐 factor old/tentative error，model fidelity mismatch 的底层来源仍为 `UNKNOWN`。
- 与历史 A05 比较，86 个公共非计时字段在 122/167 个完成行上精确一致，公共 failure 终态也一致；仅排除计时
  与版本化诊断字段。Stage 1 保持 0/2，Stage 2/cache/有效估计均未产生。证据及唯一后续建议见
  [`t10_a06_failed_checkpoint_diagnostic_20260909T065842Z`](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。
- 本轮指挥会话接受 `REVIEW_ACCEPTED_A06_BOUNDED_CHECKPOINT_DIAGNOSTIC_SCOPE`，没有外部 REVIEW、重建或重跑。
  其中 `64*epsilon` 明确只作敏感性判据；既有 `InspectFirstLinkedLmTry` 含 diagnostic solve，A06 的准确说法是
  observer 没有新增该类 solve，而非整个 observer 零重复求解。

### A07 actual LM 方向定向诊断

- 小图实际 linked TRYDELTA 捕获 3 个导航 key/15 维，max-digits 文本解析与 actual accepted delta 及
  `Values::retract/localCoordinates` round-trip 在 `2e-15` 内一致，预登记三点方向 FD 3/3 通过；证明捕获、
  解析及求值方法本身在良态图上成立。`/usr/local` GTSAM 未修改，A07 未新增 solve。
- step/ramp 各唯一运行一次、预期 exit 1、无 timeout/retry。step 12/12、ramp 10/10 个 actual trial 均完整
  捕获 24 key/120 维，预登记 3+3 个 rejected direction 的 linked predicted decrease 与事后同一 linearization
  逐值一致；历史 122/167 行的 86 个公共非计时字段及失败终态精确一致。
- 六个方向的 18 个 FD 均未满足原判据：低/中 lambda 的 FD 仍指向下降但幅值仅为解析值的一部分，`1e4`
  则 step 为 `g^T u=-4.1794e-6`、FD 约 `+1.9717e-4`，ramp 为 `-2.5635e-6`、FD 约 `+6.9950e-6`，均发生
  符号反转。A06 单一主导坐标在同三步长通过，故不能把不一致归给该坐标。
- 每个 selected trial 的逐 factor fsum/long-double difference 与 direct graph decrease 同号且均为目标上升，
  排除了“仅 graph.error 总量相减导致符号错误”。不过绝对 factor 变化相对 net change 为
  `5.28e4--6.85e6` 倍；UWB expression 最大，pose prior 与 IMU 构成主要对消。当前没有逐 factor analytic
  directional derivative，因此这是唯一剩余缺口，不能把某一 factor class 定为 bug。证据见
  [`t10_a07_exact_lm_direction_diagnostic_20260909T082434Z`](evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)。

## 唯一后续动作

审查 A10 已实现生成链路、已完成的九次有限 development pilot 和具体 validation 准入提案。
本轮指挥已接受 A09，无需再以等待外部 REVIEW 阻塞交付。A10 仍无有效 AUTO Stage2/score；
若需推进首次 conditional LM 驻点阻塞，须另行裁定有限范围；本轮不追加进程、不改真值/solver/cap。
正式 validation/test/gate lock 未获准。

## 需要外部提供或裁定的最少信息

- calibration/validation/test 的 split manifest，包含 recording/base-trajectory/真实 RNG seed 身份；test 先封存。
- 合成数据的完整生成 provenance，或真实数据的 fixed beta、sigma、geometry/time、GT/measured-reference provenance。
- 待审数值表、每策略相同的有限调参次数、operating points、排序函数和总计算预算 `B_total`。
- A02-R01/A03、停滞诊断与 A04 已独立复审接受；本轮指挥会话已限定接受 A05 与
  `T10-A05-RF1/RF2/RF1-N01`。默认/A02 V1 的 rejection 后 small-change 已独立观察为
  2 trials/1 rejection/0 accepted/1 no-update；lambda exhaustion 末次 rejection 不重复计数，异常聚合仍保留
  `INCOMPLETE`。A02/V2
  是否迁移默认或进入正式方法仍需另行裁定，不能自动迁移。A06/A07/A08 已由本轮指挥会话限定接受；A09 为
  `REVIEW_ACCEPTED_A09_BOUNDED_STAGE2_BUDGET_SCOPE`（指挥会话）。进入正式 validation 仍至少需要：独立 split 与完整生成或真实
  calibration/noise/geometry/time provenance；版本化 validation role/scheduler identity；预声明的有限数值表、
  operating-point 排序和 `B_total`；以及能产生至少一个 eligible group 的、未用作 development 的 AUTO validation 单元。

## A10 前各阶段历史 `NOT_RUN` 记录（当前增量见页首）

除 Stage 1 诊断基线、A02 cap=50 对照、A03 cap=500 限定诊断和历史 A05 V2 分别记录的 step/ramp 定向运行
（各阶段各 `1/1`）、停滞定位唯一 step replay、A04 唯一 step 活图重捕获、A08 pre/post step/ramp 各一次及 focused build/tests 外，新数据、score、
T10 validation admission/执行、任何阈值 sweep、gate lock、test、完整 19-cell batch、正式 RQ1–RQ4、U14、
T11 prefix、T12 metrics、T13 paper result、trajectory/ATE/RPE、性能/RSS、论文图表生成均为 `NOT_RUN`。
本轮 A05 RF1-N01 review-fix 的 step/ramp replay 为 `NOT_RUN (0/2)`；A06 与 A07 checkpoint observer 各自为
固定 step/ramp 各 `1/1`。A08 pre-fix factor capture 与 post-fix bounded run 分别各 `1/1`；post-fix Stage 2
实际运行但两条均失败。A09 step/ramp 各 `1/1`、无 retry，Stage 2 收敛并封存 cache；score unavailable，
decision/final、正式 validation/sweep/gate lock/test/RQ、trajectory metrics 仍 `NOT_RUN`。没有额外 estimator replay。
动态 runner exception serialization 未注入执行；C++ exception result 的 `INCOMPLETE` 有断言，实际 runner
contract 覆盖 default/A02/V2 的 `COMPLETE` CSV/JSON。C1–C3 不升级。证据见
[`t10_a05_counter_review_fix_20260909T060028Z`](evidence/t10_a05_counter_review_fix_20260909T060028Z/VERIFICATION.md)。
A06 证据见
[`t10_a06_failed_checkpoint_diagnostic_20260909T065842Z`](evidence/t10_a06_failed_checkpoint_diagnostic_20260909T065842Z/VERIFICATION.md)。
A07 证据见
[`t10_a07_exact_lm_direction_diagnostic_20260909T082434Z`](evidence/t10_a07_exact_lm_direction_diagnostic_20260909T082434Z/DECISION.md)。
A08 证据见
[`t10_a08_factor_jacobian_correctness_20260909T095029Z`](evidence/t10_a08_factor_jacobian_correctness_20260909T095029Z/VERIFICATION.md)。
