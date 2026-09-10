# R08 独立复审与最小下一任务

结论：研究方向正确，已经从单seed调试进入两条新基础轨迹的有限比较；实际运行和负结果有价值。
但只接受“有限validation矩阵执行且失败保留”，不接受工程门完整通过、12输入完整闭环或gate准入。
不能将下一步概括为“只剩LOS，修好后即可锁gate/test”。本轮不修改estimator、不启动优化。

## 独立核验与真实进度

复核 STATUS、R08协议、上一轮任务和合同边界后，先检查原始CSV、源码和失败状态，再参考交付VERIFICATION。
1320项预评价冻结文件哈希全部匹配；冻结核验后读取对应scenario truth，独立复算36份有效final的
全记录/历史ATE RMSE/P95及可用配对差值，同时分别核验decision/final bias RMSE。
[独立审计结果](evidence/t10_a19_r08_independent_review_20260910/AUDIT.json)。
未重建完整GTSAM图、重跑工程测试或独立重算333-bit证书；这些以封存日志和当前源码为依据。

12输入实际为8个SCORED、3个FAILED、1个INTERRUPTED。8个SCORED各尝试5个final，36个有效、4个失败；
其中7个输入有structured与suppress_all配对，另一个参考失败。不能把“执行5个final”写成“5个均有效”。
原表：[INPUT_RESULTS.csv](evidence/t10_a19_r08_validation_compare_20260910T115109Z/INPUT_RESULTS.csv)、
[PAIRED_RESULTS.csv](evidence/t10_a19_r08_validation_compare_20260910T115109Z/PAIRED_RESULTS.csv)。

7个配对中，structured历史RMSE为1好6差，P95为0好7差。turn01陡ramp最明显：
RMSE增加28.183407mm、P95增加43.758707mm；turn02/step1仅RMSE改善0.219404mm，P95反而增加0.256794mm。
三个gate在turn01 step1/2/3实际接受修正，RMSE分别比全抑制高0.453320/0.746240/0.552924mm；
其余可评分输入均Suppress。由此可以说“当前固定配置没有展示重复轨迹收益”，不能说普遍有害或统计显著。
两条LOS无有效final，LOS代价未评价；两条base及其配对场景不是12次独立运动试验。

## 对研究方向更关键的新发现

独立读取8个组的scoring/group_0/N.csv，全部为6400乘单位阵。每段16个观测、sigma=.05m，
该名义信息量相同。故在这批评分上严格有

`eta = lambda_min(R)/6400 = 1/(6400*s^2)`。

复算最大差6.66e-16；eta约.5825–.7482、s约.01445–.01638m，两道当前门槛全部通过。
三gate实际仅由gamma区分Use/Suppress，8/8决策相同。
这不仅是“暂时没看到不同决定”：当前信息尺度下两项分数是确定映射，full规则可等效为另一s阈值加gamma。
不应在同一种N上不断加seed或扫阈值来证明eta有独立贡献，也不应据此全局删除eta。
下一研究检验应按合同加入实际noise/count变化，使N发生变化；弱几何单独改变R而固定N，仍不足以单独
破除这一映射。噪声、观测数、几何各作一个因素，避免笛卡尔积及按结果选场景。
原始分数汇总：[GROUP_RESULTS.csv](evidence/t10_a19_r08_validation_compare_20260910T115109Z/GROUP_RESULTS.csv)。

## 工程与证据问题，按阻塞优先级

1. **LOS链路工程门漏测。** turn01 LOS为已知空C request错误；turn02 LOS已经完成无C Stage2，
   仍因评分器拒绝空support失败。当前[a19_r08_pipeline.cpp](../../tools/paper/a19_r08_pipeline.cpp)第855行
   无条件调用ScoreRefitRecoverability；[nlos_scoring.cpp](../../src/nlos_scoring.cpp)第179行明确拒绝空support。
   这不是数值不收敛，也不是新的方法难题。上一轮已明确要求同生产入口零候选全链fixture，实际门只覆盖
   refit/inference局部测试、no-C recovery/fallback及prepare，未覆盖automatic→empty Stage2→score→final。
   V2又主要检查dispatch源码和prepare，因而继续漏过下一接口。必须补完整路径，不能再靠两个真实LOS逐层探错。

2. **另有两个不同的ramp失败，不能省略。** turn02陡ramp在Stage2返回
   `conditional range metadata does not match graph`，已有17次接受call，非“从未进入Stage2”。
   [nlos_refit.cpp](../../src/nlos_refit.cpp)第1001–1012行将key、常数和残差核对合并成一个错误，
   第413行起RangeFactorMatchesExpected含逐位残差比较。可能是metadata不一致，也可能是表达式求值顺序，
   当前日志未给出失败obs/字段，不能独立认定根因，更不能直接放宽检查。
   turn01缓ramp的四个Suppress final/fallback则真正遇到lambda exhaustion：empty_recovery/outer1已converged，
   outer2为1 call/10拒绝/0接受，handoff=0；这是不同的数值失败。
   原始目录：[turn01缓ramp](evidence/t10_a19_r08_validation_compare_20260910T115109Z/attempts/a10_val_turn_01_seed20101_ramp_gentle/output/)。
   它应保留为可靠性数据，不应借机重开无上限求解器研究。

3. **中断不是零候选。** turn02缓ramp的stage1/status、partition等文件为0字节；汇总器把缺失CSV读为[]，
   导致INPUT_RESULTS候选/段/组等列写0。这些应为UNKNOWN/NOT_EXPORTED，不是观测到的零。
   保存的file_access.trace末端可独立确认Stage1 outer58，尾部含NUL；完整final完成阶段与宿主崩溃根因
   无法从现有有效日志独立核验，目录存在不能证明阶段完成。原失败/中断不应重标成功。

4. **汇总表缺阶段标签。** PAIRED_RESULTS的accepted_bias_rmse_m和bad_correction_rate实际取
   decision_time（[aggregate第174–176行](../../tools/paper/a19_r08_aggregate.py)），与轨迹final列并列但无前缀。
   JSON正确区分两个阶段，独立复算一致；这不是指标造假，但新表必须加decision_/final_字段，不能沿用含混命名。
   本次审计最初按final解读该CSV列时断言停止，查明取值来源后分别复算，记录在AUDIT中。

证据允许接受本轮有限比较；不允许宣布正式gate锁定、LOS无退化、eta增量、held-out或论文完成。
R08实际validation上下文与遗留UNBLINDED_DEVELOPMENT导出标签并存已披露，需版本化澄清而非事后改写旧包。

## 最小剩余路线与停止事项

先闭合两个LOS并做失败分类/汇总修正；随后预登记少量noise/count诊断和有限缓存工作点选择；
只有完整LOS与失败记账、参数/指标锁定后才消耗未见test。真实多链路重复、可信简单基线、独立标定、
外部数据与prefix证据仍未补齐，synthetic验证不能取代它们。
停止：回seed10101；重复同N输入追逐eta增量；为了positive result改gamma/预算/精度；
只测局部模块然后用正式输入探接口；每个ticket生成新schema或一套新runner；把无记录记为零。

## 下一轮可执行prompt

执行 **T10-A19-R09：LOS全链收口与R08证据修正**。目标是取得两条LOS的真实all_range配对结果，
并把其余失败分清；不以更多工程检查代替最终输出，不在本轮锁gate或跑test。

1. 先读STATUS、本复审、R08协议和完整方法/实验合同。登记限定amendment及新执行身份：
   允许两条已使用validation LOS各一次correctness复核运行，历史R08失败原样保留；这不是新独立样本或恢复盲测。
   不回seed10101，不生成/读取30101–30103。保护冻结包与用户未提交改动。

2. 修正runner对空support的分流：完整验证无C graph/Values/mask身份后，score输出明确
   NOT_APPLICABLE_NO_CANDIDATES及空分数组，继续原final/export/evaluator；不把评分器的非空校验直接删掉。
   有候选但无eligible与零候选分开；保留全部候选及Suppress参考。无C禁止inexact handoff，原四项AND不变。
   LOS all_range必须从相同原始初始化、原始全部valid ranges和同数值策略独立求解，不能把Suppress轨迹改名，
   也不能继承discovery优化后的状态冒充原始初始化对照。别再复制另一整套estimator/runner。

3. 科学运行前，同一生产入口的实际小fixture必须走完 raw初始化→automatic零候选→Stage2→score不适用
   →五策略final及独立all_range→export→独立evaluator；另测非空候选零eligible及mixed分支。
   必须能在pre-fix入口复现此次LOS故障、post-fix通过；只做prepare或局部refit测试不算该门通过。
   复用现有ABI/身份检查，只运行受影响回归。统一真实producer/reader的角色解释并保留旧输出历史标签，
   不因R09任务号改schema。工程门通过后自动进入第5步，不停在ready。

4. 用R08冻结材料只读定位两个ramp失败：陡ramp列出失败metadata谓词、obs/factor/key/常数/残差及其
   binary64差值；缓ramp核对最后accepted状态、outer2证书、generic/驻点/外层停止条件。
   若封存内容不足以恢复现场，明确UNKNOWN及最小缺失项；本轮不新增diagnostic solve或重跑ramp，
   不绕过metadata检查、不放宽容差/证书、不引入no-C handoff。不要把两种失败合成一个solver故障。

5. 两条原validation LOS各一次fresh raw执行，固定P1、R08模型/solver/333bit/所有阈值及初始化。
   每条进程树最多700s、无retry，独立目录与新票；每final保持至多一次fallback。
   本轮总预算4000s，其中scheduled硬限3000s、1000s reserve不自动使用，工程/诊断/评价计入scheduled。
   普通失败封存并停止该行，不再现场修复重跑；共享correctness缺陷停批次。不得重跑整个12输入矩阵。

6. 冻结所有估计输出后独立评价，给出两条LOS相对all_range的同时间ATE RMSE/P95/完整匹配数、
   failure/fallback/成本与冻结LOS容忍判据。无候选coverage/risk=UNDEFINED，失败=UNAVAILABLE。
   新corrected汇总用decision_/final_分列；缺失/损坏状态不得转0；每个新运行与R08原失败分列，
   不增加独立样本数、不改写原指标或用成功覆盖失败。若改变非空路径，明确其受影响结果范围，
   不静默把旧成功包当作新实现的验证。

7. 更新STATUS与claim–evidence，回答LOS是否闭合、两个ramp是否定位、哪些门仍不能准入。
   将本复审已证实的8组N=6400I写入后续诊断设计依据；只提出固定预算的noise/count单因素矩阵与
   公平有限缓存选择计划，本轮不执行、不调gate。只有未来不同N下的held-out证据才能讨论eta增量；
   无增量时按冻结论文结构简化gate/收窄claim，不保证eta有效。
   到两条LOS结果或停止边界即封存，不再续接新的求解器支线。
