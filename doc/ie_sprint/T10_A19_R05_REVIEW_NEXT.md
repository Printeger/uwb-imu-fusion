# R05 独立复审与 R06 最小任务

2026-09-10。结论：接受**完整输入 Stage2 收敛的日志级里程碑**；完整交付仍为
`CHANGES_REQUESTED_OUTPUT_AND_AUDIT`。本轮未修改估计器、未重跑优化或读取评价truth。

## 当前完成了什么

R05首次真正越过R02 outer15：Stage1完成90轮，发现2段/32候选；Stage2只在outer15发生一次受限
handoff，继续到outer24，记录的最终联合四项AND全部成立。不能继续把当前问题称为“求解器仍卡outer15”。
当前支持域内不再扩展求解器支线，下一步应转向输出完整性、真实评分与收益比较。

本轮实际用Python重算了封存CSV/JSON计数、终行阈值关系，校验了封存结果hash及当前实际runner/core/
GTSAM/MPFR/GMP身份，见
[READ_ONLY_AUDIT.json](evidence/t10_a19_r05_independent_review_20260910/READ_ONLY_AUDIT.json)及
[审计脚本](evidence/t10_a19_r05_independent_review_20260910/read_only_audit.py)。

R05封存根为 `doc/ie_sprint/evidence/t10_a19_r05_identity_score_compare_20260910T083223Z/`。
终行来自 `attempts/attempt1/pilot/output/stage2/outers.csv:25`：

| 最终联合检查 | 实际记录 | 原阈值 | 判断 |
|---|---:|---:|---|
| relative objective change | 1.0840894946e-15 | 1e-8 | 通过 |
| scaled step | 9.4781582582e-10 | 1e-6 | 通过 |
| projected gradient/KKT | 1.7746221616e-12 | 1e-8 | 通过 |
| navigation gradient | 6.3933990901e-7 | 1e-6 + 1.8539714947e-10 | 通过 |

outer23梯度仍为1.5369014050e-6，记录没有提前判收敛；outer24内层自身也正常收敛。
逐block合计与汇总一致：Stage1 327calls/468trials；Stage2 96calls/189trials，95接受、94拒绝、0未决。
outer15记录的12个试步P/D、步长、allowance满足交接保护条件，inner当时仍为false，没有把交接写成内层收敛。

**核验限制：**最终graph/Values没有完整导出，本轮不能独立恢复同一终态并重新计算目标、梯度或KKT。
上述是交叉检查运行记录及实现控制流的结果，不是独立数值重算终态，更不是已证明修正有效。
score、eligibility、Use/Suppress、五final和收益评价全部NOT_RUN；没有决策时accepted risk为
`UNAVAILABLE_NOT_RUN`，不能套用“已完成决策但零接受”的UNDEFINED解释。

## 需要修复的具体问题

1. **Stage2导出器不支持live C。** `tools/paper/a19_r04_pipeline.cpp:425-452` 在成功refit后先导出
   Values，再生成factor/content identity，随后才调用ScoreRefitRecoverability。
   `tools/paper/a17_graph_io.h:67` 只支持x/v/b，其余直接throw。遇到scalar c即退出，进程exit2/82.672844s。
   这是收敛后的I/O故障，不是数值失败。
2. **旧报告对残缺文件的描述不准确。** 实数文件 `stage2/final_values.csv` 只有246行，类型全部是B，
   共41个bias key；没有X/V/C。因此不能称为“已保存全部X/V/B、只缺C”。factor_metadata及content_identity
   尚未写出。这份文件不能作为可消费checkpoint，不能跳过检查或补填C后继续宣称原运行完整。
3. **handoff审计同时有格式和语义错误。** 24份中23份未触发文件含裸`nan`，严格JSON解析失败。
   `tools/paper/a18_optimizer.h:101` 又把所有文件的inner_converged硬编码false；这23个block_status的
   converged实际为true。不能只把nan替换掉就算修好，必须让字段与真实block状态一致，区分“guard未评估”
   和“inner未收敛”。这些错误目前没有推翻CSV/独立block_status的收敛记录，却妨碍可靠自动审计。
4. **工程链没有调用出错的生产导出路径。** `test/test_nlos_refit.cpp` 的R05直接握手测试在得到refit后
   直接进入score，并用既有final exporter写最终产物，跳过runner的Stage2 Values导出。该测试还明确
   使用engineering check-only回调，不能称为生产certified callback全链验证。它确实支持接口握手，
   却没有覆盖此次失败点。下一轮必须用真实含C的Values调用同一个生产writer/reader及后续函数。

## truth事件的处理

`EVALUATION_TRUTH_BOUNDARY_INCIDENT.json` 披露：代理在decision/final冻结前查看generation manifest及
部分step bias truth；文件记录可执行程序/阈值此后未改变，estimator访问记录为0次truth打开。
本轮只读该事件记录，没有打开truth，也未独立重建代理原始访问过程。

不能把该事件说成estimator已经读取GT，也不能因此抹掉收敛工程记录。按R05协议，该run不可用于独立收益
评价。换ticket重跑同seed也不会使已看过的truth重新未知。后续同输入可在**披露已见truth、冻结全部方法/
工作点、仅修I/O**的条件下做非盲development描述性比较；不能称独立validation、盲测或eta泛化证据。
真正独立验证留给方法冻结后的未见数据，不在本轮临时新增场景或通过换seed追好结果。

## 下一步给 Codex 的 prompt

执行 **T10-A19-R06：完整Stage2输出与真实评分/收益比较**。
工作目录 `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie`。先读STATUS、冻结结构/roadmap、完整两份合同、
R05协议、本复审和READ_ONLY_AUDIT.json。保护dirty worktree、R05所有残缺/失败文件与truth事件记录。
本prompt授权以下I/O及审计修复、必要工程门和通过后一次fresh非盲development运行，不再扩solver支线。

1. **先登记范围与证据标签。** 仍用原精确development schema、默认关闭、consumable=false；若新增
   可选审计字段需明确版本/兼容性，不因R06任务编号盲目更名。更新真实source/library/implementation/run
   身份与新ticket。声明本seed已有truth暴露，后续只作`UNBLINDED_DEVELOPMENT`描述性比较，不能独立
   validation或升级claim。不得把已知信息重新包装为未见数据。

2. **只修输出，不改数值方法。** 支持实际Stage2 X/Pose3、V/Vector3、B/ConstantBias、C/double完整
   Values，保持稳定key/type/维度与binary64精确值。复用合适现有writer或扩展当前helper，并让reader
   同步支持；不能跳过C、从truth/Stage1幅值补C、用post-fit residual替代C，不能产生pseudo-range。
   对同一refit.graph/Values保存完整Values、factor/support metadata和内容identity，关联实际raw/config/
   重建路径。预验证类型/有限性/key集合及输出完整性；成功文件/清单原子提交，失败残片不得伪装成可消费
   final_values。明确区分solver_converged、export_complete、score_complete，保留真实首次错误。

3. **修严格JSON和审计语义。** 未评估/不适用数值使用null和明确原因，禁止裸nan/inf；真实非法非有限
   状态仍应失败，不能都转null掩盖。inner_converged来自真实CheckedLmResult，guard_evaluated/qualified/
   handoff状态分开；block实际计数与未评估guard字段分开。覆盖正常内层收敛、qualified handoff、guard
   rejection、未评估状态。只改变导出和审计，不改变guard触发、accepted Values、P/D判据或求解行为。

4. **工程门必须走此次失败的同一函数。** 在含非零C、零边界C和多个段的真实小型joint graph上，调用
   生产Stage2 writer→reader，逐key/type/维度/bit比较X/V/B/C，核对graph/Values identity，重新审计
   同一graph的objective/KKT/gradient一致。测试缺key、错误type、unsupported值、非有限值和写失败时
   不发布完整成功产物。使用严格JSON parser检查全部审计文件，并核对inner字段与block_status一致。
   随后从同一小型refit结果沿生产导出→score导出→decision→final→独立fixture evaluator走完成功分支；
   不能绕过生产writer，不能只让另一个测试exporter成功。无需为此重跑完整seed作工程fixture。
   R05已验证的身份握手及R04 CMake/ABI/预算机制，在仍覆盖当前构建时复用，受影响必要检查实际执行。

5. **工程门通过后直接运行一次完整raw。** 新identity、新目录、新ticket，冻结P1 step seed10101
   ORIGINAL_RAW_NO_CHECKPOINT。R05残缺文件不作续跑checkpoint。Stage1 outer<=500、conditional<=50、
   Stage2 outer<=200、自动<=900s；原四项AND、handoff、lambda、333bit、IMU模型、L1/TV、partition、
   score和全部阈值不变。保存完整终态且导出通过才进入score，保存全部group的真实eta/s/gamma与
   eligibility/unavailable原因及计算来源，不能把未知填零。首次新失败停止依赖分支，不临场改参重试。

6. **有有效评分就继续完成原比较。** 冻结decisions后执行suppress_all、structured_debias、
   fit_only(gamma<=1)、s_fit(再加s<=0.10m)、full_gate(再加eta>=0.10)。每策略独立900s，共4500s，
   自动加final总5400s，至多原合同一次fallback；等价复用满足原条件，失败/空接受行保留。
   estimator全过程无truth/GT/oracle；开发期间只读raw、接口schema文档和独立工程fixture数据，
   不再提前打开真实evaluation文件甚至表头。可以在读取前编写支持冻结数据合同的评价器。

7. **评价时序与结论必须真实。** 所有可运行decision/final及方法/代码/配置hash冻结后，独立evaluator
   才读取evaluation-only truth，记录读取时间与冻结manifest。报告完整记录/[3,6]s raw-frame ATE
   RMSE/P95/matches及相对suppress_all差值，decision/final bias-field error、accepted RMSE、
   bad-correction、good rejection、coverage、retained fraction、failure/fallback和成本，
   epsilon_bad=0.20m。已完成决策且零接受时risk=UNDEFINED；未运行评价为UNAVAILABLE_NOT_RUN。
   结果注明非盲development，不因换ticket恢复独立性。所有gate同决策时明确无eta增量比较证据。
   若全不可评分，按原协议报告零eligible并停止final，不改工作点求结果。

到完整配对表或首次预登记失败结束；不新增精度/求解器/阈值搜索/场景/held-out研究。交付原失败的同函数
回归、完整Values往返和严格审计证据、命令/退出码/实际身份、真实score及五策略比较或失败记录。
更新STATUS/CLAIM_EVIDENCE，R05事件与错误报告修正说明保留，C1–C3不升级。

## 本轮审计执行记录

`python3 doc/ie_sprint/evidence/t10_a19_r05_independent_review_20260910/read_only_audit.py`
exit0，结果与日志在同目录。首次临时检查程序试图用Python默认JSON parser继续读取裸小写nan，exit1；
改为严格判无效，并只用正则提取已明确的Boolean检查不一致，未修改任何R05文件。未运行新estimator、
终态恢复/数值重算、score、final或truth evaluator；不把本轮文件审计称为重做实验。
