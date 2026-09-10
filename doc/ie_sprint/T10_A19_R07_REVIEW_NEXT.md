# R07 独立复审与下一步

本复审接受 R07 的限定工程交付与单输入配对 development 结果；结束 seed10101 求解器支线。
不接受正式 gate lock、eta 增量、普遍轨迹收益、跨运动泛化或 C1–C3 升级。本轮不修改 estimator、不启动优化。

## 实际证据与独立核验

先读 STATUS、R07 协议、冻结结构/路线图与合同，再检查原始产物、源码及指标，形成判断后参考交付 VERIFICATION。
独立核验 [387 项冻结 payload](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/PRE_EVALUATION_FREEZE.json)，无 hash mismatch。
核验五个 final 的有效状态、mask、终行四项 AND 和驻点阈值，确认四个 Suppress 轨迹字节一致。
在冻结核验后读取已披露的 evaluation-only truth，使用 NumPy 插值、位置误差和百分位数独立重算全部
配对轨迹指标，以及 structured final 的 bias RMSE。结果见 [本轮 AUDIT.json](evidence/t10_a19_r07_independent_review_20260910/AUDIT.json)。

原始运行目录为 [attempt1/pilot/output](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/attempts/attempt1/pilot/output/)。
Stage1 90 outer、Stage2 24 outer、2 段/32 候选、1 个 eligible group，五个 final 各执行一次且有效，fallback 0。
运行日志报告 exit0、83.586104785 s、estimator forbidden truth open 0；不把复审说成独立重跑。
受影响测试日志实际为 [refit/export 8 项](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/engineering/test_refit_retry.log)
及 [inference/policy 15 项](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/engineering/test_inference_retry2.log) 通过。

| 方案 | 候选使用 | 全记录 ATE RMSE | 历史 [3,6]s ATE RMSE | 历史 P95 |
|---|---:|---:|---:|---:|
| suppress_all | 0/32 | 0.023427203651 m | 0.015116977089 m | 0.021046873577 m |
| structured_debias | 32/32 | 0.024065220556 m | 0.015879789319 m | 0.022902577500 m |
| fit_only / s_fit / full_gate | 0/32 | 与 suppress_all 相同 | 与 suppress_all 相同 | 与 suppress_all 相同 |

同时间配对为全记录41点、历史16点。使用两段修正使历史 RMSE/P95 增加 0.762812/1.855704 mm，
全记录 RMSE/P95 增加 0.638017/0.251497 mm。它回答的是这一个已见输入上的轻微负收益，不能外推显著性或普遍有害。
原表：[PAIRED_RESULTS.csv](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/PAIRED_RESULTS.csv)。

eta=0.5839784741、s=0.0163572990 m 均通过当前工作点；两段 gamma=0.1431107780/1.4164987079，
最大 gamma 超过1，三个 gate 全部抑制同一组。因此 full_gate 并未比 fit_only 或 s_fit 提供不同决策。
structured bias RMSE=0.005767514545 m、bad correction=0/32（epsilon_bad=0.20m），与轨迹负收益并不矛盾。
被三个 gate 拒绝的 good bias corrections 也是32/32；这不能再解释成 gate 伤害轨迹，本次配对反而是抑制略好。
单组32条候选不是32次独立试验；零接受条件风险是 UNDEFINED。候选只占32/328条raw观测，
structured 保留328/328，Suppress 保留296/328。已拟合准幅值不保证候选还能提供足够导航增益，
冻结结构的 navigation-information 注释已经区分这两个问题；本轮未独立量化该机制。

## R07 是否解决了原阻塞

是，在这个输入上解决了。当前 [nlos_inference.cpp](../../src/nlos_inference.cpp) 为 accepted-empty recovery
和 fallback 传递独立 request；[nlos_refit.cpp](../../src/nlos_refit.cpp) 无C分支构造实际 remaining raw-range 常数，
调用 development callback 并拒绝 inexact handoff。runner 为各阶段使用独立日志目录。
原始 [recovery trace](evidence/t10_a19_r07_suppress_certified_compare_20260910T104117Z/attempts/attempt1/pilot/output/final/suppress_all/recovery_refit_iterations.csv)
只有2轮，最终梯度1.3312178737e-7，阈值仍1e-6，四项AND全真；共同参考339 factors/123 X-V-B keys/0 C。
这支持结束本seed的修复，不支持保证所有新场景均收敛。

边界：没有重建完整 GTSAM 图或独立重算每个333-bit证书；build、fixture、file-open审计以封存日志和当前源码为依据。
普通 implementation.diff 是空文件，因为相关源码未被 Git 跟踪；不能用空diff证明没有改动。
当前源码/二进制在387项冻结核验内，但未凭完整前后源码快照逐行证明所有旧路径不变。
R05 truth 暴露永久保留，结果仍 UNBLINDED_DEVELOPMENT。

## 下一步的最小选择

先以固定 P1/固定工作点完成两条新 validation 基础轨迹上的六场景，共12个输入的有限验证，
检查负收益、gamma 全拒绝、LOS 和 ramp fragmentation 是否重复出现；不立即执行旧 A10 的整个
3-support × 12-input 搜索加 fixed-partition 大矩阵。该首批是 validation characterization，不锁gate、不冒充held-out test。
之后才在预声明的有限缓存候选上选择工作点，并在未见 test 基础轨迹上检验；若无增量按冻结结构缩减claim。
低信息/噪声/样本数诊断、fixed-partition诊断、正式基线、真实多链路、外部数据与prefix仍是后续缺口。

不能直接复用 R07 命令批跑：生成器 CLI 只接 development；pipeline 的 Stage2 request 总允许非空C handoff，
且 `eligible != 0` 才运行 final；evaluator 固定 `generation['scenarios']['step']['truth']`。
这些是可以在 fixture 中验证的批量入口限制，不应等每个真实输入失败后再发下一张修复ticket。

## 可直接交给 Codex 的 R08 prompt

执行 **T10-A19-R08：有限 synthetic validation 准入与固定工作点12输入比较**。
目标是取得完整逐输入分数、决策和配对收益表，判断 R07 观察是否重复；不得以接口、文档或预检通过代替实际运行。

1. 先读 STATUS、本复审、R07协议、冻结论文结构/roadmap、完整方法/实验合同，以及
   T10_A10_VALIDATION_ADMISSION.md / T10_A10_SPLIT_PROPOSAL.json。
   接受R07限定结果并结束seed10101求解器支线。保留全部历史失败、A08数值支持域限制与R05暴露。
   先登记本轮amendment：仅开放绑定split的synthetic validation入口及下述有限矩阵；不是默认迁移或test准入。
   更新合同对R07无C策略扩展的交叉引用，不能靠删除历史限制完成“准入”。

2. 输入固定采用旧提案的两条独立基础轨迹 a10_val_turn_01/seed20101、a10_val_turn_02/seed20102，
   每条 LOS、step1、step2、step3、ramp_gentle、ramp_steep，共12个。沿用提案精确运动/噪声/场景定义，
   不根据R07误差重新设计幅值或挑场景。生成前核验这些reservation未被消费或提前评估；若已被使用，
   如实记录用途，不能把旧数据重新标未见。seed10101/10102及其基础轨迹永久development；
   test30101–30103不生成、不读取、不运行。两个validation基础轨迹不代表任意运动泛化。

3. 复用现有解析生成器和 C++/GTSAM。只补参数化运动/六场景、role/split/ancestry/content身份绑定、
   逐scenario evaluator和有限编排。原始传感器文件与truth分离；estimator不得收到truth路径或状态。
   validation上下文必须贯穿真实producer/consumer/final身份，不能只改Python role或把validation伪装development。
   只作必要版本化的role envelope，不重建通用scheduler/迁移历史cache；旧development schema保持可读且不跨role混用。
   补齐零候选/零eligible正常结果：零候选Stage2用无C严格certified路径、禁止handoff；
   有候选但全不eligible必须保留候选、按合同Suppress并得到参考final，不跳过整个结果。
   score不适用与numerical failure分开，不能强造eligible。structured基线遇不适用/失败也据实记账。

4. 科学输入运行前，以同一生产入口的真实小fixture一次覆盖 LOS零候选、存在候选但零eligible、
   多组mixed Use/Suppress、无C recovery/fallback，以及step/ramp按scenario/obs_id正确读取truth的独立评价。
   验证完整identity握手、跨role/错parent/test输入拒绝、no-C handoff拒绝、启动父目录与CMake ABI。
   再做完整raw prepare但不优化。保存实际源码快照和hash，不再把untracked源码的空git diff当修改记录。
   工程correctness问题在此修复；通过后自动进入已登记矩阵，不停在“具备运行条件”。

5. 固定P1支撑参数、R07数值模型与策略、333bit、容差、lambda和四项AND；gate仍eta>=.10、s<=.10m、
   gamma<=1，epsilon_bad=.20m。不扫P2/P3、不调gamma、不增solver预算、不引入新的数值方案。
   每个输入从raw独立初始化，只产生一次自动Stage1/2/score；五策略复用这个输入的同一冻结候选与分数，
   各final至多一次、fallback至多一次。另对两条LOS执行同配置/初始化/数值策略的all_range参考，
   不能拿candidate-suppress reference冒充all_range。已有真实基线入口若不兼容，只补同策略接线，
   不开发新的baseline方法；不能用先验oracle空partition作为自动发现结果。

6. 运行前登记B_total=14400s，scheduled累计wall硬限10800s，3600s保留不自动消费，并将准备/评价成本列入。
   单输入沿用R07自动900s、每final900s和总5400s上限；并行度1，外层累计预算优先，实际remaining不足则不再启动。
   83.586s只是单development输入观测，用它给出明确标estimated的12输入成本及不确定性，
   不把各单项cap的和写成可保证完成的预算。旧A10每进程120s提案不再默认套用，预算变更在启动前登记。
   固定顺序、无自动retry/换seed/失败退款加试。普通数值失败是一个数据点，记录后继续其余独立已登记输入；
   数据泄漏、错误truth关联、身份或共享correctness缺陷才停止受影响批次。预算耗尽行写NOT_RUN_RESOURCE。

7. 所有12输入的score/decision/final与代码、配置、评价器先冻结，再独立读取validation truth。
   evaluator按manifest选scenario及truth，不再硬编码step。报告每条基础轨迹/场景的候选数量、段长、
   short/boundary、group/eligible/unavailable、eta/s/gamma、三gate具体pass位及决策差异、
   全记录/[3,6]s同时间配对ATE RMSE/P95、bias error、bad acceptance、good rejection、三种coverage、
   failure/fallback和成本。必须区分decision-time与final-time，并独立重算选定输出。
   零接受risk=UNDEFINED；失败=UNAVAILABLE；全零候选不当作gate成功。
   LOS额外报告相对all_range差值，沿用旧提案RMSE+.05m/P95+.10m及新增失败的容忍规则，
   同时呈现实际差值，不能因容忍宽松隐藏退化。统计以base为单位，两条base只报逐条结果，
   不把12场景或packet当12/数百次独立重复。相同决定可验证完整身份后复用，但记录真实执行次数。

8. 交付12输入完整表、冻结包、命令/exit/日志、独立指标核对、当前STATUS和claim–evidence，回答：
   修正收益是否跨输入重复？LOS有何代价？三个gate在哪些组做出不同决定？
   全门控相对s_fit是否已有可比较差异？若没有，明确“本矩阵未辨别”，不能宣称eta有效或无效。
   此轮不选择/锁定gate，不跑held-out、不自动追加弱几何或阈值搜索；结束时给出下一次有限缓存选择与
   缺失低信息/模型失配诊断的具体方案。固定分段、正式baseline矩阵、prefix、真实数据缺口继续列为未完成。
   本任务取得表或触及边界后停止；不得返回seed10101继续追求正收益。
