# R06 独立复审与 R07 最小任务

2026-09-10。结论：**完整自动评分已完成；structured_debias有真实final及绝对误差；共同参考失败使配对
轨迹收益仍不可得。** 接受该限定 `UNBLINDED_DEVELOPMENT` 里程碑，不接受“修正改善轨迹”或“eta有效”。
本轮没有新跑estimator、修改实现或求解参数。

## 独立核对的结果

R06根目录：`doc/ie_sprint/evidence/t10_a19_r06_live_c_score_compare_20260910T092530Z/`。
本轮先核验amended freeze的119项payload hash，再从冻结输出重算小矩阵谱，以及在冻结后读取已披露
evaluation truth独立重算轨迹和bias误差，详见
[AUDIT.json](evidence/t10_a19_r06_independent_review_20260910/AUDIT.json)、
[可复现脚本](evidence/t10_a19_r06_independent_review_20260910/audit.py)。脚本exit0，未调用任何优化器。

- 完整Stage2导出125 keys/1027 scalars，X/V/B各41、C为2；所有hex与binary64 bits一致。
  24份handoff严格JSON有效，inner_converged与各block_status一致。没有独立重建整个GTSAM终态图重新
  求梯度，本检查不代替该项数值证明。
- 保存的 `scoring/group_0/N.csv,R.csv` 复算
  `eig(N)=[6400,6400]`、`eig(R)=[3737.462234469297,5455.245266518915]`，
  eta=`0.5839784741358278`，s=`0.01635729904348021 m`，与生产分数一致。
- `segments.csv` 的两段gamma为 `0.1431107780`、`1.4164987079`。组内最大gamma超过1，故整个组
  Suppress；eta>=0.10与s<=0.10m均通过。三个gate决定一致，这次拒绝由gamma驱动，不能归因于eta。

| 策略 | 冻结决定 | 候选接受 | 实际final | 配对轨迹收益 |
|---|---|---:|---|---|
| structured_debias | Use两段 | 32/32 | OK，无fallback | 无共同参考，UNAVAILABLE |
| suppress_all | Suppress两段 | 0/32 | recovery与fallback均失败 | UNAVAILABLE |
| fit_only / s_fit / full_gate | Suppress两段 | 各0/32 | 三者各自recovery与fallback均失败 | UNAVAILABLE |

structured_debias独立复算：完整41点 raw-frame ATE RMSE=`0.024065220556 m`、P95=`0.039408040294 m`；
历史[3,6]s共16点 RMSE=`0.015879789319 m`、P95=`0.022902577500 m`。
final accepted bias RMSE=`0.005767514545 m`，bad-correction=`0/32`（epsilon_bad=0.20m）。
这只能说明该已见truth输入上修正幅值误差较小、产生了一条有这些绝对误差的轨迹；没有证明比抑制候选更好。

原evaluator的decision-time good-correction rejection也被复算支持：三个gate都拒绝32/32条按固定
bias-error阈值判为good的候选。其意义是此输入上的保守拒绝；bias-good不等同已证实trajectory-beneficial，
不能据此临时放宽gamma。32条来自2段/1组/1个seed，不是32次独立泛化试验。

coverage需要分清分母：总raw观测328，候选32，候选占比32/328；报告的candidate_use_coverage=1指
structured_debias使用32/32候选，并非全部328条都是候选。它的overall_retained_fraction为328/328。

## 共同参考失败：比“200轮不够”更具体

原始证据位于
`attempts/attempt2/pilot/output/final/suppress_all/recovery_refit_iterations.csv`及对应fallback CSV。
第1轮梯度降至1.10794e-4，第2轮降至 **1.1159186055e-6**，随后直到第200轮目标和梯度保持相同；
第3--200轮conditional_lm_iterations=0，记录的lambda达到1e5。最后objective/step/KKT均通过，仅导航
驻点不通过：1e-6+1.3320362282e-10小于当前梯度。因此不能把它改标成功，也没有依据直接增加outer上限。

四个Suppress策略的recovery/fallback共8条200行数值轨迹一致，inference ID也一致。它们不是8个独立的
算法失败机制，而是同一决定、同一路径的重复执行；保留各行失败分母，同时不要据此作独立重复实验统计。

源码静态审查找到了明确的policy覆盖差异：

1. `src/nlos_inference.cpp:750` 只在development request存在且accepted非空时调用development refit。
   accepted为空时落入常规 `RunFrozenCandidatePolicy`。
2. `src/nlos_refit.cpp:684-699` 的空C分支直接构造默认CheckedLmOptions并调用
   `RunCheckedConditionalLm`，绕过development certified callback；默认policy为GTSAM_CHECK_ONLY_V1。
3. `src/nlos_inference.cpp:872-875` 的fallback也无条件调用常规refit。

因此R06的Use与Suppress处理没有完全共用同一种数值接受策略。当前日志与旧LM在微小变化时停止、外层
反复重新启动的解释相符。**尚未封存该无C失败端点的P/D证书，不能证明改用certified路径必然收敛。**
这属于先前默认/无C边界刻意保留的覆盖范围，下一步需要明确development scope amendment，不能静默
将其宣称为原协议已授权的默认行为或纯日志修复。

## 唯一下一步

限定扩展现有certified P/D接受与严格驻点检查到 **development final的全部抑制/无C recovery和fallback**。
不新增求解器，不改最终阈值，不提高200轮预算，不加先验/抖动。无C问题没有可交回的偏置更新，
**禁止inexact handoff**。Stage1/有C的Stage2及旧legacy/default调用不变。

先用小型真实candidate-excluded图证明该精确分支实际调用certified callback及fallback；随后一次原输入
完整development运行取得同一政策下的共同参考和配对表。不把R06失败状态或Stage2联合轨迹冒充参考。
本轮先解决计算可比性，保留gamma导致的全组拒绝这个观察；不要同时调gate掩盖该现象。

本seed继续标UNBLINDED_DEVELOPMENT。R05暴露及R06 hash-prefix-only evaluator amendment保留；本次
核验119项冻结文件一致，独立读truth在此之后。未证明盲测、泛化、统计显著性、LOS无退化或eta增量，
C1–C3不升级。取得配对表后应离开本seed的工程修复循环，转有限预登记validation与未见数据证据。

## 可直接交给 Codex 的 R07 prompt

执行 **T10-A19-R07：统一全部抑制分支的development数值policy，取得共同参考与配对收益**。
工作目录 `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie`。
先读STATUS、冻结结构/roadmap、完整METHOD/EXPERIMENT合同、R06协议、本复审及AUDIT.json。
保护dirty worktree、R06全部成功/失败结果、truth事件与冻结manifest。此prompt授权下面的限定amendment、
实施、工程验证及通过后一次完整运行；不只交计划或测试通过，不重复请求同范围授权。

1. **登记唯一scope扩展。** 将既有 `PAPER_CERTIFIED_PAIR_REDUCTION_V1` 用于development final的
   accepted为空、candidate全部抑制的无C recovery/fallback。显式记录原路径为check-only，新路径为
   现有333-bit P/D+原generic AND navigation stationarity；新实现/运行/库身份如实更新。
   原stage1、非空C Stage2/其handoff、score/gate、IMU模型、beta、方向/retract、lambda、初始化及
   最终四项AND不改。旧无development request调用保持兼容；不因任务R07自动更改payload schema。

2. **补全三处真实调用链。** FinalInferenceEngine的空accepted recovery要保留显式development request；
   SegmentRefitter空C路径在opt-in时执行同一个certified callback；fallback也使用匹配的development
   policy，而不是回退成旧check-only。传入实际共同参考图中的全部remaining raw range常数与metadata，
   候选32条仍排除，不能把它变成all-range估计。封存输入对应339 factors/123 X/V/B keys、0 C，先核对
   实际图再执行，不靠硬编码计数替代身份审计。无C绝不允许INNER_NUMERICAL_STALL_INEXACT交接或把
   lambda exhaustion、0 accepted、gradient超标改写为收敛。原900s/call/outer上限均保持。
   recovery与fallback使用独立输出命名空间，避免重复outer目录覆盖；fallback仍最多一次。

3. **只运行直接必要工程门。** 真实小图构造非空candidate partition但accepted为空，检查被排除obs、
   raw factor/Values对应、实际callback计数与policy，不用mock/check-only冒充certified执行。
   验证recovery正常、真实或受控失败后fallback走同一policy，日志相互隔离；梯度不达标/未决证书/身份
   错误仍失败，default路径不受影响。用现有封存Stage2构建相同candidate-excluded静态图可作不求解
   的图/身份检查，不能把R06缺失的参考终态伪造成可用checkpoint。相同CMake构建、prepare握手和生产
   export/score/final/evaluator现有回归在覆盖当前改动时复用，其他受影响项实跑并记录非零测试数。

4. **工程门通过后一次fresh全流程。** 新ticket/新输出，从原冻结P1 step seed10101完整raw原始初始化，
   不从旧失败轨迹续跑。Stage1 outer<=500、conditional<=50、Stage2 outer<=200、自动阶段<=900s；
   每final<=900s（含一次fallback），final总<=4500s、全树<=5400s。Stage1/Stage2/score应保持原数值
   定义；如出现非预期差异先解释是否触及了非目标路径，不临场改参数。只有同一最终图/Values真通过
   所有停止条件并完整导出才算有效参考。

5. **冻结原五策略与工作点。** suppress_all、structured_debias、fit_only(gamma<=1)、
   s_fit(再加s<=0.10m)、full_gate(再加eta>=0.10)，epsilon_bad=0.20m。不得因为已知32条bias-good
   被拒绝而放宽gamma/eta/s，不做sweep。五行都保留；只有完整decision/mask/目标/初始化/实际solver
   身份等价时才复用共同参考并标SAME_DECISION及真实执行次数。R06历史失败不改标为R07成功。

6. **完成对比再停止。** 所有可运行decision/final与代码/配置hash冻结后，独立evaluator读取truth。
   estimator始终不读GT/oracle；本seed继续UNBLINDED_DEVELOPMENT，不恢复独立性。
   输出全记录及[3,6]s共同时间关联下ATE RMSE/P95/matches、相对suppress_all差值，以及decision/final
   bias RMSE、bad-correction、good rejection、候选占比、候选使用率、retained fraction、failure/
   fallback和成本。bias-good不等于trajectory-beneficial，0/32不能称总体零风险；三个gate同决定就
   明确该输入无eta增量比较。
   若无C在相同严格policy下仍首次失败，封存实际graph/Values/最后接受状态、完整P/D、lambda和梯度
   证据后停止受影响分支，不再加outer/放宽1e-6/做高精度新estimator。失败也必须进入五策略表。

交付最小policy覆盖变更及amendment、真实分支回归、实际命令/退出码/身份/计数、参考及structured
结果的配对表或首次失败证据，更新STATUS/CLAIM_EVIDENCE。只有配对数据存在才回答轨迹改善/恶化。
成功后结束该单seed工程支线，下一研究任务转预登记有限validation/held-out设计，不继续以此seed调gate。
本轮不锁正式gate、不升级C1–C3、不增加新场景或依赖。
