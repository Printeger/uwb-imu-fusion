# T10-A17 限定 amendment / 实施前协议

登记 UTC 2026-09-10T00:22:11Z，先于原型实现和测试。指挥接受 `REVIEW_ACCEPTED_A16_FIXED_ENDPOINT_PRECISION_AUDIT_SCOPE`，来源本指挥会话；独立复审 `/tmp/t10-a16-commander-review-7d4bmy57/REVIEW.md` 已读取并封存。仅接受A16固定端点审计，不是T10完成/solver收敛。T10 IN_PROGRESS；A14失败、A15 124个factor FD失败、A12负结果、A08历史15/18保留。

## 授权实现的唯一边界

实现 `PAPER_CERTIFIED_PAIR_REDUCTION_V1` 默认关闭的独立 C++ paper development 首 conditional navigation block 原型。复用当前raw loader/Initializer/GraphBuilder、同u=0的首conditional graph、现有GTSAM LevenbergMarquardtOptimizer的阻尼构建/solve和native retract，实际trial只解一次，无shadow solve，无Python estimator。Python仅复用A16固定状态的MPFR残差算术并计算区间P/D，可由一个持续certificate子进程服务C++请求；它不持有优化状态、不产生方向、不重新retract。不改/usr/local、production core/legacy入口或依赖。

独立入口要求显式`--policy PAPER_CERTIFIED_PAIR_REDUCTION_V1`和development首block模式；缺省拒绝启动原型，非法值拒绝。原A14 config文件字节不变，策略由独立声明绑定，模型仍PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1。原型不写正式run_status/trajectory/partition/Stage2缓存；产物schema为A17_FIRST_BLOCK_DIAGNOSTIC_ONLY、consumable=false。策略身份覆盖实际config/raw/initial图、源码、factor公式/精度/错误schema及实际MPFR/GMP/core/GTSAM；旧缓存reader对原型产物必须拒绝。本轮不铺开Stage2/cache/final生产集成，不实现scheduler。

## 固定数值语义

按A16唯一草案：333bit MPFR有向区间，所有binary64常量精确导入。P从实际未阻尼linear graph的固定J/r/delta计算`-r^T(Jdelta)-.5||Jdelta||²`；D从同一native端点对的完整原factor公式/full R计算`.5 sum(r0-r1)(r0+r1)`。不减大目标，不计damping为目标；不将native残差的补偿求和当完整误差界。公式/分支来源保留A16定义与近零近似，不正交化，不修改PIM。

P/D半宽均须<=精确十进制1e-15；Phi<=0或Dhi<=0为可证拒绝，按原lambda规则继续。区间跨零或过宽为NUMERIC_REDUCTION_UNRESOLVED并立即结束block；未知factor/未支持近pi/分支不确定为NUMERIC_REFERENCE_UNSUPPORTED；非有限为NUMERIC_REFERENCE_NONFINITE。证书/IPC/身份损坏显式失败，无无证书接受/精度重试。

仅在Plo>0、Dlo>0并证明`Dlo/Phi > 原binary64 minModelFidelity(1e-3)`时接受。ratio下界除法RNDD，上界RNDU；比较在MPFR执行；跨门不确定，明确上界<=门则拒绝。向GTSAM lambda更新传入的fidelity用mpfr_get_d(RNDD)，输出hex/bits并验证转换值不大于认证下界且仍严格大于原阈值；转换无穷/下溢到门内则显式失败。保留原lambda更新公式；当前useFixedLambdaFactor=true仍完整核对，不用普通double除法作证书。

保留原SEQUENTIAL_CHOLESKY、isotropic damping、lambdaInitial=1e-5/factor10/lower0/upper1e5和原默认值；实际参数在测试前读取校验，遇到不同拒绝而非改参数。V2 internal rel=0，外部generic rel=1e-6/abs=1e-8/errorTol原值；原AuditNavigationStationarity tol1e-6/roundoff8/五尺度1不变，generic AND stationarity才首block合格。接受一次trial不等于Stage1收敛。计数分别保存calls/solves/trials/accepted/rejected/unresolved；认证失败trial不是可证拒绝，必须单列。

## 工程门（先冻结，不因失败放宽）

- 同A16初始/base/trial graph/Values hash和371factor、123key/615delta、1886native残差严格一致；静态333bit D与A16区间重叠且点差<=1e-18，P与独立精确有理数J/r/delta求和的结果包络相符，与A15稳定GN差<=1e-20；P/D半宽<=1e-15，完整逐factor证据。
- 同base/lambda=1.0000000000000002e-14（已核对A15 trial delta表，按封存binary64精确恢复）使用原GTSAM实际单次solve；615个delta和native端点逐位等于A15/A16。该静态回归允许一次原方向重建，不构造第二shadow optimizer/求解，不沿checkpoint运行优化；pilot禁止warm start。
- 有效算术反例：正/负/零D及P、精确大目标相消、native假下降被证书拒绝、fidelity低/等/高于门及跨门、过宽/非有限、未知factor/近pi/区间分支不确定；保守ratio及RNDD binary64转换用精确有理数验证。固定精度，不调判据。
- 原默认策略和V2/驻点/计数的现有focused regression，独立原型不启用时native路径与源码/库hash保持；默认关闭/非法policy/development作用域拒绝，旧Stage2 reader拒绝原型schema。至少一个工程小图对照native同调用数/停止/计数（非本场景额外pilot）。不因工程门失败扩充科学预算。
- 构建/接口/测试实现错误可以修正并保留失败；任何未解决工程门失败使pilot NOT_RUN，不为通过放宽精度/阈值。全部门通过后自动运行唯一pilot，无需再次询问。

## 唯一pilot预算与停止

只用A14原P1 step seed10101 raw/config，显式新policy，从原raw重新初始化。外层配置outer500/inner50/refit200原样保留，但执行范围只outer1首conditional block，绝不调用chain/Stage2。最多50次iterate调用、原lambda上界与内部V2规则，整进程外部硬超时120秒（含certificate子进程），一次、无retry/预算追加/参数或精度搜索。

首block generic AND stationarity成立、显式失败、lambda耗尽、50call上限或timeout即停止；结果全部保留，未运行阶段NOT_RUN，未评候选/段/组/eligible/score为NA。原型不生成正式缓存；validation/test/T11/gate/final/scheduler NOT_RUN，C1–C3不升级。更新STATUS/READINESS/CLAIM_EVIDENCE、归档命令/source/raw/config/actual libs/失败后停止等待review。

实施前勘误：初稿把checkpoint lambda暂写为1e-6；读取A15 call17 trial1原始delta账本后确认1.0000000000000002e-14，现已更正。此时尚未实现/编译/执行任何A17原型或数值测试；不是结果驱动变更。初稿保留为PROTOCOL_INITIAL_TYPO.md。查找不存在的conditional_lm_trials.csv一次exit1也保留，实际lambda来自conditional_lm_trial_deltas.csv。
