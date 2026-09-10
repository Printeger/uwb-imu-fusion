# T10-A18 实施前限定 amendment、工程门与预算

登记 UTC 2026-09-10T01:00:00Z，先于实现/测试。指挥接受 REVIEW_ACCEPTED_A17_CERTIFIED_FIRST_BLOCK_PROTOTYPE_SCOPE；来源本指挥会话及 /tmp/t10-a17-commander-review-j5heubp8/REVIEW.md（已封存）。仅接受A17首block原型；T10 IN_PROGRESS，A14失败、A15全部FD失败、A12负结果、A08历史15/18与全部claim限制保留。

## 唯一实现范围

PAPER_CERTIFIED_PAIR_REDUCTION_V1 数学/接受语义不变；实现版本 A18_CPP_MPFR_333_V1。进程内C++直接调用已安装MPFR4.0.2/GMP；333bit有向区间、精确binary64输入、A16/A17五类factor及原SO3近零近似/未支持近pi失败规则不变。保留完整PIM/full R，P来自实际native未阻尼J/r/delta，D来自两个native端点的完整残差配对恒等式。native solve/retract每trial一次，无shadow solve/高精度retract/无证书快速接受/精度搜索，不改/usr/local或依赖。

以显式development请求接入现有AutomaticSupportProvider真实循环；默认入口不启用。请求绑定策略/实现/实际依赖身份，独立于只读diagnostic observer；实际条件导航调用新策略，chain/partition/四项AND原实现原样复用。每次构造range factor时同步传递同一binary64 beta+当前u常量、anchor/lever/raw measurement，不能从首轮u=0推断。返回与落盘schema均标A18_STAGE1_DIAGNOSTIC_ONLY、consumable=false；实际旧cache reader须拒绝，入口在Stage1返回后停止，不调用Stage2/cache/final。不增加生产配置默认策略或通用调参接口。

A17 C++异常改为保留明确NONFINITE/UNRESOLVED/UNSUPPORTED分类；trial计数、失败状态与原因同时落盘，认证失败不算可证拒绝。

## 冻结工程判据

1. 全部43个A17封存pair，零iterate、固定333bit。每pair完整factor集合/常量/端点/实际linear.csv身份检查；新旧P/D区间须重叠，中心差<=1e-18，半宽<=精确十进制1e-15；接受/拒绝一致，fidelity区间比较和RNDD double转换由精确有理数独立核对。不同运算组织可有不同包络宽度，但不得改变公式。
2. 同base/lambda实际native方向和端点逐位一致；用封存pair重建linearization/retract检查，不使用checkpoint初始化科学pilot。非零动态bias工程fixture来自真实provider条件图，核对常数、残差/方向及新旧证书；原生残差公式sanity max absolute1e-8未白化/1e-6白化（A16原判据），方向/端点逐位一致。
3. 正/负/零P/D、相消、假下降、fidelity低/等/高/跨门、区间过宽、非有限、未知factor/不支持分支；必须测试C++异常传播/计数/失败输出。默认关闭、原stationarity与计数、provider链/partition回归实际通过。接口影响的core/runner/tests重建，实际动态库身份记录。
4. 一次完整43-pair静态批次外部wall<=10秒，包含输入加载/证书求值/输出（不把准备计时藏入进程外）。固定输入，不并行分摊成本；环境CPU/MPFR/GMP/编译/库身份登记。实现bug可修正并保留失败；不能放宽门或删pair。
5. P/D各半宽<=1e-15；Phi<=0或Dhi<=0拒绝；跨零/过宽/跨fidelity门立即UNRESOLVED。正P/D且RNDD(Dlo/Phi)>原binary64 minModelFidelity才接受；除法/比较/输出转换均保守，RNDD double必须仍过门且不高于下界；非有限/未知/域或分支不确定显式失败。无证书不继续lambda作为精度重试。

## 唯一科学运行预算

所有工程门通过后自动一次P1 step seed10101，从A14原raw/config重新Initializer；仅显式新数值策略，输出隔离。原新IMU模型、模型/priors/初始化、outer500、每conditional50、原lambdaInitial1e-5/factor10/lower0/upper1e5、V2internal rel0、外部原generic、stationarity/roundoff/尺度、L1/TV/ADMM/partition/全部容差不变。完整Stage1含交替导航/chain；整个进程树timeout KILL 900秒，一次ticket，无retry/warm start/追加预算/参数搜索。Stage1完成或首次失败立即返回；Stage2/score/cache/final/gate/validation/test/T11/scheduler NOT_RUN。

逐outer保存目标/step/KKT/navigation四项AND、conditional calls/trials/接受拒绝/证书成本、失败原因及lambda；只有原四项AND通过才冻结实际partition。失败不称零候选，未评candidate/segment/group/eligible/unavailable为NA；short可由实际partition报告，Stage2 boundary未评为NA。保留全部命令/exit/失败/source/raw/config/实际库身份及truth隔离。工程门未通过pilot NOT_RUN。完成更新STATUS/READINESS/CLAIM_EVIDENCE和归档后停止等待review。
