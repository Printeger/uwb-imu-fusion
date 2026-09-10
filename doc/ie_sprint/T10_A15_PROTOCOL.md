# T10-A15 运行前协议

登记时间：2026-09-09T15:26:33Z，早于实现、构建及唯一捕获。
指挥接受 `REVIEW_ACCEPTED_A14_OPT_IN_IMU_MODEL_AND_BOUNDED_PILOT_SCOPE`；来源为本用户指挥会话，非外部 REVIEW.md。仅限定交付接受；T10 IN_PROGRESS，保留 A14 pilot failure、A12 阻尼负结果、A08 15/18 和 C1–C3 限制。

唯一输入为 A14 原 P1 step seed10101 config/raw，新 IMU opt-in 模型。原 V2 outer500/inner50/refit200、所有容差/初始化/模型/依赖不变。从 raw 初始化，一个 estimator 进程，硬限120秒，无 retry。仅首次 conditional block，结束后无条件阻止 chain。零 InspectFirstLinkedLmTry、零 shadow optimizer、零 budget extension。

新增默认关闭 passive terminal capture 观测模式：原生 TRYDELTA binary64 max_digits10，全 calls/trials/base/Values/delta 和分支 stdout。解析、导出、驻点审计不参与决策。全部捕获后先核对 A14 初始 graph/Values hashes 严格相等、123 keys/615 dim/371 factors（40 IMU/328 UWB/3 priors），17 calls/16 accepted/42 trials/26 rejected、末态 E=2325.3170707993281、lambda=100000.00000000007。目标容差2e-12，梯度各分量容差2e-12 absolute + 2e-10 relative，计数严格相等。失败则停止静态复算，不重试。源码/ELF变化单列，不据此替代内容身份。

通过后静态检查只选 call16最后accepted、call17首个与最后拒绝actual delta，以及末态最大尺度梯度坐标。完整方向用 u=delta/||delta||2（原实际delta同时用于真实trial复算），所有 X/V/B物理尺度沿用1 rad/m/(m/s)/(m/s²)/(rad/s)，native retract。

FD步长固定 {1e-3,3e-4,1e-4,3e-5,1e-5,3e-6,1e-6,3e-7,1e-7}，不追加。逐factor白化残差Jacobian方向误差阈值：maxabs(FD-Ju) <= 1e-6 + 1e-5*maxabs(Ju)。目标方向导数阈值 abs(FD-gTu) <= 1e-7 + 1e-5*abs(gTu)，同时报告原 graph.error 差分和逐残差恒等式累加。每步独立判定；不得因某一步通过遮盖失败。支持导数一致性须残差检查在至少三个相邻步长同时通过；目标检查未达到此要求记未被FD确认，结合残差证据与消减/舍入审计判定证据边界，不放宽阈值。

完整保存基点/实际trial/各FD正负白化残差、基点白化Jacobian、逐factor贡献及映射；比较 linked objective/linearized subtraction、sum 0.5*(r_old-r_new)*(r_old+r_new)、-sum(r*Jdelta)-0.5 sum((Jdelta)^2)，并输出 long double 累加。long double 仅改善累加，不能消除binary64残差求值误差，非高精度真值，不接入接受决策。linked epsilon*oldLinearizedError、minModelFidelity、relativeErrorTol及实际分支均追溯实际库源码。

原 generic AND stationarity 保持；末态近门槛不算成功。静态程序零optimizer iterate/solve。所有未到达阶段NOT_RUN、未评数量NA；不读truth/GT、不跑其他场景/Stage2/gate/validation/test/T11。交付唯一最小后续建议；改变数值/接受/停止语义须另交PROPOSED amendment，本轮不实施。
