# T10-A13 定向静态审计协议

登记UTC: 2026-09-09T13:54:44.728469+00:00

指挥接受 `REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE`。这是A12限定诊断接受，不是estimator成功、validation准入或外部REVIEW.md。T10 IN_PROGRESS，保留A12两臂不驻点/B未达预登记改善的负结果，A08历史15/18及全部历史限制。

只用原P1 step seed10101 raw/config、A12恢复过的A11 call50 Values，以及A12 stationarity_physical/weak_index=1的冻结615维方向。禁止truth/GT读取或hash、新数据、其他场景/参数搜索、chain/Stage2/validation/test/T11。零optimizer构造/iterate，无求解预算增加；诊断仅C++既有GTSAM图/PIM和静态线性代数，不新增estimator。

实施为A12恢复代码的独立静态扩展，保留原文件：相同loader/init/GraphBuilder/first conditional替换、完整key/type检查。初始graph/Values及call50 graph/Values四个内容SHA精确匹配A12方可继续；目标/五类梯度按A12原容差核对。失败停止，不以近似恢复作结论。恢复前后身份一致，保持live graph/Values不变。

对实际40个CombinedImuFactor只读导出PIM参数、biasHat、deltaT、完整15x15预积分协方差、实际noise model R、未白化Jacobian与原白化Jacobian。跟踪本仓构造/Reset/IntegrateBetween、实际linked GTSAM update/propagation/Gaussian whiten。原生PIM副本仅用同一raw样本/时间/参数重放预积分，核对末态与实际PIM，非另一个图或估计器。

沿同一native update产生的F，线性分解Sigma_s <- F Sigma_s F' + Q_s。源分别为measurement_acc、measurement_gyro、integration、bias_RW_acc、bias_RW_gyro、biasAccOmegaInt_acc、biasAccOmegaInt_gyro、其acc/gyro_cross；所有15x15交叉项保留。禁止通过改变参数、置零参数或构造替代factor进行消融。总和必须匹配实际PIM covariance（maxabs <=1e-12+1e-10*maxabs(actual)），均值/时间也必须匹配（1e-12+1e-10*scale）。本审计不纠正上游传播公式或遗漏项。

冻结方向u按原615列映射，||u||2=1（tol1e-12）；每factor kappa_f=||J_f u||²，和应匹配A12 sigma_min²（1e-10+1e-8*scale）。导出全部371factor及5x5导航类别交叉曲率。IMU使用原完整Sigma和原未白化方向z=H u，v=Sigma^{-1}z；静态线性代数仅解15x15 covariance系统，不解导航状态。covariance来源归因为a_s=v'Sigma_s v，其和=kappa_f；这是固定完整precision下的敏感性/分解，不是把各Sigma_s逆相加，也不代表来源独立提供信息。完整残差block交叉项、R Sigma R'≈I及原白化一致性核对，tol1e-10+1e-8*scale。若不通过，归因标不可用而不放宽判据。

静态C++采集一进程，外部120s；离线矩阵审计60s。编译/诊断接口错误保留并可修；不增加任何optimizer调用。查明单位/dt缩放后给出一致/不一致/证据不足结论；如不一致，仅交付PROPOSED amendment和唯一有模型依据的最小修复方案，不修改生产协方差/默认/solver/GTSAM。不把I6或大条件数本身视为bug。完成即停止。
