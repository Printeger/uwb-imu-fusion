# T10-A14 amendment 与运行前协议

状态：TECHNICALLY_ACCEPTED_OPT_IN_DEVELOPMENT_IMPLEMENTATION_ONLY；登记于2026-09-09 UTC，源码实施及测试之前。
指挥接受 `REVIEW_ACCEPTED_A13_DIRECTED_IMU_COVARIANCE_AUDIT_SCOPE`，技术接受[A13唯一草案](T10_A13_AMENDMENT_DRAFT.md)。不是效果通过、默认迁移或validation准入。T10 IN_PROGRESS；A12负结果与A08历史15/18限制保留。

## 模型 amendment
唯一 opt-in `paper.imu_covariance_model: PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1`；缺省/legacy `LEGACY_GTSAM_COMBINED_DEFAULT_V1`。
biasHat是固定预积分线性化点，Bi/Bj仍未知live状态；不另设独立逐采样积分bias噪声，所以显式K=biasAccOmegaInt=0。
不是已知真实bias、不是posterior归零。Qa/Qg/Qba/Qbg/Qi、完整15维交叉项、初始化/priors、solver/GTSAM/容差/传播近似不改。
不开放任意K；未知模型显式拒绝。legacy GraphBuilder/ImuPreintegrator默认保留I6。
身份绑定实际六矩阵、gravity、tangent/Rot3 convention、采样与native传播版本；加入common、discovery/support、Stage2 producer/request兼容和final context。
raw/cache/obs_id不变；全部模型相关结果与checkpoint跨模型失效。既有cache可保留读取审查，未绑定新语义的旧cache不允许warm start。

## 工程验收门（先冻结，失败不放宽）
- legacy A12初始/终态graph和Values四个SHA精确一致，123 keys/371 factors/40IMU/328UWB。原目标atol=2e-12,rtol=2e-12，五类梯度atol=2e-10,rtol=2e-12；类型/keys精确。
- 两模型mean、biasHat、初始Values及未白化residual/Jacobian精确一致（同环境）；不比较跨模型objective优劣。
- 每个实际PIM新Sigma与A13封存五个非K源矩阵直接求和：maxabs差<=1e-15+1e-10*maxabs(expected)。全225项比较，不以总矩阵减K代替。
- 对称误差<=1e-15+1e-12*maxabs(Sigma)；原矩阵最小特征值>=-1e-12*谱范数；LLT成功、R Sigma R^T-I maxabs<=1e-8。不加jitter，不删交叉项。
- constructor/Reset后K严格0或I6、Sigma清零，按原40段/样本重放均值和Sigma精确一致。
- native Values.retract：原A12冻结完整弱方向（615坐标，不替换成求解方向）用于工程导数回归，仅在恢复的历史checkpoint静态评价，不作为新模型warm start。
  步长固定{1e-4,1e-5,1e-6}，逐factor残差中心FD对Jd：maxabs差<=1e-6+1e-5*maxabs(Jd)；图级目标中心FD对g.d：差<=1e-3+1e-5*abs(g.d)。全部结果保留；单坐标不替代完整方向。
- 只改模型必须改变common/support/Stage2/request/final身份；实际cache兼容入口拒绝交叉模型及缺失绑定的旧cache；非法配置拒绝。
- 重建core/runner及受接口影响tests；运行IMU、GraphBuilder、config、paper input/methods/cache、discovery/refit/inference必要回归（工程fixtures，不运行其他recording/held-out）。记录命令、退出码及实际映射库。
任一数值或工程门未通过：科学pilot NOT_RUN，保留具体失败，不临时改判据或策略。

## 条件式唯一development运行（事先固定）
仅当上述门全部通过，运行原A10 P1 step seed10101 raw，原config追加唯一paper模型字段。
从原raw初始化；automatic discovery/refit/score，outer500/inner50/refit200，原V2和四项AND、support/short/boundary不改。
一进程，timeout --signal=KILL 120s，无retry。observer可用既有outer1诊断且不启用first-block budget或shadow optimizer。
自然到达chain/Stage2/score允许；final_inference保持false，无gate决策/final sweep。未到阶段NOT_RUN，未评计数NA。
不读取truth/GT，不生成数据、不运行LOS/ramp/P2/P3/validation/test/T11，不实现validation scheduler、不锁gate、不升级C1–C3。
证据：`evidence/t10_a14_conditional_imu_20260909T142807Z/`。
