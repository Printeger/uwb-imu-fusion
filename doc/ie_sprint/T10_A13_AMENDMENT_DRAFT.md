# A13 amendment 草案：显式 conditional-on-live-bias IMU 协方差

状态：`PROPOSED_NOT_ACCEPTED_NOT_IMPLEMENTED`。本轮只交付草案，未修改合同生效定义或生产代码。

指挥接受 `REVIEW_ACCEPTED_A12_FIXED_CHECKPOINT_SCALE_AND_DAMPING_DIAGNOSTIC_SCOPE` 只接受A12限定诊断。
T10 IN_PROGRESS；保留A12不驻点/B未达改善判据、A08历史图级15/18及所有历史证据限制。

## 证据与限定结论

[A13静态证据](evidence/t10_a13_imu_covariance_20260909T135328Z/VERIFICATION.md)证明：同一P1 step seed10101 call50
恢复身份通过；实际40个PIM的`biasAccOmegaInt=I6`。Reset不改该参数，而每个dt都会把其对角块与
测量噪声密度相加后注入预积分协方差。相对声明的测量噪声，其acc/gyro源分别大250000/25000000倍。
这不是把初始bias prior传播一次，也不是已声明的bias random walk。

**结论为“不一致”：实际协方差模型含A10 development声明没有覆盖、没有来源或物理尺度依据的额外项。**
A10明确允许“生成bias恒定、估计器仍用非零random walk”的建模差异；该已声明差异不是此次缺陷。
GTSAM允许用户显式选择bias积分不确定性，并不因默认I6就构成通用GTSAM bug。
本审计不证明该项是非收敛的唯一根因，不证明修改后必收敛，更不证明原模型下的A12负结果失效。

## 唯一建议的模型与最小改动

仅提议一个具名、显式opt-in的paper development模型：`PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1`。
其联合likelihood以graph中未知`B_i/B_j`为条件，`biasHat`是用于一阶预积分修正的固定线性化点，
不是已知真实bias。当前`biasCorrectedDelta(B_i)`和15维Combined factor已经保留live bias及其相关性。
不再同时假设一个没有数据/模型来源、每采样独立再抽取的“积分bias选择噪声”。

由**不包含这种额外独立随机源**的模型定义推得`Q_aux=0`；在当前GTSAM接口中，其唯一实现是
显式`biasAccOmegaInt=Matrix6::Zero()`。这是待批准的条件模型选择，不是把未知真实bias或其posterior
不确定性设零，也不是看到结果后试零、1e-5或先验方差。它不宣称消除预积分一阶bias修正的高阶误差。
不能把初始`B0` prior协方差直接填进去：该prior是状态约束，且此接口的对角传播实际按密度/逐采样注入，
既不具有一次性初始协方差传播的时间相关结构，也未提供从prior到该项的单位换算或独立性证明。

其他设置全部保留：

- 测量密度`Qa=(.002)^2 I3`、`Qg=(.0002)^2 I3`，不改raw噪声或生成器。
- bias random walk `Qba=(.01)^2 I3`、`Qbg=(2e-5)^2 I3`及完整15维相关传播保持。
- 原`integrationCovariance=1e-9 I3`保持，并明确登记为单位m²/s的积分近似模型量；不是独立传感器标定。
- 原初始化、启发式pose/velocity/bias priors、原生参数化、目标项集合、solver/容差/停止/预算不变。
  **白化权重和目标数值会改变**，因此不能宣称objective语义完全不变；必须新身份、新证据。
- 不同时修GTSAM传播中的其他近似。其非零`biasAccOmegaInt`交叉块的dt语义不在本次已验证支持域；
  当前交叉参数恰为零，而传播产生的rotation/position/velocity/bias协方差交叉项仍必须完整保留。

## Paper / legacy 实施边界（全部待实施）

1. `ImuPreintegrator`增加显式模型枚举参数；已有两参数调用默认仍为历史
   `LEGACY_GTSAM_COMBINED_DEFAULT_V1`，逐字节复现原参数，包括I6。不修改/usr/local或依赖版本。
2. `GraphBuilder`只增加同一模型选择的传递；默认legacy路径保持。paper runner通过一个经校验的
   显式配置字段（建议`paper.imu_covariance_model`）选择新模式。只允许上述一个新模型，
   不开放任意K数值候选或调参范围；缺省保持历史模型，不能静默迁移已有paper结果。
3. 新模式构造时设置一次K=0；Reset只更新biasHat/清零累计量，不根据优化结果、post-fit residual、
   GT或prior covariance重新设置K。实例化每个PIM时导出实际完整参数及版本。
4. 在METHOD合同base IMU模型说明及EXPERIMENT合同noise/provenance/失效规则中登记上述条件模型，
   明确现有biasRW和积分近似的synthetic/engineering来源。不修改冻结论文或假称真实独立标定。

## 身份失效范围

raw imu/uwb、recording/cache/obs_id、时间计划及生成器不变；旧数据可继续读取。
均值预积分与输入初始化应保持（需回归证明），不能把“预期不变”写成实际通过。

启用新模式后，所有依赖PIM covariance、白化IMU Jacobian的graph/linearization、common preparation、
discovery/refit/score/final结果及估计协方差都必须使用新身份。旧A08/A09 cache、A10/A11/A12/A13 checkpoint
只能作为历史原模型证据，不能混作新模式的warm start、score或matched comparison。
A12固定checkpoint可仅作为专门标注的历史调试对象，不是新模型端到端初始化。

实际代码中`CalibrationContextHash`只覆盖几何/beta；`Stage2RefitConfigHash`主要覆盖refit/solver。
不能仅把新字段写进日志。建议新增规范化`ImuCovarianceModelIdentity`，绑定模型版本、实际六类矩阵
(Qa,Qg,Qi,Qba,Qbg,K)、重力及native preintegration convention；将其纳入common preparation、
discovery/support context、Stage2 producer/cache/request兼容检查与final context。
已有完整graph内容hash会随白化变化，但早期请求/缓存身份也必须拒绝跨模式复用，不能只靠最终hash补救。
这是现有身份接线的最小扩充，不实现validation scheduler。

## 必要回归与验收门（本轮全部NOT_RUN）

- 参数/生命周期：legacy模式I6保持，新模式K0显式固定；constructor/Reset/40段实例的参数与单位可追溯。
  未声明模型/非法值显式失败，不回退为“看起来能运行”的参数。
- 协方差：同原样本与dt核验完整15×15传播、对称性、PSD/可白化及所有交叉项；测量密度按1/dt、
  biasRW按dt、积分项按dt；新模型不存在额外K源。不得对协方差做只留对角或加jitter的补救。
- 均值/雅可比：两模式的预积分均值、biasHat、初始Values、未白化残差/Jacobian应一致；白化值按新Sigma
  改变且通过native retract下定向/逐factor导数检查。若失败先定位，不连带修改solver或GTSAM。
- Legacy回归：保持现有默认的参数、原A12恢复身份及公共数值；重建受ABI影响的core/runner/tests，
  明确不能混用旧头文件二进制。相关IMU、GraphBuilder、paper/config及cache身份测试为必要集合。
- 身份反例：同raw/相同scalar solver而只换模型，request/cache/common/final身份应不同且交叉消费被拒绝。
  保留旧artifact可复核性，不删除或重标历史失败。
- 通过上述静态/工程门后，若需确认估计进展，必须另行预登记单一development运行及固定预算。
  本草案不授予该运行、不自动扩到LOS/ramp或validation/test，不保证eligible、驻点或科学收益。

A13本轮不实施上述改动或回归，不锁gate、不升级claim；交付本草案后停止。
