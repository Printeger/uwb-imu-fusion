# own_vicon 初始化修复后 LOS 流程结果

## 结论

三项初始化修复已经生效，并恢复了 IE/GTSAM 的基础定位输出。`robust_cauchy` 在同一完整 own-vicon
`15-31-28` recording 上首次成功导出918个pose、覆盖完整73.36s，aligned ATE RMSE为 **0.951271 m**。
这回答了“轨迹是否根本算不出来”：旧失败主要发生在错误初始化后的preliminary/raw-reference阶段；修复后
raw reference和Cauchy都能收敛。

Recover-vs-Reject 仍未形成有效配对。producer成功冻结253个候选观测、8段，并使raw reference收敛，
但Stage2固定checkpoint recovery总预算耗尽，未发布Stage2 cache。`suppress_all`和`lcb_fixed_full`因此都
按合同保留`NOT_RUN`，`delta_RR`和`delta_RR_NLOS`均为NA。当前剩余阻塞已经从基础FGO初始化缩小到
Stage2 navigation stationarity/求解预算，不是anchor/IMU数据让所有定位算法都无法输出。

## 修复与实际触发

- Trilateration现在先计算候选点目标，只提交有限且降低range平方误差的步；拒绝步不改变当前位置。
  只有步长、梯度或相对目标变化达到收敛且局部Hessian可观时才发布`p_out`，未收敛返回`false`。
- 静止判定不再使用硬编码`0.01`。本条首2秒acceleration-norm variance为`0.0308523`，当前冻结
  `sigma_a=0.357`给出阈值`sigma_a^2=0.127449`，因此正确进入重力方向对齐，估计gyro bias为
  `[0.0136144,-0.000980001,-0.00393499] rad/s`。
- 每次运行自动检查anchor几何，不写per-recording侧别。该布局最小/最大singular-value ratio为
  `0.000325028`，低于固定`1e-3`，且平面近水平，因此自动构造两侧2.5D seed；实际3D raw-range目标选择
  下侧。seed为`[-0.0103251,-0.0931572,-0.517479]m`，收敛p0为
  `[-0.0216184,-0.0852544,-0.526389]m`，旧p0的z为`-5.827m`。

2.5D不读取moving-tag Vicon，不拟合高度、beta或noise。由于首帧raw range仍有未校正正偏差，初始化z与
evaluator中的Vicon高度仍存在误差；这项修复只避免近共面几何从anchor质心进入错误的高度分支。

## 逐方法结果

| method | status | ATE RMSE (m) | NLOS-window RMSE (m) | ATE P95 (m) | range before/after RMSE (m) | candidates | accepted | fallback/failure |
|---|---|---:|---:|---:|---:|---:|---:|---|
| suppress_all | NOT_RUN | NA | NA | NA | NA / NA | 253 obs / 8 seg | NA | producer Stage2 failed |
| lcb_fixed_full | NOT_RUN | NA | NA | NA | NA / NA | 253 obs / 8 seg | NA | producer Stage2 failed |
| robust_cauchy | SUCCESS | 0.951271 | 0.467489 | 1.693017 | 0.440256 / 0.440256 | NA | 0 | false |
| SFUISE-ToA | SUCCESS | 0.297530 | 0.268811 | 0.377887 | 0.440256 / 0.440256 | NA | 0 | false |

Cauchy轨迹有918个有限pose，时间为0.0089--73.3694s，并非只生成一半。冻结评价在10Hz网格使用0.02s
nearest容差，而C++关键帧约12.5Hz，周期相位使360个网格点匹配，coverage为0.490463；SFUISE的高频轨迹
匹配725点，coverage为0.987738。窗口有一个并集、时长2.240277s；Cauchy/SFUISE分别匹配12/23个窗口点。
range指标使用3585个可评planned `obs_id`，两种baseline都未应用recovery offset，before/after相同。

| delta | value | status |
|---|---:|---|
| `ATE(lcb_fixed_full)-ATE(suppress_all)` | NA | no Stage2 cache / no RR pair |
| `Window(lcb_fixed_full)-Window(suppress_all)` | NA | no Stage2 cache / no RR pair |

## 剩余失败位置

raw reference在55次迭代内将objective从`1.0144452069e9`降到`4267.527197`，最大位置范数3.914m，状态为
`CONDITIONAL_LM_CONVERGED`。Stage2随后将objective继续降到`2823.712672`；28个outer iteration的
objective、step和KKT检查最终通过，但`max_scaled_navigation_gradient_objective=0.806003`仍远高于固定
`1e-6` stationarity阈值。固定checkpoint恢复预算耗尽后明确失败。这是当前无RR pair的直接原因；本轮没有
修改Stage2容差、预算、support或恢复规则。

## 验证、复现与产物

协议为[OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md](OWN_VICON_INITIALIZATION_FIX_PROTOCOL.md)。隔离输出为
`experiments/results/own-vicon-flow-initfix-20260912A`；锁ID
`6f5e6cba8ebfa65f0d2dd863242b872208b257736595b8b4f684e5f86616b15a`，最终
`LOCK_CORE_INPUT_IMPLEMENTATION_PASS`。方法实际启动3/5树：producer exit1、Cauchy exit0、SFUISE exit0；
两个RR依赖格未启动，不算科学进程。

定向与回归验证为32项C++测试和24项Python测试通过。标准evaluator在科学产物封存后因异构method row的
`window_samples`字段未进入首行CSV header而exit1；失败私有目录保留为
`own-vicon-flow-initfix-20260912A.failed-standard-evaluate`。已有post-seal wrapper只采用字段并集重新序列化，
对同一科学artifact评价exit0；未重跑算法。完整命令、退出码和hash见
[`evidence/own-vicon-initialization-fix-20260912A`](evidence/own-vicon-initialization-fix-20260912A/README.md)。

本结果是单条development correctness证据。没有调detector/LCB/solver/noise，失败与NA均保留；不构成
Recover优于Reject、跨数据集精度、正式外参或held-out结论。
