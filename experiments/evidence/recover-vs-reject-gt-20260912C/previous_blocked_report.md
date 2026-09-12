# Recover vs Reject：准入阻塞交付

三条冻结录制均为 **BLOCKED_GEOMETRY_OR_IMU**；科学进程 **0/15**，12方法单元全部NOT_RUN，3条配对差值NA。
测量白名单准备与evaluator-only窗口枚举已执行；不把它们写成估计器、恢复或定位通过。

## 准入原因

作者README声明角速度在IMU系、加速度在rig系。已查的作者材料没有闭合CSV加速度的符号、测量原点及上游变换实现。
厂商SDK说明不能替代作者CSV处理来源。未用GT、逐录制calib外参或测距残差猜方向；没有发布错误标注IMU系的cache。
s3/s4的radio tag1实际位于CAD location3；八anchor使用v2直接身份。静态beta未校正、未标定；审计误差是总误差。

[作者README](https://github.com/utiasASRL/starloc/blob/d3ad541/README.md)、[作者论文](https://arxiv.org/html/2309.05518v1)、
[厂商IMU说明](https://docs.stereolabs.com/docs/development/zed-sdk/modules/sensors/imu)。

## 12个方法单元

| recording | method | ATE RMSE | window RMSE | range before/after | status |
|---|---|---|---|---|---|
| zigzag_s4 | suppress_all | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s4 | lcb_fixed_full | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s4 | robust_cauchy | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s4 | SFUISE-ToA | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| loop-3d_s3 | suppress_all | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| loop-3d_s3 | lcb_fixed_full | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| loop-3d_s3 | robust_cauchy | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| loop-3d_s3 | SFUISE-ToA | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s3 | suppress_all | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s3 | lcb_fixed_full | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s3 | robust_cauchy | NA | NA | NA / NA | NOT_RUN: geometry/IMU |
| zigzag_s3 | SFUISE-ToA | NA | NA | NA / NA | NOT_RUN: geometry/IMU |

候选观测/段、decision/final接受数、覆盖率、fallback和退出码均NA，未运行不填零。完整字段见metrics.csv。

## 配对差值

| recording | Recover−Reject ATE | Recover−Reject window | RR / 四方法共同样本 |
|---|---|---|---|
| zigzag_s4 | NA | NA | NA / NA |
| loop-3d_s3 | NA | NA | NA / NA |
| zigzag_s3 | NA | NA | NA / NA |

正差值表示Recover更差；fallback不能计为恢复收益。没有科学结果支持胜负判断。

## 独立审计诊断（不是detector support）

| recording | 全部合格事件 | 去重窗口 | 并集时长s |
|---|---:|---:|---:|
| zigzag_s4 | 14 | 11 | 89.497655 |
| loop-3d_s3 | 4 | 4 | 20.811030 |
| zigzag_s3 | 12 | 8 | 33.127726 |

严格e>0.5m、≥2s、≥5包、gap≤1s；逐link保留全部事件，不只最长段。没有有效planned obs_id，主range比较保持NA。

![zigzag_s4审计诊断](/home/mint/ws_fusion_uwb/evaluator_private/icra/recover_vs_reject/recover-vs-reject-20260912T122319Z/zigzag_s4/audit_diagnostic.png)

![loop-3d_s3审计诊断](/home/mint/ws_fusion_uwb/evaluator_private/icra/recover_vs_reject/recover-vs-reject-20260912T122319Z/loop-3d_s3/audit_diagnostic.png)

![zigzag_s3审计诊断](/home/mint/ws_fusion_uwb/evaluator_private/icra/recover_vs_reject/recover-vs-reject-20260912T122319Z/zigzag_s3/audit_diagnostic.png)

## 产物与复现

[冻结协议](RECOVER_VS_REJECT_PROTOCOL.md)；[几何准入记录](recover_vs_reject_geometry.json)。
输出目录：`/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie/experiments/results/recover-vs-reject-20260912T122319Z`。evaluator私有目录：`/home/mint/ws_fusion_uwb/evaluator_private/icra/recover_vs_reject/recover-vs-reject-20260912T122319Z`。
执行命令使用 `/usr/bin/python3 experiments/scripts/run_recover_vs_reject.py {prepare,execute,evaluate,verify} --run <上述目录>`，每阶段独立调用。
prepare/execute/evaluate退出码2表示准入阻塞；verify退出码0表示hash核验通过。
真实v2 cache发布、C++ prepare、PL producer/Stage2/final、SFUISE数据接入及科学运行均NOT_RUN。
基础数学/报告辅助函数仅由工程fixture验证，不能称真实非空恢复或真实ROS接入通过。
需要独立来源闭合CSV IMU坐标、specific-force符号与测量原点，并完成对应C++/ROS准入后才能继续科学矩阵。
名单和参数不因阻塞改变。核心/默认/旧审计/历史结果未修改；T10=C2-C、T11=C、C1–C3不升级；未提交/push。

## 实际验证与实现边界

20/20工程测试exit0；包括真实bwrap隔离和ROS序列化fixture往返。ROS range为float32，
往返按该wire精度核验；IMU为float64、时间为纳秒。未把工程fixture当真实数据接入。
独立复核exit0：120747条参考误差与旧审计差值<1e-12m，30个合格事件逐段完全匹配，
12方法格/3差值/15任务记账完整。核心源码、冻结参数、原始输入前后hash核验exit0。

入口的prepare已实现白名单staging与准入检查；execute当前实现阻塞准入的完整账本。
由于三条均未准入，本轮**没有接通或验证STAR-loc成功分支的C++ producer/final调度与SFUISE ROS数据桥**。
可运行v2 cache转换、变换和评价数学有工程fixture，真实cache发布、实际backend配对校验、
真实非空恢复/fallback均NOT_RUN。解除来源阻塞后仍需接通并验证成功分支，不能只修改status绕过准入。

- [20项测试日志](evidence/recover_vs_reject_20260912/engineering.log)
- [实际命令与退出码](evidence/recover_vs_reject_20260912/commands.json)
- [锁定manifest与源码/输入hash](evidence/recover_vs_reject_20260912/lock.json)
- [12格完整指标CSV](evidence/recover_vs_reject_20260912/metrics.csv)
- [3条配对差值CSV](evidence/recover_vs_reject_20260912/paired_differences.csv)
- [15任务账本](evidence/recover_vs_reject_20260912/execution.json)
- [独立复核](evidence/recover_vs_reject_20260912/independent_verification.json)
