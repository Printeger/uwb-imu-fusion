# SFUISE ToA baseline adapter

状态：`DONE / COMPLETE_WITH_RETAINED_FAILURES`。本任务只接入用户提供的未修改 SFUISE
checkout，不修改其核心算法。

## 冻结输入与方法边界

仅运行官方 ISAS Walk1/2/3 配置，启动前强制检查 `if_tdoa=false`、`topic_uwb=/rtls_flares`。
SFUISE commit 固定为 `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`。每条原始 bag 的 SHA-256
必须与我们 estimator 使用的 clean measurement cache 的 `base_source_sha256` 完全相同。
rosbag 只播放 `/waveshare_sense_hat_b`、`/rtls_flares`、`/anchor_list`；不播放 GT，
不挂载或传入 detector、support、innovation、segment、recovery、oracle 或 range-error artifact。

SFUISE 保留作者配置，包括原生 ToA offset 和原生 UWB rejection；这是 SFUISE baseline 本身，
不能改写成我们的 detector/recovery，也不按结果改阈值。

## 轨迹转换

官方 spline 状态为 navigation frame 中的 body/IMU pose，并在线估计 navigation→UWB/map 变换。
官方可视化位置额外包含旋转后的 body→tag offset。统一 evaluator 的参考点为 body/IMU 原点，
所以 adapter 直接计算：

```text
q_U_B = q_U_N * q_N_B
p_U_B = q_U_N * p_N_B + t_U_N
```

即不加入 tag lever。输出为统一 TUM schema：
`timestamp x y z qx qy qz qw`。时间戳取原始 ToA sensor timestamps，adapter 从最终累计 spline
插值，不读取 GT。原生 path exporter 依赖 GT timestamps，本任务不使用它。

## 冻结统一评价

每条序列的 evaluation interval 在运行前由对应 clean UWB cache 的首末 sensor timestamp固定；
全部方法使用相同 GT 文件、`nearest_within_tolerance=0.02s`、同一闭区间、同一 tracker/body
同点假设、`SE3_SCALE_1_PER_TRAJECTORY`。SFUISE不单独重对齐、不单独裁剪有利区间。
评价调用仓库 `tools/paper/evaluate_runs.py::trajectory_metrics`；每格记录ATE RMSE、状态、
matched samples、runtime和failure reason。

本方法四列复用已存在且source SHA一致的clean运行artifact，但在本任务中重新调用统一evaluator；
SFUISE三列fresh运行。复用是trajectory artifact provenance，不是复用SFUISE或跨方法innovation。
Walk2已有 Robust FGO算法失败必须保留，不能为填表而重试或改参数。

## 复现命令

从仓库根目录执行：

```bash
python3 experiments/scripts/run_sfuise_toa.py
python3 experiments/scripts/evaluate_sfuise_baseline.py \
  --sfuise-batch experiments/results/sfuise-toa-20260912T104509Z-8543708554f5
```

第一条命令会创建唯一输出目录，实际使用时把第二条命令的参数换成第一条打印的目录。
构建位于 `/home/mint/ws_fusion_uwb/evaluator_private/icra/sfuise_baseline/build_ws`；第三方 checkout、
catkin 构建和大数据均不复制进 Git。实际本轮 SFUISE batch 为
`experiments/results/sfuise-toa-20260912T104509Z-8543708554f5`，统一评价输出为
`experiments/results/sfuise-comparison-20260912T105246Z-e224f3a369ec`。

## 主表与运行状态

ATE RMSE 单位为米。全部可用单元均由同一个 evaluator 在各序列预先固定的共同区间内计算；不同方法
保留自身原生轨迹采样率，因此 matched sample 数不同。

| Sequence | Base FGO | Robust FGO | SFUISE | suppress_all | lcb_fixed_full |
|---|---:|---:|---:|---:|---:|
| ISAS Walk1 | 0.163853268 | 0.163847231 | 0.109076557 | 0.163853268 | 0.163853268 |
| ISAS Walk2 | 0.228472954 | NA | 0.075431056 | 0.228472954 | 0.228472954 |
| ISAS Walk3 | 0.178023331 | 0.175866603 | 0.079883493 | 0.178023331 | 0.178023331 |

| Sequence | Method | Status | samples | runtime (s) |
|---|---|---|---:|---:|
| Walk1 | Base / Robust / SFUISE / suppress / LCB-full | success / success / success / success / success | 229 / 229 / 967 / 229 / 229 | 1.331 / 1.526 / 68.659 / 0.419 / 0.393 |
| Walk2 | Base / Robust / SFUISE / suppress / LCB-full | success / **failure** / success / success / success | 293 / 0 / 1239 / 293 / 293 | 1.079 / 2.063 / 85.548 / 0.433 / 0.429 |
| Walk3 | Base / Robust / SFUISE / suppress / LCB-full | success / success / success / success / success | 314 / 314 / 1338 / 314 / 314 | 1.464 / 1.670 / 90.999 / 0.430 / 0.449 |

这里的 SFUISE runtime 是包括 1x rosbag playback、启动和导出的端到端 wall time；adapter 同时保留
SFUISE 原生 window runtime 均值 23.407/21.832/23.784 ms。其他四列使用封存 run 的 estimator
elapsed time，二者语义在 CSV 中保留，不能把数值当作严格同口径速度排名。

## 坐标、时间与失败记录

SFUISE 原生可视化 path 把 body→tag offset 加到位置上，并以 GT 消息时刻触发输出。为保持 GT 隔离，
本 adapter 不使用该 path：它从官方累计 spline 与在线 navigation→UWB/map calibration 重建 body/IMU
原点，按原始 ToA sensor timestamp 采样。官方 `EstimationInterface` 的 calibration 输入在启动时 remap，
阻断其依赖 GT 的可视化分支；SFUISE core 与 adapter 仍分别接收实际 calibration 输出。Walk1/2/3
分别导出 967/1239/1338 个样本，三段官方 core 均成功退出。

Walk2 Robust FGO 保留 `ESTIMATION_FAILED: FINAL_LM_FAILED:CONDITIONAL_LM_MAX_ITERATIONS`，ATE 为 NA；
没有重试或修改数学逻辑。suppress_all 与 lcb_fixed_full 三段均为 `NO_CANDIDATES`，因此轨迹与 Base
相同，这不构成恢复收益证据。

开发过程中第一次 catkin 调用因继承的 `PWD` 指向仓库而拒绝非 workspace 根目录；修复只是在子进程
环境显式设置 build workspace。统一评价第一次在系统 Python 3.8 因 `str.removeprefix` 不可用而退出；
改为兼容切片后成功。两项都属于 adapter/harness 兼容修复，没有触及 SFUISE 或本方法核心代码。

未实现/未运行：TDoA、Walk1/2/3 以外序列、其他数据集、批量矩阵、参数 sweep、正式外参标定、
held-out 论文结论。tracker 原点与 body/IMU 原点的单位外参仍是此前用户授权的开发假设；所有方法
共用该假设和 alignment，SFUISE 没有单独选择更有利的转换。
