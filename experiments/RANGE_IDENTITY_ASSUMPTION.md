# Walk1 单位外参与同一时基诊断

用户于 2026-09-12 明确授权：官方未提供外参/时钟来源时，采用最可能的单位外参与共同时间戳重跑 evaluator。
运行前固定 T_anchor_GT=I4、T_marker_IMU=I4、时钟 scale=1/offset_s=0。
这表示 GT pose 与 anchor 处于同一坐标系、marker pose 代表 IMU pose，时间单位均为秒且无漂移/偏移。
保留官方 IMU→tag [0.1,-0.025,0] m 杆臂和 beta=-toa_offset。
这些假设未获独立标定验证；来源类别 USER_AUTHORIZED_ASSUMPTION，不能标 independent。
只复用已有 Walk1 clean 四方法 artifacts，不运行 estimator，不搜索或拟合变换/时钟。
保持 linear position + SLERP、0.05 s 最大 bracket gap、无外推；保留旧结果。
新逐观测和汇总 CSV 必须携带 ASSUMED_GEOMETRY_AND_CLOCK 标签。

## 实际结果

唯一 evaluator 运行 exit0，状态 EVALUATED_ASSUMPTIONS；没有 estimator 调用、拟合或重试。
19,400 行中 17,064 行可评价（每方法 4,266/4,850），2,336 行保留 INVALID_OBSERVATION，
不因本轮假设改动原 validity；无 GT gap/超出支持域排除。
四方法使用相同原始观测且本次 applied_correction 全为0，raw/corrected 指标相同。
重复方法行不当作独立测量样本；每方法总体指标如下（m）：

| 方法 | 样本数 | RMSE | MAE | signed median | absolute P95 |
|---|---:|---:|---:|---:|---:|
| M0 | 4266 | 1.696637133 | 1.511731778 | -1.389659918 | 2.679216685 |
| M1 | 4266 | 1.696637133 | 1.511731778 | -1.389659918 | 2.679216685 |
| M3 | 4266 | 1.696637133 | 1.511731778 | -1.389659918 | 2.679216685 |
| M4 | 4266 | 1.696637133 | 1.511731778 | -1.389659918 | 2.679216685 |

[逐观测 CSV](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-17cd8b00ede14cdf9d52a48f323f9afe/range_metrics.csv)、[anchor/segment 汇总](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-17cd8b00ede14cdf9d52a48f323f9afe/range_summary.csv)、
[方法汇总](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-17cd8b00ede14cdf9d52a48f323f9afe/method_summary.json)、[完整假设快照](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-17cd8b00ede14cdf9d52a48f323f9afe/calibration_snapshot.json)。
所有 CSV 携带 reference_status 和 assumed_fields；旧缺标定配置/结果保持。
当前误差条件于单位外参/同一时基，不能区分物理 range error 与假设带来的几何/时间误差，
不据此确认外参、生成 oracle NLOS label 或声称 recovery 收益。

## 复现

```bash
python3 experiments/scripts/evaluate_range_gt.py --batch-dir experiments/results/walk1-clean-20260912T094703Z-d8f65646d1f3 --calibration /home/mint/ws_fusion_uwb/evaluator_private/icra/walk1_range_identity_assumed.json --allow-identity-assumptions
```

15/15 工程测试 exit0，包括默认拒绝假设、显式允许并传播标签、拒绝非单位变换和假定静态 bias，
以及已有物理隔离测试。[运行身份](evidence/range_identity_20260912/execution.json)、
[状态](evidence/range_identity_20260912/evaluation_status.json)、[测试日志](evidence/range_identity_20260912/tests.log)。
未实现/未运行：独立外参时钟标定、其他数据、非空补偿真实案例、estimator rerun、参数搜索、正式论文准入。
