# own_vicon 15-31-28 单条流程结果

> 本页记录初始化修复前的封存结果。修复后的fresh run及当前结论见
> [OWN_VICON_INITIALIZATION_FIX.md](OWN_VICON_INITIALIZATION_FIX.md)。

## 结论

`2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle` **只能部分走通当前冻结流程**。
measurement cache、C++ prepare、共同初始化核验、ROS 往返和 SFUISE-ToA 均通过；SFUISE 正常导出并在
726 个有效 10 Hz 样本上得到 ATE RMSE **0.297893 m**。冻结 GTSAM producer 在 detector 已冻结 support 后的
raw-reference LM 达到100次迭代上限，未发布 Stage2 cache。因此 `suppress_all` 和 `lcb_fixed_full` 都不能运行，
不存在有效 RR 配对，`delta_RR` 与 `delta_RR_NLOS` 均为 NA。Cauchy 也在 preliminary LM 达到上限。

这说明 bag、4-anchor 输入和 SFUISE 桥并未整体失效，但当前冻结 GTSAM 初始化/优化链不能在该输入上收敛。
不把 SFUISE 成功解释为本方法 recovery 成功，也不隐藏另外三个方法单元的失败或未运行。

## 输入与工程核验

- 完整 bag：14680 条 IMU、14479 条 range、3669 个 UWB message；C++ 实际建立918个 keyframe，规划3624条 UWB。
- `/livox/imu` acceleration 由原始 g 固定乘9.81到 m/s²；gyro 原值按 rad/s。四个专用测试均通过。
- 四个 anchor 使用同 bag 静态 Vicon 中位位置；各 anchor 最大分量漂移不超过0.019305 m。
- tag0/rig/IMU 使用零杆臂近似；header stamp 只减一次共同 epoch，不拟合时间偏移。
- ROS 往返为14479/14680条，最大时间误差小于1.0 ns，输出 bag 不含 GT 和 support topic；SFUISE probe 通过。
- 锁 `fc9331c30b58d0550c464a4c8b3fae2f54a78429753616be8688f561f9ecb2b1` 最终核验通过。

prepare A 因 `time_basis` 非 cache 枚举失败；B 因相对时间又叠加 epoch 导致空窗口失败。两者均在优化器前停止，
科学任务为0，目录保留。C 修正适配器合同后才进入唯一一次科学执行。这两次不是 detector/solver 参数重试。

## 逐方法结果

| method | status | ATE RMSE (m) | NLOS-window RMSE (m) | ATE P95 (m) | range before/after RMSE (m) | candidates | accepted | fallback/failure |
|---|---|---:|---:|---:|---:|---:|---:|---|
| suppress_all | NOT_RUN | NA | NA | NA | NA / NA | 260 obs / 10 seg | NA | producer failed |
| lcb_fixed_full | NOT_RUN | NA | NA | NA | NA / NA | 260 obs / 10 seg | NA | producer failed |
| robust_cauchy | FAILURE | NA | NA | NA | NA / NA | NA | NA | `PRELIMINARY_LM_FAILED:CONDITIONAL_LM_MAX_ITERATIONS` |
| SFUISE-ToA | SUCCESS | 0.297893 | 0.268623 | 0.380129 | 0.440256 / 0.440256 | NA | 0 | false |

SFUISE 没有本方法 recovery offset，所以 range before/after 相同。range 指标覆盖3585条有 Vicon reference 的 planned
observation。窗口内轨迹有23个样本；evaluator 独立复核到1个合格事件/1个并集窗口，时长2.240277 s，
总 range reference 14344条，与先前审计一致。

| delta | value | status |
|---|---:|---|
| `ATE(lcb_fixed_full)-ATE(suppress_all)` | NA | no Stage2 / no RR pair |
| `Window(lcb_fixed_full)-Window(suppress_all)` | NA | no Stage2 / no RR pair |

## 失败位置与几何诊断

detector 本身完成且在 Stage2 前冻结 support：3604个 signal row、8个 forward alarm、6个 backward alarm、
260个候选观测和10段，`gt_or_oracle_read=false`。producer 随后的 raw-reference 初始 objective 为
`4.06986456475943e11`，LM 在冻结的100次上限停止；Stage2、decision、final graph 都没有运行。

C++ 日志显示没有找到静止初始化段，使用 identity orientation；首帧4-anchor trilateration 给出
`[-0.299819, 0.122701, -5.82726] m`。四个 anchor 的高度只跨0.032670 m，中心化 anchor 坐标的奇异值为
`[5.185029, 3.718844, 0.001685]`，最小/最大比为`3.25e-4`。封存后的 GT-only 诊断得到 PDOP
中位数7.48、P95 20.17，VDOP几乎相同；首帧线性 trilateration 矩阵条件数约4087。首帧四条 raw range
相对近似 Vicon 几何均有约0.33--0.40 m正误差，且本轮按合同不校正静态 beta。

这些证据支持“近共面4-anchor、未校正正偏差和 identity IMU orientation 共同造成高度初始化/优化困难”这一解释，
但不能仅由一次失败唯一归因。SFUISE 使用自身初始化、优化和 rejection，0.298 m ATE 说明数据不是普遍不可用。

## 复现与产物

冻结协议：[OWN_VICON_FLOW_PROTOCOL.md](OWN_VICON_FLOW_PROTOCOL.md)。完整隔离输出：
`experiments/results/own-vicon-flow-20260912C`（24 MB）；A/B 工程失败目录同级保留。
主表为 `metrics.csv`，配对表为 `paired_differences.csv`，实际任务及退出码为 `execution.json`，锁为 `lock.json`。

执行命令：

```text
PYTHONPATH=experiments/scripts:/opt/ros/noetic/lib/python3/dist-packages:/home/mint/ws_fusion_uwb/devel/lib/python3/dist-packages \
/usr/bin/python3 experiments/scripts/run_own_vicon_flow.py {prepare,preflight,execute,verify} --run experiments/results/own-vicon-flow-20260912C
```

C 的 prepare/preflight/execute/verify 退出码均为0；方法真实退出码另列于表中。首次 evaluator 因成功行新增
`window_samples` 而 CSV header 取自首个失败行，退出1；失败私有目录保留为 `own-vicon-flow-20260912C-failed-evaluator-A`。
封存后 wrapper `evaluate_own_vicon_flow_postsealed.py` 只修复字段并集序列化，SHA-256 为
`539491c2039d238d0555ce7cfbc92cd3c9b6a0d86f26a61a1e0e3748077d8adc`，重读未变的科学 artifacts 后退出0。
无科学进程重跑，无参数调整，不升级 C1--C3、T10/T11 或 held-out claim。
