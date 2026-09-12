# 0912 ICRA 五数据集 manifest 验证

状态：`DONE / FIVE_FAMILY_MANIFEST_VALIDATION_PASS / EXPERIMENT_ADMISSION_NOT_GRANTED / ESTIMATOR_NOT_RUN`。

## 范围与判据

本任务只建立和验证 HUEC、MILUV、own_vicon、SFUISE、starloc 的实验 inventory 与统一数据合同。未修改或运行 estimator、CUSUM detector、NLOS recovery，未启动批量/精度实验，也未根据数据内容调整方法参数。

PASS 判据限定为：五个 family 均有逐 recording 条目；主文件及构成 recording 的 UWB/IMU/GT/时间/标定/anchor 文件由路径与 SHA-256 锁定；bag/CSV 选定内容完成结构、ID、有限性和时间顺序扫描；schema/语义/TDoA 拒绝/split 暴露检查通过；所有异常和未知 metadata 保留。它不是正式 VAL/TEST 准入或传感器质量认证。

## 实际 inventory

| family | recording | 内容检查 |
|---|---:|---|
| HUEC | 8 | 8 个动态 combined bag 的四路 absolute range、IMU、local GPS 全消息扫描 |
| MILUV | 3 | 每序列 `uwb_range.csv`、`imu_px4.csv`、`mocap.csv` 全行扫描并绑定 `timeshift.yaml` |
| own_vicon | 3 | UWB/IMU/Vicon 0–4 pose 全消息扫描；两个无索引源 bag 仅在临时副本 reindex 后恢复扫描 |
| SFUISE | 3 | Walk1/2/3 的 UWB、anchor、IMU、Vive GT 全消息扫描 |
| starloc | 22 | 每序列 UWB、IMU+embedded GT、calib 及版本化 anchor 文件扫描/绑定 |

总计 39 条：36 条 `LOCAL_CONTENT_VALIDATED`，3 条 `LOCAL_CONTENT_VALIDATED_WITH_RECORDED_ISSUES`。角色为 6 DEV、33 UNASSIGNED、0 VAL、0 TEST。

已登记而未隐藏的问题：

- HUEC NLOS/Trajectory_A/Case_2 的 `/dwm1001/anchor5` 第 1156 条 topic 内消息在时间 `1730041547.8471816 s` 有 `rssi=-inf`；该消息 `distanceFromTag` 有限。manifest 不删除该观测，也不把 RSSI 当 range。
- own_vicon 两个 obstacle 原 bag 无 ROS index。只读原文件失败后，metadata inspector 在系统临时目录复制并 reindex 临时副本，分别恢复约 11.635 s/569 条 UWB 和 11.745 s/578 条 UWB；原始文件路径、字节和 SHA-256 仍是身份，`data/` 未修改。该恢复不证明源 bag 尾部完整。
- HUEC range 消息没有 tag ID；必须字段以 `UNKNOWN/null` 记录，未发明 tag ID。
- 未闭合的 survey、外参、clock offset/drift、独立 beta、许可证和开发暴露继续列于每条 `unresolved_items`。未知开发暴露只允许 UNASSIGNED。

## TDoA 拒绝

Schema 只允许 `ABSOLUTE_TOA`、`ABSOLUTE_TWR`、`ABSOLUTE_RANGE`。顶层 policy 明确拒绝 `TDOA`、`TIME_DIFFERENCE_OF_ARRIVAL`、`RANGE_DIFFERENCE` 和 `ANCHOR_PAIR_DISTANCE_DIFFERENCE`；validator 另做语义检查。15 个工程测试含大小写 TDoA、差分和 UNKNOWN 反例，均按预期失败。

本次条目的测量类型证据来自实际绝对距离字段：HUEC `distanceFromTag`、MILUV `range/range_raw`、own_vicon `nodes[].dis`、SFUISE `ranges[].range` 及作者 ToA 声明、starloc README 的 raw distance `range`。未把 passive、GT range、bias 或 TDoA 流纳入主 UWB 输入。

## 执行与结果

最终复现命令从仓库根执行：

```bash
python3 experiments/scripts/inspect_dataset_metadata.py \
  --recover-unindexed \
  > experiments/datasets/five_dataset_metadata_observed.json

python3 experiments/scripts/build_dataset_manifest.py
python3 experiments/scripts/validate_manifest.py --check-hashes
python3 experiments/scripts/test_validate_manifest.py
```

最终退出码依次为 0、0、0、0。权威日志位于 [validation_20260912_v2](datasets/validation_20260912_v2/)：

- `metadata_stderr.log`：0 bytes；最终 inspector 无 stderr；
- `build_manifest.log`：生成 39 条 recording；
- `static_hash.log`：0 errors、42 warnings，`FIVE_FAMILY_MANIFEST_VALIDATION=PASS`；
- `tests.log`：15 tests 全部通过。

最终身份：

```text
five_dataset_metadata_observed.json  c2aa3624c374d6eb4249239624a1abf8977a644a50b80ab502e3adcc2e872a18
dataset_manifest.json                0ea29e67a21c315ccb9434594b22a4cdbd47df573f704ba32b1dcecd0ec7d1b1
dataset_manifest.schema.json         1ad53fd2acc2f7f09fdc4083de9e5432e4da8c8ff1837178195d90dd216acc9e
```

最终成功前保留以下工程失败事实：metadata inspector 首次把 SFUISE 实际 `fpp/rxp` 误写为不存在的 `fp_power/rx_power`，exit 1；第二次假设无 header 的 `/anchor_list` 有 header，exit 1。修正为 bag 实际消息字段和 bag-record time 后通过。validator 首次因 artifact 路径条件缺右括号而 py_compile、validator、tests 均 exit 1；修正后完整重跑通过。这些修复只影响实验基础设施脚本。

## 未执行与准入限制

`catkin build`、核心 CTest、estimator、CUSUM、NLOS recovery、accuracy evaluation、批量实验全部 `NOT_RUN`。所有 39 条 recording 的 `formal_admission` 仍为 `NOT_ADMITTED`。下一步若要形成正式 VAL/TEST，必须另立锁定协议并逐条闭合数据合同缺口；本次 PASS 不授权继续运行或调参。
