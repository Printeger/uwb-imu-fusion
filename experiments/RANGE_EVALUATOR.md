# ISAS Walk1 evaluator-only per-range residual

状态：`IMPLEMENTED / ENGINEERING_PASS / REAL_RANGE_UNAVAILABLE_CALIBRATION`。
本任务只新增 evaluator 与 estimator 文件系统隔离，不改 src/、include/、tools/paper/、
config/ 中的核心算法、CUSUM、recovery、静态 bias 或 solver 参数。

## 几何与测距定义

统一 `T_A_B` 表示把 B 坐标转换为 A 坐标。GT TUM 为 `T_G_M(t)`，M 是 `vive/tracker_1`。
每条观测先在传感器时基关联 GT，再计算：

```text
p_tag_A = (T_anchor_GT * T_G_marker(t) * T_marker_IMU * [lever_IMU_tag; 1])[:3]
gt_range = norm(p_tag_A - anchor_A)
raw_range_error = measured_range - static_range_bias - gt_range
corrected_range = measured_range - applied_correction
corrected_range_error = corrected_range - static_range_bias - gt_range
```

同时包含 marker→IMU 和 IMU→tag 杆臂，不重复施加 lever；旋转随 GT pose 插值。
不能用估计轨迹 SE(3) alignment、在线导航 calibration 或 post-fit residual 生成 GT geometry。
校正取**实际 final-use**记录，不能把 Stage2 c_hat 自动当已施加校正。M3 补偿为0；
M4 仅 accepted 且 final_use 且非 fallback 时应用 delta，核验其等于 Stage2 c_hat。
fallback/suppressed 保留 estimated_bias，但 applied_correction=0。失败 run 不假造有效 correction。
M0/M1 没有 detector，candidate 留空并标 NOT_RUN_BASELINE，不能说 detector 输出了0候选。
未进入计划的 observation 仍保留，标 NOT_TESTED_UNPLANNED。protocol-invalid 观测不计误差指标。

## 时间规则（运行前登记）

GT 原始时间通过显式 `scale*t+offset_s` 映射到 range sensor time。
必须给 clock 来源，不能把 bag record time 混作 sensor header time。
位置线性插值，姿态最短弧 SLERP；最大 bracket gap **0.05s**，严格大于即 GT_GAP_EXCEEDED。
等于阈值可插值；精确 GT timestamp 直接关联，不受相邻 gap 影响；不外推，不跨 gap，
GT 时间必须严格递增，不能以自动排序/去重掩盖异常。
每行记录左右 GT 时间与实际 bracket gap。此规则只属于 range evaluator，
不修改既有 trajectory evaluator 的 nearest 0.02s 协议。

## 官方 SFUISE 来源审计

用户指定仓库固定到 commit `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`。
[来源 SHA/URL ledger](evidence/range_gt_20260912/official_source_ledger.json) 记录下载的源码身份。
本地 Walk1 bag 的 Git blob `5fa9741eca085a1a8514e53c5eec5297a7effcc0` 与官方 tree 完全一致。

| 已确认项 | 定义和范围 |
|---|---|
| ToA | [官方 Walk1 config](https://github.com/KIT-ISAS/SFUISE/blob/75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d/sfuise/config/config_test_isas-walk1.yaml) 的 if_tdoa=false；本 evaluator 拒绝所有 TDoA/未知类型 |
| IMU/body→tag | 同 config 的 offset=[0.1,-0.025,0]m；[Residuals.h](https://github.com/KIT-ISAS/SFUISE/blob/75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d/sfuise/include/Residuals.h) 先在 body 姿态下旋转该 lever，再平移至 tag |
| bias 符号 | 官方 ToA residual 为 distance−toa−distance_offset；本任务 beta=−toa_offset，不是直接复制同号 |
| anchor 顺序 | [SplineFusion.cpp](https://github.com/KIT-ISAS/SFUISE/blob/75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d/sfuise/src/SplineFusion.cpp) 按 map 升序 anchor ID 分配 offset |
| anchor geometry | [EstimationInterface.cpp](https://github.com/KIT-ISAS/SFUISE/blob/75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d/sfuise/src/EstimationInterface.cpp) 读 bag anchor_list 前20条平均；本地已盘点坐标，仅是作者提供的 map geometry |
| GT | 同文件直接读 tracker_1_ref pose。q_nav_uwb/t_nav_uwb 是在线估计的导航→UWB 变换，不是已给出的 GT→anchor 外参；代码没有给本任务所需 T_anchor_GT/T_marker_IMU 的数值 |

本 evaluator 的 source-declared beta（m）：

| anchor ID | beta = −toa_offset |
|---|---:|
| 7475 | +0.0700 |
| 9524 | −0.1539 |
| 10548 | +0.0751 |
| 15155 | −0.1409 |
| 20276 | +0.0247 |

这些是作者公开常数，标定 recording/独立性未报告；我们没有从 Walk1/TEST 重新拟合。
只允许作为带来源声明的 evaluator 校正项，**没有写回 estimator**，其 fixed_beta_by_link 仍为 {}。
这也不是“独立 LOS 标定已验证”的结论。
目前真实残差的阻塞：T_anchor_GT、T_marker_IMU、GT_time_to_sensor。没有用 identity/0 猜测。

## 物理隔离

[evaluate_range_gt.py](scripts/evaluate_range_gt.py) 是独立可执行 evaluator，不导入/启动 estimator。
输入是终止状态的 estimator artifacts 与私有 calibration/GT；只向
`/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-UUID/` 写新文件。
禁止写回 raw measurement、estimator config/cache/runs。派生 range CSV 不复制回 experiments/results。
现有 trajectory evaluator 的新产物也移到 evaluator_private/icra/trajectory。

[estimator_isolation.py](scripts/estimator_isolation.py) 在每次真实 backend 调用上使用 bwrap 空 root
mount namespace：只挂载系统 runtime、实际 binary/共享库、单个 stripped config、锁定 clean
measurement manifest/imu/uwb、实际 Stage2 payload allowlist（读权限）和 batch/runs（写权限）。
没有宿主 home/data/GT/private evaluator mount；网络与进程 namespace 隔离；不允许隐式 fallback
到无隔离模式。缺 bwrap/库/合法 cache 则明确失败，不能绕过。
物理隔离边界是 estimator 进程；host scheduler 管理 artifacts，不参与 range GT 计算。
运行目录只由 estimator/scheduler 写入；evaluator 不接收自定义输出目录，避免误写该白名单目录。
输入库路径显式配置，不继承用户 LD_PRELOAD/Python 用户路径或任意 UIFGO 环境。

## 输入合同与输出

calibration JSON schema 名 `isas_walk1_range_calibration_v1`，必须为 ISAS-Walk1 绝对 ToA/TWR/range。
字段：gt_path/gt_sha256、tag_id、T_anchor_GT、T_marker_IMU、lever_IMU_tag_m、
anchors_m（ID→3D）、static_bias_m（ID→m）、GT_time_to_sensor（scale/offset_s）、association。
每项有 sources[path,sha256,recording_id]，来源须为独立标定或 PUBLISHED_DATASET_CONSTANT；
禁止 target recording 自标定或 fit_on_evaluation_data=true。来源文件校验 hash；null 保持未知。
有效变换检查 SO(3)，非有限数、重复 obs_id、错 link、缺 mask/compensation 等直接拒绝，不静默 join。

range_metrics.csv 包含用户要求全部字段，另含 run_id/method/tag_id/static_range_bias、valid/planned、
GT 关联左右边界、detector/correction/evaluation 状态。obs_id 始终是字符串，保留64位身份。
range_summary.csv 同时按 run/method/anchor 和 run/method/anchor/segment 分组，分别汇总 raw/corrected。
median 是**有符号误差中位数**；P95 是**绝对误差95百分位**；RMSE/MAE 单位m，
sample_count 只计有效误差，total_observations/unavailable_count 同时保留；空集合输出空值，不填0。
NO_SEGMENT 是未分段的独立组，不冒充 detector segment。
evaluation_status.json 保存 input/artifact hashes、association、缺口、数量和退出状态。
所有 source 在评估前后 hash 核对，修改即失败；不输出 oracle NLOS label。

## 使用与实际验证

```bash
python3 experiments/scripts/test_range_gt.py
python3 experiments/scripts/run_walk1_smoke.py
python3 experiments/scripts/evaluate_range_gt.py \
  --batch-dir experiments/results/walk1-clean-20260912T094703Z-d8f65646d1f3 \
  --calibration /home/mint/ws_fusion_uwb/evaluator_private/icra/walk1_range_calibration.json
```

真实 calibration 文件已创建，已知作者常数已填入；其余缺口为 null。
补齐有来源的真实外参/时钟后只重跑 evaluator，不重新运行 estimator 或改变算法。
代码返回0=有可评估数据、2=标定缺失或零可评估观测、1=输入/结构错误。

| 实际检查/运行 | exit / 结果 |
|---|---|
| range/isolation tests | 0，15/15；[日志](evidence/range_gt_20260912/tests.log) |
| harness regression | 0，5/5；[日志](evidence/range_gt_20260912/harness_tests.log) |
| 首次隔离 smoke 094624Z | harness exit1；首 backend exit127，隔离进程缺显式 libuwb_imu_fgo 搜索路径。未开始估计；失败目录保留 |
| 修复 library path 后隔离 smoke 094703Z | exit0；四方法+producer成功，四方法所有既有指标与隔离前 exact；[对照](evidence/range_gt_20260912/isolation_comparison.json) |
| 旧 smoke artifacts 的首次 range evaluator | exit2，19400条保留，标定缺失；私有原结果保留 |
| 最终隔离 smoke artifacts 的 range evaluator | exit2，19400条保留，80条汇总，0条有效几何GT；UNAVAILABLE_CALIBRATION |

最终 [range_metrics.csv](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-73415e57bce94794bff29a3ebb150826/range_metrics.csv)、[range_summary.csv](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-73415e57bce94794bff29a3ebb150826/range_summary.csv)、
[状态](/home/mint/ws_fusion_uwb/evaluator_private/icra/range/range-73415e57bce94794bff29a3ebb150826/evaluation_status.json)；Git evidence 仅保存 [hash和路径](evidence/range_gt_20260912/private_artifact_hashes.json)，不复制派生数据到 estimator 路径。

测试包括解析杆臂旋转与非零 frame translation、校正符号、实际接受/fallback/suppression、
精确 timestamp/gap边界/不外推、缺 bias/外参拒绝、TDoA、自标定拒绝、重复ID与错link。
真实 bwrap probe 尝试打开 GT 与含 gt_range/range_error/oracle 的私有文件，被拒绝；
改写这些文件后同一 measurement consumer 输出 hash 完全不变，测量输入不可写。
这些测试证明当前挂载边界，不把测试程序称为真实数学 estimator；另一次真实四方法 smoke
证明当前实际 binary 能在该边界运行。没有声称旧的非隔离历史 run 已获得此证明。

未完成：真实 range RMSE（缺独立几何/时钟关系）；真实非空 compensation 与 fallback case
本轮 NOT_RUN（工程 fixtures 已验证）；其他 recording/TDoA/批量矩阵不支持。
没有新增定位收益结论，没有标定拟合、核心 build/CTest、参数更改、commit 或 push。

## 2026-09-12 用户授权假设模式

用户随后授权以单位外参与共同时间戳作 evaluator 诊断。显式 --allow-identity-assumptions
仅允许 source=USER_AUTHORIZED_ASSUMPTION 的 T_anchor_GT=I、T_marker_IMU=I、scale=1/offset=0；
不允许假定 bias/anchor/lever，不改变默认来源检查。逐行与汇总标注 ASSUMED_GEOMETRY_AND_CLOCK。
[新运行结果与复现](RANGE_IDENTITY_ASSUMPTION.md)；前文 UNAVAILABLE 为历史严格来源模式结果。
