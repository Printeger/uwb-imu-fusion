# ICRA 数据集 manifest 与统一数据合同

任务卡：`0912-ICRA-FIVE-DATASET-VALIDATION`，2026-09-12。
本合同补充[实验合同](../doc/ie_sprint/EXPERIMENT_CONTRACT.md)，不改变[方法合同](../doc/ie_sprint/METHOD_CONTRACT.md)、estimator、CUSUM、NLOS recovery 或科学参数。

当前交付为 `FIVE_FAMILY_MANIFEST_VALIDATION=PASS / EXPERIMENT_ADMISSION=NOT_GRANTED / ESTIMATOR=NOT_RUN`。
这里的 PASS 仅表示五类本地 recording 已进入 inventory，schema、身份、路径、选定内容字段和 SHA-256 静态检查通过；它不表示某条记录已获正式实验准入，也不表示定位精度或同步/标定质量通过。

## 文件与版本

- [dataset_manifest.json](datasets/dataset_manifest.json)：唯一运行时读取的 inventory manifest，`icra_dataset_manifest_v2`。
- [dataset_manifest.schema.json](datasets/dataset_manifest.schema.json)：JSON Schema draft-07；未知字段报错。
- [five_dataset_metadata_observed.json](datasets/five_dataset_metadata_observed.json)：39 条 recording 的只读 bag/CSV 内容盘点证据。
- [inspect_dataset_metadata.py](scripts/inspect_dataset_metadata.py)：只读 metadata/选定字段扫描；不调用 estimator。
- [build_dataset_manifest.py](scripts/build_dataset_manifest.py)：显式重建 manifest，只写 manifest 文件，不修改 `data/`。
- [validate_manifest.py](scripts/validate_manifest.py)：schema、语义、路径及 SHA-256 静态检查；不写任何文件。
- [DATA_VALIDATION.md](DATA_VALIDATION.md) 与 [VERIFICATION.md](VERIFICATION.md)：本任务范围、实际命令、失败和结果。

顶层 `inventory_artifact` 用 SHA-256 绑定内容盘点证据。每条 recording 的 `local_validation.artifact_pointer` 指向该证据中的对应节点。`recording_files` 绑定构成 recording 的 bag 或 UWB/IMU/GT/时间/标定/锚点文件；`raw_data_path` 是主 UWB 文件或包含全部传感器的 bag。

## Recording 必填合同

| 字段 | 合同 |
|---|---|
| `dataset_family`, `recording_id` | family 限 HUEC/MILUV/own_vicon/SFUISE/starloc；二元身份唯一 |
| `raw_data_path`, `raw_sha256`, `recording_files` | 仓库相对 `data/<family>/...` 路径及文件 SHA-256；不得用共享 `latest` 或目录名代替文件身份 |
| `uwb_measurement_type`, `measurement_type_evidence` | 只允许 `ABSOLUTE_TOA`、`ABSOLUTE_TWR`、`ABSOLUTE_RANGE`；必须给本地字段或作者来源证据 |
| `tag_ids`, `anchor_ids` | `{status,value,evidence,notes}` fact；已知 ID 为非空唯一字符串数组，未知必须显式为 null |
| `anchor_coordinates_source` | 文件/topic、frame、单位和来源；独立 survey 未闭合时不得冒充已闭合 |
| `imu_source` | 文件/topic、设备或 frame、加速度/角速度单位；不确定的设备缩放明确保留 |
| `gt_source`, `gt_reference_point` | evaluator 专用来源和实际被跟踪原点；不能只写 Vicon/Vive/GNSS 名称 |
| `tag_imu_extrinsics`, `gt_extrinsics` | 变换值、方向和来源；缺失时为 UNKNOWN，不以零或 identity 填充 |
| `time_units` | UWB/IMU/GT 分别记录 s/ms/us/ns；无换算定义的 ticks 不接受 |
| `time_synchronization` | 各时间字段、公共秒映射、epoch 与 offset/drift 证据；共享时间列不自动证明同步准确 |
| `range_calibration` | 原始/处理列、单位、已有校正、固定 beta 来源及独立性；GT/bias 列只准 evaluator 使用 |
| `physical_obstruction` | 材料、link、时间和标注来源（若存在）；目录或文件名只算 recording-level 声明 |
| `role`, `split_group_id` | DEV/VAL/TEST；开发暴露未知时只能 `UNASSIGNED` 且 group 为 null |
| `used_during_method_development`, `development_use_evidence` | true/false/null；true 必须 DEV，未知不能假定 false 或分配 TEST |
| `metadata_status`, `local_validation` | 本地内容 PASS 或带已登记问题的 PASS；与正式实验 admission 分离 |
| `formal_admission`, `estimator_runs_this_task` | 本版本固定 `NOT_ADMITTED`、`NOT_RUN` |
| `license`, `unresolved_items` | 本地许可证据和所有未闭合项，不因 validator exit 0 删除 |

Fact 状态统一为 `OBSERVED`、`SOURCE_DECLARED`、`UNKNOWN`、`NOT_APPLICABLE`。前两者必须有非 null 值；后两者必须为 null。`OBSERVED` 只说明本地内容扫描到，`SOURCE_DECLARED` 只说明配置、README 或作者材料有声明，两者都不自动证明标定独立、物理真值或论文准入。

## 明确拒绝 TDoA

本论文只接受单 tag–anchor 的绝对 ToA/TWR/range。拒绝 TDoA、time/range difference、anchor-pair distance difference，以及通过改列名、取绝对值或改单位伪装的差分数据。

该约束由三层实现：schema 的 `uwb_measurement_type` 只有三个绝对量枚举；manifest 顶层显式列出 rejected 类型；validator 独立检查 policy 和每条 recording。测试覆盖 `TDoA/TDOA/tdoa/TDOA_RANGE_DIFFERENCE/UNKNOWN`，这些输入均导致失败。混合数据源未来只能由独立 adapter allowlist 选择绝对 range stream，不能把 TDoA 混入。

## 五类本地 inventory 结果

| family | recording 数 | split/开发暴露 | 本地内容结果 | 未闭合边界 |
|---|---:|---|---|---|
| HUEC | 8 | 全部 UNASSIGNED/UNKNOWN | 8 个动态 `combined.bag` 全量扫描 UWB/IMU/GT；1 条 anchor5 消息的 RSSI 为 `-inf`，range 本身有限 | bag 无 tag ID；独立锚点 survey、杆臂、GNSS 参考点外参、clock、beta、split |
| MILUV | 3 | random/circular 为 DEV/used=true；cirObstacles UNASSIGNED/UNKNOWN | `uwb_range.csv`、`imu_px4.csv`、`mocap.csv` 全行结构/有限性/时间检查通过 | raw `range_raw` 与项目 loader 所用 `range` 的正式选择、timeshift 应用、外参、许可；cirObstacles 锚点来源 |
| own_vicon | 3 | no-obstacle 为 DEV/used=true；两个 obstacle UNASSIGNED/UNKNOWN | no-obstacle 原 bag 全量通过；两个 obstacle 原 bag 无 index，在临时副本 reindex 后分别恢复约 11.6/11.7 s 并全量扫描，原文件未改 | 原 bag 完整性、IMU 单位、锚点 survey、Vicon/tag/IMU 外参、clock、beta、许可 |
| SFUISE | 3 | Walk1/2/3 全部 DEV/used=true | UWB、anchor、IMU、GT 所有消息的身份、有限性和时间顺序通过 | 独立 survey、Vive 外参、clock、beta、bag 许可；均仍 NOT_ADMITTED |
| starloc | 22 | 全部 UNASSIGNED/UNKNOWN | 22 组 `uwb.csv`、`imu.csv`、`calib.json` 与版本化 anchor 文件通过结构/有限性/时间检查 | tag–IMU 外参、混合 IMU frame 的 adapter 语义、survey 不确定度、clock、beta、split、许可 |

共 39 条 recording：36 条 `LOCAL_CONTENT_VALIDATED`，3 条 `LOCAL_CONTENT_VALIDATED_WITH_RECORDED_ISSUES`。后者是 HUEC 的一条非有限辅助 RSSI 和两个原始 own_vicon bag 无索引。它们没有被删除、修写或当作正式实验可用性 PASS。

当前角色为 6 条 DEV、33 条 UNASSIGNED、0 条 VAL、0 条 TEST。DEV 包括 ISAS Walk1/2/3、MILUV random/circular 和 own_vicon no-obstacle。所有时间窗、prefix、低冗余版本和注入派生数据必须继承基础 recording 的 split group；未知开发暴露阻止分配正式 VAL/TEST。

## 统一输入与真值隔离合同

1. 原始测距不可覆盖。规范观测至少保存 `recording_id, source_message_ordinal, range_ordinal, obs_id, tag_id, anchor_id, timestamp_s, z_raw_m`，以及原始质量字段和所有选择 mask。重复时间戳、裁剪、排序、缓存和方法切换不能重编号。
2. suspected NLOS 在候选阶段前保留。format/range/known-anchor/keyframe/candidate/final/fallback mask 分列记录，不能以缺行代替失败。
3. 统一长度 m、时间 s、加速度 m/s²、角速度 rad/s；同时保存原始单位和换算。frame、轴向、四元数 xyzw 与变换方向必须明确，未知变换不填 identity。
4. `t_common_s=scale*t_native+offset_s`；若有 drift，记录模型和来源。prefix 必须在初始化前裁剪所有输入，不能用全 bag anchor 平均、未来 GT 插值或未来同步信息。
5. 固定静态 `beta`、动态分段 `c_s` 和 IMU bias 分开。残差仍为 `h + beta + dynamic_offset - z_raw`。缺独立 LOS beta 时记录 `MISSING_CALIBRATION`，不能描述成已标定零值。
6. GT、obstruction、bias/gt_range、oracle/injection 列只供独立 evaluator。MILUV/starloc 这类同 CSV 含测量和 GT 的数据，正式 adapter 必须先生成 measurement-only 输入并记录列 allowlist 与 hash。
7. bias truth、`z-h(GT)-beta` 带噪参考与 post-fit residual 分开。inventory PASS 不改变最终图、Values、残差与协方差合同。

## PASS 与正式准入边界

本任务 PASS 的判据是：五类 family 均有实际 recording；39 条主/支持文件路径和 SHA-256 可核对；必填字段齐全；TDoA 被拒绝；已知开发暴露不泄漏为 TEST；未知 metadata 和三项本地问题被结构化保留；validator 与工程反例通过。

正式实验准入仍需另立锁定协议，闭合所选 recording 的独立 survey、外参、同步、range/beta 标定、许可、split 和 measurement-only adapter。未来每个 run 使用 `experiments/results/<run_id>/` 隔离输出并保留失败、fallback 和 zero coverage；不能根据结果调参数。本任务没有启动这些工作。

## 0912 range evaluator 来源补充

[官方源代码审计与私有 evaluator](RANGE_EVALUATOR.md) 已补充 Walk1 的作者 lever 和
static beta 数值/符号。beta=-toa_offset；只供 evaluator，estimator 空 beta 表未变。
官方来源未给出闭合的 GT→anchor、marker→IMU 变换或独立时钟标定，range 数字仍不可用。
前述“未找到 beta 来源”为最初盘点快照，此节与 Walk1 manifest notes 是后续更新；其他 recording 不升格。
