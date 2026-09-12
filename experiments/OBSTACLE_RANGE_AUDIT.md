# 五数据集 obstacle / persistent positive range-error audit

状态：`AUDIT_COMPLETE_WITH_REFERENCE_LIMITATIONS`。2026-09-12；本轮只运行独立 evaluator，
核心 estimator、recovery、定位 benchmark、参数调整、提交/push 均 **NOT_RUN**。
任务边界和全部描述性阈值见 [预登记协议](/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie/experiments/OBSTACLE_RANGE_AUDIT_PROTOCOL.md)。

## 结论

1. **最值得优先用于真实持续偏置实验的是 STAR-loc，尤其 zigzag_s4 的 tag1→anchor7/6。**
   原始误差存在长平台，段外误差明显更低，几何重算与作者 Vicon reference 一致。
   这支持“真实 persistent positive range-error”，不等于已获得逐包 NLOS label 或证明 recovery 有益。
   MILUV nominal random/circular 也有持续正误差段；不能由 nominal 名称推断全部 LOS。
2. **cirObstacles_1_random3_0 没显示比默认 random 更强的总误差分布，并且采样太稀疏。**
   每 link median interarrival 2.545–2.569s，
   每 link仅 77–81 包。
   所有相邻包都超过固定1s continuity gap，最长严格连续段为0s；这是**连续性不可确认**，不是没有 NLOS。
   用户两次提到的同名 recording 只统计一次。CIR 没有读取。
3. **两个 own_vicon obstacle 及 HUEC 动态 NLOS 更适合候选 no-harm/selectivity 场景，而非当前强持续 NLOS 主证据。**
   自有数据约0.3–0.4m的持续正基线确实存在，nominal也有；未有独立 beta，不能把全部基线归为动态 NLOS。
   15:46:27 有短促异常，15:48:55更稳定；两条均没有 >0.5m、持续≥2s且≥5包的连续段。
   HUEC四条 NLOS 的 raw median 较小，但少量大负误差会抬高 RMSE；不应称为“全体测距准确”。
   本轮没有定位运行，因此 no-harm 只是后续实验用途建议，**不是已证明无伤害**。
4. **存在明确的数据解释/几何风险。** STAR-loc v1 radio/marker ID 不同；自有旧配置与bag anchor明显不符；
   HUEC GNSS高度基准需+1m，个别参考高度异常；自有两bag缺index，静态CSV含footer/损坏行。
   这些问题和参考类型不符项均保留，不能解释成 positive NLOS。

## 范围和指标

| 数据类 | recordings | 源观测行 | 有效正测距 | 可评残差 |
| --- | --- | --- | --- | --- |
| HUEC_dynamic | 8 | 61702 | 61702 | 61651 |
| HUEC_static | 767 | 67107 | 67105 | 0 |
| MILUV | 3 | 25013 | 25013 | 25010 |
| SFUISE | 3 | 17750 | 17750 | 0 |
| own_vicon | 3 | 19046 | 19046 | 18883 |
| starloc | 22 | 463345 | 463345 | 463345 |

共 806 条本地 recording/静态配置文件，1078 条 tag–anchor 汇总、996 条 anchor 汇总。
“源观测行”已排除明确标记的统计footer，但保留无效/空白记录；valid_ranges另列。
HUEC静态缺 13 个预期height/distance组合文件，见 [缺项清单](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/missing_static_grid.json)。
缺文件不能直接计为无线丢包。SFUISE 是 Vive，静态HUEC是激光参考，均不符合本轮限定的 Vicon/GNSS；
它们已接入count/dropout审计，error/persistence输出NA。没有使用别的传感器补GT。

统一 `e_raw=z_raw-||p_tag_GT-a_GT||`，无 beta subtraction、无demean、无range/GT拟合外参或时钟；
它包含固定偏置、动态excess、测量噪声及参考误差，**不是 latent bias truth**，也不是post-fit residual。
主表P95为绝对误差P95；CSV另有signed P95及positive-tail条件均值/P95、e>0/0.2/0.5/1m计数和比例。
最长段逐tag–anchor计算；e≤门槛、无效range/GT或gap>1s中断，anchor/recording汇总取link最长，不跨tag串段。
“明显持续”仅为预登记描述：e>0.5m且持续≥2s、N≥5；不是检测器、置信保证或NLOS真值标签。
所有图为全原始有效误差散点，未用折线跨掉包连接、未剪掉大误差；时间从各recording首UWB起算。

## 哪些 recording / link 有明显持续正误差

以下按持续时间展示12条代表link；完整 qualifying 集合与起止位置见
[persistent_links.csv](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/persistent_links.csv)，同时保留left/right-censored标记。
“段外中位数”只作离线描述对照，不作为减偏置、校准或 estimator preprocessing。

| 数据类 | recording | tag | anchor | 起点 s | 终点 s | 持续 s | N | 段均值 m | 段外中位数 m |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| starloc | zigzag_s4 | 1 | 7 | 0.0241 | 19.1704 | 19.1462 | 549 | 0.7279 | 0.3425 |
| starloc | zigzag_s4 | 1 | 6 | 78.4582 | 93.1261 | 14.6679 | 409 | 0.6141 | 0.3767 |
| starloc | loop-3d_s3 | 1 | 6 | 63.1619 | 72.0156 | 8.8537 | 262 | 0.7548 | 0.2682 |
| starloc | zigzag_s3 | 1 | 6 | 123.2420 | 130.7239 | 7.4820 | 223 | 0.8523 | 0.2667 |
| starloc | zigzag_s2 | 2 | 6 | 29.5835 | 36.8374 | 7.2539 | 101 | 0.7679 | 0.2085 |
| starloc | loop-2d_s2 | 1 | 6 | 67.3976 | 74.3210 | 6.9234 | 96 | 0.7877 | 0.3431 |
| starloc | eight_s3 | 1 | 6 | 59.1291 | 65.7942 | 6.6650 | 185 | 0.7628 | 0.5664 |
| starloc | apriltag_s3 | 1 | 7 | 99.6595 | 106.1304 | 6.4710 | 189 | 0.6381 | 0.2260 |
| MILUV | default_1_random3_0 | 10 | 3 | 172.3773 | 178.6939 | 6.3166 | 38 | 0.8627 | 0.1256 |
| starloc | loop-2d_s3 | 1 | 6 | 45.5595 | 51.5778 | 6.0184 | 181 | 0.7270 | 0.2289 |
| starloc | zigzag_s2 | 1 | 6 | 48.4868 | 54.5030 | 6.0162 | 88 | 0.8394 | 0.2781 |
| starloc | loop-3d-z_s3 | 1 | 7 | 44.5865 | 50.3634 | 5.7770 | 176 | 0.6714 | 0.2245 |

满足描述条件的全部 recording：starloc: apriltag_s3, eight_s2, eight_s3, ell_s3, grid_s3, loop-2d-fast_s2, loop-2d-fast_s3, loop-2d_s1, loop-2d_s2, loop-2d_s3, loop-2d_s4, loop-3d-z_s3, loop-3d_s2, loop-3d_s3, zigzag_s2, zigzag_s3, zigzag_s4; MILUV: default_1_circular3D_0, default_1_random3_0; own_vicon: 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle。

zigzag_s4 anchor7最长段从记录开头出现，属于left-censored，不能声称观测到了真实onset；
anchor6有记录内部的平台，更适合包含前后正常参考的实验。s3的loop-3d/zigzag和MILUV默认random/circular
可作为第二组真实候选。建议保留完整 recording 与其他anchor，不只导出漂亮片段。
这些序列已经因本轮GT审计产生development曝光，不能随后当未见held-out test。

## nominal/default 与 obstacle 对照

| 数据类 | recording | N raw | N error | median m | mean m | RMSE m | absolute P95 m | >0.5m % | 最长>0.5m s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| MILUV | cirObstacles_1_random3_0 | 954 | 954 | 0.1244 | 0.1944 | 0.3180 | 0.7914 | 11.2159 | 0.0000 |
| MILUV | default_1_circular3D_0 | 9515 | 9512 | 0.1061 | 0.2177 | 0.3905 | 0.9394 | 17.2624 | 3.9071 |
| MILUV | default_1_random3_0 | 14544 | 14544 | 0.1121 | 0.1783 | 0.3088 | 0.7800 | 10.4648 | 6.3166 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 14479 | 14344 | 0.3172 | 0.3178 | 0.4432 | 0.4555 | 2.6980 | 2.2403 |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | 2258 | 2234 | 0.3568 | 0.3673 | 0.3935 | 0.4837 | 4.1629 | 0.8399 |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | 2309 | 2305 | 0.3628 | 0.3672 | 0.3696 | 0.4368 | 0.0000 | 0.0000 |
| HUEC_dynamic | LOS_Trajectory_A_Case_1 | 8405 | 8397 | 0.0524 | 0.0196 | 0.5615 | 0.3459 | 0.1191 | 0.5021 |
| HUEC_dynamic | LOS_Trajectory_A_Case_2 | 8219 | 8203 | 0.1021 | 0.0583 | 0.7681 | 0.4138 | 0.4511 | 1.2992 |
| HUEC_dynamic | LOS_Trajectory_B_Case_3 | 6645 | 6637 | -0.0138 | 0.0037 | 0.5684 | 0.3511 | 0.1205 | 0.3000 |
| HUEC_dynamic | LOS_Trajectory_B_Case_4 | 7253 | 7253 | 0.0211 | 0.0337 | 1.0363 | 0.3195 | 0.0965 | 0.2964 |
| HUEC_dynamic | NLOS_Trajectory_A_Case_1 | 9447 | 9439 | 0.1179 | 0.0698 | 0.8628 | 0.3599 | 0.1377 | 0.4012 |
| HUEC_dynamic | NLOS_Trajectory_A_Case_2 | 9156 | 9156 | 0.1326 | 0.0772 | 0.6896 | 0.3968 | 0.2840 | 0.5980 |
| HUEC_dynamic | NLOS_Trajectory_B_Case_3 | 6297 | 6294 | -0.0093 | 0.0010 | 0.4155 | 0.3287 | 0.0159 | 0.0000 |
| HUEC_dynamic | NLOS_Trajectory_B_Case_4 | 6280 | 6272 | 0.0734 | 0.0438 | 0.6182 | 0.3967 | 1.2277 | 1.2979 |

这是不同recording的原始分布比较，不是控制住姿态、距离、速度和采样率后的障碍因果效应。
MILUV obstacle与default random的median仅相差约厘米量级，default circular的尾部反而更重。
自有nominal的anchor1也存在约2秒正偏段及离散大异常，不能把no_obstacle名字当作零偏标定。
HUEC没有大持续正段，但negative mismatch需单独诊断；完整极值与无效计数见
[extreme_and_invalid.csv](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/extreme_and_invalid.csv)。

STAR-loc无统一“obstacle recording”标签。作者区分手持高度和Jackal安装：s1/s2/s3/s4/s5是setup，
不是每包传播类别；s2提高rig位置以增加LOS比例。相同轨迹跨setup可描述比较，不能混同为同运动配对。
详见 [作者setup说明](https://arxiv.org/html/2309.05518v1#S3.SS1)。

## 几何、时间与数据质量核验

- MILUV：锁定作者commit `399ad46f4e6e88c3c98b40aae33fe8e2c6719cff`，三条都用constellation0。GT为ifo001 Vicon body；
  天线位置为p+R·lever，tag10/11分别使用作者非零杆臂，不把body当tag或混用两个tag。
  `range_raw`在校正前保存，`range`经antenna-delay/power校正；本轮只统计前者。
  timeshift.yaml是共同epoch origin，CSV已经减过一次，不能再次给GT单独平移。
  线性位置/SLERP、不外推、GT bracket≤0.05s；与发布GT range的差异有少数局部异常，不能把两种插值声称精确相同。
  [作者预处理](https://github.com/decargroup/miluv/blob/399ad46f4e6e88c3c98b40aae33fe8e2c6719cff/preprocess/process_uwb.py)、
  [时间处理](https://github.com/decargroup/miluv/blob/399ad46f4e6e88c3c98b40aae33fe8e2c6719cff/preprocess/cleanup_csv.py)、
  [anchor](https://github.com/decargroup/miluv/blob/399ad46f4e6e88c3c98b40aae33fe8e2c6719cff/config/uwb/anchors.yaml)、
  [tag](https://github.com/decargroup/miluv/blob/399ad46f4e6e88c3c98b40aae33fe8e2c6719cff/config/uwb/tags.yaml)。
- STAR-loc：从作者测量时刻Vicon tag_pos重算距离；v1的8项radio→marker映射经过逐包GT几何恒等验证，
  v2/v3同号。全部重算与作者gt_range最大绝对差 2.66e-15m。v1映射来源是发布GT的身份解析，
  不是独立外参标定，严格只在evaluator使用，不能交给estimator作GT-derived adapter。
  由rig/tag点反算的body lever仅作一致性诊断；没有用于估计器。camera tf不用于本轮测距几何。
  [作者README](https://github.com/utiasASRL/starloc/blob/d3ad541/README.md)、
  [v1 marker文件](https://github.com/utiasASRL/starloc/blob/d3ad541/mocap/uwb_markers_v1.csv)。
- own_vicon：按用户声明近似共点，tas_uwb_0为移动参考、1..4为anchor；两端在各包时刻插值，
  不从旧配置搬anchor、不将测距拟合到GT。旧config与同bag Vicon anchor的距离差范围
  0.272–0.472m，
  见[逐anchor差异](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/own_anchor_config_discrepancy.csv)。未修改旧config。
  最大anchor坐标分量跨度 0.0193m；这含mocap抖动/异常，
  不是测距拟合证据。header同一epoch且无倒序，bag−header延迟已有记录，但这不能证明独立时钟偏移为零。
  两条obstacle原bag无索引，只对临时副本reindex；可读UWB窗口仅约11秒，不假设未封存尾部也已恢复。
- HUEC动态：作者trajectory.csv提供处理后的GNSS位置，作者评估对z加1m；若把z直接当tag高度会引入几何错误。
  /odometry/local_gps、pose.csv、LS/ESKF结果不能不加区分当成同一GT；本轮采用声明的trajectory.csv，未读取LS/ESKF输出。
  时间从ns转s，约8Hz GT采用≤0.25s bracket，未外推/搜索延迟。
  GNSS→UWB坐标转换和NTP由作者声明，独立杆臂/heading/同步不确定度未完整给出。
  LOS Case2存在z相对高度达到1.63m；原作者评估会按abs(z)<0.5筛掉一些GT，本轮保留并报告，
  不靠删不利参考降低RMSE。各条都有少量大负range/reference误差，其原因未定位，不能作为positive NLOS证据。
  [作者论文](https://www.nature.com/articles/s41597-025-05887-9)、
  [作者评估源码](https://github.com/cucudasluv/UWB-dataset/blob/main/Technical_validation/localization_error_analysis.py)。
- HUEC静态：4488行明确统计footer不算measurement；
  完全空白文件和截断行保留为invalid，原文件未修写。激光GT超出本轮限制，不能回答它们是否有持续正偏。
  Transmission/Reception counter仅报告首末采样之间的增量差，遇到reset/不一致则NA，见
  [static_counter_audit.csv](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/static_counter_audit.csv)；不把累计历史计数当本recording包数。
- SFUISE：仅absolute ToA测距count/dropout；Vive非Vicon/GNSS，且历史单位GT→anchor假设未闭合。
  因此不给新range误差或NLOS结论，保留三条UNAVAILABLE。不是数据读取失败。

## dropout 和统计文件

主交付：

- [逐recording总表](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/recording_summary.csv)
- [逐recording/anchor总表](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/anchor_summary.csv)
- [逐recording/tag/anchor完整表](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/link_summary.csv)
- [nominal/obstacle对照](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/nominal_obstacle_comparison.csv)
- [GT关联状态](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/association_status.csv)
- [全部几何、GT/IMU时序与header延迟](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/geometry_audit.json)

link表包含N、median/mean/RMSE/absolute和signed P95、positive tails、四阈值最长段、
interarrival median/P95/max、>1s gap数量/总时间、>3×median gap、cadence缺样本估计、首尾覆盖缺口。
无真实发包计划时cadence估计不是packet-loss rate；MILUV CIR记录的慢轮询也不能写成大量无线丢包。
未匹配GT和无效range分别计数；e不可用时最终summary的最长段为NA。

## 诊断图

- [MILUV/cirObstacles_1_random3_0](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/MILUV/cirObstacles_1_random3_0/range_error.png)
- [MILUV/default_1_circular3D_0](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/MILUV/default_1_circular3D_0/range_error.png)
- [MILUV/default_1_random3_0](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/MILUV/default_1_random3_0/range_error.png)
- [own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle/range_error.png)
- [own_vicon/2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/own_vicon/2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle/range_error.png)
- [own_vicon/2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/own_vicon/2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle/range_error.png)
- [HUEC_dynamic/LOS_Trajectory_A_Case_1](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/HUEC_dynamic/LOS_Trajectory_A_Case_1/range_error.png)
- [HUEC_dynamic/NLOS_Trajectory_A_Case_1](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/HUEC_dynamic/NLOS_Trajectory_A_Case_1/range_error.png)
- [starloc/loop-2d_s1](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc/loop-2d_s1/range_error.png)
- [starloc/loop-2d_s2](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc/loop-2d_s2/range_error.png)
- [starloc/loop-2d_s3](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc/loop-2d_s3/range_error.png)
- [starloc/loop-2d_s4](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc/loop-2d_s4/range_error.png)
- [starloc/zigzag_s4](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/recordings/starloc/zigzag_s4/range_error.png)

## 执行证据与限制

最终run：`20260912T111904Z-cbdedc41`；evaluator exit0，806条处理完成，0个执行失败，源SHA前后0变化。
9/9工程测试通过，包含杆臂/符号、gap/缺失GT断段、坏geometry拒绝、footer/损坏行、全无效recording保留。
独立summary再核验所有recording行数/obs_id唯一性/RMSE及每个最长事件成员，exit0。

实际命令：

```bash
/usr/bin/python3 experiments/scripts/test_obstacle_range_audit.py
/usr/bin/python3 experiments/scripts/audit_obstacle_ranges.py
/usr/bin/python3 experiments/scripts/summarize_obstacle_audit.py /home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41
```

[执行manifest](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/execution.json)、[source SHA](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/source_hashes.json)、
[固定官方来源下载ledger](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/sources/ledger.json)、[逐recording验证](/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/20260912T111904Z-cbdedc41/report-da32f56f/validation.json)。
原始逐观测CSV和13幅图均在evaluator_private，未导出至estimator cache。
两次工程失败历史保留：首轮751个静态解析失败且v1同号几何结果作废；第二轮2个无时间行处理失败。
报告生成首轮因全NA列的读取类型检查失败（exit1），原日志保留；显式数值类型处理后复核通过，
未修改观测或统计值。执行日志位于仓库 experiments/evidence/obstacle_range_audit_20260912/。
最终版本没有更改阈值、选取新时窗或删掉失败数据来优化结论。

本报告是数据资格和原始误差审计，不是全大规模定位实验、恢复收益报告、独立beta标定或正式held-out验证。
T10=C2-C、T11=C、C1–C3不升级。
