# UWB–IMU 主流程精度与耗时报告

> 本报告由 `tools/run_dataset_benchmark.py report` 自动生成。
> 标定口径：GT-assisted interface calibration；GT 未作为状态初值或算法调参输入。
> SFUISE 保留官方 UWB 残差剔除，因此不是与 current nominal 完全对称的算法边界。

## 结论摘要

- current 主表有效/总计：36/40；官方 SFUISE baseline：33/40；current 双标签补充：10/11。
- 数值只汇总 coverage ≥80% 且运行成功的序列；有限但显著发散的轨迹仍按实验合同保留，可由 APE max、时间序列和轨迹图识别。

## 数据接口审计

| 数据族 | IMU 接口与单位 | 坐标/杆臂 | UWB/anchor | GT 与标定口径 |
|---|---|---|---|---|
| HUEC | `/imu/data`；acc m/s²、gyro rad/s（不执行错误的 deg→rad） | body；tag lever `[-0.055,-0.055,0.668]` m | 4 anchor topics，topic 自带坐标 | `/odometry/local_gps` tag pose；数据自带标定 |
| MILUV | `imu_px4.csv`；m/s²、rad/s | PX4 body；官方 tag10/11 杆臂 | processed `range`；官方 constellation | mocap body pose 投影到 tag；官方标定 |
| SFUISE | Waveshare IMU；m/s²、rad/s | IMU/body；lever `[0.1,-0.025,0]` m | `/rtls_flares` + anchor list；Walk ToA offset | VIVE body 投影到 tag 后拟合 VIVE→anchor world 固定变换（GT-assisted） |
| own_vicon | Livox acc ×9.80665；gyro rad/s | 联合拟合 IMU→tag 固定旋转/杆臂/时延 | bag 内 anchor 1–4 Vicon 稳健中值 | tag0；三录制联合 GT-assisted interface calibration；原 bag 不修改 |
| STAR-Loc | acc 已为 rig m/s²；gyro rad/s | `C_r_i=C_c_rᵀC_c_i`；CAD tag 杆臂 | setup 选择 v1/v2/v3；s1–s4 `range_calib`，s5 `range`；v1 marker ID 固定映射到 radio ID | `uwb.csv` 同时刻官方 `tag_pos_*`；官方几何，v1 ID 对应由 v2 房间几何恢复 |
| simulation | 200 Hz m/s²、rad/s | body；冻结 tag 杆臂 | 8 anchors；seed 20260901；无丢包/NLOS | 解析真值仅供评价；measurement bootstrap |

Preflight：**PASS**；发现 40 条缓存。

| 数据集 | 序列 | 状态 | IMU | UWB | IMU Hz | 初始 | GT覆盖 | 距离几何P95 [m] |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| HUEC | LOS_Trajectory_A_Case_1 | PASS | 47047 | 8405 | 200.1 | 9.778 | 0.988 | 0.414 |
| HUEC | LOS_Trajectory_A_Case_2 | PASS | 51038 | 8219 | 200.2 | 9.725 | 0.984 | 0.339 |
| HUEC | LOS_Trajectory_B_Case_3 | PASS | 37019 | 6645 | 200.2 | 9.750 | 0.985 | 0.464 |
| HUEC | LOS_Trajectory_B_Case_4 | PASS | 40020 | 7253 | 200.2 | 9.726 | 0.978 | 0.550 |
| HUEC | NLOS_Trajectory_A_Case_1 | PASS | 62927 | 9447 | 200.2 | 9.757 | 0.992 | 0.322 |
| HUEC | NLOS_Trajectory_A_Case_2 | PASS | 55523 | 9156 | 200.2 | 9.721 | 0.981 | 0.311 |
| HUEC | NLOS_Trajectory_B_Case_3 | PASS | 34934 | 6297 | 200.2 | 9.753 | 0.984 | 0.437 |
| HUEC | NLOS_Trajectory_B_Case_4 | PASS | 35646 | 6280 | 200.2 | 9.727 | 0.986 | 0.483 |
| MILUV | cirObstacles_1_random3_0 | PASS | 40941 | 954 | 249.1 | 9.809 | 1.000 | 0.755 |
| MILUV | default_1_circular3D_0 | PASS | 27738 | 9515 | 249.3 | 9.806 | 1.000 | 0.813 |
| MILUV | default_1_random3_0 | PASS | 42317 | 14544 | 249.4 | 9.804 | 1.000 | 0.718 |
| SFUISE | ISAS-Walk1 | PASS | 4839 | 4850 | 81.9 | 9.981 | 1.000 | 0.252 |
| SFUISE | ISAS-Walk2 | PASS | 6229 | 6205 | 81.9 | 9.968 | 1.000 | 0.263 |
| SFUISE | ISAS-Walk3 | PASS | 6684 | 6695 | 81.9 | 9.959 | 1.000 | 0.243 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | PASS | 14680 | 14479 | 202.2 | 9.715 | 1.000 | 0.460 |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | PASS | 2279 | 2258 | 201.5 | 9.706 | 1.000 | 0.483 |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | PASS | 2313 | 2309 | 202.3 | 9.708 | 0.998 | 0.437 |
| simulation | figure_eight_nominal_seed_20260901 | PASS | 24601 | 19672 | 200.0 | 9.807 | 1.000 | 0.196 |
| starloc | apriltag_s3 | PASS | 13396 | 27056 | 202.2 | 9.776 | 1.000 | 0.371 |
| starloc | eight_s2 | PASS | 5383 | 10873 | 202.2 | 9.685 | 1.000 | 0.610 |
| starloc | eight_s3 | PASS | 11264 | 23256 | 202.2 | 9.738 | 1.000 | 0.606 |
| starloc | ell_s3 | PASS | 12535 | 26481 | 202.2 | 9.617 | 1.000 | 0.475 |
| starloc | grid_s3 | PASS | 13055 | 27130 | 202.2 | 9.753 | 1.000 | 0.380 |
| starloc | loop-2d-fast_s1 | PASS | 3989 | 7602 | 202.2 | 9.752 | 1.000 | 0.619 |
| starloc | loop-2d-fast_s2 | PASS | 3989 | 7725 | 202.2 | 9.733 | 1.000 | 0.845 |
| starloc | loop-2d-fast_s3 | PASS | 6288 | 13187 | 202.2 | 9.764 | 1.000 | 0.174 |
| starloc | loop-2d-v2_s5 | PASS | 9947 | 3340 | 202.2 | 9.764 | 1.000 | 0.155 |
| starloc | loop-2d_s1 | PASS | 7409 | 15374 | 202.2 | 9.764 | 1.000 | 0.573 |
| starloc | loop-2d_s2 | PASS | 9900 | 19528 | 202.2 | 9.793 | 1.000 | 0.697 |
| starloc | loop-2d_s3 | PASS | 12013 | 24827 | 202.2 | 9.762 | 1.000 | 0.245 |
| starloc | loop-2d_s4 | PASS | 25378 | 41607 | 202.2 | 9.763 | 1.000 | 0.314 |
| starloc | loop-2d_s5 | PASS | 7985 | 2680 | 202.2 | 9.753 | 1.000 | 0.165 |
| starloc | loop-3d-z_s3 | PASS | 17018 | 35640 | 202.2 | 9.754 | 1.000 | 0.466 |
| starloc | loop-3d_s1 | PASS | 10021 | 19762 | 202.2 | 9.767 | 1.000 | 0.326 |
| starloc | loop-3d_s2 | PASS | 9125 | 17156 | 202.2 | 9.758 | 1.000 | 0.897 |
| starloc | loop-3d_s3 | PASS | 11508 | 23566 | 202.2 | 9.813 | 1.000 | 0.438 |
| starloc | loop-3d_s5 | PASS | 4158 | 1404 | 202.2 | 9.576 | 1.000 | 0.174 |
| starloc | zigzag_s2 | PASS | 8704 | 17970 | 202.2 | 9.774 | 1.000 | 1.488 |
| starloc | zigzag_s3 | PASS | 23096 | 47372 | 202.2 | 9.738 | 1.000 | 0.484 |
| starloc | zigzag_s4 | PASS | 36538 | 49809 | 202.2 | 9.762 | 1.000 | 0.448 |

### GT-assisted 固定接口标定

- own_vicon：联合 3 条录制，时延 3.115 ms，杆臂 `[0.008358, -0.001873, 0.044515]` m，杆臂 1σ `[0.002116, 0.002327, 0.001694]` m；gyro/accel 拟合 RMSE 分别为 0.3776 rad/s、0.6160 m/s²。
- SFUISE：联合 3 条 Walk 的缺失 VIVE→anchor world 变换，保留/剔除 3158/180 个拟合样本，残差 RMSE/median 为 0.1998/0.1615 m。
- STAR-Loc v1：`uwb_markers_v1.csv` 首列为 Vicon marker ID，数据量测使用 radio ID；按 v2 不变房间几何冻结映射 `10→11, 11→7, 12→4, 13→10, 15→5, 16→6, 6→12, 9→9`。`gt_range` 仅用于映射后的独立残差审计，未参与拟合。

接口规则包括：HUEC 保持原始 m/s²、rad/s；MILUV 使用 PX4 与官方 constellation/tag 杆臂；SFUISE 使用官方 ToA offset；own_vicon 使用三条录制联合 GT 辅助接口标定；STAR-Loc 旋转角速度、恢复 v1 marker/radio ID 对应，并按 setup 选 anchor/range 字段。

## 族级汇总

| 数据集 | 方法 | 有效序列 | 序列等权 APE RMSE [m] | pooled APE RMSE [m] | 序列等权 RPE-1s RMSE [m] | pooled RPE-1s RMSE [m] |
|---|---|---:|---:|---:|---:|---:|
| HUEC | current | 5 | 1309.7956 | 1986.1334 | 1773.9051 | 2675.9035 |
| HUEC | sfuise | 7 | 25.0422 | 46.9667 | 2.8541 | 3.8920 |
| MILUV | current | 2 | 0.9960 | 1.0136 | 1.0770 | 1.0721 |
| SFUISE | current | 3 | 0.5916 | 0.6395 | 0.9891 | 1.0462 |
| SFUISE | sfuise | 3 | 0.1435 | 0.1434 | 0.5785 | 0.5733 |
| own_vicon | current | 3 | 1.0358 | 1.1950 | 0.3666 | 0.5255 |
| own_vicon | sfuise | 3 | 0.9843 | 1.1360 | 0.2224 | 0.4250 |
| simulation | current | 1 | 0.0991 | 0.0991 | 1.1426 | 1.1426 |
| simulation | sfuise | 1 | 0.2837 | 0.2837 | 0.2300 | 0.2300 |
| starloc | current | 22 | 1.1524 | 2.3290 | 1.7873 | 3.4033 |
| starloc | sfuise | 19 | 12363.0484 | 18917.9931 | 429.9440 | 528.9895 |

## 逐序列精度结果

主指标为原 anchor/world 坐标系、相同 UWB 标签点上的 APE；coverage < 80% 不进入汇总。

| 数据集 | 序列 | 方法 | 状态/原因 | Coverage | APE RMSE | mean | median | P95 | max | 对齐 APE | RPE-1s 平移 RMSE/median/P95 | RPE-1s 旋转 RMSE/median/P95 [deg] | 姿态 APE RMSE [deg] |
|---|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| HUEC | LOS_Trajectory_A_Case_1 | current | SUCCESS | 0.983 | 3226.5968 | 244.8687 | 1.1005 | 9.4471 | 74126.0551 | 3224.8751 | 4275.5638/1.7266/20.9252 | 54.9230/12.4682/136.8785 | 111.6853 |
| HUEC | LOS_Trajectory_A_Case_1 | sfuise | SUCCESS | 0.983 | 96.7842 | 31.0915 | 1.4021 | 253.5184 | 523.1514 | 88.4995 | 7.0312/1.4906/21.2752 | 22.6573/3.7043/55.7854 | 131.4147 |
| HUEC | LOS_Trajectory_A_Case_2 | current | RuntimeError: RUN_FAILED: commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551353348 (Symbol: x1028).  Thrown when ... | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| HUEC | LOS_Trajectory_A_Case_2 | sfuise | SUCCESS | 0.909 | 71.5871 | 12.7387 | 1.7206 | 2.4148 | 585.4313 | 70.1814 | 6.5771/0.9404/2.3514 | 19.1416/3.2163/49.7111 | 176.4148 |
| HUEC | LOS_Trajectory_B_Case_3 | current | SUCCESS | 1.000 | 833.9228 | 69.5909 | 1.1859 | 7.0958 | 16215.7890 | 833.3729 | 1153.8206/1.6399/24.2565 | 53.6939/12.6177/138.3737 | 108.7379 |
| HUEC | LOS_Trajectory_B_Case_3 | sfuise | SUCCESS | 1.000 | 1.3764 | 1.3438 | 1.2719 | 1.9575 | 2.2007 | 0.3770 | 1.3862/0.9450/2.5800 | 21.1823/6.1281/58.8907 | 123.8195 |
| HUEC | LOS_Trajectory_B_Case_4 | current | RuntimeError: RUN_FAILED: commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551352705 (Symbol: x385).  Thrown when a... | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| HUEC | LOS_Trajectory_B_Case_4 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| HUEC | NLOS_Trajectory_A_Case_1 | current | RuntimeError: RUN_FAILED: commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551352889 (Symbol: x569).  Thrown when a... | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| HUEC | NLOS_Trajectory_A_Case_1 | sfuise | SUCCESS | 1.000 | 1.5710 | 1.4877 | 1.3852 | 2.4435 | 2.8236 | 0.5868 | 1.0267/0.6794/1.8892 | 32.5322/29.5662/53.1003 | 108.6092 |
| HUEC | NLOS_Trajectory_A_Case_2 | current | SUCCESS | 0.998 | 2394.0218 | 239.3413 | 1.7406 | 51.5435 | 53551.9317 | 2391.4519 | 3305.8506/1.5518/312.3033 | 61.5605/14.1018/145.9396 | 110.3397 |
| HUEC | NLOS_Trajectory_A_Case_2 | sfuise | SUCCESS | 0.998 | 1.3111 | 1.1877 | 1.0711 | 2.2723 | 3.1385 | 0.8179 | 1.1904/0.8858/2.1799 | 42.1885/36.0935/70.2640 | 109.2587 |
| HUEC | NLOS_Trajectory_B_Case_3 | current | SUCCESS | 0.997 | 74.1844 | 8.0783 | 1.2093 | 7.7201 | 1638.8562 | 74.1105 | 105.6401/1.3156/15.5720 | 57.5022/13.5842/144.4807 | 96.4502 |
| HUEC | NLOS_Trajectory_B_Case_3 | sfuise | SUCCESS | 0.998 | 1.2721 | 1.2278 | 1.1568 | 1.8844 | 2.7985 | 0.4492 | 1.4365/1.3501/2.4073 | 21.3561/6.3873/53.6740 | 119.3522 |
| HUEC | NLOS_Trajectory_B_Case_4 | current | SUCCESS | 1.000 | 20.2522 | 4.9533 | 1.4721 | 11.0830 | 271.0591 | 20.1729 | 28.6505/1.3282/34.8041 | 57.7073/12.9527/145.7468 | 91.2247 |
| HUEC | NLOS_Trajectory_B_Case_4 | sfuise | SUCCESS | 1.000 | 1.3933 | 1.1586 | 0.9079 | 2.9060 | 3.3107 | 0.5976 | 1.3307/1.1593/2.2156 | 21.3275/3.7355/62.8680 | 101.5649 |
| MILUV | cirObstacles_1_random3_0 | current | RuntimeError: RUN_FAILED: commit terminal at backend_update:  Indeterminant linear system detected while working near variable 7061644215716937915 (Symbol: b187).  Thrown when a... | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| MILUV | cirObstacles_1_random3_0 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| MILUV | cirObstacles_1_random3_0 | current_all_tags | RuntimeError: RUN_FAILED: commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551352723 (Symbol: x403).  Thrown when a... | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| MILUV | default_1_circular3D_0 | current | SUCCESS | 0.996 | 0.9235 | 0.7835 | 0.7484 | 1.6706 | 2.6535 | 0.6647 | 1.1017/1.0103/1.8154 | 80.3035/60.3218/152.3555 | 120.1189 |
| MILUV | default_1_circular3D_0 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| MILUV | default_1_circular3D_0 | current_all_tags | SUCCESS | 0.998 | 0.9626 | 0.8186 | 0.8209 | 1.6038 | 3.9522 | 0.8418 | 0.9059/0.5633/1.7847 | 61.8172/42.9701/119.4725 | 48.5613 |
| MILUV | default_1_random3_0 | current | SUCCESS | 0.997 | 1.0685 | 0.7743 | 0.5340 | 2.2848 | 3.8699 | 1.0608 | 1.0523/0.8121/1.7592 | 70.8560/47.2586/146.7777 | 121.2228 |
| MILUV | default_1_random3_0 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| MILUV | default_1_random3_0 | current_all_tags | SUCCESS | 0.997 | 0.9211 | 0.6552 | 0.3995 | 2.1950 | 2.7063 | 0.9190 | 0.5901/0.3031/1.1717 | 48.8069/28.1452/103.5557 | 36.8282 |
| SFUISE | ISAS-Walk1 | current | SUCCESS | 0.998 | 1.0575 | 0.2961 | 0.1623 | 0.6192 | 18.5445 | 1.0535 | 1.6986/0.5321/1.5119 | 44.6524/27.0654/82.9210 | 105.1350 |
| SFUISE | ISAS-Walk1 | sfuise | SUCCESS | 1.000 | 0.1572 | 0.1385 | 0.1200 | 0.2936 | 0.3637 | 0.1176 | 0.7278/0.5596/1.2956 | 21.5681/13.3887/42.4131 | 163.1921 |
| SFUISE | ISAS-Walk2 | current | SUCCESS | 0.999 | 0.2414 | 0.1960 | 0.1644 | 0.4289 | 1.0302 | 0.2321 | 0.4922/0.3881/0.8426 | 31.4634/21.9683/58.7306 | 102.3106 |
| SFUISE | ISAS-Walk2 | sfuise | SUCCESS | 1.000 | 0.1226 | 0.1148 | 0.1101 | 0.1842 | 0.2094 | 0.0902 | 0.5094/0.4469/0.8792 | 20.9287/14.2819/39.2868 | 177.7657 |
| SFUISE | ISAS-Walk3 | current | SUCCESS | 0.997 | 0.4760 | 0.2216 | 0.1712 | 0.4555 | 8.4315 | 0.4728 | 0.7764/0.4192/0.9145 | 48.1736/24.3815/113.8387 | 101.6131 |
| SFUISE | ISAS-Walk3 | sfuise | SUCCESS | 1.000 | 0.1509 | 0.1410 | 0.1408 | 0.2343 | 0.2565 | 0.0986 | 0.4982/0.4493/0.7948 | 25.5877/17.4162/46.1208 | 168.4001 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | current | SUCCESS | 0.999 | 1.2531 | 1.1878 | 1.2490 | 1.6180 | 4.3188 | 0.4652 | 0.5563/0.1834/0.7207 | 38.9486/9.0522/111.4122 | 79.8475 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | sfuise | SUCCESS | 0.999 | 1.2015 | 1.1631 | 1.2730 | 1.5527 | 1.6435 | 0.2996 | 0.4766/0.3322/0.9410 | 6.9620/4.0456/15.0573 | 49.3585 |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | current | SUCCESS | 0.989 | 1.1078 | 1.0410 | 0.8182 | 1.6935 | 1.8070 | 0.4039 | 0.5263/0.1976/1.1199 | 31.0036/12.4581/68.1079 | 165.9188 |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | sfuise | SUCCESS | 0.991 | 1.0485 | 0.9964 | 0.8006 | 1.7290 | 1.8053 | 0.3349 | 0.1880/0.0358/0.3759 | 4.0473/0.5484/9.4861 | 16.5206 |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | current | SUCCESS | 1.000 | 0.7466 | 0.7466 | 0.7478 | 0.7565 | 0.7598 | 0.0135 | 0.0173/0.0144/0.0296 | 6.0037/4.7181/10.5655 | 169.1406 |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | sfuise | SUCCESS | 0.991 | 0.7030 | 0.7030 | 0.7023 | 0.7048 | 0.7049 | 0.0030 | 0.0026/0.0022/0.0053 | 0.4856/0.0743/0.8641 | 172.5932 |
| simulation | figure_eight_nominal_seed_20260901 | current | SUCCESS | 0.999 | 0.0991 | 0.0899 | 0.0849 | 0.1682 | 0.2878 | 0.0989 | 1.1426/0.9304/2.0502 | 64.4396/47.4021/123.6353 | 91.5635 |
| simulation | figure_eight_nominal_seed_20260901 | sfuise | SUCCESS | 0.999 | 0.2837 | 0.2832 | 0.2818 | 0.3011 | 0.3780 | 0.0840 | 0.2300/0.2173/0.3464 | 0.4024/0.2598/0.7678 | 19.4238 |
| starloc | apriltag_s3 | current | SUCCESS | 1.000 | 0.4035 | 0.2947 | 0.1985 | 0.8434 | 2.0017 | 0.3465 | 0.6264/0.4644/1.1057 | 62.7241/39.1015/144.5517 | 145.9447 |
| starloc | apriltag_s3 | sfuise | SUCCESS | 1.000 | 21328.2136 | 15650.7336 | 11437.5120 | 43219.2278 | 48219.9804 | 14519.9187 | 679.3159/467.8980/836.7497 | 29.1045/21.0552/55.8022 | 124.2107 |
| starloc | eight_s2 | current | SUCCESS | 1.000 | 0.8591 | 0.6538 | 0.4531 | 1.8164 | 3.7961 | 0.5069 | 1.0496/0.9099/1.6555 | 78.7557/57.5796/150.3285 | 137.6219 |
| starloc | eight_s2 | sfuise | SUCCESS | 1.000 | 3110.8453 | 2099.9747 | 1075.0684 | 7010.4199 | 8122.9929 | 2335.6054 | 226.0674/120.6346/438.4416 | 43.0367/40.5816/62.3120 | 129.4469 |
| starloc | eight_s2 | current_all_tags | SUCCESS | 1.000 | 0.6010 | 0.4643 | 0.3506 | 1.3323 | 1.9831 | 0.4959 | 0.9746/0.7142/1.8954 | 86.5119/68.6911/156.5551 | 118.6915 |
| starloc | eight_s3 | current | SUCCESS | 0.999 | 0.4573 | 0.3761 | 0.3492 | 0.8759 | 1.9521 | 0.3241 | 0.7390/0.6178/1.2137 | 68.1940/44.1845/137.4084 | 145.7069 |
| starloc | eight_s3 | sfuise | SUCCESS | 0.999 | 11263.8276 | 8419.2714 | 6318.7310 | 22757.5084 | 25176.7059 | 7634.8124 | 584.6890/266.5793/515.6028 | 26.8894/26.7237/37.2342 | 131.9561 |
| starloc | ell_s3 | current | SUCCESS | 1.000 | 0.4234 | 0.3176 | 0.2165 | 0.8806 | 1.7453 | 0.3779 | 0.8575/0.7628/1.4144 | 65.8539/39.4161/140.0378 | 153.3871 |
| starloc | ell_s3 | sfuise | SUCCESS | 0.999 | 25048.7301 | 17139.5407 | 9979.0447 | 54175.0608 | 60990.8825 | 18395.7589 | 989.5682/525.5353/1219.1023 | 59.5953/57.8683/74.7197 | 139.6217 |
| starloc | grid_s3 | current | SUCCESS | 1.000 | 0.3984 | 0.3494 | 0.2853 | 0.7255 | 1.5562 | 0.2159 | 0.4505/0.1664/1.1036 | 56.5606/27.9672/135.3689 | 168.3142 |
| starloc | grid_s3 | sfuise | SUCCESS | 1.000 | 6169.8539 | 3894.2911 | 1336.6530 | 13834.4638 | 15482.3380 | 4793.2771 | 245.6269/129.3680/329.0548 | 16.2863/5.6579/45.2719 | 115.3545 |
| starloc | loop-2d-fast_s1 | current | SUCCESS | 0.997 | 0.6937 | 0.5833 | 0.4662 | 1.3207 | 1.7947 | 0.5752 | 1.8904/1.8395/2.7520 | 104.4815/88.1027/172.4950 | 144.5707 |
| starloc | loop-2d-fast_s1 | sfuise | SUCCESS | 1.000 | 1447.4163 | 1033.2674 | 706.5028 | 3000.4877 | 3387.9910 | 1018.8233 | 132.3317/88.8073/218.4555 | 55.0432/58.9914/68.7026 | 134.4824 |
| starloc | loop-2d-fast_s1 | current_all_tags | SUCCESS | 0.997 | 0.5930 | 0.4731 | 0.3596 | 1.2982 | 1.8628 | 0.5441 | 1.4955/1.3064/2.4450 | 98.0594/87.0486/161.7281 | 119.9650 |
| starloc | loop-2d-fast_s2 | current | SUCCESS | 1.000 | 1.0333 | 0.7781 | 0.6121 | 1.5905 | 6.5831 | 0.8182 | 2.0577/1.8033/3.2024 | 101.0967/87.9552/166.1219 | 132.2220 |
| starloc | loop-2d-fast_s2 | sfuise | SUCCESS | 0.997 | 1811.0031 | 1294.7627 | 856.4308 | 3776.4507 | 4262.9731 | 1275.4052 | 150.6879/111.7648/265.1387 | 54.4829/56.5213/71.5343 | 134.7958 |
| starloc | loop-2d-fast_s2 | current_all_tags | SUCCESS | 1.000 | 0.7380 | 0.6244 | 0.5221 | 1.3075 | 2.0274 | 0.4921 | 1.6493/1.3679/2.9667 | 107.4426/97.0469/171.6711 | 119.8748 |
| starloc | loop-2d-fast_s3 | current | SUCCESS | 1.000 | 0.3149 | 0.2635 | 0.2008 | 0.6187 | 1.0571 | 0.1764 | 1.2462/1.2457/1.7077 | 45.5882/31.2126/91.9902 | 137.9563 |
| starloc | loop-2d-fast_s3 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| starloc | loop-2d-v2_s5 | current | SUCCESS | 1.000 | 1.2389 | 0.7463 | 0.5111 | 2.0245 | 14.0930 | 1.1643 | 1.7250/0.9025/2.7096 | 57.5955/22.8199/147.5431 | 155.8296 |
| starloc | loop-2d-v2_s5 | sfuise | SUCCESS | 1.000 | 10541.4898 | 7435.0163 | 4731.8776 | 21477.6282 | 23609.2216 | 7540.9694 | 356.0387/350.3485/520.8295 | 18.7120/14.0877/33.6181 | 115.5861 |
| starloc | loop-2d_s1 | current | SUCCESS | 0.998 | 0.5373 | 0.4371 | 0.3404 | 1.0634 | 2.1204 | 0.4445 | 1.1222/1.0412/1.7754 | 80.2224/57.0937/154.7015 | 143.6455 |
| starloc | loop-2d_s1 | sfuise | SUCCESS | 1.000 | 5969.4981 | 4208.1427 | 2785.3121 | 12533.0394 | 14104.7401 | 4261.1511 | 305.9596/195.2029/476.8635 | 30.5812/31.2210/41.7435 | 135.2294 |
| starloc | loop-2d_s1 | current_all_tags | SUCCESS | 0.998 | 0.5106 | 0.4118 | 0.3400 | 0.9459 | 2.4148 | 0.4757 | 1.0751/0.9154/1.8921 | 80.6674/62.1081/148.5278 | 122.1462 |
| starloc | loop-2d_s2 | current | SUCCESS | 0.999 | 1.3032 | 0.7686 | 0.6406 | 1.3540 | 23.0562 | 1.1303 | 1.6765/0.7384/1.3481 | 90.2565/69.3783/164.3233 | 144.2122 |
| starloc | loop-2d_s2 | sfuise | SUCCESS | 0.999 | 13376.2127 | 9566.0278 | 6763.8856 | 27689.0492 | 30855.7541 | 9445.2774 | 810.3046/366.6129/708.6303 | 24.7364/22.6507/32.0466 | 131.4370 |
| starloc | loop-2d_s2 | current_all_tags | SUCCESS | 0.999 | 0.7130 | 0.6553 | 0.5944 | 1.2065 | 1.5102 | 0.4482 | 0.8173/0.7461/1.3314 | 95.8187/77.9974/166.7726 | 133.3486 |
| starloc | loop-2d_s3 | current | SUCCESS | 1.000 | 0.3754 | 0.2919 | 0.2205 | 0.6901 | 2.1564 | 0.2472 | 0.7250/0.6800/1.0476 | 46.4285/25.5956/95.4283 | 138.4815 |
| starloc | loop-2d_s3 | sfuise | SUCCESS | 0.999 | 19514.9624 | 14270.2936 | 9995.2275 | 39745.2860 | 44095.1354 | 13336.6647 | 561.6907/429.2655/835.2302 | 18.9306/18.0228/26.9485 | 120.5419 |
| starloc | loop-2d_s4 | current | SUCCESS | 0.999 | 0.4709 | 0.2660 | 0.1281 | 1.2083 | 2.8097 | 0.4617 | 0.5864/0.3085/1.3072 | 45.0571/19.6453/107.0952 | 159.9122 |
| starloc | loop-2d_s4 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| starloc | loop-2d_s5 | current | SUCCESS | 1.000 | 0.9498 | 0.7143 | 0.5752 | 1.7066 | 5.6284 | 0.7525 | 1.3197/1.0572/2.3058 | 56.5793/19.0846/145.4470 | 164.5865 |
| starloc | loop-2d_s5 | sfuise | SUCCESS | 1.000 | 6077.7572 | 4328.2157 | 3140.9305 | 12728.3187 | 14452.3501 | 4311.5808 | 275.5365/257.3308/506.2424 | 20.5282/16.8040/33.3745 | 124.5561 |
| starloc | loop-3d-z_s3 | current | SUCCESS | 1.000 | 0.4237 | 0.3037 | 0.2278 | 0.6396 | 2.3843 | 0.3755 | 0.6497/0.5264/1.1193 | 62.8131/36.2488/142.4586 | 162.7035 |
| starloc | loop-3d-z_s3 | sfuise | SUCCESS | 0.999 | 17395.0514 | 14014.4567 | 15671.8455 | 31627.4772 | 34663.8991 | 11003.4507 | 396.0133/332.4755/635.6357 | 16.3512/9.8422/28.2998 | 140.7280 |
| starloc | loop-3d_s1 | current | SUCCESS | 0.999 | 0.5174 | 0.4083 | 0.3219 | 1.1021 | 2.1203 | 0.4764 | 0.9530/0.8913/1.5020 | 85.1964/61.3547/158.8369 | 146.7139 |
| starloc | loop-3d_s1 | sfuise | SUCCESS | 1.000 | 12423.5642 | 9338.7692 | 7320.6206 | 24226.1304 | 26493.5627 | 8232.6042 | 495.7820/329.7107/505.8062 | 36.7427/26.5755/72.8503 | 132.9554 |
| starloc | loop-3d_s1 | current_all_tags | SUCCESS | 0.999 | 0.4614 | 0.3799 | 0.3166 | 0.9158 | 2.1256 | 0.4421 | 0.9556/0.8336/1.6020 | 91.6738/76.0649/158.4952 | 133.5662 |
| starloc | loop-3d_s2 | current | SUCCESS | 1.000 | 0.8625 | 0.7122 | 0.6310 | 1.4018 | 5.5417 | 0.5927 | 1.0314/0.7973/1.6523 | 90.1980/68.0840/162.8901 | 144.2867 |
| starloc | loop-3d_s2 | sfuise | SUCCESS | 1.000 | 8090.1197 | 5986.1913 | 4861.1184 | 16509.6376 | 18347.2378 | 5535.9782 | 326.1383/238.5446/469.1883 | 27.2948/20.7182/52.5855 | 133.7639 |
| starloc | loop-3d_s2 | current_all_tags | SUCCESS | 1.000 | 0.7203 | 0.6114 | 0.5131 | 1.3460 | 1.7989 | 0.4681 | 0.8585/0.7313/1.5044 | 100.1757/81.2432/166.5212 | 131.6433 |
| starloc | loop-3d_s3 | current | SUCCESS | 0.999 | 0.4516 | 0.3698 | 0.3066 | 0.8029 | 2.2937 | 0.3110 | 0.7147/0.5872/1.2227 | 66.6141/43.1941/141.4626 | 142.4987 |
| starloc | loop-3d_s3 | sfuise | RuntimeError: RUN_FAILED: SFUISE_PROCESS_EXIT_-11 | 0.000 | NA | NA | NA | NA | NA | NA | NA/NA/NA | NA/NA/NA | NA |
| starloc | loop-3d_s5 | current | SUCCESS | 1.000 | 0.7999 | 0.5730 | 0.4001 | 1.7856 | 3.3359 | 0.7080 | 1.4339/1.2274/2.2528 | 67.6235/32.6204/163.9396 | 159.0451 |
| starloc | loop-3d_s5 | sfuise | SUCCESS | 1.000 | 1705.9676 | 1139.2627 | 576.6470 | 3687.9295 | 4113.4684 | 1283.4828 | 164.2092/122.4644/267.7238 | 27.3054/24.4572/45.5149 | 91.1096 |
| starloc | zigzag_s2 | current | SUCCESS | 1.000 | 11.7976 | 1.8365 | 0.5422 | 2.0316 | 239.2941 | 11.7827 | 17.2257/0.8590/3.2703 | 95.8856/73.2884/169.8293 | 147.5054 |
| starloc | zigzag_s2 | sfuise | SUCCESS | 1.000 | 10161.7534 | 7280.1685 | 5188.8140 | 20755.9214 | 22976.6786 | 7107.5007 | 432.5012/312.1084/560.1631 | 35.4722/15.6590/74.1388 | 127.7433 |
| starloc | zigzag_s2 | current_all_tags | SUCCESS | 1.000 | 0.6077 | 0.5074 | 0.4292 | 1.1698 | 2.0348 | 0.4112 | 0.9609/0.8929/1.4440 | 88.8866/70.1035/164.7826 | 139.3552 |
| starloc | zigzag_s3 | current | SUCCESS | 0.999 | 0.4773 | 0.3779 | 0.2881 | 1.0055 | 1.7471 | 0.3568 | 0.7483/0.6489/1.2416 | 50.2998/31.1009/102.7331 | 134.9375 |
| starloc | zigzag_s3 | sfuise | SUCCESS | 1.000 | 27716.0590 | 20476.2260 | 14564.4584 | 60769.1286 | 71347.6008 | 20079.8147 | 634.7989/332.6040/1052.7959 | 37.8932/26.2865/69.0699 | 137.5970 |
| starloc | zigzag_s4 | current | SUCCESS | 1.000 | 0.5628 | 0.3494 | 0.2033 | 1.6308 | 3.8463 | 0.5375 | 0.4918/0.2775/0.8036 | 68.0286/38.7602/152.6004 | 160.3596 |
| starloc | zigzag_s4 | sfuise | SUCCESS | 1.000 | 31745.5938 | 24132.6214 | 20046.0300 | 60468.7603 | 67549.9144 | 20829.2075 | 401.6764/382.2832/656.0897 | 14.1645/8.9923/28.9089 | 115.7736 |

### 公共有效序列的成对差值

负值表示 current 的 raw-frame APE RMSE 更小。

| 数据集 | 序列 | current − SFUISE [m] |
|---|---|---:|
| HUEC | LOS_Trajectory_A_Case_1 | 3129.8126 |
| HUEC | LOS_Trajectory_B_Case_3 | 832.5463 |
| HUEC | NLOS_Trajectory_A_Case_2 | 2392.7107 |
| HUEC | NLOS_Trajectory_B_Case_3 | 72.9123 |
| HUEC | NLOS_Trajectory_B_Case_4 | 18.8589 |
| SFUISE | ISAS-Walk1 | 0.9003 |
| SFUISE | ISAS-Walk2 | 0.1189 |
| SFUISE | ISAS-Walk3 | 0.3251 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 0.0516 |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | 0.0594 |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | 0.0436 |
| simulation | figure_eight_nominal_seed_20260901 | -0.1846 |
| starloc | apriltag_s3 | -21327.8101 |
| starloc | eight_s2 | -3109.9862 |
| starloc | eight_s3 | -11263.3704 |
| starloc | ell_s3 | -25048.3068 |
| starloc | grid_s3 | -6169.4555 |
| starloc | loop-2d-fast_s1 | -1446.7227 |
| starloc | loop-2d-fast_s2 | -1809.9698 |
| starloc | loop-2d-v2_s5 | -10540.2509 |
| starloc | loop-2d_s1 | -5968.9608 |
| starloc | loop-2d_s2 | -13374.9095 |
| starloc | loop-2d_s3 | -19514.5870 |
| starloc | loop-2d_s5 | -6076.8074 |
| starloc | loop-3d-z_s3 | -17394.6277 |
| starloc | loop-3d_s1 | -12423.0468 |
| starloc | loop-3d_s2 | -8089.2572 |
| starloc | loop-3d_s5 | -1705.1677 |
| starloc | zigzag_s2 | -10149.9558 |
| starloc | zigzag_s3 | -27715.5817 |
| starloc | zigzag_s4 | -31745.0310 |

## 图表

### HUEC

逐序列 APE RMSE

![逐序列 APE RMSE](figures/HUEC_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/HUEC_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/HUEC_ape_cdf.png)

成对精度差

![成对精度差](figures/HUEC_paired_delta.png)

LOS_Trajectory_A_Case_1 XY/高度轨迹

![LOS_Trajectory_A_Case_1 XY/高度轨迹](figures/HUEC_LOS_Trajectory_A_Case_1_trajectory.png)

LOS_Trajectory_A_Case_1 3D 轨迹

![LOS_Trajectory_A_Case_1 3D 轨迹](figures/HUEC_LOS_Trajectory_A_Case_1_trajectory_3d.png)

LOS_Trajectory_A_Case_1 APE 时间序列

![LOS_Trajectory_A_Case_1 APE 时间序列](figures/HUEC_LOS_Trajectory_A_Case_1_ape_time.png)

### MILUV

逐序列 APE RMSE

![逐序列 APE RMSE](figures/MILUV_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/MILUV_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/MILUV_ape_cdf.png)

default_1_random3_0 XY/高度轨迹

![default_1_random3_0 XY/高度轨迹](figures/MILUV_default_1_random3_0_trajectory.png)

default_1_random3_0 3D 轨迹

![default_1_random3_0 3D 轨迹](figures/MILUV_default_1_random3_0_trajectory_3d.png)

default_1_random3_0 APE 时间序列

![default_1_random3_0 APE 时间序列](figures/MILUV_default_1_random3_0_ape_time.png)

### SFUISE

逐序列 APE RMSE

![逐序列 APE RMSE](figures/SFUISE_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/SFUISE_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/SFUISE_ape_cdf.png)

成对精度差

![成对精度差](figures/SFUISE_paired_delta.png)

ISAS-Walk1 XY/高度轨迹

![ISAS-Walk1 XY/高度轨迹](figures/SFUISE_ISAS-Walk1_trajectory.png)

ISAS-Walk1 3D 轨迹

![ISAS-Walk1 3D 轨迹](figures/SFUISE_ISAS-Walk1_trajectory_3d.png)

ISAS-Walk1 APE 时间序列

![ISAS-Walk1 APE 时间序列](figures/SFUISE_ISAS-Walk1_ape_time.png)

### own_vicon

逐序列 APE RMSE

![逐序列 APE RMSE](figures/own_vicon_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/own_vicon_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/own_vicon_ape_cdf.png)

成对精度差

![成对精度差](figures/own_vicon_paired_delta.png)

2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle XY/高度轨迹

![2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle XY/高度轨迹](figures/own_vicon_2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle_trajectory.png)

2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle 3D 轨迹

![2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle 3D 轨迹](figures/own_vicon_2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle_trajectory_3d.png)

2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle APE 时间序列

![2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle APE 时间序列](figures/own_vicon_2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle_ape_time.png)

### simulation

逐序列 APE RMSE

![逐序列 APE RMSE](figures/simulation_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/simulation_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/simulation_ape_cdf.png)

成对精度差

![成对精度差](figures/simulation_paired_delta.png)

figure_eight_nominal_seed_20260901 XY/高度轨迹

![figure_eight_nominal_seed_20260901 XY/高度轨迹](figures/simulation_figure_eight_nominal_seed_20260901_trajectory.png)

figure_eight_nominal_seed_20260901 3D 轨迹

![figure_eight_nominal_seed_20260901 3D 轨迹](figures/simulation_figure_eight_nominal_seed_20260901_trajectory_3d.png)

figure_eight_nominal_seed_20260901 APE 时间序列

![figure_eight_nominal_seed_20260901 APE 时间序列](figures/simulation_figure_eight_nominal_seed_20260901_ape_time.png)

### starloc

逐序列 APE RMSE

![逐序列 APE RMSE](figures/starloc_ape_box.png)

逐序列 1 s RPE

![逐序列 1 s RPE](figures/starloc_rpe_box.png)

pooled APE CDF

![pooled APE CDF](figures/starloc_ape_cdf.png)

成对精度差

![成对精度差](figures/starloc_paired_delta.png)

loop-3d_s3 XY/高度轨迹

![loop-3d_s3 XY/高度轨迹](figures/starloc_loop-3d_s3_trajectory.png)

loop-3d_s3 3D 轨迹

![loop-3d_s3 3D 轨迹](figures/starloc_loop-3d_s3_trajectory_3d.png)

loop-3d_s3 APE 时间序列

![loop-3d_s3 APE 时间序列](figures/starloc_loop-3d_s3_ape_time.png)

### all

全部运行 coverage

![全部运行 coverage](figures/all_coverage.png)

### timing

current 阶段耗时

![current 阶段耗时](figures/current_timing_stack.png)

current solver 分位耗时

![current solver 分位耗时](figures/current_timing_quantiles.png)

SFUISE 端到端墙钟

![SFUISE 端到端墙钟](figures/sfuise_wall.png)

## 耗时分析

current runner 的 `timing.csv` 分解 adapter/bootstrap、IMU preintegration、prepare、commit、state query；SFUISE 使用其原生 average-window runtime 和进程墙钟，二者不作逐阶段直接排名。

### 全量自然运行

| 数据集 | 序列 | 方法 | 状态 | 墙钟 [s] | CPU [s] | epoch/轨迹样本 | 原因 |
|---|---|---|---|---:|---:|---:|---|
| HUEC | LOS_Trajectory_A_Case_1 | current | SUCCESS | 2.732 | 2.731 | 2342 |  |
| HUEC | LOS_Trajectory_A_Case_2 | current | FAIL | 1.188 | 1.188 | 1028 | commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551353... |
| HUEC | LOS_Trajectory_B_Case_3 | current | SUCCESS | 1.992 | 1.992 | 1828 |  |
| HUEC | LOS_Trajectory_B_Case_4 | current | FAIL | 0.526 | 0.526 | 385 | commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551352... |
| HUEC | NLOS_Trajectory_A_Case_1 | current | FAIL | 0.723 | 0.723 | 569 | commit terminal at backend_update:  Indeterminant linear system detected while working near variable 8646911284551352... |
| HUEC | NLOS_Trajectory_A_Case_2 | current | SUCCESS | 2.912 | 2.910 | 2596 |  |
| HUEC | NLOS_Trajectory_B_Case_3 | current | SUCCESS | 1.865 | 1.865 | 1713 |  |
| HUEC | NLOS_Trajectory_B_Case_4 | current | SUCCESS | 1.882 | 1.882 | 1711 |  |
| MILUV | cirObstacles_1_random3_0 | current | FAIL | 0.278 | 0.278 | 186 | commit terminal at backend_update:  Indeterminant linear system detected while working near variable 7061644215716937... |
| MILUV | default_1_circular3D_0 | current | SUCCESS | 1.460 | 1.460 | 1230 |  |
| MILUV | default_1_random3_0 | current | SUCCESS | 2.208 | 2.208 | 1885 |  |
| SFUISE | ISAS-Walk1 | current | SUCCESS | 0.925 | 0.925 | 882 |  |
| SFUISE | ISAS-Walk2 | current | SUCCESS | 1.201 | 1.201 | 1140 |  |
| SFUISE | ISAS-Walk3 | current | SUCCESS | 1.310 | 1.309 | 1222 |  |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | current | SUCCESS | 3.480 | 3.480 | 3569 |  |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | current | SUCCESS | 0.443 | 0.443 | 469 |  |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | current | SUCCESS | 0.418 | 0.418 | 478 |  |
| simulation | figure_eight_nominal_seed_20260901 | current | SUCCESS | 4.700 | 26.366 | 2421 |  |
| starloc | apriltag_s3 | current | SUCCESS | 5.505 | 5.505 | 5008 |  |
| starloc | eight_s2 | current | SUCCESS | 3.591 | 19.277 | 1995 |  |
| starloc | eight_s3 | current | SUCCESS | 4.806 | 4.806 | 4256 |  |
| starloc | ell_s3 | current | SUCCESS | 5.468 | 5.468 | 4868 |  |
| starloc | grid_s3 | current | SUCCESS | 5.354 | 5.354 | 4906 |  |
| starloc | loop-2d-fast_s1 | current | SUCCESS | 2.397 | 13.851 | 1268 |  |
| starloc | loop-2d-fast_s2 | current | SUCCESS | 2.631 | 14.380 | 1431 |  |
| starloc | loop-2d-fast_s3 | current | SUCCESS | 2.549 | 2.548 | 2408 |  |
| starloc | loop-2d-v2_s5 | current | SUCCESS | 0.870 | 0.870 | 800 |  |
| starloc | loop-2d_s1 | current | SUCCESS | 4.827 | 26.315 | 2589 |  |
| starloc | loop-2d_s2 | current | SUCCESS | 6.866 | 38.473 | 3625 |  |
| starloc | loop-2d_s3 | current | SUCCESS | 4.944 | 4.944 | 4568 |  |
| starloc | loop-2d_s4 | current | SUCCESS | 8.640 | 8.638 | 7852 |  |
| starloc | loop-2d_s5 | current | SUCCESS | 0.676 | 0.676 | 634 |  |
| starloc | loop-3d-z_s3 | current | SUCCESS | 7.232 | 7.232 | 6440 |  |
| starloc | loop-3d_s1 | current | SUCCESS | 6.510 | 36.074 | 3483 |  |
| starloc | loop-3d_s2 | current | SUCCESS | 6.380 | 36.046 | 3387 |  |
| starloc | loop-3d_s3 | current | SUCCESS | 4.583 | 4.583 | 4195 |  |
| starloc | loop-3d_s5 | current | SUCCESS | 0.345 | 0.345 | 316 |  |
| starloc | zigzag_s2 | current | SUCCESS | 6.192 | 34.034 | 3304 |  |
| starloc | zigzag_s3 | current | SUCCESS | 9.552 | 9.551 | 8584 |  |
| starloc | zigzag_s4 | current | SUCCESS | 10.732 | 10.731 | 9108 |  |
| HUEC | LOS_Trajectory_A_Case_1 | sfuise | SUCCESS | 129.577 | 88.078 | 2361 |  |
| HUEC | LOS_Trajectory_A_Case_2 | sfuise | SUCCESS | 139.236 | 95.588 | 2294 |  |
| HUEC | LOS_Trajectory_B_Case_3 | sfuise | SUCCESS | 104.037 | 64.802 | 1847 |  |
| HUEC | LOS_Trajectory_B_Case_4 | sfuise | FAIL | 111.523 | 4.775 | 0 | SFUISE_PROCESS_EXIT_-11 |
| HUEC | NLOS_Trajectory_A_Case_1 | sfuise | SUCCESS | 169.133 | 91.730 | 2641 |  |
| HUEC | NLOS_Trajectory_A_Case_2 | sfuise | SUCCESS | 150.088 | 94.708 | 2615 |  |
| HUEC | NLOS_Trajectory_B_Case_3 | sfuise | SUCCESS | 98.972 | 64.412 | 1732 |  |
| HUEC | NLOS_Trajectory_B_Case_4 | sfuise | SUCCESS | 100.781 | 62.346 | 1730 |  |
| MILUV | cirObstacles_1_random3_0 | sfuise | FAIL | 115.077 | 4.644 | 0 | SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_circular3D_0 | sfuise | FAIL | 79.012 | 3.935 | 0 | SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_random3_0 | sfuise | FAIL | 114.247 | 5.036 | 0 | SFUISE_PROCESS_EXIT_-11 |
| SFUISE | ISAS-Walk1 | sfuise | SUCCESS | 42.921 | 19.808 | 969 |  |
| SFUISE | ISAS-Walk2 | sfuise | SUCCESS | 50.915 | 23.883 | 1240 |  |
| SFUISE | ISAS-Walk3 | sfuise | SUCCESS | 53.713 | 27.264 | 1338 |  |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | sfuise | SUCCESS | 48.088 | 31.059 | 3665 |  |
| own_vicon | 2025-10-24-15-46-27_vicon_lidar_uwb_imu_obstacle | sfuise | SUCCESS | 17.170 | 6.282 | 565 |  |
| own_vicon | 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle | sfuise | SUCCESS | 17.125 | 5.689 | 575 |  |
| simulation | figure_eight_nominal_seed_20260901 | sfuise | SUCCESS | 72.930 | 50.588 | 2458 |  |
| starloc | apriltag_s3 | sfuise | SUCCESS | 70.466 | 58.813 | 5094 |  |
| starloc | eight_s2 | sfuise | SUCCESS | 36.057 | 80.653 | 1891 |  |
| starloc | eight_s3 | sfuise | SUCCESS | 61.607 | 48.989 | 4344 |  |
| starloc | ell_s3 | sfuise | SUCCESS | 67.659 | 55.314 | 4952 |  |
| starloc | grid_s3 | sfuise | SUCCESS | 68.430 | 57.212 | 4993 |  |
| starloc | loop-2d-fast_s1 | sfuise | SUCCESS | 29.227 | 54.621 | 1253 |  |
| starloc | loop-2d-fast_s2 | sfuise | SUCCESS | 28.884 | 56.627 | 1355 |  |
| starloc | loop-2d-fast_s3 | sfuise | FAIL | 39.610 | 2.999 | 0 | SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-2d-v2_s5 | sfuise | SUCCESS | 54.202 | 36.575 | 819 |  |
| starloc | loop-2d_s1 | sfuise | SUCCESS | 44.785 | 109.550 | 2513 |  |
| starloc | loop-2d_s2 | sfuise | SUCCESS | 55.393 | 146.496 | 3372 |  |
| starloc | loop-2d_s3 | sfuise | SUCCESS | 64.728 | 51.964 | 4653 |  |
| starloc | loop-2d_s4 | sfuise | FAIL | 108.189 | 5.341 | 0 | SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-2d_s5 | sfuise | SUCCESS | 45.562 | 30.359 | 653 |  |
| starloc | loop-3d-z_s3 | sfuise | SUCCESS | 87.176 | 77.125 | 6526 |  |
| starloc | loop-3d_s1 | sfuise | SUCCESS | 55.888 | 144.877 | 3256 |  |
| starloc | loop-3d_s2 | sfuise | SUCCESS | 51.460 | 134.729 | 3104 |  |
| starloc | loop-3d_s3 | sfuise | FAIL | 61.933 | 3.695 | 0 | SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s5 | sfuise | SUCCESS | 29.961 | 16.687 | 335 |  |
| starloc | zigzag_s2 | sfuise | SUCCESS | 50.558 | 130.456 | 3108 |  |
| starloc | zigzag_s3 | sfuise | SUCCESS | 113.150 | 101.390 | 8670 |  |
| starloc | zigzag_s4 | sfuise | SUCCESS | 120.570 | 114.662 | 8584 |  |

### current 稳定计时（warm-up 后 5 次）

| 数据集 | 序列 | repeat | CPU affinity | 墙钟 [ms] | CPU [ms] | adapter [ms] | bootstrap [ms] | throughput [epoch/s] | RTF |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| HUEC | LOS_Trajectory_A_Case_1 | 1 | 4 | 2625.74 | 2620.03 | 84.19 | 0.07 | 891.9 | 0.0112 |
| HUEC | LOS_Trajectory_A_Case_1 | 2 | 4 | 2635.86 | 2635.80 | 87.12 | 0.08 | 888.5 | 0.0112 |
| HUEC | LOS_Trajectory_A_Case_1 | 3 | 4 | 2621.44 | 2621.13 | 89.03 | 0.06 | 893.4 | 0.0111 |
| HUEC | LOS_Trajectory_A_Case_1 | 4 | 4 | 2603.93 | 2603.89 | 86.03 | 0.07 | 899.4 | 0.0111 |
| HUEC | LOS_Trajectory_A_Case_1 | 5 | 4 | 2698.22 | 2698.11 | 85.40 | 0.08 | 868.0 | 0.0115 |
| MILUV | default_1_random3_0 | 1 | 4 | 2260.57 | 2260.43 | 91.48 | 0.13 | 833.9 | 0.0110 |
| MILUV | default_1_random3_0 | 2 | 4 | 2236.75 | 2236.66 | 90.45 | 0.13 | 842.7 | 0.0109 |
| MILUV | default_1_random3_0 | 3 | 4 | 2236.89 | 2234.36 | 91.64 | 0.13 | 842.7 | 0.0109 |
| MILUV | default_1_random3_0 | 4 | 4 | 2231.97 | 2231.92 | 89.99 | 0.16 | 844.5 | 0.0109 |
| MILUV | default_1_random3_0 | 5 | 4 | 2251.61 | 2251.59 | 108.61 | 0.20 | 837.2 | 0.0110 |
| SFUISE | ISAS-Walk1 | 1 | 4 | 921.35 | 921.33 | 17.37 | 0.05 | 957.3 | 0.0156 |
| SFUISE | ISAS-Walk1 | 2 | 4 | 903.82 | 903.79 | 17.76 | 0.05 | 975.9 | 0.0153 |
| SFUISE | ISAS-Walk1 | 3 | 4 | 919.65 | 919.58 | 17.68 | 0.05 | 959.1 | 0.0156 |
| SFUISE | ISAS-Walk1 | 4 | 4 | 923.66 | 923.63 | 18.49 | 0.08 | 954.9 | 0.0156 |
| SFUISE | ISAS-Walk1 | 5 | 4 | 916.22 | 916.21 | 19.68 | 0.07 | 962.7 | 0.0155 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 1 | 4 | 3397.78 | 3397.68 | 52.44 | 0.14 | 1050.4 | 0.0463 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 2 | 4 | 3410.90 | 3410.82 | 51.34 | 0.13 | 1046.4 | 0.0465 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 3 | 4 | 3431.44 | 3431.32 | 53.09 | 0.15 | 1040.1 | 0.0468 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 4 | 4 | 3431.31 | 3431.23 | 51.07 | 0.13 | 1040.1 | 0.0468 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 5 | 4 | 3406.77 | 3406.68 | 49.82 | 0.14 | 1047.6 | 0.0464 |
| simulation | figure_eight_nominal_seed_20260901 | 1 | 4 | 2792.04 | 2791.35 | 59.84 | 0.20 | 867.1 | 0.0227 |
| simulation | figure_eight_nominal_seed_20260901 | 2 | 4 | 2817.94 | 2817.82 | 60.08 | 0.21 | 859.1 | 0.0229 |
| simulation | figure_eight_nominal_seed_20260901 | 3 | 4 | 2782.18 | 2782.10 | 59.89 | 0.19 | 870.2 | 0.0226 |
| simulation | figure_eight_nominal_seed_20260901 | 4 | 4 | 2823.00 | 2822.92 | 59.26 | 0.25 | 857.6 | 0.0230 |
| simulation | figure_eight_nominal_seed_20260901 | 5 | 4 | 2801.67 | 2801.56 | 64.22 | 0.22 | 864.1 | 0.0228 |
| starloc | loop-3d_s3 | 1 | 4 | 4638.77 | 4638.61 | 57.37 | 0.85 | 904.3 | 0.0465 |
| starloc | loop-3d_s3 | 2 | 4 | 4694.18 | 4694.08 | 57.72 | 1.69 | 893.7 | 0.0471 |
| starloc | loop-3d_s3 | 3 | 4 | 4875.11 | 4874.99 | 58.87 | 1.69 | 860.5 | 0.0489 |
| starloc | loop-3d_s3 | 4 | 4 | 4784.82 | 4784.68 | 64.12 | 1.60 | 876.7 | 0.0480 |
| starloc | loop-3d_s3 | 5 | 4 | 4902.02 | 4901.64 | 57.98 | 1.78 | 855.8 | 0.0491 |

current 端到端 5 次统计：

| 数据集/序列 | wall p50/p95/p99/max [ms] | CPU p50 [ms] | throughput p50 [epoch/s] | RTF p50 |
|---|---:|---:|---:|---:|
| HUEC/LOS_Trajectory_A_Case_1 | 2625.74/2685.75/2695.73/2698.22 | 2621.13 | 891.9 | 0.0112 |
| MILUV/default_1_random3_0 | 2236.89/2258.78/2260.21/2260.57 | 2236.66 | 842.7 | 0.0109 |
| SFUISE/ISAS-Walk1 | 919.65/923.19/923.56/923.66 | 919.58 | 959.1 | 0.0156 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 3410.90/3431.41/3431.43/3431.44 | 3410.82 | 1046.4 | 0.0465 |
| simulation/figure_eight_nominal_seed_20260901 | 2801.67/2821.99/2822.80/2823.00 | 2801.56 | 864.1 | 0.0228 |
| starloc/loop-3d_s3 | 4784.82/4896.64/4900.95/4902.02 | 4784.68 | 876.7 | 0.0480 |

### current 每 epoch 阶段分位数（5 次 repeat 的中位数）

| 数据集/序列 | 阶段 | p50 [ms] | p95 [ms] | p99 [ms] | max [ms] |
|---|---|---:|---:|---:|---:|
| HUEC/LOS_Trajectory_A_Case_1 | IMU preintegration | 0.0285 | 0.0435 | 0.0685 | 0.5878 |
| HUEC/LOS_Trajectory_A_Case_1 | prepare total | 0.0591 | 0.0902 | 0.1700 | 0.6891 |
| HUEC/LOS_Trajectory_A_Case_1 | no-UWB update | 0.0571 | 0.0869 | 0.1665 | 0.6740 |
| HUEC/LOS_Trajectory_A_Case_1 | UWB commit/solver | 0.9950 | 1.3869 | 1.7724 | 2.6972 |
| HUEC/LOS_Trajectory_A_Case_1 | state query | 0.0001 | 0.0001 | 0.0002 | 0.0009 |
| MILUV/default_1_random3_0 | IMU preintegration | 0.0319 | 0.0475 | 0.0807 | 0.5811 |
| MILUV/default_1_random3_0 | prepare total | 0.0632 | 0.0898 | 0.1757 | 0.7032 |
| MILUV/default_1_random3_0 | no-UWB update | 0.0607 | 0.0865 | 0.1680 | 0.6855 |
| MILUV/default_1_random3_0 | UWB commit/solver | 1.0238 | 1.3683 | 1.7489 | 3.3839 |
| MILUV/default_1_random3_0 | state query | 0.0001 | 0.0001 | 0.0002 | 0.0004 |
| SFUISE/ISAS-Walk1 | IMU preintegration | 0.0112 | 0.0180 | 0.0344 | 0.3054 |
| SFUISE/ISAS-Walk1 | prepare total | 0.0413 | 0.0615 | 0.1274 | 0.4114 |
| SFUISE/ISAS-Walk1 | no-UWB update | 0.0388 | 0.0579 | 0.1190 | 0.3969 |
| SFUISE/ISAS-Walk1 | UWB commit/solver | 0.9711 | 1.3100 | 1.5946 | 1.9980 |
| SFUISE/ISAS-Walk1 | state query | 0.0001 | 0.0001 | 0.0002 | 0.0004 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | IMU preintegration | 0.0095 | 0.0153 | 0.0293 | 0.5875 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | prepare total | 0.0387 | 0.0579 | 0.1206 | 0.7038 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | no-UWB update | 0.0365 | 0.0549 | 0.1173 | 0.6868 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | UWB commit/solver | 0.9264 | 1.2681 | 1.6703 | 3.0525 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | state query | 0.0001 | 0.0001 | 0.0002 | 0.0007 |
| simulation/figure_eight_nominal_seed_20260901 | IMU preintegration | 0.0155 | 0.0215 | 0.0480 | 0.5354 |
| simulation/figure_eight_nominal_seed_20260901 | prepare total | 0.0480 | 0.0705 | 0.1537 | 0.7296 |
| simulation/figure_eight_nominal_seed_20260901 | no-UWB update | 0.0445 | 0.0647 | 0.1454 | 0.7084 |
| simulation/figure_eight_nominal_seed_20260901 | UWB commit/solver | 1.0144 | 1.3877 | 1.8098 | 2.8377 |
| simulation/figure_eight_nominal_seed_20260901 | state query | 0.0001 | 0.0001 | 0.0002 | 0.0006 |
| starloc/loop-3d_s3 | IMU preintegration | 0.0084 | 0.0159 | 0.0254 | 0.3654 |
| starloc/loop-3d_s3 | prepare total | 0.0397 | 0.0674 | 0.1157 | 0.4869 |
| starloc/loop-3d_s3 | no-UWB update | 0.0367 | 0.0618 | 0.1076 | 0.4719 |
| starloc/loop-3d_s3 | UWB commit/solver | 1.0317 | 1.5385 | 1.8689 | 2.9368 |
| starloc/loop-3d_s3 | state query | 0.0001 | 0.0001 | 0.0002 | 0.0150 |

### SFUISE 端到端耗时

SFUISE 上游 CMake 强制写入 `Debug` 类型；隔离工作区不改源码，通过 `-O3 -DNDEBUG` 形成 Release-equivalent 二进制，报告保留该差异。

| 数据集 | 序列 | repeat | CPU affinity | 墙钟 [s] | CPU [s] | RTF | 原生窗口均值 [ms] | 回放倍率 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| HUEC | LOS_Trajectory_A_Case_1 | 1 | 4 | 128.88 | 88.91 | 0.5479 | 36.068 | 2.0 |
| HUEC | LOS_Trajectory_A_Case_1 | 2 | 4 | 129.37 | 89.59 | 0.5500 | 36.305 | 2.0 |
| HUEC | LOS_Trajectory_A_Case_1 | 3 | 4 | 129.20 | 88.57 | 0.5492 | 35.920 | 2.0 |
| HUEC | LOS_Trajectory_A_Case_1 | 4 | 4 | 129.10 | 88.45 | 0.5488 | 35.775 | 2.0 |
| HUEC | LOS_Trajectory_A_Case_1 | 5 | 4 | 129.02 | 88.75 | 0.5485 | 36.059 | 2.0 |
| SFUISE | ISAS-Walk1 | 1 | 4 | 42.65 | 19.69 | 0.7215 | 28.214 | 2.0 |
| SFUISE | ISAS-Walk1 | 2 | 4 | 42.87 | 19.67 | 0.7252 | 28.048 | 2.0 |
| SFUISE | ISAS-Walk1 | 3 | 4 | 42.74 | 19.69 | 0.7230 | 27.987 | 2.0 |
| SFUISE | ISAS-Walk1 | 4 | 4 | 42.85 | 19.59 | 0.7249 | 27.828 | 2.0 |
| SFUISE | ISAS-Walk1 | 5 | 4 | 42.88 | 19.58 | 0.7253 | 27.867 | 2.0 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 1 | 4 | 48.18 | 31.46 | 0.6565 | 38.770 | 2.0 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 2 | 4 | 48.29 | 31.29 | 0.6579 | 38.495 | 2.0 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 3 | 4 | 48.16 | 30.92 | 0.6562 | 37.996 | 2.0 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 4 | 4 | 48.18 | 30.76 | 0.6564 | 37.870 | 2.0 |
| own_vicon | 2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 5 | 4 | 48.06 | 31.03 | 0.6549 | 38.385 | 2.0 |
| simulation | figure_eight_nominal_seed_20260901 | 1 | 4 | 72.98 | 50.22 | 0.5933 | 38.072 | 2.0 |
| simulation | figure_eight_nominal_seed_20260901 | 2 | 4 | 72.85 | 50.05 | 0.5923 | 37.911 | 2.0 |
| simulation | figure_eight_nominal_seed_20260901 | 3 | 4 | 73.04 | 50.31 | 0.5938 | 38.066 | 2.0 |
| simulation | figure_eight_nominal_seed_20260901 | 4 | 4 | 73.04 | 50.69 | 0.5938 | 38.447 | 2.0 |
| simulation | figure_eight_nominal_seed_20260901 | 5 | 4 | 72.89 | 50.32 | 0.5926 | 38.208 | 2.0 |

SFUISE 端到端 5 次统计：

| 数据集/序列 | wall p50/p95/p99/max [s] | CPU p50 [s] | RTF p50 | 原生窗口均值 p50 [ms] |
|---|---:|---:|---:|---:|
| HUEC/LOS_Trajectory_A_Case_1 | 129.10/129.34/129.37/129.37 | 88.75 | 0.5488 | 36.059 |
| SFUISE/ISAS-Walk1 | 42.85/42.88/42.88/42.88 | 19.67 | 0.7249 | 27.987 |
| own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle | 48.18/48.27/48.28/48.29 | 31.03 | 0.6564 | 38.385 |
| simulation/figure_eight_nominal_seed_20260901 | 72.98/73.04/73.04/73.04 | 50.31 | 0.5933 | 38.072 |

SFUISE 稳定计时失败项：

| 数据集 | 序列 | repeat | 原因 |
|---|---|---:|---|
| MILUV | default_1_random3_0 | 1 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_random3_0 | 2 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_random3_0 | 3 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_random3_0 | 4 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| MILUV | default_1_random3_0 | 5 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s3 | 1 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s3 | 2 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s3 | 3 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s3 | 4 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |
| starloc | loop-3d_s3 | 5 | WARMUP_FAILED: SFUISE_PROCESS_EXIT_-11 |

## 仿真 direct nominal / ROS fde=off 等价性

状态：**FAIL**；direct/realtime 样本数 2421/2421；标签位置最大差异 0.29254758015691174 m；姿态最大差异 179.96417722377464 deg；禁止的 GT 订阅：[]。

原因：numeric mismatch, max dt=1.4210854715202004e-14。

## 失败与审计口径

所有 FAIL/NA 均保留在逐序列表中；非有限轨迹、求解失败或 coverage < 80% 不进入族级数值汇总。估计器输入 allowlist 仅含 `imu.csv`、`uwb.csv`、`anchors.csv`，`gt.csv` 由 manifest 标为 evaluator-only。

## 复现

```bash
python3 tools/run_dataset_benchmark.py prepare
python3 tools/run_dataset_benchmark.py calibrate
python3 tools/run_dataset_benchmark.py preflight
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target nominal_dataset_runner -j2
python3 tools/run_dataset_benchmark.py run --method current
python3 tools/run_sfuise_benchmark.py run
taskset -c 4 python3 tools/run_dataset_benchmark.py timing
taskset -c 4 python3 tools/run_sfuise_benchmark.py timing
python3 tools/check_sim_realtime_equivalence.py --playback-rate 2
python3 tools/run_dataset_benchmark.py evaluate
python3 tools/run_dataset_benchmark.py report
```
