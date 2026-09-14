# own_vicon 15-31-28 单条流程检查

## 冻结输入

- recording：`2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle`，完整 bag，tag0，anchor 1--4。
- UWB：`/nlink_linktrack_nodeframe3` 的 header stamp 与原始 `nodes[].dis`；每个 node 保留源 message、range 和全局 observation 序号。
- IMU：`/livox/imu` header stamp；acceleration 固定乘 9.81（g→m/s²），gyro 原值 rad/s，不读取无效 orientation。
- 时间：以首个 UWB/IMU header stamp 的较早者为零，不拟合 UWB--IMU offset。
- 几何：anchor 1--4 的同 bag Vicon 静态位置中位数；tag0、rig、IMU 原点采用零杆臂近似。

同 bag anchor survey 和零杆臂来自已曝光 development 数据，状态为 `USER_APPROXIMATE_COLOCATION`。
moving-tag Vicon 不进入 measurement cache、初始化、detector 或 estimator，只供隔离评价。

## 冻结运行

复用 `sfuise_walk1_pl_bidirectional_clean.yaml`。只替换 cache、4-anchor geometry 和零杆臂。
依次执行一个 producer、共享唯一 Stage2 cache 的 `suppress_all` 与 `lcb_fixed_full`，以及独立
`robust_cauchy`（scale 2.3849）和 SFUISE-ToA（静态 ToA offset 全零）。每个科学进程树最多1800秒，
总数最多5，无算法重试或参数修改。producer 失败时两个依赖方法记 `NOT_RUN`，独立方法继续。

## 判定与评价

工程流程要求 measurement cache、C++ prepare、Cauchy prepare identity、CSV→ROS 往返及 SFUISE
零测量启动通过。科学流程按真实退出状态记账；正常导出不等于精度合格。

评价用 tag0 Vicon position 作为近似 UWB antenna reference，完整 UWB 首末时刻、10 Hz 网格、估计最近邻
0.02秒、GT插值 bracket≤0.05秒、scale=1 SE3。窗口为各 link 全部 `range−Vicon distance>0.5m`、
持续≥2秒、至少5包、gap≤1秒的并集。Recover−Reject 为 `lcb_fixed_full−suppress_all`，正值表示 Recover 更差。
报告全部失败、fallback、空 support 和负收益；不据结果调参，不升级论文 claim。
