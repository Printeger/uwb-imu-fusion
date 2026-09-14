# own_vicon 初始化修复与 LOS 重跑协议

## 输入与边界

只使用 `data/own_vicon/2025-10-24-15-31-28_vicon_lidar_uwb_imu_no_obstacle.bag`、tag0、完整
header-stamp UWB/IMU 和全部4个 anchor。沿用 `OWN_VICON_FLOW_PROTOCOL.md` 的单位、时间、几何、零杆臂、
方法、预算和评价。moving-tag Vicon 始终留在 evaluator 私有路径，初始化和 estimator 不可读取。

## 冻结修复

1. Trilateration 对每个 LM 候选点重新计算原始 range 平方残差。只有目标严格降低的有限步可以提交；
   拒绝步保留当前位置并增大 damping。只有有限结果达到步长、梯度或相对目标变化收敛条件才返回成功；
   未收敛不发布 `p_out`。
2. 首2秒静止判定保留 acceleration norm variance 与 gravity magnitude 条件，方差阈值唯一取配置
   `sigma_a^2`。不更改 own-vicon 继承的 `sigma_a=0.357 (m/s^2)/sqrt(Hz)`。
3. 每次运行自动计算 anchor 的中心化 singular values；最小/最大比 `<=1e-3` 且平面法向
   `|n_z|>=0.9` 时进入2.5D。法向统一指向世界 +Z，seed 的两侧候选只由 anchor 最佳拟合平面和首批原始
   range 构造。先按实际3D raw-range平方目标选侧；目标不可分时，只有anchor平面和两候选相对世界
   `z=0` 能给出唯一近侧时才使用该非GT高度侧信息，否则返回歧义失败。不新增侧别配置。

## 工程门与科学运行

静止阈值、接受/拒绝、未收敛返回、近共面自动选侧及普通3D兼容测试必须通过，并完成 C++ 重建。
随后使用全新隔离目录运行 prepare、preflight、execute、evaluate、verify。科学树仍为一个
`pl_bidirectional_cusum` producer、共享其唯一 Stage2 cache 的 `suppress_all`/`lcb_fixed_full`、
`robust_cauchy`、`SFUISE-ToA`，每树1800秒、最多5树、串行、无算法参数重试。

逐方法报告 ATE_RMSE、NLOS-window RMSE、ATE_P95、range before/after、candidate/accepted segments、
fallback/failure；RR 差值仍为 Recover 减 Reject，正值表示 Recover 更差。工程通过不等于定位或恢复成功。
