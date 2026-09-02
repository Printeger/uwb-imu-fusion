# P2/P3 数学、融合、ROS 与性能验证报告

> 非认证声明：本实现、测试、统计工具和保护级输出仅供研究验证，不构成安全认证、适航结论或运行授权。

## 当前结论

本轮完成了 P2/P3 验证基础设施和关键回归实现，并执行了正式 H0/noncentral Monte Carlo 与 43 秒 ROS 场景矩阵；noncentral、持续 step 恢复仍有失败，full sweep、Method A/B 600 epochs 与 600 秒性能矩阵尚未执行。因此当前结论是 **IMPLEMENTED_UNVERIFIED**，不得表述为 P2/P3 门禁整体通过。

| 项目 | 绑定值 |
|---|---|
| 基线 git SHA | `254f2906a46841e9d14cc1c8e014ec870a91946e` |
| 工作树 | dirty（本报告对应尚未提交的实现变更） |
| 权威配置 hash | `b66a2b3ec6eaf56c` |
| 根 seed | `20260901` |
| 构建类型 | Release |
| 环境 | Linux，ROS Noetic，GTSAM 4.2 系列，Eigen 3 |

## 已实际验证

| 门禁/测试 | 实际状态 | 样本数 | 结果/阈值 | artifact 或命令 |
|---|---:|---:|---|---|
| Release 增量构建 | PASS | 1 | 编译成功 | `catkin build uwb_imu_pl --no-status` |
| 全部 GoogleTest | PASS | 60 distinct cases（catkin aggregate 120） | 0 error / 0 failure | `catkin run_tests uwb_imu_pl --no-status` |
| dense SVD oracle 随机满秩回归 | PASS | 1000 | covariance 最大要求 `<1e-8`；测试全部满足 | `test_dense_oracle` |
| rank deficient / unmonitorable fail-close | PASS | 2 fixtures | infinite slope，不返回大有限数 | `test_dense_oracle` |
| 10 秒事件顺序重复性 | PASS | 2 次、2202 events/次 | `(timestamp, IMU-before-UWB, sequence)` 完全一致 | `test_deterministic_event` |
| Method B covariance/mean information downdate | PASS | 单元 fixtures | 数值误差 `<1e-10` | `test_realtime_incremental` |
| stale UWB graph/version 审计 | PASS | 1 fixture | 拒绝且 graph/version/covariance 不变 | `test_realtime_incremental` |
| 1 秒 UWB drop + `8→6→4→8` | PASS | 4 UWB epochs | DOF/group/commit 恢复 | `test_realtime_incremental` |
| v2 logger 精确表头、结构化 summary 与 invalid-formal 规则 | PASS | 3 fixtures | 精确表头匹配，数值无效 conditional 不标 formal | `test_run_logger` |
| 正式 H0 Monte Carlo | **PASS** | 16 jobs × 200,000 = 3,200,000 | 16/16 target `P_FA=1e-5` 位于 95% exact CI；16/16 KS `p>0.01`（最小 `p=0.0656`） | `results/p2_monte_carlo/h0.csv` |
| 正式 noncentral Monte Carlo | **FAIL** | 160 jobs × 100,000 = 16,000,000 | 154/160 theory-in-exact-CI；6 个门禁失败，全部原样保留 | `results/p2_monte_carlo/noncentral.csv` |
| 早期 12 秒 nominal ROS smoke / schema | **FAIL / PASS** | 217 integrity epochs | 122 commit、94 ALERT、1 次 fail-closed numerical reinit；该失败未删除 | `/tmp/uwb_pl_ros_smoke6.O57oge/`（临时） |
| Dogleg + Cholesky、skip=10，43 秒 figure-eight nominal | **PASS** | 836 integrity epochs | 836 commit、0 reject、全程 AVAILABLE、0 processing error/reinit；PE mean/P95/max `0.081/0.145/0.229 m`；v2 schema PASS | `/tmp/uwb_pl_ros_chol_dogleg_skip10_43.D0HvoH/`（临时） |
| 上述 43 秒 warm timing（前 100 epochs 排除） | **PASS（当前时长）** | 736 epochs | core mean/P50/P95/P99/max `19.18/14.71/57.51/81.48/106.19 ms`；mean `<50`、P99 `<100` | 同上 `timing.csv` |
| 43 秒 straight nominal（固定连续 yaw fixture） | **PASS** | 838 epochs | 838 commit、0 reject/reinit；PE mean/P95/max `0.079/0.150/0.232 m`；schema PASS | `/tmp/uwb_pl_ros_final43_straight_fixed.XO3St1/`（临时） |
| 43 秒 circle nominal | **PASS** | 836 epochs | 836 commit、0 reject/reinit；PE mean/P95/max `0.085/0.162/0.252 m`；schema PASS | `/tmp/uwb_pl_ros_final43_circle.bCTWaZ/`（临时） |
| figure-eight step `1.0 m` | **检测 PASS / 恢复受限** | 838 epochs | onset 后 `43.9 ms` 首报；持续整组拒绝后 1 次显式数值重初始化，无静默 reassociation | `/tmp/uwb_pl_ros_fault43_step.NDZ30U/`（临时） |
| figure-eight ramp `0.05 m/s` | **检测 PASS** | 837 epochs | bias 约 `0.611 m` 首报；330 reject；0 reinit | `/tmp/uwb_pl_ros_fault43_ramp.VHPYV7/`（临时） |
| figure-eight magnitude sweep `0–2 m` | **检测 PASS** | 835 epochs | bias 约 `0.512 m` 首报；502 reject；0 reinit | `/tmp/uwb_pl_ros_fault43_sweep.NdsLXv/`（临时） |
| figure-eight single-anchor outage | **PASS** | 836 epochs | truth active 后 group/DOF `8→7`（过渡 1 帧 group 5）；836/836 commit、全程 AVAILABLE | `/tmp/uwb_pl_ros_fault43_outage.1s8x3i/`（临时） |
| 缩小 Monte Carlo 功能 smoke | PASS（仅功能） | H0 `16×1000`；noncentral `160×1000` | 仅验证 runner/CI/KS 输出，不用于门禁 | `/tmp/uwb_pl_quick.*/mc/`（临时） |
| 缩小 sweep 功能 smoke | PASS（仅功能） | 1 seed、2 epoch/组合、4104 raw rows | block-bootstrap 汇总生成成功 | `/tmp/uwb_pl_quick.*/sweep*.csv`（临时） |

## 明确保留的未执行项

以下项目状态均为 **NOT RUN**，不是 PASS：

- 100 seeds × 200 epochs 全 geometry/noise/fault sweep 与 small-noise covariance 最终统计；
- delay/drop/gap/anchor-change 的 ROS topic-level rostest；对应 estimator-level deterministic fixtures 已通过；
- global/post-fit 独立 calibration、`P_FA=10^-2/10^-3/10^-4` ROC 和 history `1/10/50/200`；
- Method A/B 三轨迹各 200 epochs 的 600 个真实多历元 comparison；
- Release `3×(600 s×20 Hz)` timing、CPU affinity 与性能门槛；
- Debug clean build、Release clean build和实际 rostest（本轮只做 Release 增量构建）。

早期短时 ROS smoke 的失败原因与 artifact 仍保留：无信赖域的 iSAM2 Gauss–Newton 会产生过大的 pose/velocity 步长，并最终在 Cholesky 消元时报告旧 pose 变量附近不定。正式候选改为 Dogleg 信赖域，研究配置 `relinearize_skip=10`；随后三条 43 秒 nominal 均无告警、拒绝或重初始化并通过当前时长性能门槛。step 持续故障虽然快速检出，但由于尚无 FDE、只能拒绝整个 UWB batch，长期 inertial-only 后仍发生一次显式受控重初始化；故障恢复不能提升为 PASS。以上结果仍不能替代 `3×600 s` 门禁。

正式 Monte Carlo 使用同一 root seed 的场景独立派生 seed，manifest 记录 UTC、git SHA/dirty、配置 hash、Release/编译器/OS/CPU/RAM/GTSAM/Eigen 和完整命令。H0 整体 PASS；noncentral 因 6/160 exact-CI 失败而整体 FAIL。没有通过重跑或筛 seed 消除这些统计失败。

| DOF | anchor | η ratio | theoretical P_MD | empirical P_MD | 95% exact CI |
|---:|---:|---:|---:|---:|---:|
| 1 | 1 | 1.00 | 0.001000 | 0.001320 | [0.001105, 0.001565] |
| 1 | 3 | 0.50 | 0.186367 | 0.189560 | [0.187135, 0.192003] |
| 1 | 3 | 1.00 | 0.001000 | 0.001250 | [0.001041, 0.001489] |
| 1 | 8 | 0.25 | 0.746485 | 0.742830 | [0.740109, 0.745537] |
| 2 | 3 | 0.25 | 0.781844 | 0.777660 | [0.775070, 0.780234] |
| 5 | 6 | 0.25 | 0.832078 | 0.828690 | [0.826340, 0.831021] |

## 实现闭环

- run schema 升级为 `uwb-imu-pl/v2`；v2 增加三类 detector、group/model-valid、扩展 timing、稳定 event sequence、truth CSV 和结构化 summary。
- 正式 conditional 路径仍独占 commit/PL 决策；global/post-fit 无经验阈值时明确记录为未校准诊断。正式/性能默认关闭全图 residual 扫描。
- 当前 `(Pose3,v,bias)` 15×15 joint covariance 改为对 iSAM2 Bayes-tree 当前 clique ancestor closure 做 15 列 `Rᵀ/R` selected-inverse 三角求解；不重建 nonlinear graph、不重新 factorize，回归仍与 full-graph batch oracle 比较。
- iSAM2 正式 UWB/IMU 路径使用 Dogleg 信赖域抑制 range/IMU 联合状态的无界 Gauss–Newton 步长；`relinearize_skip=10` 的 43 秒回归同时满足功能与当前时长 core mean/P99 门槛。
- Method B 在线配置仍由 loader 拒绝；新增 information/vector downdate 候选仅供 shadow 验证。
- 仿真栈显式 `/imu/data→/sim/imu` remap；nominal/H0 默认关闭 packet loss 和随机 NLOS，stress 可由 launch 参数独立打开。
- straight 往返轨迹保持固定 yaw，移除速度过零时 `0↔π` 的非物理瞬时航向跳变。
- 仿真发布 `/uwb_sim/fault_truth`，实时 run 同步记录 `ground_truth.csv` 与 `fault_truth.csv`；run directory 可显式指定绝对路径。
- IMU gap 发布 UNAVAILABLE、记录事件并受控重初始化；stale frame 显式拒绝且事件时间使用处理 watermark，避免日志 timestamp rollback。

## 复现命令

```bash
catkin build uwb_imu_pl --no-status
catkin run_tests uwb_imu_pl --no-status

# 正式 P2 数学采样（默认即计划样本数）
rosrun uwb_imu_pl integrity_monte_carlo \
  config/realtime_uwb_imu_pl_research.yaml results/p2_monte_carlo

# 正式 snapshot sweep（默认 seeds 20260901–20261000、200 epochs）
rosrun uwb_imu_pl snapshot_integrity_sweep \
  config/realtime_uwb_imu_pl_research.yaml results/p2_sweep.csv --resume
python3 tools/summarize_snapshot_sweep.py \
  results/p2_sweep.csv results/p2_sweep_summary.csv

# ROS 仿真；run_directory 可使用绝对路径
roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  trajectory:=figure_eight run_directory:=/absolute/path/to/run

python3 tools/validate_run_schema.py /absolute/path/to/run
python3 tools/generate_integrity_report.py /absolute/path/to/run \
  --markdown results/report.md --html results/report.html
```

其余正式 sweep 和长时性能运行完成后，应由 raw results 重新生成最终报告；任何失败必须保留为 FAIL，并附 CI、耗时和 artifact 路径。
