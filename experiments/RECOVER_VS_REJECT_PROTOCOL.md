# Recover vs Reject — 2026-09-12 冻结任务卡

用户实施计划为本轮授权依据；这是带标定限制的 development 预实验。固定顺序为
`zigzag_s4`、`loop-3d_s3`、`zigzag_s3`；完整 recording、tag1、全部 v2 anchors。
不换录制、不新增录制/注入/扫描，不提交或 push。T10=C2-C、T11=C、C1–C3 不升级。

## 准入与输入

只用原始 `range`，不用 range_calib、CIR、GT-derived bias、静态去均值。beta 为
MISSING_CALIBRATION，数值不校正；讨论总正误差，不声称分离动态 NLOS。
允许独立官方作者 CAD + 厂商近似变换；禁止本序列 GT 或逐录制拟合外参和测距优化几何。
必须明确加速度/角速度坐标、符号、测量原点和上游处理来源，然后统一到 IMU 原点。
无法闭合则 `BLOCKED_GEOMETRY_OR_IMU`，不得生成谎称 IMU 系的可运行 cache。

源 CSV 用列白名单抽取 measurement-only staging；完整文件 SHA 仅存 host provenance，
估计器身份只绑定白名单测量投影及已准入变换，改变 evaluator GT 不得改变输入身份。
每 UWB 源行一个消息，不合并/挪动时间；源 ordinal 为零基数据行号，physical CSV line=ordinal+2。
obs_id 复用 C++ StableObservationId。必需 RSSI 为相等中性占位0，非实测质量。
未使用 orientation 为 has_orientation=false、单位 quaternion 占位，不消费 GT/SDK orientation。
准入后复用 nlos_measurement_cache_v2（opaque transform hash，无伪造 injection ancestry）。
GT、窗口在 evaluator 私有目录；estimator mount allowlist 排除源 CSV 和 evaluator 目录。

## 方法与预算

共同模板 `config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml`；仅替换数据路径、
准入几何与输入单位。噪声、初始化、step=4、solver/recovery 不变；不按残差标定。
每条唯一 `pl_bidirectional_cusum` producer，forward/backward kappa=0.5，h分别
7.0234689587858723 / 7.0234689587858714，gap/reset/交集/分段保持现有实现。
Stage2 前冻结 support；suppress_all 与 lcb_fixed_full 共享 Stage2 cache、Values、support、
raw/IMU、input plan、nominal sigma、原始初值和 common preparation identity。
lcb_fixed_full 只沿用现有 c_hat−2sigma_local>0 与数值资格，接受后用完整 c_hat；无新 gate。
空 support 有效，不重跑 detector；producer失败阻塞两依赖方法，独立方法继续。

每条按 producer、suppress_all、lcb_fixed_full、robust_cauchy、SFUISE-ToA 串行。
Cauchy scale=2.3849。SFUISE 75bf5a32 保留原生初始化/优化/rejection，静态 ToA offset=0
明确未校正；同 raw range/IMU/几何的独立 ROS 桥，不读取 support/GT/recovery。
它是系统参考，不能称只差 recovery。每科学进程树1800秒，共最多15任务，失败不重试参数。

## 评价（任何科学运行前冻结）

参考点为 tag1 天线；估计从 IMU 原点按同一准入杆臂转换，GT 为作者 Vicon tag position。
区间为原始 tag1 UWB 首末时刻，网格从首时刻起10Hz；估计 nearest≤0.02s，GT线性插值
bracket≤0.05s、不外推。无效GT样本断开插值，不跨缺失补齐。
ATE 为 scale=1 SE3。RR 在共同有效时刻分别全区间对齐，窗口沿用同一对齐。
另列四方法共同覆盖，不让SFUISE缺失缩短主RR。无可评时刻/缺输出保持NA。
审计正误差窗口（非NLOS truth）为逐link全部 e>0.5m、持续≥2s、≥5包、gap≤1s
合格段的时间并集；无效/缺GT/非超限立即断段，重叠窗口只计一次。
range before=z_raw−h_GT，after=z_raw−delta_final−h_GT，同有效planned obs_id；
仅最终实际恢复应用offset，suppressed/fallback不伪装校正，恢复子集另表。
逐方法保留ATE_RMSE、窗口RMSE、ATE_P95、range before/after RMSE、候选观测/段数、
decision/final accepted段数、覆盖率、fallback、failure。
逐recording delta_RR=ATE(recover)−ATE(reject)，窗口同号，正值表示Recover更差；
缺失NA，fallback差值不作为成功恢复收益。

## 工程门与交付

测试源行/ID/单位/变换/符号、ROS序列化往返、GT隔离及GT变化身份不变、配对身份拒绝；
空候选/零接受/恢复/fallback/producer失败/超时/负收益报告；全部事件/窗口去重/GT缺口/
全区间对齐。先运行工程门，再消费科学任务。核心与冻结配置前后hash保持。
若准入阻塞，允许完成测量白名单、evaluator-only审计、12格/3差值的NA交付；
真实cache发布、三条C++ prepare、SFUISE接入/科学运行明确NOT_RUN，不声称已通过。
交付 `RECOVER_VS_REJECT.md`、CSV、锁定manifest、命令/exit/log/hash及独立输出目录。
