# MILUV Recover vs Reject：运行前冻结协议

用户授权：仅已审计且具有真实persistent positive range error的MILUV recording，运行冻结PL CUSUM、同cache的suppress_all/lcb_fixed_full及Cauchy/SFUISE；不隐藏负收益，不按结果调参。

固定顺序：default_1_random3_0、default_1_circular3D_0；完整recording，ifo001的tag10、全部constellation0 anchor（0..5）。两条tag10均已在先前审计中有合格正误差段。当前后端为单tag/单杆臂，明确不混合tag11；不新增recording、不改名单、不新增注入。

使用原始range_raw及imu_px4.csv白名单。每个源UWB行单独消息，源行号与obs_id稳定；不合并异步测距时间，不使用range/std/bias/gt_range、CIR、passive、相机IMU或GT去偏。作者cleanup_csv只选列及共同epoch平移；当前CSV已减过timeshift，不再平移。依据官方399ad46f源码，PX4 IMU与mocap body按同一轴系建模、加速度为Rᵀ(a−g)，不另转轴/取负/拟合时间。物理IMU与mocap body原点沿用作者共点近似，tag10非零杆臂[0.13189,-0.17245,-0.05249]m；原点近似作为development限制，不声称精确独立外参。作者发布constellation0 survey固定，无测距或本批GT几何优化。beta缺独立标定，明确未校正，不把总误差当纯动态NLOS。

共同模板config/paper/ie0911/sfuise_walk1_pl_bidirectional_clean.yaml：噪声、初始化、step4、solver和recovery全部原样；只替换输入路径和上述几何。前后kappa均0.5，h分别7.0234689587858723/7.0234689587858714，gap/reset/交集/分段原实现不变。每条仅一个producer，support在Stage2前冻结；成功后发布同一Stage2 cache供两种final。共同input/IMU/plan/sigma/原始Values/准备身份/support/Stage2 Values逐项核验，差异拒配。LCB仍c_hat−2σ_local>0及既有数值资格，接受后完整c_hat，一次原suppress fallback。空support为有效情况，不触发重跑。

Cauchy scale=2.3849，与RR同输入/初值；SFUISE锁定75bf5a32，消费相同原始测量与几何，保留原生初始化/优化/rejection，显式零ToA offset。仅数据相关topic/频率/杆臂/单位改变；采样系数1保留所有消息。SFUISE是系统参考，不声称只差recovery。GT、审计窗口、support和recovery输出不挂载到其独立进程。

工程先行：白名单/稳定ID/原始数值/几何/时间、GT不可见与GT变更不影响cache、实际CSV→ROS全行往返、C++ prepare-only与Cauchy prepare-only相同共同初值、SFUISE零测量启动、pair拒配/空候选/恢复/fallback/超时/负收益fixture。科学串行最多10个进程树，每树1800s，失败继续独立方法；producer失败时两个final NOT_RUN，不伪造cache，不调参或外部重试。

评价独立：参考点tag10天线；GT在原mocap时刻先按p+R*lever换算，再在固定网格线性插值。区间为tag10原始UWB首末，首时刻起10Hz；估计nearest≤0.02s、GT bracket≤0.05s、无外推。RR共同有效时刻分别一次全区间scale1 SE3对齐，窗口不重新对齐；四方法共同覆盖另表，SFUISE缺失不缩短RR。正误差窗口仅evaluator用原raw与Vicon几何，逐link全部e>0.5m、持续≥2s、至少5包、gap≤1s合格段的并集；使用与原审计一致的raw时刻位置线性/姿态SLERP，不能把这些窗口传给detector。

range指标在共同valid planned obs_id上：before=z_raw−h_GT，after=z_raw−delta_final−h_GT。只应用最终实际恢复offset，suppressed/fallback不校正；恢复子集另报数量和paired RMSE。失败/缺失字段NA，invalid producer不以占位0冒充有效空support。ATE/P95/窗口/覆盖率/候选观测和段/decision及final接受段/fallback/failure逐recording逐method输出。delta_RR与delta_RR_NLOS均Recover−Reject，正值更差；fallback不记成功恢复收益，零接受/空候选不算恢复胜利。

交付MILUV_RECOVER_VS_REJECT.md、8格CSV、2条配对差值、隔离输出、锁定manifest/命令/退出码/hash/图，并更新STATUS/claim ledger。已有STAR-loc/审计产物保持；不修改核心/默认/依赖、不提交/push。这是已GT曝光的development预实验，不升级held-out、T10/T11或C1–C3。
