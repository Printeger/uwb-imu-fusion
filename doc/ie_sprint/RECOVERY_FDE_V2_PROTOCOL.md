# 0911-RECOVERY-FDE-V2 用户授权实施协议

状态：DONE / LOCALIZATION_AND_ENGINEERING_GATES_PASS / WALK1_ZERO_CANDIDATE。授权来源：本轮用户给定“恢复基础定位，再修复空候选与标准化 FDE”实施计划。
本协议为当前任务卡，替代旧下一任务/停止边界；历史合同和结果原样保留。

1. PAPER_RAW_REFERENCE_LM_V2：paper 共同准备在完整 raw Gaussian graph 上执行一次 checked LM，失败终止；robust 从该 Values 开始，all-range/FDE 复用 reference。prepare-only 与 cache replay 不重复优化。保留 planned ranges、yaw、固定杆臂、噪声、p、迭代上限。记录 raw objective 前后、状态、迭代、尺度和 graph/Values identity。
2. FDE_SUCCESS_EMPTY_V2：仅 reference success、FDE success、全部 planned tested、原始 partition 零段，且实际 graph/Values/metadata/obs_id/noise 内容审计通过时发布 SUCCESS_EMPTY；Stage2 optimizer=0。四候选 final 在完全同图时复用结果，final optimizer=0；旧交替停止条件 NOT_APPLICABLE，不伪造 CONVERGED trace。有候选而全部 suppress 仍用原 final/fallback。
3. IMU_AIDED_POSTFIT_FDE_V2：定位门通过后，在同一完整 Gaussian reference 上白化线性化，用一次稀疏 QR 求 d_i=||(I-AA†)e_j||²，Qrii=sigma_i²*d_i，w_i=r_i/sqrt(Qrii)。沿用尺度、秩、数值审计，无稠密全图逆/projector、damping、jitter、新 prior；不可确认或不可分辨即失败，不退回 r/sigma。p=.99、1 DoF、严格 w²>6.6348966010212145、仅 r<0 候选；temporal 参数不变。保存 measurement sigma、residual variance、w、诊断 r/sigma、测试状态、linearization identity，拒绝跨版本 cache。
4. 定位门：fresh legacy full Walk1 aligned RMSE 与 .169642149m 差<=.01m；paper all-range/Cauchy 均<1m，完整 planned trajectory。独立 evaluator 才读既有锁定 GT，20ms、scale=1 SE(3)。先关闭定位原因，再进入 detector 升级。
5. 工程门：reference success/failure、身份反例、empty/all-suppress区别、cache发布/replay/零优化；projector/Schur、异方差、完整 nuisance/IMU交叉协方差、高leverage/零冗余/近退化/等号/符号/damping独立性。受影响测试及完整 CTest。
6. 生产门：固定 Walk1 起始后[8,11]s smoke，full Walk1 五方法串行；其余五输入只加载/prepare。真实空/非空候选均接受。不展开六输入精度矩阵，不按结果调参。

所有运行用新隔离目录，记录命令/退出码/日志及失败和 NOT_RUN。保护 doc/v2/ie_0911/、冻结结构/roadmap、旧结果/默认；实施阶段不自动提交/push。任务完成后用户另行明确授权提交并推送本轮实现，受保护的未跟踪 `doc/v2/ie_0911/` 仍不纳入提交。T10=C2-C、T11=C、C1–C3不升级。局部 Gaussian 检验不消除数据依赖 prior/标定缺口，单观测1%不等于整段1%。

## 基础定位 correctness 补充（实施前登记）

实际链接库反例 engineering/linked_robust_before.log（exit1）：r=.6m、sigma=.15m、Cauchy k=2.3849，实际loss=1.5714894027765274而标准loss=3.806335111274521；FD梯度1.0554429921150188，而IRLS目标梯度6.9935006712883085。本机Robust继承Base::squaredMahalanobisDistance，调用已robust加权whiten，再把加权距离送入loss。paper-only PAPER_STANDARD_ROBUST_LOSS_V2 覆盖此距离为底层Gaussian距离，保留原WhitenSystem/IRLS、k、solver、预算和全部因子。只修目标/线性化不一致，不改/usr/local或legacy；影响paper robust历史结果，旧结果保留并禁止混用。

## 来源与定位门事实

Kuusniemi (2005), *User-Level Reliability and Quality Monitoring in Satellite-Based Personal Navigation*，原始PDF正文p.72式54、p.76式60（PDF页102/106）已读取。原文局部检验采用残差协方差对角归一化；本实现保留signed w以选择r<0，仅平方用于双边检测，不增加原文global-test/isolation步骤。来源：https://www.ucalgary.ca/engo_webdocs/other/Dissertation_Heidi_Kuusniemi__Sep05.pdf 。PDF及提取文本保存在本轮隔离目录；临时PDF读取工具仅安装到/tmp/ie_fde_pdf_reader，不改变SDK/ROS/GTSAM/项目依赖。

定位门实测：legacy=0.1696421486772529m，paper all-range=0.1638532681570079m，Cauchy=0.16384723094111373m，均按锁定20ms、scale=1 SE(3)独立评价。对应localization_fixed_metrics.json及legacy_eval；Cauchy先前warm-only迭代失败保留。raw reference94次、objective2029100318.3603289→128.93460199296939；本轮不改变yaw/杆臂/噪声/迭代上限。完整CTest29/29、smoke/full Walk1五方法与30项prepare均已通过；详见 [结果与证据](../ie_0911/RECOVERY_FDE_V2_RESULT.md)。

## 阈值实现精度说明（收口澄清）

上文6.6348966010212145为理论分位数；本轮依用户“不改变阈值”约束，实际保留既有 `Chi2inv(.99,1)` 查表值6.6349（src/chi2.cpp未改），身份与artifacts记录实际值，严格大于及等号测试均以该固定值为准，未因结果调整。
