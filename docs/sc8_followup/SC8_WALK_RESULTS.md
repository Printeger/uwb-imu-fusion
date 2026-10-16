# SC8 Walk Real-Data Follow-up

状态：**IN_PROGRESS，W0 已执行，W1–W4 继续同一 Goal。** 本文件为本轮唯一结果报告，不改写旧 SC8。

## W0：输入、版本和参考资格

入口 HEAD `00b8486c4f45024daa046b2f4ec20a0eec98afb3`，较设计核对版本多一个演示文稿提交。原有未跟踪 `docs/sc8/SC8_RESULTS.html/.pdf` 保留。仓库及上级路径未发现 AGENTS.md。ROS Noetic、GTSAM 4、原生 runner/DSO 实际存在；入口 ELF/DSO SHA256、SFUISE pin、每个源 bag 和 CSV 哈希见 [冻结合同](../../config/sc8_walk_followup.yaml)。SFUISE 固定 `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`。生产配置、AL、risk、故障合同未改。

三条 bag 均来自 `/home/mint/ws_fusion_uwb/data/SFUISE/`，重新读取构建本轮 cache。IMU 原始 m/s²、rad/s；range 按官方 anchor ID 加一次固定 offset，`ra=0` 保留并标无效；5 anchor，tag 27956。官方 IMU→tag 偏移 `[0.1,-0.025,0] m` 保持为估计器模型参数，其独立实测误差界未知。

评分 GT 改为 bag 实际 tracker 位姿，明确 **REFERENCE_OPEN**，不沿用旧适配器将 tracker 当 IMU 加杆臂后得到的物理 tag 声称。输入变换没有使用 TEST GT。`T_AB` 将 B 坐标映到 A，`T_WI=T_WT*T_TI`，`p_Wtag=p_WI+R_WI*l_Itag`。世界变换拟合不能替代 `T_TI`。

官方 [SFUISE 论文](https://isas.iar.kit.edu/pdf/RAL23_Li.pdf)、本地固定版本 README/配置/接口均已查阅。发布配置给出 IMU-tag offset，没有给出 tracker→IMU 完整安装外参及不确定度；bag topic 清单见合同，无静态外参闭合证据。因此物理 tag APE、body ARE、0.5/1s RPE 均 **NA**，真实 raw PL/HMI 概率资格 **UNQUALIFIED**。不据此中断 FDE 工程研究。

Walk1 CAL 预先以时间前60%拟合、后40%留出。有限 `[-0.1,0.1] s` 时间搜索得到 gyro→tracker 诊断时移 -0.0383347 s；训练旋转残差 RMSE 0.0602826 rad/s，留出 0.0487206 rad/s，identity 0.485762 rad/s。旋转 det=1，正交误差 1.15e-15。三方向激励奇异值 6.768/3.501/3.327；这只支持旋转相关诊断，gyro 对平移的 Jacobian 为零，不能据此声称完整6D外参已辨识。诊断矩阵不应用到输入或 TEST GT，不提供伪精度排名。

需要外部补充：tracker→IMU 安装旋转/平移、tracker→tag 测点定义、时钟偏移/漂移与误差界、世界重力方向验证。可采三轴转动与不同姿态平移的独立 CAL，包含静止段和同步事件，测量上述杆臂并报告尺寸误差；另留独立 TEST。当前数据不能提供可认证的有限完整外参误差界。

确定性测试：左右乘/逆变换/tag 杆臂、重力与角单位、已知旋转和37 ms时移共3测试 PASS。真实 Walk1 前缀22个原生状态的未来 GT 篡改测试 PASS：两次初始化、完整 trajectory、提交/动作/源截止逐值相同。生成脚本已预登记 Walk2 定向源批次、Walk2四类及 Walk3联合确认的弱/强额外故障，时刻/幅值在 FDE 结果前冻结。

## 后续执行与验收状态

W1–W4 尚未完成；不授予结项状态。WF01–WF04 的输入/参考与小测试证据已建立，物理外参资格保持 OPEN；WF05–WF18 待本 Goal 后续实际运行、实现、回归与报告更新。

```bash
python3 tools/sc8_walk_followup.py prepare
python3 tools/test_sc8_walk_followup.py
python3 tools/sc8_walk_followup.py noninterference
python3 tools/sc8_walk_followup.py baseline
```

本轮 cache 尚需用于后续执行，未清理。禁止自动 push。资源中断时从上述 baseline 与本轮 cache 续跑，不重建另一个 Goal。
