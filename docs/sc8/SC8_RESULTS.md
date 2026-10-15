# SC8 实验结果与执行记录

**状态：IN_PROGRESS / W0。正式 E01–E08 尚未开始；本文件不是结项声明。**

入口 HEAD：`fa5b3dc9ffe3c88492352491d79ba022f373bfb5`。本地与设计时版本相同，工作区入口干净，无适用 AGENTS.md。保留用户既有数据和证据，不 push。实际运行矩阵位于 `config/sc8_experiments.yaml`，已展开 143 个唯一进程计划，上限 160；接口开发重试另外计账。

本轮目标为可靠证据，不以算法全胜、所有 PL 满足 AL、或 1000 历元实时作为成功条件。T-SIM、T-REAL、T-SEMI 分开；实测不具有 SC7 raw 源资格，PL=NA/拒绝。

## W0 数据与接口

全量本地只读库存已确认 R1 Walk2、R2 Walk3、R3 无障碍 15-31-28、R4 障碍 15-46-27、R5 loop-3d-z_s3、R6 loop-2d-fast_s3，另有 CAL-W Walk1、CAL-O 15-48-55、CAL-S loop-2d_s3。本轮未下载外部数据，未启动历史 40 条 campaign。九条均由当前适配器新生成输入并完成元数据审计，未将旧 CSV 当新算法结果。

Walk 世界变换只拟合 CAL-W：844 个保留样本，拟合残差 RMSE 0.178204 m。GT 仅用于接口标定与评分，未用于估计器状态初值、故障决策或镜像选择。不能将标定残差视作独立真实 GT 的误差界。

CAL-O 的角运动激励不足以可靠识别完整外参：独立候选拟合得到约 0.176 s 时移、水平杆臂标准差约 0.08/0.07 m。保留诊断后，固定使用 R3 原录制前 20%（截至 14.678774643 s）作接口标定，14.678774643–16.678774643 s 为隔离段，R3 主评分从 16.678774643 s 开始。没有重置估计或风险。该族标记 `GT-assisted / test-involved segmented calibration / CALIBRATION_OPEN`，不称独立录制盲测。前缀拟合诊断：gyro RMSE 0.147482 rad/s、accel RMSE 0.287109 m/s²、时移 0.0155421 s；不据此授予完整 6D 外参可辨识性。

STAR 使用官方几何与现有固定 ID 对应，不用 R5/R6 GT 拟合。20 ms 公共组包含不同物理时刻的测距；原时间保留，SFUISE RTLSStick 使用组末时间。这属于未认证的异步近似，不能称同步测距或继承仿真 raw 源定理。R5 最大 IMU 缺口约 257 ms，完整保留；接口时差上限由传感器 metadata 确定，不调整噪声。

实测 requested `O-FULL` 对应的研究 SC7 源 producer 仍拒绝。新增明确命名的 `O-FDE-ENG` 调用现有 general production pipeline，`O-ENG-NOFDE` 调用其 off 路径；这不是 SC7 完整恢复/PL 的替代验收。将分别报告实际候选、原生提交、失败、输出 coverage 与拒绝。通用 pipeline 不能借用未认证的 Gaussian 诊断签发 raw PL。

SFUISE 本地 SHA 为 `75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d`，remote `KIT-ISAS/SFUISE`，入口源码干净且保持只读。官方测距剔除开启。另构建该固定源码的只读观察版本，新增输入截止、输出时刻、core/CPU 计时；原始数值语句逆向剥离后逐字节相同。确定性双实例检查通过：1746 条相同输入事件、122875 次 knot 逐项核对，状态/标定/官方剔除开关一致。SFUISE 官方传感器初始化和 ours 独立初始观测/共用传感器初始化的区别必须披露，不称两种初始化提供完全相同的信息。

## 已完成的局部验证（不是正式主结果）

- 独立评分器 9 项测试通过，并与本地冻结 `evo v1.29.0` 核对 APE、ARE、body SE(3) RPE。包括姿态符号、单位、杆臂、时移、刚体变换、错序/重复、缺帧与 nearest-rank 慢值测试。
- 原始输入测试通过：SC8-H/U/A/G/J 各 1401 个 IMU 样本、1120 条 range，共用独立初始观测及全部标准噪声 draw；纯 IMU case 无新增 UWB 故障。UWB 2.25 m/3 历元、加速度 20 m/s²/500 ms、陀螺 1 rad/s/500 ms，在 raw 层按 manifest 注入。原 SC7 F0 与其显式 SC8 等价描述的原始行完全一致。
- 当前健康仿真冒烟 ON/OFF 各 80 次提交，160/160 条件 raw 合格；状态、H/V PL、source、generation、parent、版本与 bound_method 一致。未放开任何窗口或 source guard。
- R1 SFUISE 20 s 固定前缀运行成功，一次求解导出在线与历史。不能将这条接口冒烟的数值用作 E02 正式结果。
- R1 工程 FDE 两次配置预检失败已保留：缺 estimation_tuning 版本、以及 nominal-only tuning 对 on profile 的拒绝。修正为 runner 显式调用同一个现有传感器初始化函数，未修改核心 guard。第三次实际求解完成，约 499.98 s，期间观测到 `NO_VALID_CANDIDATE`、`AMBIGUOUS_UNAVAILABLE`、原生提交与显著积压。

## 断点与复现

本轮临时输入、原始逐帧记录和执行 journal 仅在 `.cache/sc8/`。尚未审计的唯一证据不清理。每次执行保存实际 source/ELF 身份、输入摘要、命令和退出状态。入口原二进制与当前增量构建的二进制不混为一列。

当前可用命令：

```bash
python3 tools/sc8_experiments.py preflight --build
python3 tools/sc8_sfuise_observer.py
python3 tools/test_sc8_metrics.py
python3 tools/test_sc8_inputs.py
python3 tools/sc8_experiments.py register
python3 tools/sc8_experiments.py run --group E00
```

`register` 保留已经存在的登记；`run` 复用有身份的已执行记录。显式 `--retry-failed` 保存旧失败到原 attempt 目录，新建后续 attempt。尚未完成 W0，因此没有正式计算冻结 SHA、W1 主结果、论文图或完成状态。
