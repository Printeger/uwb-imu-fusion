# SC8 开发包使用

本包是下一轮实验设计，没有运行新实验，也没有生成虚构实验图。设计时读取的远端为`fa5b3dc`；上传的旧精度/耗时报告用于数据库存与排查，不用来判定新版能力。

## 放置

将本目录放入仓库：

```text
docs/requirements/sc8/
  SC8_EXPERIMENT_HANDBOOK.md
  GOAL_SC8.txt
  SC8_PLAN.template.yaml
  README.md
```

`SC8_PLAN.template.yaml`是待实现/登记的研究计划，不是当前runner已经识别的配置。以Handbook为主合同；实际落地时冻结为`config/sc8_experiments.yaml`或现有等价配置。

## 启动一个连续Goal

把`GOAL_SC8.txt`全文提交到可访问本地ROS/GTSAM、SFUISE与数据的Codex工作区，或输入：

```text
/goal 执行 docs/requirements/sc8/GOAL_SC8.txt。
以同目录 SC8_EXPERIMENT_HANDBOOK.md 的 E00–E08 与 A01–A24 为合同。
先核对最新代码和数据接口，预注册精选数据、配对方法、窗口与故障，
然后实际运行、评分、绘图并完成 docs/sc8/SC8_RESULTS.md。
不要全排列、不要从旧结果填新表、不要把真实数据的经验PL当仿真风险认证。
一个Goal内W0–W4连续执行，每包本地commit后继续，不自动push。
```

支持Goals的客户端可查看`/goal`、暂停`/goal pause`、恢复`/goal resume`。预算中断不是完成。恢复时从同一报告的最后冻结run ID继续，不再重建SC8计划。

## 五个最重要的实验约束

1. SFUISE在线首发和最终历史值分列；官方UWB剔除保留。
2. 精选六条实测候选，先验证最新接口，不按算法谁赢来挑数据。
3. 100–1000历元必须同时尊重FGO/完整性窗口约束；大窗允许呈现非实时结果。
4. 仿真数学保证、实测经验、实测叠加故障三类分开；缺PL不合成虚构Stanford点。
5. 图表由真实输出生成，完整失败分母和校准/GT参与范围可见。

## 最后检查

应收到：一份SC8主报告、实际冻结manifest、精简metrics与图表、可复现脚本和本地阶段commit。核心图含配对精度、H/V Stanford、故障恢复时序、窗口Pareto、模块breakdown和长流资源。没有依据的能力保留NA/OPEN；算法不赢、长窗不实时不意味着实验失败，但必须真正测试并解释。

官方Goals说明：https://developers.openai.com/cookbook/examples/codex/using_goals_in_codex
