# UWB-IMU-IE sprint 协作约束

## 材料读取顺序

1. 先读 `doc/ie_sprint/STATUS.md`，确认当前任务、前置条件和 scope amendment。
2. 再读当前任务卡，以及 `STATUS.md` 链接的冻结论文结构与 roadmap。
3. T01 尚未建立合同前，方法相关判断只以冻结论文结构和 roadmap 为依据，不提前发明合同内容。
4. T01 建立后，任何方法或实验修改前必须完整阅读并遵守
   `doc/ie_sprint/METHOD_CONTRACT.md` 和
   `doc/ie_sprint/EXPERIMENT_CONTRACT.md`；若实现与合同冲突，先登记 amendment，不能静默改变定义。
5. 只读取当前任务所需源码、配置、日志和证据；不得把计划中的接口当作已经实现。

## 项目与任务边界

- 复用现有 IE/GTSAM 后端；不得另建一套完整 Python estimator。
- 只完成 `STATUS.md` 指定的当前任务及其直接阻塞，不扩展研究问题。
- 保持 legacy 行为兼容；确需 correctness fix 时记录原因、影响和验证证据。
- T00 只审计、复现旧基线和清点数据，不开发 NLOS 方法；T01 才建立方法合同、实验合同和论文工程。
- 保护已有未提交改动，不覆盖用户结果，不升级依赖或迁移工作空间，除非当前任务明确授权。

## 方法与数据约束

- paper path 在候选阶段前不得丢弃 suspected NLOS；保留原始观测、稳定 `obs_id` 和全部选择 mask。
- 严格区分固定静态测距偏置 `beta`、动态分段偏置 `c_s` 和 IMU bias 状态。
- 报告信息量时不得加入 LM damping、已丢弃的 L1/TV 项或人工先验。
- accepted `c_s` 必须保留在最终联合 graph 中；不得同时加入重复的 corrected pseudo-range。
- 最终轨迹、bias、残差和协方差必须来自同一最终 graph/`Values` 结果。
- 正式端到端估计不得读取 oracle/GT；prefix 输入必须在初始化前裁剪，禁止未来观测泄漏。
- 不得用 test label 调参，也不得省略失败、fallback 或 zero-coverage run。
- bias truth、测量参考和 post-fit residual 必须分别标注，不能互相替代。

## 证据与交付约束

- 每个 run 使用隔离输出目录；不得把共享的 `latest` 或 `trajectory` 文件当作结果数据库。
- 区分“源码声明”“当前环境观察”和“实际运行通过”。记录实际命令、退出码、日志/产物路径；未运行项写 `NOT_RUN`。
- 不得把 TODO、README、stdout 占位、文件存在或编译成功写成实验通过。
- 论文数字只能由锁定指标生成；不得编造结果、数据或引用。
- 每个任务结束更新 `doc/ie_sprint/STATUS.md`；T01 创建 `paper/CLAIM_EVIDENCE.md` 后，同时更新相关 claim–evidence 状态。

## 0911-STEP1 用户授权 amendment（2026-09-11）

本轮用户实施计划授权第一步 LCB 固定部分补偿及同集合全额消融，替代旧下一任务边界。
仅 `lcb_partial` / `lcb_fixed_full` 允许按段冻结动态 offset；旧方法 accepted live C 规则保留。
复用 Stage2 共同参考 R 与幅值列映射，独立有限性、满秩、正定检查后由单位向量求解得到
`sigma_c_local=sqrt(diag(R^-1))` 米，沿用原容差，不加 jitter/damping/prior。
按段 `delta=max(0,c_hat_stage2-2*sigma_c_local)`，结构/数值不合法或 delta=0 suppress；
不串联 eta/s/gamma gate。全额 variant 严格复用上述集合，仅 delta 改为 Stage2 幅值。
最终 raw factor 残差 h+beta+delta-z，每个恢复观测一次，无 live C；sigma 为局部诊断，
不是校准置信保证或最终 bias 后验。导航、残差、协方差来自同一 final graph/Values；
保留求解判据与一次 suppress fallback，固定模式无 live-C final rescore。
仅实现、工程测试、SFUISE Walk1 起始后 [8,11]s 固定 smoke、六输入四方法加载/启动检查；
第二步精度矩阵 NOT_RUN，不按结果调 kappa/区间/阈值，不自动 push。
T10=C2-C、T11=C 与 C1–C3 限制不变，旧结果/默认/用户材料保护。

## 0911-FDE-STEP1 用户授权 amendment（2026-09-11）

本轮用户实施计划授权把论文主代码路径的 Stage1 从 `automatic_discovery` 改为
`imu_aided_fde`；旧 L1/TV automatic provider、历史结果和默认行为继续作为
legacy/development 保留。FDE 只在不读取 GT/oracle 的 preliminary tightly-coupled
graph/Values 上，对每个 planned raw UWB factor 使用其实际 unwhitened residual 和 factor
noise 做 1-DoF、`p=0.99` 的双边卡方 fault detection；只把 `r<0` 的 fault 作为正 excess-range
candidate，再按既有 gap/min-count/min-duration 做确定性逐 link 聚合。它属于
RAIM/FDE-family residual front end，不是完整 ARAIM 或 certified integrity method。

本轮保留 Stage2、共同参考 `R_c`、local sigma、LCB、suppress/final graph 与一次 fallback
定义；FDE 空候选仍为成功并进入既有 raw Stage2。`structured_bias_only` 不与 FDE 组合；
FDE producer artifacts/cache identity 必须 provider-aware，legacy/FDE cache 不得互用。
仅按用户计划实施、工程测试、Walk1 起始后 `[8,11]s` smoke 和六输入五方法串行检查/评价；
不按结果调整概率、temporal 参数、kappa、区间或阈值，不自动提交或 push。T10=C2-C、
T11=C 和 C1–C3 限制不变，冻结结构/roadmap、旧结果与 `doc/v2/ie_0911/` 受保护。
