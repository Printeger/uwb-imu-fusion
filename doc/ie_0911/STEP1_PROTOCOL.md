
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
