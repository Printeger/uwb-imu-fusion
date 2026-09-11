# Walk1 FDE 取证、唯一修复与固定场景验收结果

状态：**ENGINEERING_PASS / WALK1_GATE_FAILED / NO_RECOVERY_BENEFIT_EVIDENCE**。原算法取证后选择 C 分支；未确认 masking，未实现 D。完整证据根目录 `/home/mint/ws_fusion_uwb/res/fde_forensics_20260911_01`。

1. **基线与授权**：唯一代码基线 `c254c6acdce48a1be66febaaa935e0f2846e9992`，修改前工作区仅有受保护的未跟踪 `doc/v2/ie_0911/`。完整合同与冻结结构/roadmap已读；[实施前 amendment](../ie_sprint/FDE_FORENSIC_PROTOCOL.md) 和后续 C 分支公式先于生产修改登记。未提交或推送。

2. **旧流程与冻结部分**：完整 raw Gaussian checked LM → 完整图 post-fit residual covariance → 严格单点 `r²/Qrii>6.6349` 且 `r<0` → 原 temporal → Stage2/recovery。Stage2、Rc/local sigma、kappa=2、固定因子、优化器与 evaluator 保留。`freeze_audit.json` 逐文件核验；`nlos_inference.cpp` 唯一例外是 SUCCESS_EMPTY 的 provider 身份检查扩展，精确替换审计证明恢复数值逻辑相同。

3. **原输入及角色**：Walk1 normal clean/injected，semi-synthetic added-component-only；天然总偏置未知。原 manifest/truth 锁定 link `27956:20276`、闭区间 `[1664959678.3077347,1664959686.3077347]`、新增 `0.5 m`。111 条 affected raw / 30 条 planned，未重选窗口或输入。

4. **原始守恒**：4850 条 raw 中恰有原111 IDs变化；IMU字节、其他测量、原始身份及metadata相同。`raw_pair_audit.json`、`fixed_config_audit.json` 保存身份；新配置唯一语义增加 `nlos.fde_grouped_test=true`。

5. **physical factor 与计划**：[111行逐观测表](evidence/fde_forensics/fde_injected_observation_forensics.csv) 保留81条 `NOT_PLANNED`，其factor/residual/test数值不可用。30个actual scalar ExpressionFactor的measurement、sigma、index/keys、keyframe/obs/source时间映射已核验；[完整planned长表](evidence/fde_forensics/fde_all_planned_observations.csv) 支撑1074分母。

6. **注入与残差符号**：physical measurement的+0.5m与共同初值pre-fit的−0.5m最大误差均为0；比较使用16×binary64 epsilon×量值尺度。初值内容身份相同；注入后30/30 post-fit residual为负。不是 injection/sign correctness bug。

7. **衰减与分母诊断**：residual shift P10/P50/P90=-0.281323451/-0.206244372/-0.155445829m；attenuation P10/P50/P90=0.310891657/0.412488743/0.562646901，min/max=0.267710446/0.787192758。方差比中位数1.026504640；最大单点统计量2.200384483<6.6349。平方残差numerator、variance denominator、redundancy及min/max完整分列在[summary](evidence/fde_forensics/fde_forensic_summary.json)。衰减不能单独裁决masking。

8. **完整协方差核验**：同一4509×3435完整Gaussian白化线性化包含全部nuisance、IMU与prior行；稀疏QR秩证书3435。以单位向量projection提取PWW，没有全图稠密逆、damping、jitter或新prior。physical whitening/RHS sign/row coverage逐factor检查；PWW对称、PSD、幂等与正交审计通过，最大所报误差6.83e−15。谱及raw/projected残差均保留。

9. **aggregate/global**：rank30窗口clean/injected raw T=4.332634210/52.326346402，阈值50.892181312；projected=4.325050482/52.531986294。注入窗口reject。单点统计量和仅29.066111541，不能代替完整协方差。global DoF1074，raw257.869203986/308.251375660，projected257.852234764/304.685595150，阈值1184.750042947，均不reject。切空间残差norm0.130265966/1.888327437；不混用两种残差，不声称非线性精确calibration。

10. **hold-out**：仅移除原30 planned factors；clean/injected subset graph和共同initial身份一致，复用一次求解。原100迭代后 `CONDITIONAL_LM_MAX_ITERATIONS`，无重试。未输出未收敛subset轨迹为有效结果；held-out residual、预测协方差、solution separation均 `NOT_RUN_SUBSET_SOLVE_FAILED`。完整full轨迹已保存；masking保持INCONCLUSIVE。

11. **封存裁决**：[A–H决策表](evidence/fde_forensics/FDE_FORENSIC_DIAGNOSIS.md) 与 `diagnosis_seal.json` 在生产修复前封存并复核。A/B/E/F/H未获错误支持；C仅支持原truth-window aggregate存在；D/G INCONCLUSIVE。全图未拒绝不证明故障不可检测。

12. **唯一修复C**：opt-in provider `imu_aided_grouped_fde_v3`，缺省false保留V2。按同link/time/obs_id排序，以原严格gap>1s切最大planned连续组，原count>=2、duration>=0.01s。用e=r/sigma、T=eᵀPgg+e、DoF=rank(Pgg)，Boost.Math chi-square p=.99。d=1/sigma，GLS signed residual=(dᵀPgg+e)/(dᵀPgg+d)，负号才是positive excess。唯一显著positive link才贡献组内负residual，再走原temporal；多link返回歧义/空支持，不强选。未实现D/global-gated isolation。

13. **代码与cache**：新增独立C++ forensic入口，编译复用现有runner loader/export helpers；新增 `fde_math`、`fde_grouped` 和测试，Python只配对、编排、报告。生产无truth参数。provider/version、group规则和统计身份进入Stage1/cache；V2/V3 replay双向拒绝验证见 `cache_incompatibility_audit.json`。组协方差、谱、GLS/test count/status独立落盘并纳入V3 cache payload。概率是单检验概率，不是总体误报率。

14. **工程验证**：全工程+全部测试目标build exit0，完整CTest **31/31**，新增数学/分组8/8、旧FDE10/10与cache合同回归通过。包含异方差/完整相关窗口、rank/零冗余/近退化、严格等号与相邻浮点边界、交叉covariance手算、唯一/歧义link、gap/count/duration/符号/零候选；既有final/all-suppress/fallback回归保留。D-specific global/isolation/subset-failure生产状态机测试为NOT_APPLICABLE_BRANCH_C，不宣称实现未选分支。

15. **命令、预算与失败**：全部主运行/构建/测试命令、exit、日志和hash索引见 `commands.json`。原forensic exit1/143.148812s是subset算法失败，不重试；两次bwrap启动在provenance前因/dev/null不可用退出，trace确认estimator exec=0，增加/dev绑定后新目录继续。初次新target配置、C++ API/链接、报告列名及访问追踪将源码文件名误判为GT数据的错误均保留；修正仅涉及工程/报告，不重跑算法。实际estimator进程树1800s，无算法重试或放宽判据。bwrap隐藏truth及锁定GT，strace核验无forbidden open。

16. **固定Walk1六方法验收**：机器生成[Table A](evidence/fde_forensics/walk1_table_a.csv)、[Table B](evidence/fde_forensics/walk1_table_b.csv)、[完整JSON](evidence/fde_forensics/walk1_acceptance.json)。所有failure/fallback/coverage与窗口指标保留；下表为完整轨迹aligned RMSE(m)，20ms最近邻、scale=1 SE(3)，窗口沿用full alignment。

| condition | method | status | aligned RMSE (m) |
|---|---|---|---:|
| clean | all_range | COMPLETE | 0.163853268 |
| clean | robust_cauchy | COMPLETE | 0.163847231 |
| clean | suppress_all | COMPLETE | 0.163853268 |
| clean | structured_debias | COMPLETE | 0.163853268 |
| clean | lcb_fixed_full | COMPLETE | 0.163853268 |
| clean | lcb_partial | COMPLETE | 0.163853268 |
| injected | all_range | COMPLETE | 0.198923089 |
| injected | robust_cauchy | COMPLETE | 0.196904359 |
| injected | suppress_all | COMPLETE | 0.198923089 |
| injected | structured_debias | COMPLETE | 0.198923089 |
| injected | lcb_fixed_full | COMPLETE | 0.198923089 |
| injected | lcb_partial | COMPLETE | 0.198923089 |

clean: tested=1074, grouped tests=9, retained segments=0, point FP/FPR=0/0.0, support TP=0.
injected: tested=1074, grouped tests=9, retained segments=0, point FP/FPR=0/0.0, support TP=0.

目标 link 最大连续组覆盖 `[1664959677.5012724,1664959736.0829639]`，213 条观测；clean/injected T=72.174651285/116.140359811，rank213 阈值263.934234048，均不拒绝。与原30条truth-window检验不同，不重新切窗口。每侧9次组检验；1074条单点检验仍为0 fault。`qr_factorizations=1` 是原单点归一化计数，不包含新增组协方差投影工作。

零候选时Stage2与四final按既有SUCCESS_EMPTY复用同graph/Values，optimizer=0；Rc/local sigma/c_hat/LCB offset为NOT_APPLICABLE_NO_CANDIDATES，不能填0当恢复证据。所有实际final已运行；同图一致性见 `final_graph_audit.json`。离线delay在无TP时UNAVAILABLE_NO_TRUE_POSITIVE。

17. **矩阵准入与停止**：`NOT_RUN_WALK1_GATE_FAILED`。clean定位门及零段门通过，但injected若无新增TP/重叠support即不准入。扩大十个已准入场景矩阵未运行；原两个injected跳过项与原结果保留，不重判或重选。未要求LCB优于suppression，也没有调kappa/概率/窗口/temporal阈值。

18. **claims与限制**：支持本输入注入/plumbing/sign审计与受测Gaussian/分组工程正确性；不支持独立确认masking、普遍统计可检测性、校准完整性保证或LCB恢复收益。组最大连续窗口可稀释局部故障，本轮接受负结果。T10=C2-C、T11=C、C1–C3不升级；STATUS/amendment/CLAIM_EVIDENCE同步后停止。

数学来源：[Kuusniemi reliability testing, §5.1](https://www.ucalgary.ca/engo_webdocs/other/Dissertation_Heidi_Kuusniemi__Sep05.pdf)，[Blanch et al., solution separation and chi-square discussion](https://web.stanford.edu/group/scpnt/gpslab/pubs/papers/Blanch_et_al_IONGNSS_2012_B5_nr7_post_submission_rev3.pdf)。C是用户授权的grouped extension，不声称完整复现其经典global/local流程、ARAIM风险模型或protection level。源位置/hash见 `source_ledger.json`。
