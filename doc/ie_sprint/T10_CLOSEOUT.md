# T10 Closeout — C2-C，冻结探索性恢复结果

本次用户授权的收缩工作包已完成并冻结；不是原完整 T10 held-out RQ3 验收，也不是安全门控准入。旧完整计划停止适用。T11/T12 均 `NOT_RUN`。

## A. 最终 correctness 状态

| 项目 | 状态 | 证据 | 处理 |
|---|---|---|---|
| zero-candidate LOS | 接口修复通过；20101 对照数值失败 | 两条 LOS 均有零候选 Stage2、score N/A、五个有效 final；20102 独立 all-range + evaluator 全链通过 | 20101 独立 all-range 的 lambda exhaustion 保留，LOS 相对代价 UNAVAILABLE；不救 solver |
| ramp metadata | correctness 已修复 | 唯一定位：obs_id=14136106683168915725，factor139/outer2，期望 +0、实际 -0；测量相同。回归修复前失败、后通过 | 仅残差精确数值比较接纳有符号零；非零错测量仍拒绝。完整修复回归 score 组含短段、零 eligible，五个 final 有效 |
| no-C exhaustion | 合法数值失败，传播通过 | 旧缓 ramp 的四个 Suppress 策略均一次 fallback 后失败，无有效轨迹；新增 LOS20101 all-range 同类失败 | 不改模型、lambda、停止判据或精度 |

同时修正未知计数误记零、decision/final bias 列、空集合 bias RMSE、缺失 final 输出，以及把 policy reason 差异误当 Use/Suppress 差异的评价布尔值。改动均在同包内闭环。

构建通过；重启后实际复验：28/28 refit、20/20 inference、3/3 Python 回归、真实 LOS 完整链。初期两次编译错误及零测试的旧二进制调用不计通过，日志保留。

## B. 最小 η/s 诊断

仅既有 validation 20101/20102 的 step2 × A/B/C。A 审计复用冻结 baseline；B 使用相同 PCG64 抽样，`z_B=z_A+ε_A`，实际/名义 σ=.10 m；C 在初始化前保留原始偶数 UWB 帧，σ=.05 m，2.5 Hz，原 obs_id 与 IMU 不变。两派生条件均从 raw 重新初始化和自动找段。
P1、模型、solver、阈值 `τη=.10, τs=.10 m, τγ=1`、`εbad=.20 m` 均冻结。每个输入共享自己的 Stage2，五策略 final；四条新增输入各一次，30/30 六输入 final 有效，无 fallback。

| seed | 条件 | 每段观测数 | N | η | s (mm) | s_fit / full_gate |
|---|---|---:|---|---:|---:|---|
| 20101 | A | 16 | 6400 I | 0.585940 | 16.329901 | USE / USE |
| 20101 | B | 16 | 1600 I | 0.597689 | 32.337187 | USE / USE |
| 20101 | C | 8 | 3200 I | 0.592288 | 22.969866 | USE / USE |
| 20102 | A | 16 | 6400 I | 0.582791 | 16.373950 | SUPPRESS / SUPPRESS |
| 20102 | B | 16 | 1600 I | 0.595515 | 32.396161 | SUPPRESS / SUPPRESS |
| 20102 | C | 8 | 3200 I | 0.588247 | 23.048641 | USE / USE |

历史评价区间为 [3,6] s；A/B 匹配16点，C匹配8点。以下为 structured_debias 的 raw-frame ATE；Δ 相对同输入 suppress_all，正数为变差。

| seed / 条件 | RMSE / P95 (mm) | ΔRMSE / ΔP95 (mm) |
|---|---:|---:|
| 20101 / A | 9.471582 / 14.577266 | +0.746240 / +0.865374 |
| 20101 / B | 18.066380 / 23.420823 | +1.307603 / +1.735874 |
| 20101 / C | 14.449711 / 23.833973 | +1.185068 / +1.739797 |
| 20102 / A | 19.410106 / 25.679921 | +0.320005 / +1.409877 |
| 20102 / B | 37.023408 / 49.718406 | +0.622056 / +1.783169 |
| 20102 / C | 15.013712 / 21.004599 | -0.629876 / +0.714049 |

N 已实际变为 6400I/1600I/3200I：跨条件不再共享 `η=1/(6400s²)`。各条件内部仍有 `η=1/(N_scale·s²)`，不能声称统计独立。六条实际 s_fit/full_gate 决定完全相同，没有 η 的增量决策或收益证据。
完整 N/谱、λmin(Rc)、η/s/γ、段身份/观测数见 [GROUP_RESULTS.csv](evidence/t10_closeout_20260910T141607Z/GROUP_RESULTS.csv)；各策略 final、bias/coverage/failure 见 [FINAL_RESULTS.csv](evidence/t10_closeout_20260910T141607Z/FINAL_RESULTS.csv)。全部统计来自 [RESULTS.json](evidence/t10_closeout_20260910T141607Z/RESULTS.json)。

## C. C2 决定

**C2-C: recovery policy exploratory。** Structured 相对 suppression 的 RMSE 1好5差、P95 0好6差，未显示跨两条基础轨迹的重复收益；按照运行前登记的 C 优先规则裁决。η 虽已脱离原固定 N 映射，但 full_gate 没有区别于 s_fit 的决定。保留 Rc/η/s 及结构化偏置代码作 recoverability characterization / exploratory structured correction；不再把 safe recovery policy 或 η 必需门控作为已证实的主贡献。不作统计显著性、普遍有害或系统价值自动兜底的推断。

## D. 冻结 claims 与证据限制

| 状态 | 允许的表述 |
|---|---|
| SUPPORTED | 已测试域内 Rc/η/s 计算与局部正确性；本次两个 validation 输入的零候选生产分流；不同 noise/count 下 N 的实际改变 |
| PARTIALLY SUPPORTED | 有限合成自动全链及 graph-consistent 输出；LOS 全链20102成功，20101独立对照失败。不是全面系统/真实数据性能证据 |
| NOT SUPPORTED | η 增量操作收益、稳定结构化恢复轨迹收益、safe recovery policy、两条 LOS 均无退化 |
| NOT YET TESTED | T11 future-context/prefix、T12真实/公共数据新评估、正式 held-out RQ3与泛化、论文完整 C1/C3 |

重启损坏了新运行的评价/冻结 sidecar，保留在各 case 的 crash_history，详见 [CRASH_RECOVERY.json](evidence/t10_closeout_20260910T141607Z/CRASH_RECOVERY.json)。从完好的最终产物重建清单后独立重算，**未重跑任何估计器**；不能独立追溯已丢失的原始 pre-evaluation 哈希清单，重建包不恢复“未见”身份。保留 estimator trace 的 truth open 均为0，A另核验旧冻结输出。
当前 [独立核验](evidence/t10_closeout_20260910T141607Z/VERIFIED_RESULTS.json)：10558 项保留产物 hash、6 组 score、90 组 trajectory 指标复算通过；未另建或重优化完整 GTSAM graph。此为有限、已见 validation characterization。

## E. 冻结、精简及小型交接

不再增加阈值扫描、solver study、validation grid、gate redesign、seed hunting、A/R新研究系列；不运行旧18/27/111/230扩展矩阵。科学负结果不触发返工。
按用户磁盘清理授权，删除36,628个可再生成矩阵转储、重复二进制/归档等，共34.15 GiB；保留关键输入、源代码、配置、最终估计/评分和失败证据。旧完整归档已精简，不再声称每份旧包仍完整可验证。见 [删除账本及保留范围](evidence/retention_20260910/POLICY.json)。WSL trim 成功后宿主机延迟回收，最终复查 C: 可用空间由约1.5 GB增加到约19 GB；本任务未关闭 WSL。

- **T11先行（未执行）：** 冻结同一历史区间；初始化前裁剪 UWB/IMU；比较 prefix/full 的 Rc/s/η和适用轨迹后果；禁止继承 full 的状态/support，验证修改未来不影响 prefix。沿用冻结配置，只作诊断，不声称 gate 已安全验证。
- **T12随后（未执行）：** 在已有真实/公共输入上先确认标定/参考/时间语义，再以冻结实现和指标做可复现系统评估，报告改善、退化及全部失败。
- 后续运行只保留必要产物；不再归档可编译二进制或逐试步大矩阵。

复算入口与实际命令见 [POST_CRASH_CHECKS.json](evidence/t10_closeout_20260910T141607Z/POST_CRASH_CHECKS.json)、[运行前 manifest](T10_CLOSEOUT_MANIFEST.json) 和各 case 的 COMMAND/EVALUATION_RUN。**T10 已冻结；仅本次收缩范围完成，T11/T12 均未启动。**
