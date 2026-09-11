# 0911 第二步真实精度实验结果（2026-09-11）

状态：`EXECUTED_WITH_STAGE1_BLOCKED_METHOD_COMPARISONS / STRICT_SYSTEMIC_MAX_OUTER_RULE_NOT_TRIGGERED`。

本轮已按预登记完整执行六条 full 输入、两个低冗余条件、独立 GT 导出与评价。主矩阵六个共享
Stage1→Stage2 producer 全部失败，所以 `suppress_all`、`structured_debias`、`lcb_partial` 和
`lcb_fixed_full` 均没有有效 final trajectory；LCB 相对 suppress/structured 的收益、低冗余收益和主条件
全额补偿差异全部为 **UNAVAILABLE**，不是 0。没有 fallback、zero-coverage 胜利或失败剔除。
T10=C2-C、T11=C、C1–C3 限制不变；结果是 development/conditional evaluation，不是正式 held-out test。

## 冻结身份与接线

- 基线 commit：`0ba0755922854a95bb52aa0428daf68283aab37c`；运行前精简协议：
  [`STEP2_PROTOCOL.json`](STEP2_PROTOCOL.json)。未修改 `doc/v2/ie_0911/` 用户材料。
- 正式 runner：`sha256:025ffad222a435b192c470ff41fdfa6256ab3652982e63911305392909ded86b`；
  六输入 manifest：`sha256:c125cde2be1b5075bf48b5d625b98239bcde8ddf6bdffaeb5a475b63195f33fb`。
- 运行前只补三项 correctness 接线：Stage1 trace 的 active count/identity/相邻轮增删；scheduler 的真实
  `anchor_ids` 校验、共同请求身份和 runner CLI 透传；evaluator 的同一 GT 点 suppress 配对与改善量。
  kappa=2、Stage1 50 outer、阈值、噪声、R、固定补偿和 final factor 均未改变。
- 初次全量 CTest 因共享库已更新而两个 test executable 仍为旧 ABI 出现 2 个 SIGSEGV；显式构建
  `tests` 目标后，同一源码最终 CTest 27/27 通过。正式 runner hash 前后不变，实验结果未混用二进制。

## 实际矩阵与 Stage1 结果

六个 full producer 均在 Stage1 失败；其中只有 Walk1/Walk3 为 `MAX_OUTER_ITERATIONS`，少于预冻结的四个，
所以严格的 `NOT_RUN_SYSTEMIC_STAGE1_BLOCKER` 停止规则未触发，并按协议继续两个低冗余条件。

| 条件 | Stage1 状态 | Stage1 s | 后续四/三方法 |
|---|---|---:|---|
| SFUISE Walk1 full | `MAX_OUTER_ITERATIONS` | 6.146 | cache unavailable |
| SFUISE Walk2 full | `CONDITIONAL_LM_MAX_ITERATIONS` | 1.834 | cache unavailable |
| SFUISE Walk3 full | `MAX_OUTER_ITERATIONS` | 9.418 | cache unavailable |
| MILUV random full | `CONDITIONAL_LM_MAX_ITERATIONS` | 30.596 | cache unavailable |
| MILUV circular full | `CONDITIONAL_LM_MAX_ITERATIONS` | 17.517 | cache unavailable |
| own Vicon full | `CONDITIONAL_LM_MAX_ITERATIONS` | 3.799 | cache unavailable |
| Walk1 anchors 7475,9524,10548,15155 | `CONDITIONAL_LM_MAX_ITERATIONS` | 1.060 | cache unavailable |
| MILUV circular anchors 0,1,3,4 | `CONDITIONAL_LM_MAX_ITERATIONS` | 10.274 | cache unavailable |

Walk1 outer 50 的 objective/relative change/scaled step/navigation gradient 分别为
`63.11104164 / 2.2416e-5 / 3.7831e-3 / 0.10719`；chain KKT 通过，但 objective、step 和 navigation
stationarity 均未通过，active set 为 457 条、相邻轮变化 1 条。Walk3 outer 50 对应
`101.96424564 / 5.3044e-5 / 1.3379e-2 / 0.10918`，同样只有 chain optimality 通过，active set 581 条且
相邻轮无变化。完整最后十轮与 active-set SHA-256 在
`/home/mint/ws_fusion_uwb/res/ie0911_step2_results_20260911_01/stage1_last10.csv`。
其余 producer 在 conditional LM 内层上限处停止，多数没有完成可写入外层 trace 的一轮；
本轮没有按结果改变预算、容差或算法，也未实施 Stage1 修复。

## 可用轨迹精度

GT 由独立 exporter 输出，六份均含 source/config/trajectory hash；estimator manifest 记录
`gt_read=false`、`oracle_support_read=false`。下表只报告 scale 固定为 1 的逐 run SE(3)-aligned ATE。
固定 map↔GT frame 和 tracker/IMU 刚体点 provenance 未闭合，raw-frame 指标不报告。coverage 为输出 pose 的
GT 时间匹配覆盖；所有成功行均为 1.0。

| 输入/条件 | Cauchy 状态 | 匹配点 | RMSE m | P95 m | 水平 RMSE m | 垂直 RMSE m |
|---|---|---:|---:|---:|---:|---:|
| Walk1 full | OK | 229 | 343.3260 | 726.2887 | 313.1586 | 140.7281 |
| Walk2 full | OK | 293 | 709.9944 | 1403.2137 | 637.8539 | 311.8243 |
| Walk3 full | FAILED | — | — | — | — | — |
| MILUV random full | FAILED | — | — | — | — | — |
| MILUV circular full | OK | 1587 | 266.7025 | 543.2354 | 253.3491 | 83.3335 |
| own Vicon full | OK | 366 | 1683.4637 | 3315.0124 | 1061.8037 | 1306.3778 |
| Walk1 low-4 | OK | 229 | 342.6488 | 724.9242 | 312.1628 | 141.2892 |
| MILUV circular low-4 | FAILED | — | — | — | — | — |

这些全长 Cauchy 轨迹实际漂移到数百/数千米，不能以第一步三秒 smoke 的较小误差替代，也不构成有用定位
精度。Walk1 low-4 与 full 的数值不是 LCB 对 suppress 的配对收益，不能据此回答低冗余恢复价值。

## 直接问题回答

- **LCB 相对 suppress/structured：** 8 个条件都没有共同有效 final trajectory，RMSE/P95 改善量均
  unavailable；不能声称提高、恶化或相等。
- **低冗余：** 两个 producer 都在 Stage1 失败，证据不足；MILUV low-4 连独立 Cauchy 也失败。
- **全额补偿消融：** 预指定主条件 MILUV circular 的 full 没有 Stage2 cache/LCB 恢复集合，消融无辨别力；
  其他五个透明诊断同样 unavailable，不进入主结论。
- **测距参考：** anchor/杆臂/时基/独立静态 beta 的共同 provenance 未闭合，全部 N/A；未用 post-fit
  residual 冒充 range reference。
- **coverage/fallback：** 候选方法从未进入 final，candidate/restored/suppressed 数不可用而非零；fallback
  0 次。失败没有从 CSV 分母删除。
- **Stage1 blocker：** 严格“至少四个同为 MAX_OUTER”规则为 false，但实际 6/6 主 producer 均在 Stage1
  两类停止点失败，因此本轮新方法的真实精度比较在工程上被 Stage1 阻塞。

## 产物与复算

- 主 batch：`/home/mint/ws_fusion_uwb/res/ie0911_step2_main_20260911_01/`
- 低冗余 batch：`/home/mint/ws_fusion_uwb/res/ie0911_step2_low4_20260911_01/`
- GT：`/home/mint/ws_fusion_uwb/res/ie0911_step2_truth_20260911_01/`
- 汇总：`/home/mint/ws_fusion_uwb/res/ie0911_step2_results_20260911_01/`，包含 38 行
  `step2_results.csv`、`range_metrics.csv`、`stage1_last10.csv` 与两张真实数据图。
- 构建/测试：`/home/mint/ws_fusion_uwb/res/ie0911_step2_build_logs/`；最终 CTest 27/27，日志 SHA-256
  `52c1089c5a2a0b50a0843426a7eadf4ba5f1018fc54ef5c493519b14dc0c08ed`。

重跑主矩阵：

```bash
python3 tools/paper/run_experiments.py \
  --manifest config/paper/ie0911/six_inputs.yaml \
  --runner /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_paper_runner \
  --output-root NEW_ISOLATED_OUTPUT_ROOT
```

独立评价与最终聚合命令已逐字保存在相应 `*.command.json` 和本报告所列工具入口中。本轮到此停止：
不进入第三步，不调整 Stage1/LCB，不自动 push。
