# T10-A19 限定 amendment 与预登记协议

时间：2026-09-10T01:43:16Z

指挥接受：`REVIEW_ACCEPTED_A18_DEVELOPMENT_STAGE1_SCOPE`。来源为本指挥会话；独立复审记录为 `/tmp/t10-a18-commander-review-zl1ec9no/REVIEW.md`。该接受只覆盖 A18 development Stage1，不表示 T10、正式 validation、gate 或 claim 通过。

## 接口与身份

- 复用 `PAPER_CERTIFIED_PAIR_REDUCTION_V1`、现有 `AutomaticSupportProvider`、`SegmentRefitter::Run` 的无正则 refit 和 `ScoreRefitRecoverability`；不复制 estimator。
- 新增默认关闭、仅 `role=development` 可用的 Stage2 conditional-navigation callback。每次回调从实际 Stage2 graph 构造点同步取得 factor index、pose key、anchor、lever、raw measurement、sigma、`fixed beta + current c_s` 和 `obs_id`。partition 是本次 Stage1 自动发现的冻结 partition；幅值更新仍由原实现执行 `c_s>=0` 精确块更新。
- A19 Stage1/Stage2 请求、诊断 manifest 和输出绑定同一 `a19-policy-sha256:*` 身份；身份覆盖策略名、实现版本、冻结配置、可执行文件、core、GTSAM、MPFR 和 GMP。A18 产物不改标签、不 warm start、不作为本轮输入。
- 普通 Stage1/Stage2 API 与缺省 legacy 行为不变。A19 输出 schema 为 `A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY`、`consumable=false`；现有正式 cache reader 必须拒绝。
- Stage2 最终 joint graph 继续使用 raw range 与单个 live `c_s` factor；不加入 corrected pseudo-range。评分复用原 reference-plus-group 构图：共同 reference 排除全部候选，仅按 overlap group 加回候选；damping、L1/TV 不进入信息矩阵。

## 冻结工程门

判据在测试前冻结，不因失败放宽：

1. A18 已接受的 43 对 P/D 证书、决策和默认 legacy 回归继续通过。
2. Stage2 development 请求拒绝非 development role、缺失/错误 `a19-policy-sha256` 身份和非 V2 conditional policy；普通 `Run` 行为保持。
3. 工程 fixture 在非零 `c_s` 下证明回调收到的 conditional constant 逐位等于实际 `fixed beta + c_s`，factor/key/obs 对齐；回调方向使用原 GTSAM solve/native retract，异常状态与计数透传。
4. refit 回归证明固定 partition、`c_s>=0`、去正则目标、四项 AND、最终 live `c_s` graph 不变；scoring 回归证明共同 reference 排除全部候选，输出 eta/s/gamma 完整性及 short/boundary eligibility 规则。
5. A19 schema/身份变化，且正式 Stage2 cache reader 拒绝其诊断 manifest。构建并运行受影响 core、A18/A19 工程测试、discovery/refit/scoring/config/cache 回归；记录命令、退出码与运行动态库身份。工程门任何一项失败，科学运行写 `NOT_RUN` 并停止。

## 唯一科学运行预算

- 输入：A10 冻结 P1 step、seed10101、原 raw/config；从 raw 重新初始化，truth/GT 不可读。
- 唯一有效变化：显式启用 A19 certified numerical policy，并允许它贯穿现有 Stage1 与 Stage2 conditional navigation。
- Stage1：outer 最多 500，每个 conditional block 最多 50；原 lambda、PAPER_IMU_CONDITIONAL_LIVE_BIAS_V1、L1/TV、partition 与全部容差不变。
- Stage2：outer 最多 200；固定 Stage1 partition、原去正则目标、四项 AND、幅值非负约束和全部容差不变。
- 一个全新 estimator 进程树，外部硬超时 900 秒；无 retry、checkpoint warm start、参数/精度搜索或追加预算。Stage1、Stage2 或评分首次失败即停止；Stage2 成功才进入已有 eligibility/eta,s,gamma scoring，评分后立即停止。
- 不运行 Stage2 cache/final inference、正式 validation/test、LOS/ramp/P2/P3、T11、scheduler 或 gate 决策。未到达阶段写 `NOT_RUN`，未评数量写 `NA`；development 结果不得升级 C1-C3。
