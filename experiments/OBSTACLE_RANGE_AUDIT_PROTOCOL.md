# 0912 五数据集原始测距审计（执行前登记）

用户授权：仅审计 data/HUEC、MILUV、own_vicon、SFUISE、starloc 的真实 persistent positive
range-error 证据；不运行 estimator/recovery/定位 benchmark，不修改核心或默认配置，不提交/push。
优先 MILUV cirObstacles、两个自有 obstacle bag、HUEC NLOS，并保留同族 nominal 对照。
本文件是当前任务卡；旧实验和 claim 限制不升级。新增审计曝光属于 exploratory development，不能再当未见 test。

## 统一评价定义

- 主误差 `e_raw=z_raw-||p_tag_GT-a_GT||`，单位 m；**不减 beta**、不居中、不拟合外参/时钟。
  与旧 evaluator 的 `z-beta-h` 分开命名；本轮检查总原始测距误差，不称动态 bias truth。
- 同一 evaluator 的几何/汇总供所有 family 使用。MILUV 用 range_raw；STAR-loc 用 range；
  HUEC 用 distanceFromTag/Distance；own_vicon 用 dis。拒绝 TDoA；不读 CIR/图像/LiDAR/估计轨迹。
- GT 仅 evaluator 私有消费。MILUV mocap 插值沿用 linear position + shortest-arc SLERP，
  最大 bracket gap 0.05s，不外推。STAR-loc 使用作者已提供的测量时刻 tag GT 点，另以
  版本化 anchor 重算距离并与发布的 gt_range 交叉核验；不向 estimator 导出任何 GT 列。
- HUEC 动态使用作者 trajectory.csv 的 GNSS 参考和作者评估代码的 z+1m 规则，
  不重做由测距拟合的 alignment；GNSS 约8Hz，预先采用最大 bracket 0.25s，位置线性插值，
  同一规则用于全部 LOS/NLOS。标记 PUBLISHED_GNSS_REFERENCE，外参精度/高度基准限制保留。
- own_vicon 未知天线偏移不填已标定零：可显示 Vicon rigid-origin 距离的 PROXY 诊断，
  主 true-antenna range-error 为 UNAVAILABLE；anchor 身份仅按现有明确 ID/topic 映射。
  执行前用户补充：天线近似与 IMU 同原点，anchor/轨迹真值在各 bag 的 Vicon topic。
  据此采用 tas_uwb_0 为移动共点参考、tas_uwb_1..4 为同 ID anchor 参考，逐时插值两端位置，
  不从测距拟合；标记 USER_APPROXIMATE_COLOCATION，残差可作带近似条件的审计。
  同一 header 时基仍是未独立校准的记录时间关系，检查消息记录延迟但不搜索 clock offset。
- SFUISE 是 Vive 而不是 Vicon/GNSS，在本轮严格 GT 类型约束下只做 count/dropout，
  几何 residual 为 UNAVAILABLE，不沿用旧单位变换假设输出新的伪 GT。
- HUEC 静态参考为激光仪，超出本轮 GT 类型约束；统计 UWB/count/dropout，残差为 UNAVAILABLE。
  不把文件名标称距离冒充 GNSS/Vicon GT。空文件/缺文件与 reference 不可用均保留。

## 描述性统计（非 detector，不调参）

每 recording/anchor 和 recording/tag/anchor 均报告总样本、有效正测距数、GT匹配数、
signed median/mean、RMSE、absolute P95（另列 signed P95）、正误差比例、正尾条件均值/P95，
`e>0.2/0.5/1.0m` 的 count/fraction。primary 明显偏置描述门槛固定为 `e>0.5m`。
报告各门槛最长严格连续超限 run 的起止/持续时间/数量/均值；非超限、无效或未匹配行立即断开，
同 link 时间 gap>1s 也断开。持续事件描述为 >=2s 且 >=5 samples，不是 NLOS ground truth。
不同 tag 不串联 episode；anchor 总表 longest 从 constituent links 取最大。

dropout 单独报告 source-order 非单调/重复时间、正 interarrival median/P95/max、gap>1s
数量与累计时长、相对于 link median cadence 的 >3倍间隔及缺样本估计（不是已知发包丢包率），
recording 首尾相对缺口；静态若有 Transmission/Reception counters，另给计数差统计。
GT 未匹配率不等于 RF packet loss。没有发送计数的输入不声称真实 packet-loss rate。

## 产物与核验

新增独立 experiments/scripts 审计工具/工程测试和报告；逐观测、图、源身份与命令放
`/home/mint/ws_fusion_uwb/evaluator_private/icra/obstacle_audit/<unique-run>/`。
源数据只读；无索引 bag 仅临时副本 reindex，源 SHA 前后核对。所有 recording 保留状态，
异常不能被成功汇总掩盖。少量按 anchor 分面的 time/error 图包含 full-range 可见性。
最终更新 STATUS 和 CLAIM_EVIDENCE；方法参数/核心 build/定位/恢复均 NOT_RUN。

## 首轮工程发现与 evaluator correctness 修复

首轮 `20260912T111344Z-afb80d7c` exit1：HUEC 静态 CSV 含六类统计 footer，
不是观测；部分 anchor_id 还带 `anchor_id: 12` 文本。751 个解析失败保留，不解释成丢包。
解析器只剥离明确命名的统计 footer，保留原行号和其他无效行，记录 footer 数与原 ID。

STAR-loc v1 marker CSV 的索引不是 UWB radio ID。原按同号结果作废；不改变坐标数值、
不优化任何外参。evaluator-only 映射 radio→marker 为
`4→12,5→15,6→16,7→11,9→9,10→13,11→10,12→6`。
来源层级为由作者发布的 tag_pos/gt_range 几何恒等式解析出的 ID 对应，**不是独立标定文件**。
每条 recording、每包必须重新核验该对应计算的距离与作者 gt_range 在 1e-9m 内一致，
不一致直接失败；v2/v3 同号也做同样检查。这个映射不读取 raw range，不按误差择优，
只用于 evaluator，绝不导出为 estimator 的 GT-derived preprocessing。
因此 v1 将另标 PUBLISHED_VICON_REFERENCE_ID_RESOLVED，未来 estimator 接入仍需独立 ID 来源。
首轮所有结果保留为 superseded engineering attempt，最终输出由修正版本在新目录生成。

第二轮 `20260912T111707Z-2b07ed17` exit1：39 条动态和765条静态完成；另外两条静态
含完全空白占位行/截断行，没有有效时间的 link 触发首尾 gap 算术错误。修复为 NA 并保留无效行；
不编造时间、anchor ID 或丢包率。原结果保留，所有输入不变，修正版在第三个隔离目录评价。
