# T10-A11 first conditional navigation：运行前登记

登记时间：2026-09-09T12:27:15Z。状态：development diagnostic；T10 IN_PROGRESS。
指挥会话接受 `REVIEW_ACCEPTED_A10_SYNTHETIC_GENERATOR_AND_BOUNDED_PILOT_SCOPE`：
仅验收 A10 生成器与有限 pilot 交付，不表示 estimator 成功、validation 准入或外部 REVIEW.md。
A08 图级 FD 15/18、三个最小步长失败来源 UNKNOWN 的历史限制保留。

## 固定输入与禁止变更

只使用 `evidence/t10_a10_synthetic_pilot_20260909T115705Z/configs/P1_{los,step,ramp}_seed10101.yaml`
的原始字节、原绝对 cache 路径及原 raw 输入。全部角色 development。禁止重跑 P2/P3。
生成器、truth、随机源/noise、初始化、support、默认 policy/solver、停止容差、依赖全部不变。
A10 配置中被忽略的 `imu.use_imu_orientation_init` 注释字段也不改；实际默认 false、无 raw attitude。
truth 不复制到 estimator 输入，不读取用于任何数值判断；只对历史 truth 文件做前后 hash 检查。
所有进程记录 file-open trace，检查无 evaluation/generation manifest/GT 读取。

## 有限预算与决策（先登记，后实现/运行）

- 顺序 LOS、step、ramp，每场景最多一个 C++ estimator 进程，外部 timeout KILL 120 s；总计最多3个/360 s。
  无 retry、额外 seed、参数搜索、后续 chain、Stage2、gate 或 held-out validation/test。
- 新 CLI `--diagnostic-first-block-budget` 默认关闭，只与 outer=1 自动 discovery 配套，不能混用旧
  fixed-checkpoint shadow recovery 或 A07 的 lambda 选择诊断。内层 options 原 max=50、V2不变。
- 同一 authoritative optimizer 前50次逐次记录 error/lambda、实际接受/拒绝计数、local update norm、
  五类梯度/最大尺度梯度及 roundoff/驻点条件。A11不调用含额外 solve 的 InspectFirstLinkedLmTry。
- 仅第49/50次从该次 authoritative TRYDELTA 输出取最后一个 trial；必须接受计数增1、完整 key/dim
  解析，且 base Values 原生 retract(delta) 与实际接受 Values 的最大 local 差 <=1e-12。
  不满足则完整方向不可用，禁止续跑。保存所有 trial、完整 delta、base Values 与逐 factor 数据。
- 对两个实际接受 delta，以 u=delta/||delta||2、同一 live graph/base Values/native retract，
  一次性计算图级和每个 factor 的 gᵀu、中心 FD，步长固定 {1e-4,1e-5,1e-6}。
  原判据逐点 `abs(FD-analytic)<=5e-9+5e-3*abs(analytic)`，不得放宽或删点。
- 第50次终态额外对最大尺度梯度坐标用同样三个步长检查，保存该终态 Values。
  A10 数值复现核对 initial/current/previous error、lambda、五类梯度、最大梯度与roundoff，
  数值阈值 abs<=1e-12+1e-12*abs(A10)，计数/共同准备身份严格相等；二进制因诊断代码重编变化单独登记。
  A10没有保存第50次完整 Values，因此不冒称历史完整状态逐元素对照。
- 续跑 gate fail-closed：两次实际接受方向均完整且原生 retract 匹配；所有图级6点、所有factor点、
  第50次最大梯度坐标3点均按原判据通过。缺点、非有限量或任何失败即终止，不执行第51次。
- 若 gate 通过，允许同一 optimizer（保持 Values、lambda、外部停止条件）从51继续，最多总计200次。
  达到原 generic convergence AND navigation stationarity（原1e-6+原roundoff）或原 lambda exhaustion
  或200上限即停止。即使首块收敛也以明确 DIAGNOSTIC_STOP 返回，保证不进入chain。
  续跑仅 first-block budget diagnostic，不改默认 max、配置身份或生产缓存语义；本轮不会发布Stage2 cache。
- 导数不一致时只利用已捕获逐factor定位，给出最小反例和下次具体动作；本轮不修 solver/Jacobian，
  不写 /usr/local，不实现 validation scheduler。

## 证据与可复核性

证据目录 `evidence/t10_a11_first_block_20260909T122715Z/`。实现前封存源码、冻结文件SHA、dirty状态。
归档实际命令/退出码/失败、配置、raw输入快照、源码前后diff、实际/proc maps动态库SHA/buildID与成本。
聚合只读既有结果，无结果驱动运行扩展。失败及NOT_RUN保持分母；segment/group/eligible/score不适用，
不把未发现阶段的空文件称为零eligible。更新STATUS、T10_READINESS、CLAIM_EVIDENCE，不升级C1–C3。
