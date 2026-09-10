# R04 复审与 R05 最小任务

2026-09-10，指挥会话只读复核。未重新执行 GDB、prepare、工程测试或完整 estimator；本轮只新增复审、
下一任务prompt与交接记录。原R04运行事实依据封存栈、宏/布局记录、状态JSON和当前源码核对。

## 判断与证据

R04有实质工程进展：R03的非法free有真实栈、Eigen配置差异证据，以及CMake同构构建后的完整raw
prepare成功。它修复了阻止程序运行的软件问题。它没有新增完整输入上的收敛、评分或修正收益证据。

R04证据根为 `doc/ie_sprint/evidence/t10_a19_r04_crash_score_compare_20260910T072815Z/`：

- `diagnostic/original/gdb.stdout:5-18` 为SIGSEGV及 `free(0x41)`、`NoiseModelFactor::error`、
  `GraphLinearizationContentHash` 栈；`diagnostic/abi/eigen_macros.txt`、`layout_results.txt` 记录
  16/32-byte及SSE/AVX差异。两个probe的Pose3/PaperPosePriorFactor的sizeof/alignof仍相同，不能误写
  成这两个类型的大小已经不同；证据支持跨翻译单元Eigen分配/释放及对齐配置兼容性修复。
- `engineering/prepare_postfix_output/prepare_status.json`：371 factors、123 values，初始图/Values
  内容身份与历史一致；Stage1及conditional optimizer调用数0。
- `attempts/attempt1/pilot/output/stage1/status.json:1`：`INVALID_INPUT`，
  `A18_INVALID_DEVELOPMENT_POLICY_OR_IDENTITY`，0 outer/call/trial。这里是已知的输入身份拒绝，
  不再是R03的执行阶段未知，也不是数值不收敛。
- 当前 `tools/paper/a19_r04_pipeline.cpp:21,255-260` 把
  `A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY` 写入请求；
  `src/nlos_discovery.cpp:737-753` 只允许 `A18_STAGE1_DIAGNOSTIC_ONLY` 和
  `A19_STAGE1_STAGE2_SCORE_DIAGNOSTIC_ONLY`。不匹配由源码直接证实，无需另立求解器诊断。
- `tools/paper/a19_r04_pipeline.cpp:228-245` 的prepare-only在构造 `DiscoveryContext` 和
  `DevelopmentStage1Request` 之前就返回。因此真实prepare通过并没有验证生产者是否接受真实请求。
  这是上一轮工程门的明确缺口；不应继续用完整输入ticket充当接口握手测试。
- Stage2入口在 `src/nlos_refit.cpp:470-483` 校验policy/role/implementation prefix/
  support.solver_config_hash/callback/handoff关系，未见同一R04 schema白名单问题。
  该静态检查不是完整Stage2或下游链路已经运行通过的保证。

结论：科学验证仍未越过R02的Stage2 outer15。R03/R04没有证明受限handoff在完整输入有效，也没有否定它。
目前不需要修改研究方向、eta定义、求解器参数或最终四项AND。R04按既定停止规则不重试是合规的；下一轮
应把可确定的身份检查放到ticket之前，并在一个任务内完成修复、真实接口回归和完整运行。

## 唯一最小方案

保留R04已经采用的数据格式，明确支持该精确development schema，并让runner与producer共用其定义及
校验规则。R05是任务/运行编号，不自动意味着数据schema必须再改成R05。新代码、库、policy implementation
内容hash、run ID、ticket和输出目录仍必须如实更新；不得把新实现伪装成旧身份。

把同一真实请求的入口身份校验接入prepare/preflight。不要复制另一份白名单给预检、不要任意接受
`A19_*`前缀、不要关闭身份检查，也不要靠把请求改回旧schema绕过边界。历史正式reader继续拒绝
development产物。只做当前握手的共享定义/校验和必要的直接上下游回归，不开发通用schema注册平台。

## 交给 Codex 的下一轮 prompt

执行 **T10-A19-R05：修复development身份握手并完成一次完整评分与收益比较**。
工作目录 `/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-ie`。

先读STATUS、冻结论文结构/roadmap、完整METHOD/EXPERIMENT_CONTRACT、R04协议、本文件和R04封存状态。
保护dirty worktree及所有R04结果。登记新R05任务协议和新运行身份；本prompt授权以下限定接口修复、
工程验证及通过后的唯一完整运行，不停在计划或工程门通过，也不重复请求同一范围授权。

1. **只修确定的身份缺陷。** 统一runner实际发送与Stage1实际接受的精确development schema。
   若payload语义不变，保留 `A19_R04_STAGE1_STAGE2_SCORE_DEVELOPMENT_ONLY`，不因任务号R05再次
   引入新schema。使用共享常量/最小公共契约，并让生产入口与预检调用同一校验逻辑。
   role、provider、Stage1原policy、implementation identity、context.solver_config_hash、callback、
   conditional policy等原检查全部保留；正式路径/default-off/consumable=false边界不变。
   新实现hash、库hash、run ID/ticket及输出路径更新，历史schema与结果身份不改写。

2. **先验证精确请求，后签ticket。** 用同一CMake构建runner、同一CLI配置、同一请求构造函数、同一
   当前库身份执行prepare+contract-check；完整raw初始化建图后，必须实际校验生产运行将提交的
   DevelopmentStage1Request，而非提前return或校验手造替身。零conditional optimizer调用要明确记录。
   保留修复前准确拒绝R04请求、修复后该请求被接受的证据；不能仅断言新旧字符串相等。
   针对未知schema、错误role/provider、identity/context不一致、错误policy、缺callback，以及正式reader
   拒绝development做必要反例；实际测试数>0。

3. **检查直接上下游握手，勿让下一关再靠科学运行发现。** 用真实小型工程输入及同一请求/身份构造路径
   通过Stage1 producer→实际partition/context→Stage2→score→decision/final→独立evaluator，复用既有
   C++/GTSAM模块。记录每边producer/consumer接受的schema、provider、policy、solver hash及产物parent
   关系。完整raw的prepare不能伪造尚未产生的partition；小型工程产物不能升格为科学结果。
   只补受本次身份变更影响的检查；R04 ABI修复、CMake同构构建、真实final消费、独立预算检查在身份和
   相关实现未变化时可复用。若实现/库变化使旧检查不再覆盖当前构建，则重跑相应必要检查。

4. **工程门通过后直接消费新的唯一fresh ticket。** 复用冻结P1 step seed10101完整raw和原初始化，
   禁止从outer15或任何checkpoint续跑冒充完整输入结果。Stage1 outer<=500、conditional<=50、
   Stage2 outer<=200，自动Stage1→Stage2→score进程树<=900s。
   不改R03 handoff guard、证书333bit、原生方向/retract、lambda/阈值、IMU模型、分段/评分或最终四项AND。
   首次实际运行失败仍封存并停止依赖分支，不临场改身份/阈值后重跑；未知阶段不推断为零调用。

5. **满足条件后继续完成真实比较。** Stage2最终联合四项AND全通过且有eligible分数后，冻结所有
   decision-time产物，执行原五策略suppress_all、structured_debias、fit_only(gamma<=1)、
   s_fit(再加s<=0.10m)、full_gate(再加eta>=0.10)。保持每final独立进程树900s、共4500s、
   自动+final总5400s及至多原合同一次fallback。共享correctness故障停止受影响策略；局部失败保留行。
   等价决策复用须满足原全部等价条件。零eligible按原协议报告并停止final，不能虚造有效分数。
   保持原始range和live c_s，禁止pseudo-range重复计量；trajectory/bias/residual/covariance来自各自
   同一final graph/Values。

6. **真正交付收益或负结果。** 所有可运行decisions/final冻结后，独立evaluator才读取evaluation-only
   truth；estimator从不读取GT/oracle。明确给出实际完整输入final→evaluator命令和产物，不能用已通过
   的工程fixture代替。报告完整记录/[3,6]s raw-frame ATE RMSE/P95/matches、相对suppress_all差值、
   decision/final bias-field error、accepted bias RMSE、bad-correction、good rejection、coverage、
   retained fraction、failure/fallback和成本，epsilon_bad=0.20m保持。零接受risk=UNDEFINED；
   同决策不宣称eta增量有效。不可得项写UNAVAILABLE，不填零。

在工程阶段解决已知接口问题；不要新增求解器/高精度研究、schema平台、threshold sweep、场景或held-out。
到完整配对结果表或首次预登记失败为止。交付最小变更、共享契约与精确握手证据、实际命令/退出码/当前
runner和加载库hash、阶段计数、完整分数/五策略/评价表或失败证据。更新STATUS、CLAIM_EVIDENCE，
C1–C3不因单输入development结果升级。人话回答是否越过outer15、最终是否收敛、修正好坏、gate差异；
不能再用“11/11或更多预检通过”代替完整任务结果。
