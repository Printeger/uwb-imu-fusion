# P0-07 sixth independent reviewer — FAILED

结论：**FAILED，不是 BLOCKED**。不得更新总表、提交或创建
`golden-p0-07-corrected-exhaustive`，P1 必须继续关闭。

## 主要阻断

### 1. Recipe 未与实际 resolved config 逐值绑定

`test_p0_07_corrected_exhaustive.cpp:692-714` 只解析并范围检查另抄的
`p_fa`、总风险等常量，没有断言其等于：

- `config.detector.p_fa_per_test`
- `config.risk_v2.p_hmi_total`
- `config.fde.max_exclusion_cardinality`
- 其余 action-generation、selection、fault-contract 配置

`independentDenseActionReferences()` 甚至同时使用 config 的 `p_fa` 和
recipe 的 total risk，二者可漂移而测试仍通过。

### 2. 完整 raw/row-owner oracle 仍非完全 recipe 驱动

`completeRawReference()` 在约 `998-1113`：

- 从 `tx.recoverable_history`、`record.groups`、`tx.frozen_slots`、
  `group.raw_covariance`、`group.source_measurements` 获取
  inventory/owner/map；
- `boundary_slots`、`*1000+1/2`、epoch 3–12/13 等规则仍硬编码；
- manifest 的 `slot_rules`、`group_rules`、`row_identity` 并未真正解析后
  驱动完整 ledger；
- `window_first_epoch` 也未解析，改由 transaction/config 推出。

共同 frozen nonlinear factor/value 可以复用，但完整 owner ledger、block
inventory、covariance/noise placement 必须先从 frozen raw + recipe 构造，
生产对象只能作为 comparison target。

### 3. DenseActionReference 没有独立计算 PL/risk

`843-981` 独立重算了候选 rank、DOF、statistic、post test 和 profile
finiteness，但 `963-975` 直接硬置：

- `model_error_validated = true`
- `pl_valid = false`
- `hpl_m/vpl_m = inf`

没有逐 action、逐 hypothesis 计算 candidate Gram、protected response、
nullspace、slope、基础 PL 数值/类别，也没有完整 risk-term ledger。N01
unknown 可以使最终 formal eligibility 失效，但不能代替数学 PL/reference
的独立推导。

### 4. Recipe 中多项声明实际未执行

mode reference 在约 `1580-1593` 仍硬编码“三个 family”和 family index；
action recipe 的 keep-all、occurrence、union、max exclusion、identity
fields、selection ordering没有完整解析/校验。`observed_non_authoritative`
未被当作 expected 使用，这部分合格。

### 5. §2.5 冻结证据互相冲突

顶层证据仍记录旧 replay：

- alarm epoch 10
- 240 modes
- 6 actions

`replay-bundle/` 则记录当前：

- alarm epoch 23
- 504 modes
- 7 actions

两套文件和 hash manifest 各自通过，但 `frozen-output.json`、
`alarm-actions.tsv`、`output-hashes.sha256` 的同名冻结输出存在冲突，不能
作为唯一可审计基线。

## 其余审计结果

- MissingProvenance 真实 generator/action/dense/FDE 链产生非零 missing
  actions并拒绝，PASS。
- N01 所有检查到的公开出口 fail-closed，protected output 关闭，PASS。
- authority existing-bundle 全量复验、corrupt identity、同/异内容并发、
  old reader、原子 pointer、fsync/cleanup 矩阵，PASS。
- pipeline exception、`NotAttempted`、旧配置兼容、无环境绕过、无风险/
  阈值/coverage 改动，PASS。
- clean Release build：PASS，5m20.0s。
- tests 构建：PASS，3m37.5s。
- 关键 7 项 CTest：7/7 PASS，322.99s。
- 完整 CTest：32/32 PASS，334.41s。
- fresh runner 两次及 expanded authority matrix：PASS。
- fresh ASan+UBSan：7/7 PASS，159.378s。
- fresh LSan N01/ABI subset：2/2 PASS。
- golden-header/current-DSO ABI：80/8、2544/8、offsets、by-value
  load/destruct及三种 FDE exports PASS。
- hash/JSON/diff/patch apply-byte-reverse：PASS。
- 一次初始 sanitizer 编译因宏引号失败；一次独立 sanitized ABI canary
  出现无栈 `DEADLYSIGNAL` 循环并已终止。两次均不能计 PASS，但后续正确
  sanitizer suite 与非 sanitizer golden-header ABI 均通过。

最小修复应集中在上述五点，不得改阈值、风险预算、噪声数值、coverage
或分母。
