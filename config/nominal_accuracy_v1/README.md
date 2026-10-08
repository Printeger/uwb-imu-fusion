# 离散无 FDE 精度实验配置

这些 YAML 是 `tools/run_nominal_accuracy_study.py --profile` 的覆盖配置，**不是可以直接传给 ROS 的完整入口配置**。工具合并原始缓存对应的 `effective_nominal_config.yaml`，生成可用于 direct/ROS 的完整 YAML。所有文件均为实验配置；仅在全部兼容性、精度和耗时门槛通过后才可推荐。

- `sensor_selected_experimental.yaml`：因果初始化、IMU 初值、5 轮局部 LM、5 s 离散窗口、重线性化阈值 0.01。
- `device_statistics_experimental.yaml`：相同优化配置，额外为全部 Walk 使用同一设备参数。噪声密度由启动静止段样本统计得到，仍为暂定值；不构成 Allan 标定、随机游走标定或 FDE 噪声 overbound。

完整入口 YAML 可追加如下可选节，旧配置不含此节也可以加载：

```yaml
estimation_tuning:
  version: 1
  causal_bootstrap: true
  bootstrap_prefer_below_anchors: false
  nominal_initial_guess: imu  # cv 保留既有初值；imu 仅 nominal-only 使用
  nominal_lm_iterations: 5   # 0..5；非零必须使用 imu 初值
  nominal_lag_s: 5.0         # 0 保留原 epoch 窗口
  nominal_robust_experimental: false
  # 可选：按 [ba_x, ba_y, ba_z, bg_x, bg_y, bg_z] 指定有限正 sigma
  # bias_integration_sigmas: [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
```

未指定 `bias_integration_sigmas` 时，显式使用传给 `initialize` 的六个偏置先验 sigma 的平方作为 `biasAccOmegaInt`。它不是偏置随机游走参数，也不额外增加随机游走因子。零值会被拒绝。`version` 和未知字段严格校验。

FDE profile 非 `off` 时，IMU 初值、LM、秒制窗口、因果启动和鲁棒实验选项会被拒绝；共用的状态查询与预积分协方差修复仍然生效。`fde=off` 的完整性事务也不采用 nominal 的 LM 初值。`nominal_robust_experimental` 只接受独立逐距离协方差，并禁止用于完整性冻结、恢复候选或统计输入。

两个入口在启用 `causal_bootstrap` 时使用同一初始化器。启动窗口是首个 IMU 时间到其后 2 s；后续 UWB 必须严格晚于窗口末端，初始化数据不会再次进入主线。无该选项的旧配置保留旧入口的初始化方式。

`--export-historical` 是 direct runner 的可选诊断，会额外写出 `trajectory_historical_window.tum` 和 `historical_information_cutoff.csv`。精度主表始终读取 `trajectory.tum` 的在线当前状态，历史平滑不会替代验收输出。
