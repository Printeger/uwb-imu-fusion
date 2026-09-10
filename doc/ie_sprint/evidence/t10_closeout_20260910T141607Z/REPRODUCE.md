# T10 retained evidence replay

The original raw/config and final artifacts are retained. No optimizer replay was performed after the reboot.

From the repository root:

```sh
python3 tools/paper/t10_closeout_report.py
python3 tools/paper/verify_t10_closeout.py
python3 test/test_t10_closeout.py
```

These only recompute/check summaries and retained numerical results. Per-case `EVALUATION_RUN.json` records the exact independent evaluator command; `COMMAND.json` records the original estimator invocation. The current freeze manifests explicitly describe post-crash reconstruction and user-authorized retention, not the lost original new-run freeze lists.

Rebuild when an estimator replay is explicitly requested:

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target a19_r08_pipeline test_nlos_refit test_nlos_inference -j2
```

Existing run directories must never be reused for optimization. `t10_closeout.prepare(new_isolated_root, "reproduction", seed, "step2", condition)` regenerates B/C from the retained A inputs and binds the rebuilt binary/config/library identity; `run(case)` executes the recorded producer. This is reproducibility guidance, not authorization for another scientific sweep after T10 freeze. Do not archive rebuildable binaries or large per-trial matrix dumps. No new dependencies or workspace migration were used.
