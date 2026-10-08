# Accuracy/FDE research overlays

These are overlays for `tools/run_nominal_accuracy_study.py`, not standalone
ROS configurations. The runner writes the complete effective configuration.
`gaussian_nominal` with `--stage covariance` is the recommended research
overlay: original bootstrap/CV/epoch lag and explicit baseline integration
sigmas. Old configs retain their original behavior.
No overlay enables formal integrity qualification.

Use `--stage causal` with `bias_fixed_experimental`, `time_experimental`,
`motion_experimental`, `seed_experimental`, or `branch_experimental` to isolate
bootstrap mechanisms. Every overlay freezes bias integration sigmas at the
actual B values; initializing with a posterior prior does not silently change
the subsequent integration model. White noise and bias random walk remain
the input cache's original configured parameters.

`bootstrap_exact_uwb_times` adds actual discrete range timestamps to the 0.2s
nodes and integrates each IMU subinterval exactly once. Piecewise-linear
boundary interpolation uses only the already available two-second bootstrap
window. This does not change the production IMU integration convention.

`bootstrap_uwb_motion_check` requires four supported range-slope directions and
nondegenerate local geometry before confirming a speed upper bound <=0.15m/s.
It is an experimental model-selection rule with configured independent range
uncertainty, not a new independent zero-velocity sensor or certified probability
bound. Insufficient evidence produces MOTION_UNRESOLVED and a dynamic solve.

`bootstrap_seed_only` replaces the range-derived translation prior with a weak
sigma=1e6m regularizer at the configured origin; the seed is still a solver
initial value. Tilt/bias/velocity physical modeling remains experimental.
`bootstrap_enforce_below_anchors` requires the cache's declared below-anchor
physical premise. It chooses the lowest-objective admissible seed and rejects
a final solution above the anchor-height median; it does not infer that premise
from GT. D2's mirror regression is fixed; the result is not better than B.

`relinearize_experimental` uses `--stage covariance` and changes only the
relinearization threshold to 0.01. D1 gain was small, so it is not recommended.
`robust_nominal_experiment` also uses `--stage covariance`: only existing scalar
Huber is enabled, with B's original bootstrap/CV/epoch window. It remains
UNPROTECTED and cannot enter Gaussian FDE/PL. It improves Walk1–3 but
regresses STAR-Loc, so it is not the general recommendation. Confirmation numbers
are in `docs/accuracy_fde_progress.md`.

Example (writes a fresh result directory):

```bash
python3 tools/run_nominal_accuracy_study.py run --stage causal --all \
  --sequence 2025-10-24-15-48-55_vicon_lidar_uwb_imu_obstacle \
  --profile config/accuracy_fde_v2/branch_experimental.yaml \
  --output results/my_branch_test
python3 tools/run_nominal_accuracy_study.py evaluate --stage causal \
  --output results/my_branch_test
```

FDE same-model reference uses `UWB_IMU_PL_EXHAUSTIVE_BOUNDARY_RANK=1`,
`UWB_IMU_PL_EXHAUSTIVE_CARRIER_U=1` and
`UWB_IMU_PL_EXHAUSTIVE_MODE_RESPONSES=1`. Default skips only redundant extractor
rank and unused carrier U. Response caching is experimental and must be enabled
explicitly with `UWB_IMU_PL_MODE_RESPONSE_CACHE=1`; exhaustive overrides it.
No path changes fault scope, thresholds or budgets. Exact PL tie/alert boundary
equivalence of cached products remains unproven. Optional in-scope mode-cache audits use
`UWB_IMU_PL_MODE_CACHE_AUDIT_ATTEMPTS=10,12,32` plus
`UWB_IMU_PL_MODE_CACHE_AUDIT_DIR=/absolute/new/directory`; the parent core timer
includes diagnostic work, and must not be used as production speed evidence.
Always bind LD_LIBRARY_PATH to the actual executable's companion DSO.

Carrier-only diagnostics: set `UWB_IMU_PL_CARRIER_AUDIT_DIR` to capture one
production matrix, then run `test_history_fault_summary` with
`UWB_IMU_PL_CARRIER_BENCHMARK_FILE=<dir>/carrier.bin` and filter
`HistoryFaultSummary.CapturedProductionCarrierMicrobenchmark`. Diagnostic parent
timers and global numerical counters include extra computations; do not use
them as production performance evidence.

Reproduce the seven-sequence confirmation by passing each predeclared sequence
with `--all --sequence NAME --stage covariance --profile` and a fresh `--output`.
Use `gaussian_nominal.yaml` and `robust_nominal_experiment.yaml` separately.
Evaluation is a separate `evaluate` command; GT never selects the run settings.
The predeclared names and measured results are in the progress document and
`docs/benchmark/accuracy_fde_confirmation_20261009.csv`.
