# ICRA Experiment Roadmap v6
## SFUISE/ISAS controlled-corruption + simulation, with downstream SFUISE validation

**Project:** UWB-IMU-IE  
**Scope:** Only SFUISE/ISAS-Walk real sensor data + simulation. No MILUV, STAR-loc, self-collected data, or TDoA in this round.  
**Primary scientific question:** Can offline persistent-range-bias inference recover useful UWB measurements better than rejecting them, and can the recovered ranges improve both the native batch FGO and a stronger downstream estimator (SFUISE)?  
**Evidence boundary:** Real UWB/IMU motion and noise with controlled positive range-bias injection; simulation for geometry/redundancy/motion mechanisms. Do **not** claim validation on naturally occurring physical NLOS.

---

# 0. Non-negotiable rules

1. **Do not optimize the paper around clean ATE superiority over SFUISE.** SFUISE is currently the stronger nominal trajectory backbone.
2. **Before NLOS experiments, remove avoidable unfairness in the native FGO:** frame/reference-point audit, official static ToA calibration where valid, and UWB measurement-density audit.
3. **Do not add B-spline smoothing merely to improve ATE.** A continuous-time reformulation is out of scope unless later evidence makes it unavoidable.
4. **Once the formal NLOS experiments start, freeze the estimator, detector, evaluator, injection manifest, and method configs.**
5. **Any change to the native FGO that changes residual statistics requires detector recalibration on clean DEV data before formal injected experiments.**
6. **GT/injection truth is evaluator-only.** It must never enter the estimator, detector, range recovery, or SFUISE.
7. **Every planned run must remain visible** as `completed`, `no_candidates`, `fallback`, `failed`, or `invalid_input`.
8. **Every paper figure must have a machine-readable source file** and a vector PDF; every paper table must have CSV + LaTeX output.
9. **No result-dependent case selection.** Representative cases and sweep definitions are frozen before formal runs.
10. **Use only ToA/TWR/absolute ranges. No TDoA.**

---

# 1. Final paper experiment story

The final experimental section should answer four questions.

### Q1 — Nominal accuracy and fairness
How much of the clean-data gap to SFUISE comes from frame/reference-point handling, static UWB calibration, and range utilization rather than the NLOS method?

### Q2 — Controlled persistent faults
Under controlled persistent positive range bias on real ISAS trajectories, how robust are Base FGO, Robust FGO, SFUISE, Reject, and Recover?

### Q3 — Recovery versus rejection
With the **same detected support**, does range recovery outperform discarding those measurements? Does this remain true when the recovered/rejected measurements are fed into the stronger SFUISE trajectory estimator?

### Q4 — Operating region
Under what anchor redundancy, geometry, motion excitation, and fault conditions does recovery help, become neutral, or become unsafe/unidentifiable?

The real-data experiments answer Q1–Q3. Simulation mainly answers Q4.

---

# 2. Required repository artifact layout

Codex should create or preserve the following structure.

```text
experiments/icra2027/
├── README.md
├── VERSION.yaml
├── manifests/
│   ├── dataset_manifest.yaml
│   ├── injection_manifest_canonical.yaml
│   ├── injection_manifest_sweeps.yaml
│   └── simulation_manifest.yaml
├── configs/
│   ├── backbone/
│   ├── detector/
│   ├── methods/
│   └── sfuse/
├── audits/
│   ├── frame_reference_audit.md
│   ├── range_calibration_audit.csv
│   ├── range_density_audit.csv
│   └── backbone_parity_report.md
├── runs/
│   └── <experiment_id>/<case_id>/<method_id>/
├── metrics/
│   ├── run_status.csv
│   ├── trajectory_metrics.csv
│   ├── range_metrics.csv
│   ├── detection_metrics.csv
│   ├── segment_metrics.csv
│   └── pairwise_metrics.csv
├── exports/
│   ├── corrected_ranges/
│   ├── rejected_ranges/
│   └── sfuse_inputs/
├── tables/
│   ├── csv/
│   └── latex/
├── figures/
│   ├── data/
│   ├── pdf/
│   └── png/
├── logs/
└── paper_assets/
    ├── TABLE_I_clean_backbone.tex
    ├── TABLE_II_persistent_faults.tex
    ├── TABLE_III_ablation.tex
    ├── FIG_2_representative_case.pdf
    ├── FIG_3_fault_sweeps.pdf
    └── FIG_4_operating_region.pdf
```

**Rule:** `paper_assets/` contains only files generated from frozen experiment outputs. Never hand-edit numerical values there.

---

# 3. Unified method IDs

Use stable IDs in all CSV files and plots.

## Native FGO backbone
- `FGO_BASE`
- `FGO_ROBUST`
- `FGO_REJECT`
- `FGO_RECOVER_FULL`
- `FGO_RECOVER_PARTIAL` — ablation only
- `FGO_STRUCTURED` — ablation only

## SFUISE downstream methods
- `SF_NATIVE` — official ToA configuration on the same input recording
- `SF_REJECT` — same frozen support from our detector; affected observations removed before SFUISE
- `SF_RECOVER` — same frozen support; accepted observations replaced by our corrected ranges before SFUISE

**Important:** `SF_NATIVE`, `SF_REJECT`, and `SF_RECOVER` use the same SFUISE configuration. Do not disable or retune SFUISE's native residual rejection separately for any variant.

---

# 4. Unified metric contract

Every formal run must export the following.

## 4.1 Trajectory metrics

```text
experiment_id
case_id
method_id
sequence
status
failure_reason
n_gt_samples
evaluation_start
evaluation_end
alignment_mode
ATE_RMSE
ATE_median
ATE_P95
RPE_1s
fault_window_RMSE
post_fault_RMSE
runtime_total_s
```

Compute metrics on a **common GT sample set** for compared methods when possible.

Save both:
- raw-frame metrics;
- scale-fixed SE(3)-aligned metrics.

The paper may only choose the primary metric after the frame/reference-point audit is resolved and documented.

## 4.2 Range metrics

Per observation:

```text
timestamp
obs_id
sequence
anchor_id
raw_range
static_calibration
injected_bias
detector_candidate
segment_id
estimated_bias
recovery_accepted
applied_correction
corrected_range
```

For injected experiments additionally compute against the clean parent measurement:

```text
injected_component_error
```

If a trustworthy geometric GT range evaluator is available after frame audit, also save:

```text
gt_range
raw_range_error
corrected_range_error
```

## 4.3 Detection metrics

```text
case_id
anchor_id
precision
recall
F1
event_detected
forward_alarm_delay_s
offline_onset_error_s
offline_offset_error_s
false_positive_duration_s
```

## 4.4 Pairwise metrics

Always materialize paired comparisons instead of recomputing them by hand:

```text
case_id
backbone
delta_recover_minus_reject_ATE
delta_recover_minus_reject_fault_RMSE
delta_corrupted_minus_clean_ATE
```

Negative `delta_recover_minus_reject_*` means recovery is better.

---

# 5. Execution order

Do **not** execute experiments in paper-writing order. Execute them in dependency/risk order:

```text
R0  Freeze + provenance
 ↓
R1  Frame/reference-point audit
 ↓
R2  Static calibration + UWB density parity audit
 ↓
R3  Freeze improved native backbone + recalibrate detector
 ↓
R4  Build corrected-range export + SFUISE Reject/Recover adapter
 ↓
R5  Clean/no-harm formal runs
 ↓
R6  Canonical 15-case persistent-bias experiment  ← first major Go/No-Go
 ↓
R7  Magnitude/duration/model-mismatch sweeps
 ↓
R8  Detector + recovery backend ablations
 ↓
R9  4/5/8-anchor simulation operating-region experiment
 ↓
R10 Optional multi-fault/model-mismatch simulation
 ↓
R11 Freeze paper tables/figures + reproducibility bundle
```

---

# R0 — Freeze experiment provenance

## Purpose
Create a reproducible starting point before changing the backbone.

## Codex tasks
- Record current git commit, branch, compiler/build mode, dependencies, CPU/thread settings.
- Snapshot current estimator/detector/recovery configuration.
- Record current SFUISE commit/config.
- Create the experiment directory structure.
- Add a single script that prints the experiment fingerprint.
- Do not change algorithm behavior.

## Required outputs
```text
experiments/icra2027/VERSION.yaml
experiments/icra2027/README.md
experiments/icra2027/configs/...
```

## Acceptance gate
A clean checkout can reproduce the current Walk1 clean run and the fingerprint is embedded in its run metadata.

## Codex Prompt R0

```text
You are preparing the formal ICRA experiment infrastructure for the current
feature/uwb-imu-fusion-ie-postprocessing branch.

Task R0 is provenance only. Do not change estimator mathematics, CUSUM logic,
recovery logic, dataset values, or SFUISE.

Create experiments/icra2027/ with:
- VERSION.yaml
- README.md
- manifests/
- configs/
- audits/
- runs/
- metrics/
- exports/
- tables/csv/
- tables/latex/
- figures/data/
- figures/pdf/
- figures/png/
- logs/
- paper_assets/

VERSION.yaml must record the current repository commit, branch, build type,
dependencies if available, evaluator version, current detector/recovery configs,
and the SFUISE commit/config used by the existing baseline.

Add a command/script that prints a deterministic experiment fingerprint and
stores it in each run.

Smoke-test only the existing ISAS Walk1 clean pipeline.
Do not modify scientific behavior.

Finish with:
1. changed files,
2. exact reproduction command,
3. fingerprint,
4. any missing provenance information.
```

---

# R1 — Frame, reference-point, and evaluator audit

## Why this comes first
Current development results show large pre-alignment errors (~2.6–2.7 m) and much smaller scale-fixed SE(3)-aligned errors. Before formal accuracy claims, determine whether the mismatch comes from:
- anchor-world vs GT-world frame transform;
- tracker/body/IMU/tag reference point;
- lever arm;
- timestamp association;
- unit or axis convention.

## Codex tasks
1. Trace the coordinate frames used by anchor positions, GT/VIVE, IMU/body, UWB tag, SFUISE output, and native FGO output.
2. Trace every transform/extrinsic and its source.
3. Recompute Walk1/2/3 metrics with raw frame, scale-fixed SE(3) alignment, and common GT samples.
4. Do not estimate a transform from TEST GT and feed it back into the estimator.
5. Do not “fix” the error by silently changing alignment.

## Required outputs
```text
audits/frame_reference_audit.md
metrics/frame_audit_metrics.csv
figures/data/frame_audit_walk*.csv
```

## Acceptance gate
Either the frame/reference-point mapping is resolved and raw-frame evaluation is valid, **or** the unresolved issue is precisely documented and formal results are explicitly limited to scale-fixed SE(3)-aligned trajectory error.

## Codex Prompt R1

```text
Perform an evaluator/frame/reference-point audit on ISAS Walk1/2/3.

Do not modify the NLOS detector, recovery algorithm, solver objective, or
SFUISE.

Trace and document:
- anchor coordinate frame,
- VIVE/GT frame,
- FGO state reference point,
- IMU/body frame,
- UWB tag reference point,
- SFUISE output reference point,
- all lever arms/extrinsics,
- timestamp association and interpolation,
- axis and unit conventions.

For Base FGO and SFUISE, recompute Walk1/2/3 using exactly the same GT samples
and save both:
1. raw-frame trajectory error,
2. scale-fixed SE(3)-aligned trajectory error.

Create:
- experiments/icra2027/audits/frame_reference_audit.md
- experiments/icra2027/metrics/frame_audit_metrics.csv

The report must explain the source of the current ~2.6–2.7 m pre-alignment
error as far as the repository/data permit. Do not invent missing transforms.
Do not fit a GT-derived transform and feed it into the estimator.
List unresolved assumptions explicitly.
```

---

# R2 — Backbone parity audit: static calibration and UWB density

## Purpose
Determine how much of the clean ATE gap to SFUISE is avoidable unfairness.

## Variants
Run on Walk1/2/3 clean data:
- `B0_CURRENT`: current Base FGO.
- `B1_CAL`: B0 + official static per-anchor ToA calibration, only if independently valid.
- `B2_CAL_DENSE`: B1 + as many valid UWB ranges as the current discrete graph can legitimately consume.
- `B3_HIGHER_RATE`: optional only if B2 still shows a clear state-sampling limitation. **Do not add B-splines.**

## Required outputs
```text
audits/range_calibration_audit.csv
audits/range_density_audit.csv
metrics/backbone_parity.csv
tables/csv/backbone_parity.csv
figures/data/backbone_parity.csv
audits/backbone_parity_report.md
```

`backbone_parity.csv` must include:
```text
sequence
variant
raw_range_count
used_range_count
state_count
ATE_RMSE_raw
ATE_RMSE_aligned
ATE_P95
RPE_1s
runtime_s
```

## Decision rule
Freeze the best **scientifically justified** native discrete backbone before looking at injected-fault outcomes. If B2 remains worse than SFUISE, accept SFUISE as the stronger nominal backbone and proceed to downstream range-recovery validation.

## Codex Prompt R2

```text
Perform the clean-data backbone parity audit on ISAS Walk1/2/3.

Do not touch NLOS detector thresholds or recovery policies.

Part A — static ToA calibration:
Audit SFUISE's official per-anchor offsets:
[-0.0700, 0.1539, -0.0751, 0.1409, -0.0247].
Verify anchor ordering, sign convention, units, where the offsets are applied,
and whether they are independent calibration rather than test-GT-derived values.
Only if scientifically valid, add an equivalent static calibration option to
the native FGO.

Part B — UWB utilization:
For each Walk record:
- raw UWB observations,
- valid observations,
- currently planned observations,
- observations actually inserted into the final graph,
- per-anchor counts,
- exact drop/rejection reasons.

Create clean-data variants:
B0_CURRENT
B1_CAL
B2_CAL_DENSE

B2 should use as many legitimate UWB ranges as the existing discrete FGO can
consume without changing the NLOS method or inventing interpolation.

Only if diagnostics show that state sampling itself is the remaining bottleneck,
add B3_HIGHER_RATE within the existing discrete-state formulation. Do not add
B-splines or post-hoc trajectory smoothing.

Run all variants on Walk1/2/3 clean data with the evaluator from R1.

Save:
- audits/range_calibration_audit.csv
- audits/range_density_audit.csv
- metrics/backbone_parity.csv
- audits/backbone_parity_report.md

The report must separate gains caused by calibration, range density, and state
rate. Do not tune anything using injected-NLOS results.
```

---

# R3 — Freeze formal native backbone and recalibrate detector

## Purpose
Once the native backbone is changed, residual statistics change. Formal NLOS experiments need a new frozen detector calibration.

## Required outputs
```text
configs/backbone/FROZEN_BACKBONE.yaml
configs/detector/FROZEN_DETECTOR.yaml
metrics/clean_detector_calibration.csv
audits/formal_freeze_report.md
```

## Acceptance gate
- Walk1/2/3 clean runs complete.
- Candidate/false-alarm behavior is documented.
- No formal injected case has been used to tune detector/recovery parameters.

## Codex Prompt R3

```text
Freeze the formal native FGO backbone using only the clean-data evidence from
R1/R2.

Do not choose the backbone using any injected-fault result.

Because the backbone/calibration/range density may have changed, regenerate
the clean conditional-innovation signals on Walk1/2/3 and audit the current
CUSUM parameters.

If detector calibration must change, use clean DEV data only. Do not look at
formal injected test outcomes.

Freeze and hash:
- backbone config,
- static range calibration,
- range scheduling/density,
- detector parameters,
- recovery parameters,
- evaluator settings.

Write:
- configs/backbone/FROZEN_BACKBONE.yaml
- configs/detector/FROZEN_DETECTOR.yaml
- metrics/clean_detector_calibration.csv
- audits/formal_freeze_report.md

The report must state exactly which parameters are frozen before formal
persistent-bias experiments begin.
```

---

# R4 — Build range-treatment export and SFUISE downstream adapter

## Purpose
Make the paper test the **range recovery layer**, not whether the native FGO is a better trajectory estimator than SFUISE.

## Required export
For every observation:
```text
timestamp
obs_id
anchor_id
raw_range
candidate
segment_id
estimated_bias
sigma_bias
lcb
recovery_accepted
applied_correction
corrected_range
```

## SFUISE variants
- `SF_REJECT`: remove observations specified by the frozen Reject policy.
- `SF_RECOVER`: replace accepted observations with corrected ranges.

All SFUISE variants use the same official ToA configuration.

## Required outputs
```text
exports/corrected_ranges/<case>.csv
exports/rejected_ranges/<case>.csv
exports/sfuse_inputs/<case>/...
audits/sfuse_adapter_report.md
```

## Acceptance gate
On clean `NO_CANDIDATES` sequences, `SF_REJECT` and `SF_RECOVER` must match `SF_NATIVE` within deterministic solver tolerance.

## Codex Prompt R4

```text
Implement the downstream range-treatment export and SFUISE adapter.

Do not change SFUISE mathematics or its official ToA configuration.

From our frozen offline NLOS inference, export one row per UWB observation:
timestamp, obs_id, anchor_id, raw_range, candidate, segment_id,
estimated_bias, sigma_bias, lcb, recovery_accepted, applied_correction,
corrected_range.

Build two derived SFUISE inputs:
1. SF_REJECT: remove the observations specified by the frozen Reject policy.
2. SF_RECOVER: replace accepted observations by corrected_range according to
   the frozen recovery policy.

The native FGO Reject/Recover and SFUISE Reject/Recover experiments must use
the same frozen support/treatment semantics.

Add integrity tests proving:
- timestamps and IMU are unchanged,
- non-treated UWB measurements are unchanged,
- GT and injection truth never enter SFUISE input,
- SF_NATIVE, SF_REJECT, SF_RECOVER use the same SFUISE config.

On clean NO_CANDIDATES Walk1/2/3, verify SF_REJECT and SF_RECOVER reproduce
SF_NATIVE within deterministic tolerance.

Save an adapter audit report and machine-readable exported range files.
```

---

# R5 — Formal E1: clean/no-harm

## Data
Walk1/2/3 original.

## Methods
- `FGO_BASE`
- `FGO_ROBUST`
- `FGO_REJECT`
- `FGO_RECOVER_FULL`
- `SF_NATIVE`

## Paper Table I
Recommended columns:
```text
Sequence | Used UWB | Frozen FGO ATE | Robust FGO ATE | SFUISE ATE | Candidates
```

## Files
```text
metrics/E1_clean_metrics.csv
tables/csv/TABLE_I_clean_backbone.csv
tables/latex/TABLE_I_clean_backbone.tex
paper_assets/TABLE_I_clean_backbone.tex
```

## Codex Prompt R5

```text
Run formal E1 clean/no-harm experiments using the frozen R3 configuration.

Data:
Walk1, Walk2, Walk3 original clean recordings.

Methods:
FGO_BASE
FGO_ROBUST
FGO_REJECT
FGO_RECOVER_FULL
SF_NATIVE

Use the same evaluator, GT sample policy, evaluation interval, and alignment
definition for all methods.

Save:
- ATE RMSE
- ATE P95
- 1 s translation RPE
- used UWB count
- candidate count
- accepted recovery segment count
- runtime
- status/failure

Generate:
- metrics/E1_clean_metrics.csv
- tables/csv/TABLE_I_clean_backbone.csv
- tables/latex/TABLE_I_clean_backbone.tex

Do not tune parameters.
```

---

# R6 — Formal E2: canonical 15-case persistent-bias benchmark

## Purpose
First major scientific Go/No-Go.

## Case design
For each Walk1/2/3:
- choose one deterministic valid 10 s window after bootstrap;
- use the same window for all five anchors;
- inject each anchor separately with `+1.0 m` constant bias;
- total = `3 × 5 = 15` cases.

Freeze the manifest before running methods.

## Methods
### Native FGO
- `FGO_BASE`
- `FGO_ROBUST`
- `FGO_REJECT`
- `FGO_RECOVER_FULL`

### Strong downstream estimator
- `SF_NATIVE`
- `SF_REJECT`
- `SF_RECOVER`

## Primary metrics
- full ATE RMSE;
- fault-window RMSE;
- ATE P95;
- degradation from clean;
- detector event recall;
- bias estimation error;
- paired deltas:
  - `FGO_RECOVER_FULL - FGO_REJECT`
  - `SF_RECOVER - SF_REJECT`
  - `SF_RECOVER - SF_NATIVE`

## Required outputs
```text
manifests/injection_manifest_canonical.yaml
metrics/E2_canonical_runs.csv
metrics/E2_canonical_pairwise.csv
metrics/E2_canonical_detection.csv
metrics/E2_canonical_range.csv
tables/csv/TABLE_II_persistent_faults.csv
tables/latex/TABLE_II_persistent_faults.tex
audits/E2_GO_NOGO.md
```

## Figure 2
Predeclare representative case:
- primary: Walk1 + third anchor in sorted anchor-ID order;
- fallback: Walk2 + third anchor, only if primary is technically invalid.

Generate:
```text
figures/data/FIG_2_trajectory.csv
figures/data/FIG_2_range_timeline.csv
figures/data/FIG_2_detector_timeline.csv
figures/pdf/FIG_2_representative_case.pdf
figures/png/FIG_2_representative_case.png
```

Panels:
- (a) trajectories;
- (b) raw/injected/corrected range timeline;
- (c) conditional innovation + CUSUM support + estimated bias/admission.

## Go/No-Go report
Must include:
```text
number of 15 cases completed
median native-FGO ΔRR
count(native-FGO ΔRR < 0)
median SFUISE ΔRR
count(SFUISE ΔRR < 0)
worst-case degradation
```

## Codex Prompt R6

```text
Run formal E2 canonical persistent-bias experiments.

First create and freeze a case manifest before running any method.

For each Walk1/2/3:
- choose one deterministic valid 10 s window after bootstrap,
- use that same time window for all anchors in the sequence,
- for each of the five anchors separately add exactly +1.0 m constant bias,
- modify only raw ToA range,
- keep IMU, timestamps, GT, IDs, and non-target ranges unchanged.

Total cases: 15.

Run:
FGO_BASE
FGO_ROBUST
FGO_REJECT
FGO_RECOVER_FULL
SF_NATIVE
SF_REJECT
SF_RECOVER

For every case save:
- full ATE RMSE
- fault-window RMSE
- ATE P95
- degradation relative to clean
- detector event result
- estimated bias error
- candidate/accepted counts
- failures/fallbacks
- runtime

Materialize:
FGO_RECOVER_FULL - FGO_REJECT
SF_RECOVER - SF_REJECT
SF_RECOVER - SF_NATIVE

Do not hide losing cases.

Predeclare Figure 2 as Walk1 + third sorted anchor ID, with Walk2 + third sorted
anchor only as a technical-invalid fallback.

Generate CSV source data, vector PDF, PNG preview, and audits/E2_GO_NOGO.md.
Do not tune parameters.
```

---

# R7 — Formal E3/E4: robustness sweeps

Run only after reviewing R6.

## E3-A Magnitude
```text
duration = 10 s
shape = constant
bias ∈ {0.2, 0.5, 1.0, 2.0} m
```

## E3-B Duration
```text
bias = +1.0 m
shape = constant
duration ∈ {2, 5, 10, 20} s
```

## E4 Model mismatch
```text
one anchor
10 s
~1 m mean positive bias
shape ∈ {constant, ramp, slowly_varying_correlated}
```

Use one predeclared anchor/window per Walk. Do not form a giant Cartesian product.

## Methods
- `FGO_ROBUST`
- `FGO_REJECT`
- `FGO_RECOVER_FULL`
- `SF_NATIVE`
- `SF_REJECT`
- `SF_RECOVER`

## Figure 3
Two panels:
- magnitude vs ATE/fault-window RMSE;
- duration vs ATE/fault-window RMSE.

## Files
```text
manifests/injection_manifest_sweeps.yaml
metrics/E3_magnitude.csv
metrics/E3_duration.csv
metrics/E4_shape.csv
figures/data/FIG_3_magnitude.csv
figures/data/FIG_3_duration.csv
figures/pdf/FIG_3_fault_sweeps.pdf
figures/png/FIG_3_fault_sweeps.png
```

## Codex Prompt R7

```text
Run the frozen controlled-fault robustness sweeps. Do not change detector,
recovery, FGO, evaluator, or SFUISE parameters.

Use one predeclared anchor/window per Walk.

A. Magnitude:
duration=10 s, constant positive bias,
bias={0.2,0.5,1.0,2.0} m.

B. Duration:
bias=+1.0 m, constant,
duration={2,5,10,20} s.

C. Model mismatch:
one anchor, 10 s, approximately 1 m mean positive bias,
shapes={constant,ramp,slowly_varying_correlated}.

Do not form an unnecessary full Cartesian product.

Run:
FGO_ROBUST
FGO_REJECT
FGO_RECOVER_FULL
SF_NATIVE
SF_REJECT
SF_RECOVER

Save all per-case results and paired deltas.

Generate machine-readable Figure 3 source CSVs and a vector PDF + PNG preview.
Do not select only favorable ranges or durations.
```

---

# R8 — Formal E5/E6: detector and recovery ablations

## E5 Detector ablation
Compare:
1. marginal/ordinary innovation + threshold;
2. conditional innovation + threshold;
3. conditional innovation + forward CUSUM;
4. conditional innovation + bidirectional CUSUM.

Calibrate to comparable clean false-alarm exposure using clean DEV data.

Metrics:
- precision;
- recall;
- F1;
- event recall;
- forward alarm delay;
- onset error;
- offset error;
- false-positive duration.

## E6 Recovery backend ablation
Freeze one detector support and compare:
- `FGO_REJECT`
- `FGO_STRUCTURED`
- `FGO_RECOVER_PARTIAL`
- `FGO_RECOVER_FULL`

## Paper Table III
Detector block + backend block.

## Files
```text
metrics/E5_detector_ablation.csv
metrics/E6_backend_ablation.csv
tables/csv/TABLE_III_ablation.csv
tables/latex/TABLE_III_ablation.tex
paper_assets/TABLE_III_ablation.tex
```

## Codex Prompt R8

```text
Run the formal detector and recovery-backend ablations using the frozen
injection manifests.

Detector ablation:
1. marginal/ordinary innovation + single threshold
2. conditional innovation + single threshold
3. conditional innovation + forward CUSUM
4. conditional innovation + bidirectional CUSUM

Calibrate thresholds only on clean DEV data to comparable false-alarm exposure.

Report precision, recall, F1, event recall, forward alarm delay, offline onset
error, offline offset error, and false-positive duration.

Backend ablation:
freeze one detector support and compare:
FGO_REJECT
FGO_STRUCTURED
FGO_RECOVER_PARTIAL
FGO_RECOVER_FULL

Do not let each backend rerun a different detector support.

Generate:
- metrics/E5_detector_ablation.csv
- metrics/E6_backend_ablation.csv
- tables/csv/TABLE_III_ablation.csv
- tables/latex/TABLE_III_ablation.tex

Keep all failed/fallback cases visible.
```

---

# R9 — Simulation S1: recovery operating region

## Main grid
### Anchor count
`4, 5, 8`

### Geometry
`good, weak`

### Motion
`excited_turning, weak_straight`

Total = 12 conditions.

For each:
- 20 fixed seeds;
- one persistent positive fault;
- +1.0 m;
- 10 s;
- deterministic affected-link rule.

Total = 240 simulated trials before method multiplication.

## Methods
- `FGO_ROBUST`
- `FGO_REJECT`
- `FGO_RECOVER_FULL`

## Figure 4
Primary quantity:
```text
ΔRR = ATE(FGO_RECOVER_FULL) - ATE(FGO_REJECT)
```

x-axis:
`4, 5, 8 anchors`

Group by geometry and motion.

Also save:
- recovery acceptance rate;
- failure rate;
- local sigma/recoverability diagnostics.

## Files
```text
manifests/simulation_manifest.yaml
metrics/S1_operating_region.csv
figures/data/FIG_4_operating_region.csv
figures/pdf/FIG_4_operating_region.pdf
figures/png/FIG_4_operating_region.png
paper_assets/FIG_4_operating_region.pdf
```

## Codex Prompt R9

```text
Build/run the formal S1 simulation operating-region experiment.

This simulation is for mechanism analysis, not to replace real-data validation.

Main grid:
anchor_count={4,5,8}
geometry={good,weak}
motion={excited_turning,weak_straight}

For every condition use 20 predeclared random seeds.

Canonical fault:
- one affected link,
- +1.0 m positive bias,
- 10 s,
- constant,
- deterministic affected-link selection documented in the manifest.

Save exact simulation truth, anchor coordinates, trajectories, IMU/noise config,
fault truth, condition labels, and seed.

Run only:
FGO_ROBUST
FGO_REJECT
FGO_RECOVER_FULL

For each trial compute:
ATE,
fault-window RMSE,
delta_RR = Recover - Reject,
recovery acceptance,
failure/fallback,
local sigma/recoverability diagnostics.

Generate Figure 4 source CSV and vector PDF + PNG.
Never drop seeds where Recover loses or a method fails.
```

---

# R10 — Optional simulation stress tests

## S2 Multi-fault
Fixed:
- 5 anchors;
- good geometry;
- excited motion;
- +1.0 m;
- 10 s.

Compare:
- one faulty link;
- two overlapping faulty links.

20 seeds each.

## S3 Model mismatch
Bias processes:
- constant;
- ramp;
- slowly varying correlated;
- bias + dropout.

20 seeds each.

## Codex Prompt R10

```text
Run optional simulation stress tests only after S1 is frozen.

S2:
5 anchors, good geometry, excited motion, +1.0 m, 10 s.
Compare one faulty link vs two overlapping faulty links.
20 fixed seeds each.

S3:
canonical geometry/motion.
Compare constant, ramp, slowly varying correlated, and bias+dropout processes.
20 fixed seeds each.

Use the same frozen estimator/detector/recovery configs.
Save all results in metrics/S2_*.csv and metrics/S3_*.csv.
Do not alter the main S1 figure based on these outcomes.
```

---

# R11 — Paper artifact freeze

## Required paper assets

### Table I
`TABLE_I_clean_backbone.tex`

### Table II
`TABLE_II_persistent_faults.tex`

Rows:
- Walk1 canonical aggregate over 5 anchor cases;
- Walk2 aggregate;
- Walk3 aggregate;
- All 15 aggregate.

Columns:
- FGO Robust
- FGO Reject
- FGO Recover
- SF Native
- SF Reject
- SF Recover

Primary cell:
- median ATE RMSE;
- optionally fault-window RMSE in parentheses.

Include `completed/planned`.

### Table III
`TABLE_III_ablation.tex`

### Figure 2
`FIG_2_representative_case.pdf`

### Figure 3
`FIG_3_fault_sweeps.pdf`

### Figure 4
`FIG_4_operating_region.pdf`

## Provenance
Create:
```text
paper_assets/ASSET_MANIFEST.yaml
paper_assets/REPRODUCIBILITY_REPORT.md
```

For every asset record:
- source CSV;
- code commit;
- experiment fingerprint;
- script;
- included cases;
- excluded cases and reasons.

## Codex Prompt R11

```text
Freeze the paper-facing experiment artifacts from the completed formal runs.

Do not rerun methods with changed parameters and do not manually edit numbers.

Generate:

TABLE_I_clean_backbone.csv + .tex
TABLE_II_persistent_faults.csv + .tex
TABLE_III_ablation.csv + .tex

FIG_2_representative_case.pdf + .png + source CSVs
FIG_3_fault_sweeps.pdf + .png + source CSVs
FIG_4_operating_region.pdf + .png + source CSVs

Copy only generated LaTeX/PDF assets into experiments/icra2027/paper_assets/.

Create ASSET_MANIFEST.yaml recording, for every table/figure:
- source files,
- code commit,
- experiment fingerprint,
- script,
- included cases,
- excluded cases and reasons.

Create REPRODUCIBILITY_REPORT.md with exact commands and all
planned/completed/no-candidate/fallback/failed counts.

The report must explicitly state:
- ISAS Walk1/2/3 are one real dataset family,
- persistent NLOS-like faults on real data are controlled injected positive
  range biases,
- simulation is used for operating-region analysis,
- no TDoA datasets are used.
```

---

# 6. Decision tree after R6

## Outcome A — Strong result
If both are consistently favorable:
```text
FGO_RECOVER_FULL < FGO_REJECT
SF_RECOVER < SF_REJECT
```
then recovery remains the main paper claim.

## Outcome B — Recovery only helps native FGO
Emphasize information preservation and low-redundancy operating region. Do not claim downstream-estimator-agnostic superiority.

## Outcome C — Recovery helps SFUISE but not native final FGO
This is still useful. Reframe more strongly around offline range recovery as a downstream-usable measurement treatment layer.

## Outcome D — Recovery does not beat Reject on either backbone
Stop large recovery sweeps. Diagnose the method; do not try to rescue the claim with simulation alone.

---

# 7. Human-owned decisions

The user should personally approve:
1. R1 frame/reference-point interpretation.
2. Whether SFUISE static offsets are valid independent calibration.
3. Frozen native backbone after R2, using clean evidence only.
4. Clean-data detector recalibration before formal injection.
5. Canonical injection manifest before R6.
6. G1 result interpretation after R6.
7. Final claim wording based on paired results.

Codex should **not** choose favorable cases, hide failures, or change parameters after formal test results.

---

# 8. Minimum publishable subset if time is critical

## Must have
- R0–R4 infrastructure/fairness.
- R5 clean/no-harm.
- R6 canonical 15 cases.
- R8 backend ablation.
- R9 4/5/8-anchor operating-region simulation.
- R11 paper assets.

## Strongly recommended
- R7 magnitude/duration sweeps.
- R8 detector ablation.

## Optional
- R10 multi-fault/model-mismatch simulation.
- B3 higher-rate native FGO if B2 is already scientifically fair.
- Any new public dataset.

**Do not spend time adapting new datasets until the canonical Recover-vs-Reject result is understood.**

---

# 9. Final paper mapping

| Paper claim | Required evidence | Artifact |
|---|---|---|
| Nominal SFUISE is stronger but comparison is fair | R1/R2/R5 | Table I |
| Persistent positive range faults are handled | R6/R7 | Table II, Fig. 2, Fig. 3 |
| Recovery can preserve information relative to rejection | R6/R8 | Table II, Table III |
| Recovered ranges transfer to a stronger downstream estimator | R4/R6 | Table II |
| Recovery value depends on redundancy/geometry/motion | R9 | Fig. 4 |
| Detector temporal structure matters | R8 | Table III |
| Failures are not hidden | all formal runs | run_status.csv, reproducibility report |

---

# 10. Start here

Run only:

```text
R0
→ R1
→ R2
→ R3
→ R4
→ R5
→ R6
```

Then review `audits/E2_GO_NOGO.md`.

Only after that should Codex receive R7–R10.
