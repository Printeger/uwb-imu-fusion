# FDE next: isolated research results

Baseline: `8fe833c`; clean working tree at start. Production configurations,
probability contracts and publication gates are unchanged. All new runs use
`formal_eligible=false`, `publication_protected=false`. No extra sensor is used.

| Stage | Work | Capability / guarantee |
|---|---|---|
| S1 | Complete fixed-model reference | Finite Gaussian oracle; raw nonlinear qualification pending |
| S2 | Functional work complete; Gaussian raw qualification gap | U1 correct commit + 4 epochs; finite-episode U2; raw empirical only |
| S3 | Complete limited exit B | New matched-power/bridge-domain attempt; no IMU recovery commit |
| S4 | Pending | No claim yet |

## S1

`research/gaussian_reference.hpp` provides an executable dense conditional
Gaussian reference, not an estimator backend. Cholesky joint whitening and SVD
solve expose reference mean, covariance, parity statistic and protection map.
For an accepted candidate, `||Z f|| <= sqrt(tau)+sqrt(chi2_quantile(1-tail))`
on its parity-noise event. The coordinate fault bound uses the residual-space
pseudoinverse slope. A separate three-coordinate noise event costs another
`tail`. Taking the union over **all** candidates and all outputs covers adaptive
selection without inferring shared failure events from matching IDs or Grams.
This bound is conditional on fixed A, covariance, C and covered fault spaces.
No range-union routine is called. Unresolved state rank or nearly singular
fault response refuses; a dangerous nullspace returns infinite PL. The
floating-point nullspace classification is a numerical oracle, not interval
certification for arbitrary ill-conditioned inputs.

The research YAML fixes a 50-epoch horizon, 128-candidate cap, `tail=1e-9`,
`alpha=1e-6`, budget `4e-5`, seed 20261010 (confirmation 20261011) **before raw
stream evaluation**. Simultaneous union cost is `2*50*128*1e-9=1.28e-5`.
There is one categorical UWB episode (task probability .001), or nominal;
S1/S2 condition on no IMU fault. This is a synthetic task law, not a measured
hardware prior. All physical leaves must remain covered even for selection
candidates that remove a healthy row. The union proof is independent of the
prior because it bounds every included fault amplitude on the same noise event.
The separate prior-weighted miss-floor function rejects its over-budget test:
`3e-5+1e-7+1000*1e-4*.001=1.301e-4 > 4e-5`.

Sixteen deterministic checks pass (denominator 16, failures 0): correlated
covariance vs independent dense normal equations; cross-covariance omission;
principal-submatrix deletion vs incorrectly deleting mixed white rows; nonzero
shared noise map; finite, exactly harmless and dangerous nullspaces; unresolved
near-rank boundary; risk floors; and nonzero-lever rotation finite differences.
The finite snapshot has statistic .0238924, threshold 16.2662, PL xyz
`[2.26057,6.12974,0]`. It validates a finite bound, not this example's AL.
The exact harmless case protects a coordinate orthogonal to an undetectable
state/fault direction. No random coverage test is used to assert rare-event risk.

S1's positive is a **FROZEN_FACTOR_MODEL**. Joint covariance is an explicit
input and deletion is in raw coordinates. Raw PIM covariance, history mapping
and nonlinear remainder qualification are not inferred from this small model;
S2 must separately report its adapter's limits. GT never participates in this
reference calculation.

Reproduction (workspace already configured for ROS Noetic, Release):

```sh
cmake -S . -B /home/mint/ws_fusion_uwb/build/uwb_imu_pl
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_next_checks -j2
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib
BIN=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl
"$BIN/fde_next_checks" config/fde_next.yaml
"$BIN/fde_research_contract_checks" config/fde_imu_order1.yaml
```

Machine: Intel Core Ultra 5 125H, 18 logical CPUs; single-thread reference.
Tested source is the S1 working tree derived from baseline (commit recorded by
next stage; no self-SHA amendment). No figures are needed for this oracle.

## S2

S1 commit: `b9a9bd3`. The shared `NoisyMotionScenario` in
`scenario_material.hpp` generates the configured smooth translation and body
rotation, true specific force and gyro, and ranges at `p_body+R*lever`.
Independent PRNG streams pre-generate untruncated ADC/range Gaussian noise.
ON/OFF reuse identical inputs. Initial error is configured once, with no
per-epoch truth reset. The 50 epochs cross fixed-lag exit at epoch 31.

`gaussian_window.hpp` differentiates actual reintegrated Combined factors for
each raw sample and axis, including the shared endpoint. It constructs
`J Q J^T` with cross-factor blocks. PIM is separately reintegrated with zero
accelerometer/gyro ADC covariances to retain only auxiliary bias/process
covariance; ADC noise is not added on top of its old marginal contribution.
Observed cross-block norm is about `1.30051e-8` in native residual units.
Candidate UWB replacements are checked as literal original raw-row subsets;
the retained covariance principal submatrix is selected before whitening.
The joint Gaussian statistic drives the research alarm and post test. Existing
estimator-weight Evidence supplies a conservative candidate-search heuristic,
not a calibrated probability or a merged failure event. The Gaussian PL still
covers **every** configured fault space for each evaluated candidate. The
existing rank-update solver and atomic transaction supply the actual estimate;
its different weighting is handled by transferring the reference PL to the
actual nonlinear committed tag mean. No range-union call is used.

Finite three-epoch UWB episodes have one shared amplitude across exact onset/stop
support. Their maps sum constituent raw/history single-epoch maps; they are
not persistent-to-window-end columns. The synthetic task law allows one
UWB episode, uniformly over onset 1–50, anchor and duration 1 or 3; events
continue naturally beyond the task cutoff. No carry-in exists at initialization.
This is a conditional research law, not a hardware event-rate estimate.

Two mechanism corrections were retained: a per-state, unit-specific numerical
recovery gate replaces the mixed-unit full-window norm **only in this runner**;
and research alarms/post statistics use the joint reference covariance.
Rotation/position numerical domains are .25 rad/.25 m; velocity and bias scales
are the original initial prior sigmas (.5 m/s, .1 m/s², .01 rad/s). These scales
are not full nonlinear remainder certificates. Candidate reference means must
pass the same domain. Incomplete V1 action census refuses selection rather than
throwing away the remainder of the stream. This is not top-K acceptance.

| Flow (seed) | Commits / total | Local available | Alarms | Exclusions | Correct fault epochs / injected | Other exclusions | Tag APE RMSE / P95 / max (m) | Increment RMSE (m) | SE3 translation / rotation RPE RMSE (m/rad) |
|---|---:|---:|---:|---:|---:|---:|---|---:|---|
| U0-uwb (20261010) | 50/50 | 49/50 | 1 | 1 | 0/0 | 1 | 0.068989 / 0.102512 / 0.165751 | 0.044544 | 0.044092 / 0.021015 |
| U1-uwb (20261010) | 50/50 | 49/50 | 2 | 2 | 1/1 | 1 | 0.069311 / 0.103407 / 0.165751 | 0.044323 | 0.043872 / 0.020596 |
| U2-uwb (20261010) | 50/50 | 49/50 | 4 | 4 | 3/3 | 1 | 0.069470 / 0.102929 / 0.165751 | 0.044449 | 0.043972 / 0.020919 |
| U3-uwb (20261010) | 50/50 | 0/50 | 1 | 0 | 0/0 | 0 | 0.367938 / 0.555703 / 0.582021 | 0.067223 | 0.066338 / 0.021042 |
| U0-off (20261010) | 50/50 | 0/50 | 1 | 0 | 0/0 | 0 | 0.069253 / 0.102512 / 0.165751 | 0.044547 | 0.044087 / 0.021030 |
| U1-off (20261010) | 50/50 | 0/50 | 31 | 0 | 0/1 | 0 | 0.086662 / 0.142114 / 0.226145 | 0.052472 | 0.051439 / 0.034269 |
| U1-confirm (20261011) | 50/50 | 49/50 | 1 | 1 | 1/1 | 0 | 0.064366 / 0.119656 / 0.208344 | 0.051613 | 0.050990 / 0.017671 |

OFF alarm counts are diagnostic Gaussian tests, with no FDE selection.
All commits, including best-effort and failed qualification, remain in the
50-output denominator. Timestamp-grid output proportion is 50/50, not a
real-time delivery proportion. Formal/protected output count is 0/50 in every
flow. ON/OFF APE uses all common 50 timestamps.

U1-uwb: at epoch 15, actual tag error 0.033196 m, served local HPL/VPL 0.877375/1.023034 m. Following four selected/local-available commits: 4/4 and 4/4. Timestamp detection/recovery delay at the injected U1 epoch is 0 epochs; computation/output delay remains nonzero and is measured in S4.
U2-uwb: at epoch 17, actual tag error 0.030933 m, served local HPL/VPL 0.985751/0.999054 m. Following four selected/local-available commits: 4/4 and 4/4. Timestamp detection/recovery delay at the injected U1 epoch is 0 epochs; computation/output delay remains nonzero and is measured in S4.
U1-confirm: at epoch 15, actual tag error 0.083711 m, served local HPL/VPL 0.873153/1.024232 m. Following four selected/local-available commits: 4/4 and 4/4. Timestamp detection/recovery delay at the injected U1 epoch is 0 epochs; computation/output delay remains nonzero and is measured in S4.

U2 removes the bad current lineage on each of its three contaminated epochs;
those rows never enter the backend, and the four epochs after stop run. This
is a functional finite-episode witness, not qualification of arbitrary
hardware persistent faults. U3 refuses all 50 local protection outputs because
coplanar ranges do not supply a global mirror-branch certificate.

**Guarantee limit:** the small fixed-model S1 theorem holds. The raw runner
outputs `LINEARIZATION_UNQUALIFIED`, with `raw_qualified=false`; its noise-to-
condensed-history transform, random linearization point and complete IMU/pose
nonlinear remainder are not certified. Initial prior centre error is configured,
not a calibrated stochastic prior. `1.28e-5` is the fixed-law union charge,
**not** a raw-stream HMI bound. The healthy epoch-31 false exclusion is a new
empirical counterexample to extending frozen false-alarm calibration to this
path. Functional U1 recovery is implemented; a strictly qualified raw Gaussian
closed loop remains a core gap. No formal risk closure or deployment protection
is claimed.

Five directly relevant existing transaction/PL tests pass (0/5 failures),
covering prepare/discard, one update, principal covariance, historical atomic
replacement and research-only PL. Sixteen mathematical checks still pass.
The development prefixes were discarded after the corrected model was frozen;
the table contains one final flow per requested case and one U1 confirmation.

Reproduction, after building `fde_next_runner` in the same build directory:

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_next_runner -j2
TMP=$(mktemp -d /tmp/fde-next-replay.XXXXXXXX)
for SPEC in U0:uwb U1:uwb U2:uwb U3:uwb U0:off U1:off; do
  SCENE=${SPEC%:*}; SCOPE=${SPEC#*:}
  "$BIN/fde_next_runner" config/fde_next.yaml "$TMP/$SCENE-$SCOPE.csv" "$SCENE" "$SCOPE"
done
"$BIN/fde_next_runner" config/fde_next.yaml "$TMP/U1-confirm.csv" U1 uwb 20261011
# Summarize/inspect transient output, then remove only this created TMP.
```

## S3

S2 commit: `f2b7790`. `imu_recovery_scope.hpp` defines one physical accel-x
raw episode with exact sample IDs/timestamps and a shared amplitude over all
affected factors. The original native IMU generator supplies action recipes;
new raw response maps replace interval surrogates. Both 50 and 500 ms supports
include the trailing shared sample contribution. When an affected interval
becomes condensed, its raw episode map is explicitly unqualified; an old
interval column or disappearance of a sample does not certify carry-in removal.
Incomplete support and unqualified historical modes forbid research selection.
Production interval contracts are unchanged.

Before actual episode trials, one healthy epoch-24 design snapshot screened
the configured 20 m/s² amplitude. Its complete fixed-law parity responses are:

| Raw duration | Lambda | Conditional miss | Target beta | Explicit affected factors | Motion in bridge box | Actual isolation |
|---|---:|---:|---:|---:|---|---|
| 50 ms | 8.04809 | .999984 | .001 | 2 | yes | 0/1 episode |
| 500 ms | 387.421 | 7.83524e-19 | .001 | 10 at epoch 24, plus carry factor 25 | yes | 0/1 episode |

The acceleration bound is derived before evaluation from
`||amplitude * frequency²||=.0284445 m/s²`; angular speed is .2 rad/s.
Both lie inside the existing 4 m/s² and 2 rad/s limits. This is not the old
5.4 m/s² out-of-domain trajectory. These power figures are **conditional
frozen-model predictions**, not empirical miss rates or hardware probabilities.
The bias/process auxiliary covariance, finite-amplitude/input-dependent noise
and random linearization remain unqualified for a raw probability guarantee.

| Flow | Commits / total | First alarm | Actual IMU exclusion commits | Post-fault four recovered outputs | Tag APE RMSE / P95 / max (m) | Current UWB-only local diagnostic finite | Full pipeline selected |
|---|---:|---|---:|---:|---|---:|---:|
| I1, seed 20261010 | 50/50 | epoch 24 | 0/50 (0/1 episode) | 0/4 | 0.085047 / 0.128544 / 0.165751 | 49/50 | 10/50 |
| I2, seed 20261010 | 50/50 | epoch 20 | 0/50 (0/1 episode) | 0/4 | 0.382898 / 0.574901 / 0.586891 | 18/50 | 10/50 |

The I1 brief episode is not detected at its injection: epoch-15 statistic
130.5339 < 183.4599. The later epoch-24 alarm is mixed with history/model
inconsistency and cannot be scored as correct timely isolation. I2 first
alarms at epoch 20: 235.4541 > 203.2736, 250 ms after the first affected epoch
(timestamp delay; wall computation is additional). No IMU exclusion commit
occurs. All 50 best-effort commits remain in each error denominator.

The correct-support I2 candidate at epoch 20 is generated automatically as
`imu_accel:0:interval:15|raw-accel-x-500ms` (action 22). It removes groups
15001,16001,17001,18001,19001,20001, covering every affected factor then
available. The actual exclusion solve yields maximum blocks
`rotation=.280861 rad, position=.394342 m, velocity=3.19672 m/s,
accel-bias=.0260087 m/s², gyro-bias=.00270694 rad/s`. It exceeds the frozen
research local recovery domain; this is an **excluded-candidate** result, not
an inference from the all-in step. At epoch 21 its respective first three
blocks are .368155/.439467/3.50529. By epoch 23 the matching onset is in the
condensed boundary and exact raw-history support is not available to certify
recovery. The gate failure indicates an unsuitable local reference/limited
recovery mechanism, not a proof that the native sensors can never recover.

Other numerically valid episode actions still require a full-state,
fault-independent bridge noise/remainder certificate. `gaussianCandidate`
refuses synthetic bridge rows as unqualified rather than inheriting deleted
IMU covariance. Only passing the acceleration box would omit initial velocity/
attitude uncertainty and gravity coupling. No risk-closing bridge or Gaussian
IMU-recovery positive is claimed. A bounded candidate relinearization with
qualified bridge/history propagation remains a precise implementation gap.

Capability output is usable: `HISTORY_RAW_EPISODE_UNQUALIFIED`,
`IMU_CANDIDATE_DOMAIN_OR_PROVENANCE_FAILED` and
`BRIDGE_GAUSSIAN_UNQUALIFIED` identify refusal. `current_tag_local_hpl/vpl`
is a separate UWB-only current-position diagnostic, transferred to the actual
committed tag, available only within its .25 m local position domain. It is
never injected back as independent velocity information or used for action
selection. It illustrates why an unresolved IMU event can coexist with
possible local current-position bounds. `future_state_status=UNPROTECTED`
never inherits stationary-yaw harmlessness or the previous epoch PL.
All raw coverage and future protection remain unqualified.

Twenty-eight deterministic checks pass (0/28 failures), including a new actual
two-factor raw support test, omitted-boundary-factor refusal, history coverage
refusal, joint ADC covariance, candidate-deletion negative and reproducible
noise inputs. The existing fault-independent bridge/box test also passes (1/1).
The two affected flow qualifications were rerun after fixing the condensed-
episode refusal; earlier output qualification counts were discarded.
This is S3 limited exit B: new legal-motion candidate development, screening
and actual excluded solves are complete; legal IMU recovery remains absent.

Reproduction:

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target fde_next_runner fde_next_checks -j2
"$BIN/fde_next_checks" config/fde_next.yaml
"$BIN/fde_next_runner" config/fde_next.yaml "$TMP/screen.csv" screen imu-diagnostic 20261010 24
"$BIN/fde_next_runner" config/fde_next.yaml "$TMP/I1.csv" I1 imu-diagnostic
"$BIN/fde_next_runner" config/fde_next.yaml "$TMP/I2.csv" I2 imu-diagnostic
"$BIN/test_integrity_v2" --gtest_filter=Phase2NumericLayerPositive.ImuBridgeIsIndependentAndItsBoxMarginFinite
```
