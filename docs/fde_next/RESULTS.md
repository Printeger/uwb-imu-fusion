# FDE next: isolated research results

Baseline: `8fe833c`; clean working tree at start. Production configurations,
probability contracts and publication gates are unchanged. All new runs use
`formal_eligible=false`, `publication_protected=false`. No extra sensor is used.

| Stage | Work | Capability / guarantee |
|---|---|---|
| S1 | Complete fixed-model reference | Finite Gaussian oracle; raw nonlinear qualification pending |
| S2 | Pending | No claim yet |
| S3 | Pending | No claim yet |
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
