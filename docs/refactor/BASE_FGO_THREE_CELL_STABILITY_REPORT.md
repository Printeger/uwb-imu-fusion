# Base FGO three-cell stability audit

## Question and pass criterion

This audit asks where the current non-IE Base FGO can work reliably enough to
publish a result. Exactly three estimator cells were run once each:

1. sim-circle clean, `keyframe.step=10`;
2. SFUISE Walk1 sparse, `keyframe.step=4`;
3. SFUISE Walk1 all-UWB, `keyframe.step=1`.

All three use the current qualified common initializer, Huber 1.345
intermediate-seed gate, unchanged Cauchy 2.3849 final solve, and unchanged
`PAPER_SOLVER_CERTIFICATE_V1`. A cell is counted as stable/usable only if the
Cauchy final Values are finite and the certificate succeeds. Raw optimizer
termination alone is not success. No IE stage or GT evaluator was run.

Simulation preserves its fixture sigma of 0.1 m. Both Walk1 cells preserve
sigma 0.15 m. No solver budget, loss parameter, state cadence, input, or
certificate threshold was tuned.

## Results

| Cell | States / factors / UWB | Huber result | Intermediate gate | Cauchy result | Certificate / export | Verdict |
|---|---:|---|---|---|---|---|
| Simulation clean | 160 / 1412 / 1250 | raw converged, 0 updates, objective 5602.768071451678, gradient 39.85572883434374 | PASS | raw converged, 0 updates, objective 5424.636806556538, gradient 43.86699425427338 | FAIL / no trajectory | NOT STABLE |
| Walk1 sparse step=4 | 229 / 1305 / 1074 | raw converged, 39 updates, objective 300.1141905613664, gradient 0.29612243597616206 | PASS | raw converged, 28 updates, objective 271.6411267996103, gradient 0.5243199396112124 | FAIL / no trajectory | NOT STABLE |
| Walk1 all-UWB step=1 | 913 / 3191 / 2276 | raw converged, 14 updates, objective 844.0200102086894, gradient 3.6007261482860535 | PASS | raw converged, 43 updates, objective 621.775894312065, gradient 4.531465978234358 | FAIL / no trajectory | NOT STABLE |

Every state audit is finite. Final maximum position norms are respectively
3.974587989096187 m, 4.513968201404472 m, and 4.501712625121587 m. Thus these
are not catastrophic state explosions. The failure shared by all three cells
is narrower: generic LM convergence does not reach the frozen navigation
stationarity requirement.

The simulation cell is the clearest basic failure. Huber and Cauchy both
return generic convergence without accepting a state update; the certificate
detects large remaining gradients. Sparse Walk1 makes substantial, finite
progress and is closest to stationary, but its final scaled gradient
0.5243199396112124 still cannot be treated as a pass. Dense Walk1 also reaches
a finite low-residual basin, but its final gradient is 4.531465978234358.

## Direct answer

Under the current scientific success contract, none of these three basic
scenarios is demonstrated to work stably. The implementation can produce
finite and physically plausible terminal Values in all three, and its
intermediate seed gate works, but the final Base-FGO certificate rejects every
cell. Therefore the currently supported claim is “finite optimization
terminal is available,” not “stable certified Base FGO estimate is
available.”

One deterministic run per cell is also not a statistical repeatability study;
even a certificate pass would establish only these locked cases, not a general
stability rate.

## Evidence

Evidence is isolated under:

`experiments/icra2027/dev/BASE_FGO_THREE_CELL_STABILITY/`

Each successful invocation of the estimator path produced
`warm_start_summary.json` plus A/B/C state and UWB CSV files. None contains
`trajectory.tum`. `gt_truth_oracle_read=false` and `ie_stages_run=false`.

The first simulation command using the legacy T07 YAML stopped at configuration
preflight because that old fixture did not explicitly declare the current
fixed calibration interface. It never built a graph or invoked an optimizer.
The actual simulation cell used an effective YAML with only those interface
fields made explicit; its original data, sigma, IMU settings, anchors,
`step=10`, and full-duration window were unchanged.
