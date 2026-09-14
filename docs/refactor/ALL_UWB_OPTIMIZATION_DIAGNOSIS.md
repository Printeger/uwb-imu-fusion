# REFACTOR-GATE-06D all-UWB optimization-basin diagnosis

Date: 2026-09-13  
Task: `REFACTOR-GATE-06D / ALL-UWB OPTIMIZATION BASIN DIAGNOSIS`  
Scope: diagnostic only; no repair, GT, ATE, truth, oracle, tuning, or production parameter change

## Verdict

Primary root-cause label:

`DENSE_INITIALIZATION_FAILURE`

The exact Gate06 common initial `Values` are already far outside the physical
range scale before the first optimizer update. Across 913 states, initial
position norm has median `162.809 m` and maximum `1188.033 m`; velocity norm has
median `15.370 m/s` and maximum `63.037 m/s`. The five anchors occupy a region
only about 5.8 m across, and the selected measurements have median `3.20 m`, P95
`4.12 m`, with the known single transient at `23.94 m`.

This is not caused by observation/state association, anchor identity, frame
construction, lever-arm application, fixed sigma, or an alternate dense factor
path. All 2,276 UWB factors attach to their planned contemporaneous state with
exactly zero timestamp mismatch; every audited prediction reproduces the actual
factor equation; all 912 adjacent IMU factors have the expected keys and exact
timeline duration. The step=1 and step=4 cases use the same
`PrepareEstimatorCore -> GraphBuilder::Build` path. A read-only step=4 comparator
has essentially the same open-loop initial drift (position median `163.423 m`,
maximum `1192.269 m`), so “dense” in the classification names the failed dense
run's initialization, not a separate step=1 construction branch.

The current Cauchy loss is an important secondary contributor: at iteration 0,
94.332% of UWB factors have weight below 0.5, 81.371% have weight below `1e-3`,
and the median weight is `4.853e-6`. Thus the UWB graph was already mostly outside
the Cauchy capture basin; optimization did not drive it there. However, Cauchy is
not the primary label because the common seed itself is demonstrably nonlocal,
Huber also fails the unchanged 100-call/stationarity conditions, and Gaussian's
raw generic termination is a nonstationary false success rather than a valid
counterexample.

## 1. Initial-state audit

The exact Gate06 identities are:

- graph/linearization: `t08graphlin-sha256:8d7275386c7c8148ef8befdc07768a41ae095a44939facfb0af64c4b2380a5e5`
- initial `Values`: `t08values-sha256:b0e6c08fcf15922b266c71ae08074ded16487d72fbdaef168cf221726a8962f7`
- state / total factor / UWB factor counts: `913 / 3191 / 2276`
- all supported initial values and state timestamps: finite

| Initial quantity | Median | P95 | Maximum |
|---|---:|---:|---:|
| position norm (m) | 162.809176 | 1015.068510 | 1188.032980 |
| velocity norm (m/s) | 15.369829 | 56.251833 | 63.036904 |
| accelerometer-bias norm (m/s²) | 0 | 0 | 0 |
| gyro-bias norm (rad/s) | 0.011452963 | 0.011452963 | 0.011452963 |
| consecutive position displacement (m) | 0.930132 | 3.972095 | 7.531671 |

The seed begins locally: state 0 position norm is `3.145761 m`, and its first
UWB residual is `-0.002442 m`. It then drifts continuously. Position norm first
exceeds 5, 10, 50, 100, 500, and 1000 m at approximately 5.296, 8.998, 18.852,
24.901, 44.340, and 55.937 seconds after the first state. This sequence-wide
growth, rather than a single discontinuity or one bad anchor, is the nonphysical
initialization failure.

The construction explains the observation without changing it: after the
finite first-state trilateration/gravity initialization, `GraphBuilder::Build`
chains `Pim().predict(...)` from the previous predicted pose/velocity using the
unchanged initial bias for the full recording. UWB observations are added as
factors but do not correct this common seed before batch optimization.

Machine-readable evidence:
`experiments/icra2027/dev/ALL_UWB_OPTIMIZATION_DIAGNOSIS/audit_v2/{audit_summary.json,initial_states.csv}`.

## 2. Observation/state association audit

All 2,276 instantiated factors passed the identity checks:

- `obs_id` exists uniquely in the immutable ledger and selected measurement plan;
- the row is estimator-usable and the independent repeat representative;
- the UWB factor has exactly the planned `X(k)` key;
- measurement-plan `keyframe_id`, decoded factor key, and state timestamp agree;
- timestamp mismatch min / median / P95 / max are all exactly `0 s`;
- every sigma is binary64-equal to `0.15 m`;
- all five anchors occur in the factor set.

The 20 deterministic sequence representatives show the seed drifting away while
association remains contemporaneous:

| order | obs_id | anchor | state | t−t0 (s) | residual (m) | abs q | Cauchy w |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 4969046405526538897 | 7475 | 0 | 0.000 | -0.002 | 0.0 | 1.000 |
| 120 | 8037656541473102929 | 7475 | 47 | 3.083 | 0.528 | 3.5 | 0.314 |
| 239 | 14950727492077464545 | 7475 | 95 | 6.159 | 2.637 | 17.6 | 0.0181 |
| 359 | 3512886187398802450 | 9524 | 145 | 9.375 | 6.253 | 41.7 | 0.00326 |
| 479 | 17815666117130461745 | 20276 | 197 | 12.644 | 15.521 | 103.5 | 0.000531 |
| 599 | 2595999217848362519 | 9524 | 243 | 15.656 | 27.131 | 180.9 | 0.000174 |
| 718 | 10520228038481081579 | 7475 | 291 | 18.745 | 45.182 | 301.2 | 6.27e-5 |
| 838 | 6074580752028860456 | 10548 | 341 | 22.087 | 68.868 | 459.1 | 2.70e-5 |
| 958 | 16513082586548325454 | 9524 | 388 | 25.152 | 97.142 | 647.6 | 1.36e-5 |
| 1078 | 6879347491875146315 | 20276 | 437 | 28.461 | 140.818 | 938.8 | 6.45e-6 |
| 1197 | 12155353821666649244 | 7475 | 483 | 31.349 | 185.552 | 1237.0 | 3.72e-6 |
| 1317 | 4609524536685317248 | 20276 | 528 | 34.257 | 235.934 | 1572.9 | 2.30e-6 |
| 1437 | 8203173526784427388 | 7475 | 574 | 37.153 | 299.317 | 1995.4 | 1.43e-6 |
| 1557 | 7044714240185052323 | 20276 | 623 | 40.295 | 378.047 | 2520.3 | 8.95e-7 |
| 1676 | 6184540158400690703 | 15155 | 671 | 43.418 | 471.080 | 3140.5 | 5.77e-7 |
| 1796 | 15960509895043685125 | 15155 | 719 | 46.643 | 580.172 | 3867.8 | 3.80e-7 |
| 1916 | 17662071115019970996 | 20276 | 769 | 49.922 | 706.479 | 4709.9 | 2.56e-7 |
| 2036 | 11638328581807179105 | 20276 | 817 | 52.994 | 846.637 | 5644.2 | 1.79e-7 |
| 2155 | 15035691189467516976 | 15155 | 863 | 55.937 | 1003.601 | 6690.7 | 1.27e-7 |
| 2275 | 15402623129066363662 | 15155 | 911 | 59.043 | 1184.936 | 7899.6 | 9.11e-8 |

The 20 largest initial absolute residuals are all late-sequence states and span
all five anchors, which is inconsistent with an isolated link mapping error:

| rank | obs_id | anchor | state | t−t0 (s) | residual (m) | abs q |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 15402623129066363662 | 15155 | 911 | 59.043 | 1184.936 | 7899.6 |
| 2 | 15402627527112876506 | 7475 | 911 | 59.043 | 1181.861 | 7879.1 |
| 3 | 1606140475115259945 | 15155 | 910 | 58.990 | 1181.610 | 7877.4 |
| 4 | 15402626427601248295 | 10548 | 911 | 59.043 | 1180.256 | 7868.4 |
| 5 | 15402628626624504717 | 20276 | 911 | 59.043 | 1180.139 | 7867.6 |
| 6 | 4383153658453106452 | 15155 | 909 | 58.939 | 1178.460 | 7856.4 |
| 7 | 4383149260406593608 | 7475 | 909 | 58.939 | 1175.329 | 7835.5 |
| 8 | 4383152558941478241 | 10548 | 909 | 58.939 | 1173.820 | 7825.5 |
| 9 | 4383150359918221819 | 20276 | 909 | 58.939 | 1173.624 | 7824.2 |
| 10 | 8003455338220548853 | 15155 | 907 | 58.783 | 1168.819 | 7792.1 |
| 11 | 9226783012307668875 | 10548 | 908 | 58.834 | 1167.357 | 7782.4 |
| 12 | 8003450940174036009 | 7475 | 907 | 58.783 | 1165.774 | 7771.8 |
| 13 | 8003447641639151376 | 10548 | 907 | 58.783 | 1164.227 | 7761.5 |
| 14 | 8003449840662407798 | 20276 | 907 | 58.783 | 1164.002 | 7760.0 |
| 15 | 9238668015309626310 | 15155 | 905 | 58.627 | 1159.262 | 7728.4 |
| 16 | 9238672413356139154 | 7475 | 905 | 58.627 | 1156.223 | 7708.2 |
| 17 | 9238671313844510943 | 10548 | 905 | 58.627 | 1154.687 | 7697.9 |
| 18 | 9238673512867767365 | 20276 | 905 | 58.627 | 1154.484 | 7696.6 |
| 19 | 9238670214332882732 | 9524 | 905 | 58.627 | 1152.245 | 7681.6 |
| 20 | 15717521741085787772 | 15155 | 903 | 58.454 | 1148.857 | 7659.0 |

The complete factor-by-factor audit, including timestamps, keys, measured and
predicted ranges, anchor coordinates, sigma, residual, q, weight, and audit
flags, is `audit_v2/uwb_association_initial.csv`.

## 3. Initial UWB residual and robust-weight audit

All residual summaries below use absolute raw residual or absolute standardized
residual; signed values remain in the full CSV.

Global initial diagnostics:

- absolute raw residual mean / median / P95 / max:
  `299.043 / 162.397 / 1010.496 / 1184.936 m`;
- absolute q median / P95 / max: `1082.645 / 6736.637 / 7899.572`;
- Cauchy weight median / P05 / P95:
  `4.853e-6 / 1.253e-7 / 0.609538`;
- fraction weight `>0.5`: `5.668%`;
- fraction weight `>0.1`: `8.392%`;
- fraction weight `<0.01`: `87.390%`;
- fraction weight `<1e-3`: `81.371%`;
- fraction weight `<1e-6`: `32.645%`.

Gate06's uncertified terminal diagnostics were absolute-q median `1001.620`,
P95 `6724.565`, maximum `7889.417`, and median Cauchy weight `5.669e-6`.
The initial values are already marginally worse and in the same basin. The
terminal iterate improves the early/locally captured subset, but the median and
tail remain at the same order. Therefore the answer to the Prompt's key question
is: **the UWB factors were already mostly outside the Cauchy capture basin at
iteration 0**.

## 4. Per-anchor initial comparison

| Anchor | N | mean abs r (m) | median abs r (m) | P95 abs r (m) | max abs r (m) | median abs q | P95 abs q | max abs q | median Cauchy w |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 7475 | 479 | 291.246 | 150.905 | 1001.369 | 1181.861 | 1006.034 | 6675.790 | 7879.071 | 5.620e-6 |
| 9524 | 433 | 291.457 | 160.071 | 981.673 | 1152.245 | 1067.140 | 6544.488 | 7681.632 | 4.995e-6 |
| 10548 | 450 | 315.467 | 186.304 | 1030.634 | 1180.256 | 1242.029 | 6870.896 | 7868.374 | 3.687e-6 |
| 15155 | 464 | 298.290 | 154.185 | 1023.422 | 1184.936 | 1027.903 | 6822.817 | 7899.572 | 5.384e-6 |
| 20276 | 450 | 298.993 | 162.945 | 995.737 | 1180.139 | 1086.302 | 6638.247 | 7867.595 | 4.820e-6 |

The similar distributions across every anchor further rule out one mis-surveyed
anchor or a swapped anchor ID as the primary cause.

## 5. Graph/frame consistency audit

The actual factor inputs are:

| Anchor ID | world x (m) | world y (m) | world z (m) |
|---:|---:|---:|---:|
| 7475 | 0 | 0 | 0 |
| 9524 | 2.609999895 | 2.670000076 | 0 |
| 10548 | 5.519999981 | 0.050000001 | 1.860000014 |
| 15155 | 3.119999886 | -2.589999914 | 1.850000024 |
| 20276 | 5.500000000 | 0 | 0 |

- `X(k)` is body-to-world. The body-frame UWB lever is
  `[0.1, -0.025, 0] m`, so the antenna point is `p + R*lever` in world.
- Calibration variables are disabled in the locked Gate06 config. Fixed beta is
  empty/unavailable and contributes zero in this explicit development setup.
- The audited equation is
  `norm(p + R*lever_body - anchor_world) + fixed_beta - measured_range`.
  All 2,276 manual predictions match each actual factor prediction within the
  binary64 audit tolerance.
- Each UWB factor's only variable key is its expected `X(k)`; each actual noise
  sigma is exactly `0.15 m`.
- There are exactly 912 `CombinedImuFactor`s for 913 states. Every factor has
  keys `[X(k-1),V(k-1),X(k),V(k),B(k-1),B(k)]`; every state time is finite and
  strictly increasing; every interval has at least one interior IMU sample; and
  maximum `|PIM deltaT - timeline deltaT|` is exactly zero.
- Step=1 and step=4 share the same loader, ledger construction,
  `PrepareEstimatorCore`, initializer, and `GraphBuilder::Build`. Intended
  differences are the frozen state timeline, selected representative count,
  adjacent IMU intervals, and resulting counts: step=1 is 913 states / 2,276 UWB
  / 3,191 factors; step=4 is 229 / 1,074 / 1,305. The raw observation-ledger
  SHA-256 is equal. No alternate dense factor-construction branch exists.

No graph/association correctness bug was found, so Part 5 replays were allowed.

## 6. Controlled loss-mode diagnostic replays

All three replays report the same base graph hash and common initial `Values`
hash listed in Section 1. Their initial-state CSV files are byte-identical.
Each uses 100 calls maximum, relative tolerance `1e-6`, absolute tolerance
`1e-8`, the same states, topology, factors, measurements, fixed sigma, IMU,
priors, and initial values. Only the UWB loss wrapper differs. Objectives across
rows are not comparable because the loss functions differ.

| Replay | Raw termination | calls / accepted iterations | objective initial → terminal | last accepted Values delta norm | finite | stationarity / max scaled gradient | terminal abs r mean / median / P95 / max (m) | terminal abs q median / P95 / max | max position norm (m) | effective downweight initial → terminal |
|---|---|---:|---:|---:|---|---|---|---|---:|---:|
| R0 Cauchy 2.3849 | `MAX_ITERATIONS` | 100 / 100 | 70,287.839 → 56,797.576 | 5.739 | yes | fail / 461.053 | 287.311 / 150.243 / 1008.685 / 1183.413 | 1001.620 / 6724.565 / 7889.417 | 1186.527 | 94.332% → 65.510% |
| R1 Huber 1.345 | `MAX_ITERATIONS` | 100 / 100 | 6,100,877.479 → 2,932,098.655 | 41.345 | yes | fail / 10,478.751 | 143.558 / 0.166 / 718.947 / 906.070 | 1.105 / 4792.980 / 6040.469 | 909.678 | 94.112% → 41.916% |
| R2 Gaussian | `CONDITIONAL_LM_CONVERGED` | 2 / 1 | 10,106,419,884.450 → 9,917,830,307.218 | 3477.550 | yes | fail / 951,232.006 | 295.448 / 177.248 / 945.186 / 1119.141 | 1181.655 / 6301.241 / 7460.937 | 1121.572 | 0% → 0% |

“Effectively downweighted” was predeclared as weight `<0.5`. The replay files
also report the fraction below `1-1e-12` and the full weight distributions.

R0 reproduces the Gate06 objective, terminal residual distribution, and final
delta exactly. Huber supplies substantially more long-range attraction: by call
100, over half the UWB factors have abs q near or below the local Huber region,
and median position norm falls to `4.146 m`; however, the late trajectory tail
remains hundreds of metres away and the solver is taking a `41.345`-norm update,
so it is neither converged nor stationary. This supports Cauchy capture-basin
weakness as a secondary contributor, not an adequate standalone root cause.

Gaussian accepts one enormous `3477.550`-norm update. Its second call accepts no
state change, drives lambda to `100000`, and the linked generic small-change
check returns success. The terminal state remains grossly nonlocal and its
stationarity audit fails by roughly eleven orders above the `1e-6` qualification
tolerance. This is precisely why raw termination is separate from the Gate04
certificate; the replay is not a valid estimate and does not show Gaussian is a
repair. It is evidence of an LM conditioning/termination symptom, but there is
no dimension-matched good-initialization control to justify
`DENSE_LM_CONDITIONING_FAILURE` as the primary cause.

## 7. Optional extended replay

`NOT_RUN`.

Parts 1–6 already identify the starting basin and distinguish the loss response.
Increasing only the Cauchy iteration budget would not answer a remaining
attribution question and could be misread as a production-budget proposal.

## 8. Root-cause classification

- Primary: `DENSE_INITIALIZATION_FAILURE`.
- Secondary contributor: `CAUCHY_CAPTURE_BASIN_FAILURE` — the current Cauchy
  wrapper numerically silences most already-nonlocal UWB rows at iteration 0.
- Observed but not promoted: dense LM conditioning/generic-termination pathology.
  Huber and Gaussian expose it, but this task does not isolate state dimension
  from initialization quality.
- Ruled out by direct audit: `MEASUREMENT_STATE_ASSOCIATION_BUG` and
  `FACTOR_FRAME_CONSTRUCTION_BUG`.

The primary wording does not claim that state density creates the seed drift;
the step=4 comparator shows the same open-loop initialization pathology. It says
that the exact dense Gate06 solve begins from a failed common initialization,
which is the earliest demonstrated causal mechanism. The robust loss and dense
LM response determine how poorly that failure is recovered.

## 9. Narrowest justified next repair

Authorize a separate, versioned **common-initialization-only** repair. Its scope
should replace the full-recording open-loop IMU propagation seed with a
truth-free, causal UWB-aided initialization that periodically keeps the existing
`X/V/B` seed within the measurement basin while reusing the current physical
UWB/IMU factors and backend. It must not alter the raw ledger, repeat
representatives, association, anchor/lever/beta convention, sigma, robust scales,
LM parameters, certificate, or IE Stage1–4 mathematics.

Before any Gate06 rerun, that repair needs a dedicated regression showing, on
the same locked input and without GT, that the initial states remain finite and
the sequence-wide UWB residual/weight audit no longer exhibits the current
monotonic kilometre-scale tail. The exact initializer design and admission
thresholds require a new prompt/amendment; they were intentionally not invented
or implemented here.

## Evidence, commands, and validation

Primary artifact root:
`experiments/icra2027/dev/ALL_UWB_OPTIMIZATION_DIAGNOSIS/`.

- Protocol: `PROTOCOL.md`
- Exact audit: `audit_v2/`
- Read-only sparse comparator: `audit_step4_comparator/`
- R0/R1/R2: `replays/{R0_CAUCHY,R1_HUBER,R2_GAUSSIAN}/`
- Gate06 terminal comparison:
  `../ALL_UWB_CORRECTNESS_WALK1/audit_metrics.json`

Build and all four required diagnostic invocations exited 0. The direct
architecture guard also exited 0 with
`Gate-05 architecture dependency guards passed`. A `ctest --test-dir` probe
found no registered tests in that package-local invocation and is recorded as
`NOT_RUN_NO_TESTS_DISCOVERED`, not as test success. The dedicated binary SHA-256
is `fe19766d39d8f288ea957d5ad9d162f6fda429364fe1b789b89176d4f94d6258`;
the exact Gate06 effective config SHA-256 is
`4cd9da0df33407e15c75de3bafc1075c5d096d2a55d7a68962b99b4936710820`.

No production source behavior, checked-in solver/noise/kernel parameter,
observation mask, IMU model, IE method, evaluator, paper result, or claim was
changed. No repair, retry, sweep, extended-budget run, commit, or push occurred.

GATE06D_ROOT_CAUSE_IDENTIFIED
