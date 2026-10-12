# HYBRID raw qualification

N0–N4 completed for the declared conditional simulation scope.
**RAW_BOUND_VERIFIED_FOR_DECLARED_SCOPE**, full50. Application AL remains
**APPLICATION_AL_UNQUALIFIED**; production qualification and protected
publication remain disabled. Starting HEAD: b051b151028dfc799ac0ae5845f2fcdae40d5f7c,
branch feature/realtime-uwb-imu-pl; initial working tree clean.

## N0 — independent AL evaluation

`HybridSelectionPolicy::BoundFirstResearch` retains the numerical, post-detector,
history provenance and common-event requirements in the fixed candidate family,
and removes only the AL-dependent candidate rejection. `LegacyAlGated` remains
the default of the existing fixed entry. The raw entry already selects without
AL; its CSV now explicitly names the strategy and separates finite qualification
from application-policy qualification. Unknown raw risk still refuses protection.
`finiteBoundQualified()` checks numerical/model/risk/receipt validity without AL;
legacy `available()` retains its original meaning. Application availability also
requires a qualified application policy and timeliness.

Deterministic tests compare 2/3, 5/5 and 10/10 m for both envelope representations:
identical selected subsets, status, PL and next H (including risk and IDs) across
five epochs with an injected fault and larger fixed-model physical sigmas. The
native 24-epoch fixtures test all three post-commit evaluations, finite bounds
above legacy AL, unknown raw risk and missing application policy. Existing fixed
20-epoch source/correlation/history and 48 native transaction outputs pass.
These fixtures do not qualify a raw Gaussian stream.

Reproduce in the configured workspace:

```sh
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target hybrid_checks epoch_raim_runner -j2
export LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib:${LD_LIBRARY_PATH:-}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/hybrid_checks config/fde_next.yaml
```

## Installed integration audit

Local headers and source identify GTSAM 4.2a5, tangent preintegration, Pose3 and
Rot3 EXPMAP, matrix Rot3 representation. CMake resolves `/usr/local/lib/cmake/GTSAM`;
source inspected at `/home/mint/dep/gtsam/gtsam/navigation/CombinedImuFactor.cpp`.
Its F propagates bias states, dt times bias RW covariance, dt times position
integration covariance, and B/C times `(ADC covariance + biasAccOmegaInt)/dt`.
The latter injects fresh per-step covariance, and cannot represent one constant
latent initial-bias error shared across all integration steps. Backend initialize
maps the initial bias prior sigmas into this parameter unless explicitly supplied.
The auxiliary trace diagnostic is not a removal justification. The implemented new source law and independent physical reference are described
below; this trace alone is not raw coverage evidence.

## N1 — executable primitive law and conditional physical covariance

New isolated configuration: `config/hybrid_raw.yaml`, source law
`raw-source-rw-v1`. It inherits legacy motion, raw sigmas (.02 m/s²,
.001 rad/s, .1 m), lever, horizon50 and seed20261010. Initial truth is
conditional fixed (including nonzero initial bias); the independent 15D
observation uses the existing Pose3 right EXPMAP retract, velocity and bias
coordinates. Bias increments occur on the .005s sample grid, independent of
ADC/initial/UWB, with sigmas `.001 sqrt(dt)` m/s² and `.0001 sqrt(dt)` rad/s.
Between samples the motion remains the original smooth analytic trajectory;
no fake integration process is added to match a numerical weight.

| Primitive | Count at T=50 | Shared use |
|---|---:|---|
| Initial observation, rotation/local translation/velocity/accel bias/gyro bias | 15 | One initialization, all future states |
| ADC point samples | 501×6=3006 | Trapezoid endpoints and adjacent PIM |
| Sample-grid bias increments | 500×6=3000 | Later bias samples, PIM input and terminal bias residual |
| UWB ranges | 50×8=400 | One raw measurement, original retained IDs |
| Integration regularization and biasHat auxiliary | 0 | Nominal optimization metric only |
| Extrinsics/gravity/time/anchors | 0 | Conditional fixed known parameters |

Unique full-horizon dimension is **6421**. This is a new model, not the legacy
4171-source count with its old independent auxiliary assumptions. Source IDs,
units, timestamps and scalar standard deviations are generated before evaluation
by `rawSourceInventory`; shared endpoint ADC is counted once. Initial, ADC,
bias and UWB RNG streams are separate; candidates do not resample sources.

The backend remains unchanged and its Combined covariance remains a nominal
optimization metric. The integrity reference replaces the entire auxiliary
covariance by the physical increment response: perturb increment k in all
subsequent measured bias samples **and** b_j, while conditioning on explicit b_i
and the actual biasHat. Initial-bias uncertainty is represented by the initial
state prior, not fresh independent per-step draws. The library's integration
covariance and biasHat-selection covariance have no declared physical source,
and are therefore absent from this independent reference, rather than removed
from the backend because of their trace. All 15 residual coordinates and
within-factor bias/rotation/position/velocity cross terms are retained. Disjoint
PIM intervals have independent RW increments conditional on their explicit bias
states; their ADC endpoint remains shared.

Source derivative oracle independently mutates literal samples and b_j and
reintegrates through the public episode entry at a different step size. It tests
all six axes at all ten increments and eleven ADC points; the direct raw prefix QR is compared to
online retirement on full15 mean/covariance and adjacent PIM cross. These are
frozen derivative/covariance checks; nonlinear covariance is not declared
Gaussian, and inverse nominal Hessians are not used as physical covariance.

Installed source HEAD is `46f3a48a5b1e6b57bc5da5374c74a8f5248b6fec`;
installed CombinedImuFactor header equals the local source header.

Reproduce: build `hybrid_raw_checks` and run it with `config/hybrid_raw.yaml`
using the environment above. The N2 uniform nonlinear containment below is
required in addition to these frozen derivative checks.

N1 targeted result: five actual raw commits, lag2 retirement; full15 prefix QR
covariance maximum relative difference `1.2563e-10`, adjacent ADC cross norm
maximum `8.67008e-9`; all independent increment residual checks pass, conditional
covariance SPD without jitter. The initial normal-equation oracle lost about
`7.8e-7` relative precision because of conditioning; it was replaced with the
independent direct QR, without changing source covariance or acceptance gates.

## N2 — common-source continuous inclusion reference

The local-chart route has a specific obstruction: a Gaussian primitive event
bounds actual raw inputs, but does not make the nonlinear PIM residual or an
adaptive frozen whitener Gaussian. N1 derivatives cannot prove that missing
inclusion. The implemented alternative is a **source-aware inertial containment
reference**, carried from initialization through every raw sample. It uses no
UWB-only reference, true state, estimated-error radius or convergence test.
Nominal H/P and the shared episode selector remain the actual calculation and
transaction path. Their charts, covariance approximation, prior information,
PIM bias correction, history transformations and adaptive candidate choices may
produce any finite center; the containment theorem covers every such center
uniformly. It does not claim to certify their frozen Gaussian PL.

### Coverage proposition and lemmas

For the pre-registered 50-epoch continuous motion and healthy-IMU primitive law,
all allowed source/onset/duration1-or3 UWB episodes have arbitrary amplitude.
No initial carry-in or post50 claim is made. With `issued_k` denoting successful
source/time/numerical/receipt checks, the target is

```
sup_fault P(exists k<=50: issued_k and
 (norm(actual_tag_H - true_tag_H)>HPL_k or
  abs(actual_tag_V - true_tag_V)>VPL_k)) <= 2*6421/2^35 < 1e-6.
```

1. **One event.** E bounds each of the 6421 unique standard-normal primitive
   coordinates by 7. Chernoff gives `P(|Z|>7)<=2 exp(-49/2)<2/2^35`, using
   `ln2<.7`; union over the full inventory yields the bound above. There is no
   candidate count, visited-path fee, per-epoch renewal or repeated endpoint.
2. **Initialization.** Right Pose3 EXPMAP translation obeys `||V(phi)||<=1`
   from its integral-of-rotations representation, globally. Thus the true
   initial position lies within the observation-centered translation radius;
   velocity and bias use their coordinate supports. Rotation matrix distance
   is at most `min(2,||phi||)`. No local chart branch or convergence condition
   is needed for this reference.
3. **Bias/ADC.** At sample i, true bias minus observed initial bias is bounded
   by the initial-observation error plus the sum of the SAME first i RW
   increments. Average ADC endpoint error is bounded by the same coordinate
   event, retaining shared sources rather than redrawing them. The conservative
   sum admits every correlation, including all endpoint signs.
4. **Continuous dynamics.** Let u be the average measured force minus observed
   initial bias and R0 the observed initial attitude. On an interval of length
   d, the true body force differs from u by at most bias/ADC support plus Lf*d/2.
   The true angular speed is bounded by average measured gyro, bias/ADC support
   and Lgyro*d/2. The accumulated angular-length bound gives
   `||R(t)-R0|| <= min(2, initial_rotation_bound + accumulated_length)`.
   Hence acceleration lies in `R0*u-g +/- error`, where
   `error=rotation_diameter*||u||+force_support+Lf*d/2`.
   Interval p/v updates integrate this enclosure over the **whole continuous
   interval**: `p+=d*v+d²*a/2`, `v+=d*a`. Their inclusion follows by induction,
   not corner sampling. Lf is checked against the independent motion-domain
   inequality `rate*(g+sum|amp*freq²|)+sum|amp*freq³|`; the declared constant
   angular-rate model has Lgyro=0.
5. **Numerical source relation.** A separate interval Taylor evaluator encloses
   exact exp/SO(3), V and sinusoidal source equations. It compares the emitted
   initial observation and raw inputs to their ideal source mappings. Computed
   interval residual supports enter the initial, bias and ADC envelopes. These
   are forward-generation rounding residues, not estimator GT errors. Residues
   larger than the predeclared 1e-10 admissible numerical domain refuse the law;
   no 1e-10 padding is added to covariance or PL. This prevents an injected
   physical IMU mismatch being relabeled as numerical noise. Only the proof
   translation unit has `-frounding-math -ffp-contract=off`; its runtime directed
   arithmetic witness rejects an unsuitable build.
6. **Lever and actual output.** Add R0*lever and its rotation-diameter support
   to the position interval. For each component, the served PL is the outward
   maximum distance from that interval to the actual emitted world tag. This
   holds for **all** centers, not a fixed Gaussian matrix family. Therefore
   every nonlinear PIM/history/optimizer/selector path and wrong healthy-row
   deletion is covered without assuming linearity or independence conditional
   on selection. Arbitrary UWB errors are a conservative superset of the same
   historical/current episode, including cancellation and stopped residuals.
7. **Receipt binding.** The immutable raw prefix and its shared last11 samples
   must match the actual transaction; epoch/time/full50 scope and one-update
   receipt must match the actual state. Existing ledger retained-ID checks are
   retained. Horizon escape, source mutation and forged receipts refuse.

The reference is informative but very conservative: it retains orientation
and bias uncertainty without UWB contraction. It is a finite raw IMU-derived
truth set, not the entire world. It is not an equivalent tightening of frozen
Gaussian PL, and it does not certify nominal inverse Hessians. Gaussian source
law and conditional known calibration remain mathematical simulation assumptions;
finite seed replay does not establish the probability theorem or hardware law.

N2 actual result: **RAW_BOUND_VERIFIED_FOR_DECLARED_SCOPE**, full50 (not a
selected prefix). U0/U1 each have 50/50 finite signed conditional bounds and
50 actual single-update commits. U1 epoch15 evaluates all9 candidates and
excludes source0/measurement1501, retains7; epochs16–19 each retain8 and sign
finite bounds. The event risk upper is `3.7375139072537422e-7` for the whole
50-epoch scope. The source forward-map oracle's acceleration residue is
`4.79616e-14 m/s²` on the targeted five-epoch check. Full50 source-law generation
and certification take about43–45ms once at initialization, outside epoch service.
The adversarial coordinate-boundary/RW family covers50 epochs, with horizon and
source/receipt tamper tests. Full15 source perturbations are additionally checked
at generating true factor endpoints in the independent oracle; GT enters only
that offline oracle, source generation and post-evaluation.

Important qualification boundaries: old CSV `available`, `evidence_level`,
`model_status`, `raw_total_risk` and Gaussian H/P fields retain the legacy/frozen
meaning. New `finite_bound_qualified`, `guarantee_domain`, `bound_risk_upper`,
`bound_scope`, `source_law_id`, `remainder_status`, `scope_risk_status` and
`proof_binding_status` refer to the independent continuous inclusion reference.
They must be consumed together. Formal/hardware/protected/application fields
remain false or unknown. The new claim is not raw Gaussian-error qualification
of the old PL formula. Cross-horizon, IMU faults and unverified real hardware
remain outside this conditional theorem.

## N3 — full denominators, tightness and costs

Source/model frozen at N2. Initial U0/50 and U1/50 streams were followed by
one U2/21 and targeted U3/3 and I2/20 negatives. Final numerical review found
that horizontal norm synthesis also needs directed rounding. After that output
correction, only affected qualified U0/U1/U2 streams were repeated; their final results replace
the initial outputs below. The source law and actual estimation trajectory did
not change. No confirmation seed was needed. No repeats of old S1–S4, HGIR, architecture or power campaigns. Results
were consumed from the identical streams; AL pairs never re-run estimation.

| Stream | Scheduled/actual commits | Finite numeric | Raw qualified | Rejected/unknown | Correct exclusions | <=50ms service |
|---|---:|---:|---:|---:|---:|---:|
| U0 | 50/50 | 50 | 50 | 0 | 0 | 6 |
| U1 | 50/50 | 50 | 50 | 0 | 1 | 6 |
| U2 | 21/21 | 21 | 21 | 0 | 3 | 6 |
| U3 | 3/3 | 0 | 0 | 3 geometry refusals | 0 | 3 |
| I2 | 20/20 | 20 frozen only | 0 | 20 source-law mismatches | 0 | 6 |

Healthy UWB exclusions:0 over all144 scheduled outputs. Every row has exactly
one backend update; formal/protected/application availability remains0/144.
U2 removes the persistent source at15–17;18–21 remain finite qualified after
stopping, carrying the entire source prefix. I2's source producer cannot attest
the healthy-IMU law (its actual injected20m/s² disagrees with that law), so its
numeric Gaussian outputs do not acquire raw protection. This is simulation-law
attestation, **not an online IMU fault detector**. Domain and receipt mutations,
missing history and expiry are deterministic refusals, not removed denominators.
Zero PE-over-PL in the121 qualified stream outputs is a replay check; the
analytical common event, not this sample, proves the conditional risk bound.

| Full50 | HPL mean/median/P95/max (m) | VPL mean/median/P95/max (m) | Full service mean/median/P95/max (ms) |
|---|---|---|---|
| U0 | 39.2374/32.5697/85.7064/93.7941 | 27.9136/23.3768/60.4766/66.0843 | 146.190/165.803/181.023/213.600 |
| U1 | 39.2372/32.5705/85.7062/93.7939 | 27.9139/23.3771/60.4766/66.0843 | 149.171/168.172/186.561/276.422 |

New-law precommit frozen-envelope comparison uses two targeted snapshots of the
independent check, same6421-source radius and same input/subset: epoch1 old
H/V=12.0293/21.7439 versus joint9.15162/15.4228; epoch5 old9.01915/11.8913 versus
joint6.58891/8.19793m. **Both are frozen numerical diagnostics**, with no raw
nonlinear qualification. Full50 frozen joint mean H/V is5.05301/6.02022m (U0),
5.06646/6.02071m (U1). The wider signed inertial inclusion is a different
reference/event/proof, and these numbers are not an equivalent tightening ratio.
Legacy zero-bias/fixed-initial results above belong to another model.

U1 VPL composition, all supports in metres in the same world output direction:

| epoch | initial position/velocity | ADC + numeric | initial accel bias | RW | rotation/continuous discretization | lever | actual-center shift | VPL |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 15 | 8.68718 | .06820 | .34100 | .01218 | 3.56276 | .34633 | .42333 | 13.44097 |
| 50 | 14.81218 | .75777 | 3.78886 | .44786 | 45.52306 | .47497 | .27957 | 66.08428 |

The angle/force coupling envelope dominates late in the task; initial position
and velocity dominate startup. ADC/RW contributions are not mixed-unit traces.
These are deterministic supports on the SAME event, not independent Gaussian
standard deviations. UWB fault/selection and nonlinear backend response enter
through the measured actual-center transfer; there is no UWB-only constraint
or current/history fault Gaussian term in this containment reference. Removing
that center term would invalidate arbitrary-fault coverage. The tiny interval
arithmetic width is contained by directed endpoints rather than jitter.

Diagnostic AL postprocessing on the immutable U0/U1 traces:

| AL H/V (m) | U0 numeric/qualified pass | U1 numeric/qualified pass | Qualified application |
|---|---:|---:|---:|
| 2/3 | 0/50 | 0/50 | 0/50 |
| 5/5 | 0/50 | 0/50 | 0/50 |
| 10/10 | 1/50 | 1/50 | 0/50 |

All50 epochs remain in every denominator. The trace is identical for all pairs:
state, retained IDs, PL, risk and H are unchanged. Increasing diagnostic AL
cannot qualify frozen/unknown outputs or unknown application evidence. Tables
explain the findings; no new plots or plot/data archives are needed.

Reproduce the core oracle and minimal streams (CSV automatically removed):

```sh
python3 tools/run_epoch_architecture.py check-raw --build
python3 tools/run_epoch_architecture.py raw --scene U0
python3 tools/run_epoch_architecture.py raw --scene U1
python3 tools/hybrid_raw_research.py raw-bound --scene U2
python3 tools/hybrid_raw_research.py raw-bound --scene U3
python3 tools/hybrid_raw_research.py raw-bound --scene I2
```

Full-service stages, mean/median/P95/max in ms (same continuous streams):

| Stage | U0 | U1 |
|---|---|---|
| h_ms | 142.949/162.574/178.113/211.295 | 145.613/164.616/181.225/266.746 |
| prediction_ms | 0.323166/0.243632/0.698525/0.830028 | 0.3736/0.249211/0.723283/2.22123 |
| a_ms | 0.0681262/0.0657985/0.0837066/0.112102 | 0.0788085/0.0669275/0.158996/0.240828 |
| b_ms | 0.418678/0.421931/0.675293/0.958785 | 0.500327/0.404408/1.01339/2.50053 |
| verification_ms | 0.0141295/0.0131/0.0196584/0.056698 | 0.0153114/0.0138285/0.0276479/0.03294 |
| commit_ms | 1.73099/1.49455/3.45027/3.58781 | 1.89185/1.61062/3.60548/5.46431 |
| finalization_ms | 0.053455/0.049901/0.0691927/0.107741 | 0.0527402/0.0497695/0.0698547/0.097951 |
| h_update_ms | 0.0229187/0.023155/0.035075/0.045074 | 0.0245258/0.024083/0.0374353/0.063315 |
| cleanup_ms | 0.0270082/0.020819/0.0527005/0.227456 | 0.0271562/0.0230205/0.0586081/0.080443 |
| full_ms | 146.19/165.803/181.023/213.6 | 149.171/168.172/186.561/276.422 |

Full service includes ingestion/preparation and raw interval propagation in
addition to the named stages; their remaining mean is about .583/.594ms for
U0/U1. H includes the physical source/covariance extraction. Initialization
(source generation/certification and estimator setup), CSV output and process
shutdown are outside epoch service. These are service samples, not WCET or
hardware TTA. No C calls, speed optimization or new performance campaign.

## N4 — application AL policy

`tools/application_al_policy.py` reads APPLICATION_AL_TEMPLATE and validates the
required application mode, protected point/frame and body mapping references,
hazard clearance, directional/time-aligned deterministic or probability bounds,
source references, finite-mission risk, TTA, delay/motion/safe-action evidence and
prior selection of AL. Its output never trusts `expected_validation` or a caller's
qualified flag. This version implements a conservative horizontal-ball/world-tag
budget; vertical-critical or other protected-point budgets explicitly refuse
until a corresponding directional mapping is supplied/implemented. Unknown
horizontal/vertical safety-critical declarations are not large AL defaults.

Additional binding declarations are required: `application.expected_bound_scope`,
`application.expected_source_law_id`, and each allowance's `time_horizon_s`.
`risk_time_basis` must be `specified_finite_mission`. A claimed allowance already
in PL requires its source in `pl_included_sources`; it is then not deducted
again. Unknown/different direction/time, RMSE, duplicate sources, unsupported
units, nonpositive residual budget, AL selected after evaluation, insufficient
delay allowance and TTA excess refuse. Nonlocalization event risks use a
conservative union, not independent square-sum assumptions.

`evaluate(policy,row)` is pure postprocessing: it also requires the SAME qualified
source/scope/frame/receipt, finite-mission horizon and risk plus independently
VERIFIED timeliness. Raw `on_time` service samples alone do not establish that
field. Example fixture policy succeeds only with hypothetical test references;
`.25m clearance - .15m allowances = .10m budget` is explicitly an arithmetic
example, never a project AL. Unit/direction/time/RMSE/double-count/negative-budget,
TTA and unknown-raw-risk/timeliness tests refuse as required. No estimator or
production YAML is mutated.

The actual provided template remains **APPLICATION_AL_UNQUALIFIED**. Missing
real application mode, hazard/geometry/map/control evidence, protected-point
mapping, mission risk and time basis, delay/actuation model, approved requirements,
chosen limits and proof-scope references are returned by the validator. No robot
size, speed, map accuracy, hardware response or safe AL has been invented.
Mathematical raw coverage and an application's HMI/TTA/deployment qualification
remain distinct. Scope risk concerns PE>PL; HMI also needs AL, timely warning and
verified safe action. Protected publication remains closed.

```sh
python3 tools/hybrid_raw_research.py evaluate-al
python3 tools/test_application_al_policy.py
```

The runnable application template is tracked at
`config/application_al_policy.yaml`, retaining the supplied proposal and null
project requirements, with the additional binding/time fields above. The incoming
requirements directory is Git-ignored; its original user files were not modified.
Budget sums, subtraction, risk union and delay-motion calculations round outward,
so a floating boundary cannot silently enlarge a permitted AL. The arithmetic
fixture selects .099m strictly inside the approximate .10m upper budget; neither
number is assigned to this project's configuration. Actual bound evidence must
also attest any source claimed to be already included in PL.

Final core regression: fixed H/P and both24-epoch native fixtures, joint episode
and complete fixed81-path/adaptive-direction/512-selection counterexamples,
A/B source/rank/correlated raw deletion/lever checks, summary source/nullspace/dof
family, seven transaction/PL tests and six terminal/reference/nonprotected/ABI
tests all pass. No full historical campaign was repeated. A final finite-result
review additionally rejects final-center/horizontal-norm overflow, invalid risk
budgets, missing AL and invalid service time; only directly affected boundary
checks are rerun. These guards preserve the normal source law, state trajectory and candidate
choices. Signed horizontal PL is synthesized by directed interval square/sum/
sqrt, and only that certified upper endpoint enters CSV and AL evaluation.
The final full15 source covariance oracle independently assembles all ADC/RW
columns at generating endpoints: units-normalized covariance difference
`5.02285e-8`, adjacent cross relative difference `3.23152e-9`.
A separately scheduled missing-history one-epoch raw negative makes one actual
commit but signs no qualified bound (0/1); policy postprocessing returns0/1.
Together with the main144 scheduled rows, there are145 final evaluated outputs,
121 qualified; this extra negative is separate from the full50 denominators.


## Final RG01–RG12 audit and stage status

| Requirement | Result and inspectable evidence |
|---|---|
| RG01 | Complete: actual starting HEAD/branch and installed GTSAM audit above; original backend/production contract retained. |
| RG02 | Complete: `hybrid_prediction.hpp`, `hybrid_coordinator.hpp`, `hybrid_checks`/native AL invariants and immutable trace postprocessing. |
| RG03 | Complete: `raw_source_law.hpp`, `scenario_material.hpp`, `config/hybrid_raw.yaml`; all6421 primitive sources, sample units and true RW. |
| RG04 | Complete: `gaussian_window.hpp` physical full15 source reference; auxiliary remains nominal backend metric; no jitter. |
| RG05 | Complete within derivative/conditional covariance meaning: `hybrid_raw_checks.cpp` independent full15/cross, prefix QR/retirement and tag containment oracles; no Gaussian nonlinear covariance claim. |
| RG06 | Complete for declared scope: `raw_nonlinear_inclusion.cpp` directed continuous-domain enclosure and source numeric attestation, common-event adaptive-centre theorem, receipt consumer. |
| RG07 | Complete: final U0/U1 actual50/50 each; epoch15 measurement1501 exclusion and16–19 four finite qualified outputs. |
| RG08 | Complete: fixed full50 event/horizon, healthy IMU conditional known calibration, single UWB episode scope; cross-horizon/hardware/IMU-fault protection refused. |
| RG09 | Complete: full denominators, same-law frozen snapshots separately labeled, source-to-tag composition and conditional inclusion distributions above. |
| RG10 | Complete policy implementation: `application_al_policy.py`/tests and tracked template; actual application requirements remain missing and unqualified. |
| RG11 | Complete research validation: core safety regressions and final affected numeric checks pass; full service samples measured; hardware TTA and protected publication remain unqualified. |
| RG12 | Complete: only this new report plus old report entry, code/necessary configs/tests; five local stage commits, own cache artifacts removed, no push. |

Local stage commits: N0 `e38cd94`, N1 `449a0b1`, N2 `c81c260`, N3 `e8e100d`,
N4 is the commit containing this final report and application evaluator. Each
stage proceeded without a permission pause. Final elapsed goal time is about
120 minutes, including source audit, proof implementation, builds and evaluation;
it is distinct from epoch service cost.

Remaining work outside the fulfilled conditional research claim: calibrated
hardware/initial information and IMU-fault coverage, certification across new
missions, a tighter nonlinear reference, real application geometry/control/map/
action requirements, directional mappings beyond the implemented horizontal-ball
world-tag case, and verified end-to-end TTA. The original frozen Gaussian PL
formula itself still lacks raw nonlinear qualification; the implemented signed
bound uses the independently proved inertial inclusion transferred to the actual
fusion tag. No result claims hardware, formal or production protection.
