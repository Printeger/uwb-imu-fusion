# HYBRID raw qualification

Research implementation in progress. Production qualification and protected
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
The auxiliary trace diagnostic is not a removal justification. The new source law
and physical reference are still being implemented; no raw coverage is claimed.

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
all six axes at first/last increments; the direct raw prefix QR is compared to
online retirement on full15 mean/covariance and adjacent PIM cross. These are
frozen derivative/covariance checks; nonlinear covariance is not declared
Gaussian, and inverse nominal Hessians are not used as physical covariance.

Installed source HEAD is `46f3a48a5b1e6b57bc5da5374c74a8f5248b6fec`;
installed CombinedImuFactor header equals the local source header.

Reproduce: build `hybrid_raw_checks` and run it with `config/hybrid_raw.yaml`
using the environment above. N2 must still establish uniform nonlinear
containment before any new raw qualification is issued.

N1 targeted result: five actual raw commits, lag2 retirement; full15 prefix QR
covariance maximum relative difference `6.12865e-11`, adjacent ADC cross norm
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
