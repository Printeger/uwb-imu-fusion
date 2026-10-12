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
