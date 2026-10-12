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
