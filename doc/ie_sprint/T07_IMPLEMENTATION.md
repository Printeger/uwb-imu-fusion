# T07 deterministic scenario/cache interface

Status: `DONE/REVIEW_ACCEPTED_DEVELOPMENT_ENGINEERING_SCOPE`.
This is development/test infrastructure, not a formal RQ result and not support
for a paper claim.

## Generator boundary

`generate_t07_scenario` is the only executable that reads all three generator
inputs: a full original base config/bag, a strict `t07_scenario_v1` recipe, and
the base-component audit. It reuses `DataLoader`; it does not simulate motion or
dynamics. Full-base generation fixes the recording origin and source ordinals
before injection/cropping, requires every event to match at least one raw
observation, and enforces optional fixture-only expected counts.

T07 v1 injection is deterministic. `scenario_seed` is mandatory provenance
context but has no stochastic effect. Events use canonical `tag_id:anchor_id`,
half-open `[start_s,end_s)` sensor-time intervals, nonnegative step or ramp
components, and reject same-link overlap. Output range conservation is
`observed_range_m = base_range_m + injected_bias_delta_m`.

## Estimator cache

The self-contained estimator directory is
`<cache-root>/<cache-id-hex>/` and contains only:

- `input_manifest.json`
- `imu.csv`
- `uwb_observations.csv`

The strict manifest locks schema `t07_estimator_cache_v1`, base recording ID,
base source SHA-256, recording sensor-time origin, normalized IMU units
(`m/s^2`, `rad/s`, quaternion wxyz), UWB units (`s`, `m`, `dBm`), source-message
grouping, payload names/counts, and payload SHA-256 values. The cache ID is
recomputed from those locked fields and payload hashes without a recipe. The
reader rejects unknown fields, alternate payload names, payload/hash/count/ID
mismatch, duplicate message groups/obs IDs, or noncanonical source order. The
canonical UWB order is nondecreasing sensor time with strictly increasing
inherited source-message ordinal as the deterministic tie-break; every group is
contiguous and unique, range ordinals are exactly `0..n-1`, and inherited
source-observation ordinals are strictly increasing. IMU rows retain their
writer-assigned source row ordinals `0..n-1` and nondecreasing sensor time. The
reader validates these invariants after hash/cache-ID verification and performs
no IMU unit conversion. Reordering complete groups or IMU rows remains invalid
even if an attacker recomputes both payload SHA-256 and cache ID.

The paper runner receives the inherited base recording ID and the stored full
source message/range ordinals. `BuildPaperInputPlan` recomputes and verifies each
stored `obs_id`; it never derives identity from the cropped cache position.
Cache start/duration are applied to IMU and UWB sensor times relative to the
inherited origin before initialization.

## Evaluation-only sidecar and isolation

`<truth-root>/<cache-id-hex>/` is disjoint and contains
`truth_manifest.json` plus `injected_component_truth.csv`. It labels truth as
`INJECTED_COMPONENT_ONLY` and the total latent bias as `UNKNOWN`. The cache and
paper-runner config contain no recipe, truth, or scripted support path. The
generator validates the actual base hash/origin/recording identity and the
fixture counts; the estimator validates only its cache.

Before writing, the generator resolves each root through its nearest existing
ancestor and then canonicalizes the created roots. It rejects equal physical
paths, symlink aliases, and either-direction ancestor/descendant relations for
both roots and final cache-ID directories. Cache and truth payloads are first
written to owned staging directories under their respective roots, checked for
their exact file sets, then renamed. Any failed write/commit removes owned
staging and any partial final directory while preserving no-overwrite
semantics. A successful estimator cache therefore has exactly the three files
listed above and no truth payload.

The dual-directory staging/cleanup guarantee covers catchable exceptions. It
does not claim crash atomicity across both publications under `SIGKILL`, process
death, or power loss; this is a retained non-blocking engineering limitation.

The isolation smoke physically moves the base bag, recipe, and corresponding
truth directory while invoking the real paper runner, polls that all three
paths remain absent for the process lifetime, then restores and checks their
device/inode/mode/size identity in a `finally` block. It verifies the actual
cache ID, inherited recording ID, `input_interface=t07_cache`, and
`cache_unit_conversion_applied=false`. Nonempty discovery trace plus matching
Stage-1 diagnostics—not early `input_manifest.json` creation—prove Stage 1 ran.
Both step and ramp caches use the unchanged T06 development parameters. Their
actual `MAX_OUTER_ITERATIONS` failures and
`recoverability_score=NOT_RUN` state are retained; no parameter was changed to
manufacture success.

## Evidence boundary

The frozen fixture uses base SHA-256
`038e8158c07da81d684e39d97f79d053c0c6603f48cad922e583e8096d1ba991`,
recording ID `content-fnv1a64:56d97b371751f1cb`, origin
`1781510676.6478176`, and event counts `1:1=79`, `1:2=77`. Those counts apply
only to this development fixture. The source/config and RSSI/residual audit is
empirical consistency, not generation provenance: the runtime parameter
snapshot and realized random seed are unavailable, so no total latent-bias
recovery metric is defined.

Initial local evidence is in
[`evidence/t07_20260907T053301Z/`](evidence/t07_20260907T053301Z/); isolated
T07-R01--R03 review-fix evidence is in
[`evidence/t07_review_fix_20260907T062126Z/`](evidence/t07_review_fix_20260907T062126Z/VERIFICATION.md).
The accepted independent-review material and final closeout checks are archived
in
[`evidence/t07_final_review_20260907T070102Z/`](evidence/t07_final_review_20260907T070102Z/VERIFICATION.md).
T08,
gate/fallback/final covariance, formal RQ runs, and claim support remain out of
scope.
