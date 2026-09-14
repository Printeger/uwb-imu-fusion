# Runner linearization contract update after initializer change

Date: 2026-09-14  
Branch: `feature/uwb-imu-fusion-ie-postprocessing`  
Decision: `EXPECTED_INITIALIZER_LINEARIZATION_MIGRATION`

## Outcome

The old `a029fb...` hash was not blindly replaced. The current runner's exact
physical graph, evaluated at the deterministic pre-common-initializer Values,
reproduces both historical identities bit-for-bit:

- reference Values: `t08values-sha256:51a986505c24566054d530e575f200abfa24bb785aa1d21edee712c5c6bc46bd`;
- graph residual/Jacobian identity at that reference:
  `t08graphlin-sha256:a029fb09830efd4d17d24900cc594e25bdef7c317b5dea78602af2ccb85709e7`.

The common initializer changes only the Initial Values/linearization point:

- current Values: `t08values-sha256:1a6cb7d84db69d55b7533fe0bb1c087142981730f09fbacb51cbfa6efa5b8b76`;
- current graph residual/Jacobian identity:
  `t08graphlin-sha256:cd46d08fca3a104e35fae843fbca68288dfa2e07e07039c6e6f071fa28d6cced`.

Therefore the old flat assertion conflated graph semantics and the initializer
linearization point. The physical graph did not regress; the old current-point
linearization expectation was stale after the intentional common-initializer
redesign.

## Exact T04 audit

The preserved pre-redesign T04 artifacts and the current T04 runner were
compared. `observations.csv` is byte-identical (SHA-256
`4152fceb933f48b76e9434e9f423600aa434494059115d147d55b185c1837d3a`) and
the complete PIM covariance artifact is byte-identical (SHA-256
`7d897f322e578d5481d3e2fb00929d1bdec67d01f0111daacea5e92dd7d4fd65`).
Temporary config-file hashes differ because the test creates a new absolute
config/support path; those path identities are not physical graph inputs.

| Required semantic input | Old versus current evidence |
|---|---|
| 1. Raw observation IDs | Identical 627-row ledger; frozen sequence hash `sha256:f33599b958b25e81cabd1cd57536cd02c1ac01022aec67122b47edacfb61130d` |
| 2. Selected estimator IDs | Identical 64 representatives; `sha256:16bc33eb5af139d3a7b91ac08fc37f5b5e018df3824bb538d0545af17a93d7cb` |
| 3. State timestamps/count | 8 states; timeline `sha256:58371f82a10240c8c10ff3f4974c6702ea68da65a2e5c2aa215c8342fb2d296e` |
| 4. Factor count | 74 in both, including 64 UWB and 7 PIM factors |
| 5. Factor type sequence | `sha256:1a0c7033e7e0c2467612e6a565f7b3d45ee0befa9b74a563805e3565dabb8378` |
| 6. Factor key sequence | `sha256:c954453c8e090a25828d9e931e6ba99068707a21abf0ca8d477f94a37bea9be8` |
| 7. UWB measurements | Covered by byte-identical ledger and combined UWB physical hash below |
| 8. UWB sensor sigma | All 64 selected rows remain exactly `0.1 m` |
| 9. Anchor coordinates | `sha256:7c1b58312ea72d10580b00ad735885c74f0dac2f2ecdac0fb029de68dc6848dd` |
| 10. Lever arm | `sha256:1751173c6b6189f0503d46be5ebdd1e388d97d8ee2c76b585c457af3f1aa6825` |
| 11. IMU/PIM intervals | 7 intervals; `sha256:047a70b8a27ca08d7fcbed9a01e33ef29485d194ca5d8424bc972aff74d96232` |
| 12. Physical topology | `sha256:36f26aff489a4a09aefbcdf8b617cabf6dd87bb1c894437698f57992a72cea2a` |
| 13. Initial Values | Historical reference remains `51a986...`; current common initializer is intentionally `1a6cb7...` |
| 14. Residual/Jacobian values | Same graph at historical reference exactly reproduces `a029fb...`; current initializer point gives `cd46d0...` |

The combined UWB physical-semantics fingerprint, which binds each selected
`obs_id`, keyframe, tag/anchor, raw range, sensor sigma, anchor coordinates and
lever arm, is
`sha256:f4e373840d075c04611ef55540d012a6dfcc88fd634f074db3398eb0d16532a6`.
The seven state/PIM durations are
`0.5000400543212891`, `0.49994897842407227`, `0.5000214576721191`,
`0.4999973773956299`, `0.4999504089355469`, `0.5014615058898926`, and
`0.4985623359680176` seconds. The graph-linearization hash itself retains the
stronger per-factor runtime type, keys, factor error, linearized factor type and
augmented Jacobian at the reference Values.

## T06/T08 equivalence

The preserved T04/T06/T08 historical artifacts have the same observation and
PIM artifact hashes, and all three carry the same old `51a986...`/`a029fb...`
identities. After migration all three pass the same shared strict helper, which
checks the complete physical identity dictionary above before checking the new
`1a6cb7...`/`cd46d0...` initialization identity. No factor-level assertion was
removed or relaxed.

## Contract migration

`common_preparation.json` is now schema `uifgo_t09_common_preparation_v2` and
separates:

- `PHYSICAL_GRAPH_AT_PRE_COMMON_REFERENCE_VALUES_V1`: stable physical graph
  fingerprint at the deterministic GraphBuilder reference Values;
- `COMMON_INITIALIZER_OUTPUT_LINEARIZATION_V1`: current Initial Values and the
  graph residual/Jacobian fingerprint at those Values.

The legacy flat hash remains preserved in historical reports and as the current
physical-reference assertion. `CommonPreparationIdentityInput` and FDE context
now bind `physical_graph_sha256` to the physical-reference identity instead of
the initializer-dependent identity. This intentionally invalidates stale
initializer-conflated cache identities; estimator mathematics and factors are
unchanged.

## Validation

| Gate | Result |
|---|---|
| T04/T06/T08 runner contracts | PASS, 3/3 |
| Initializer | PASS, 13/13 |
| Core regression | PASS, 516 tests, 0 failures |
| Gate05 architecture guard | PASS |
| Full CTest | PASS, 36/36 |

Commands included the focused CTest regex, direct initializer and architecture
executions, `catkin test uwb_imu_fgo --no-deps --summarize`, and full
`ctest --output-on-failure` from `build/uwb_imu_fgo`.

## Authorized Walk1 continuation

The locked initialization-only run built 913 states, 3,191 factors and 2,276
UWB factors. It accepted all 912 prefixes with zero initialization failures and
passed `INITIALIZATION_SEED_QUALITY_V1`. Evidence is under
`experiments/icra2027/dev/RUNNER_LINEARIZATION_CONTRACT_MIGRATION/locked_walk1_initialization/`.

The unchanged Cauchy-2.3849 Base FGO then obtained raw
`CONDITIONAL_LM_CONVERGED`, but `PAPER_SOLVER_CERTIFICATE_V1` rejected it with
`NAVIGATION_STATIONARITY_FAILED:NOT_STATIONARY`; maximum scaled navigation
gradient was `6.3016170371536822`. No trajectory was exported.

The first CLI invocation omitted the runner's mandatory explicit
`UIFGO_T09_ROBUST_SCALE` environment field and exited at baseline preflight
with `robust baseline requires explicit UIFGO_T09_ROBUST_SCALE`. It did not
construct a robust baseline solve and is not counted as an estimator attempt.
The recorded Cauchy result above is from the corrected invocation with the
already frozen value `2.3849`; no value was selected or tuned from output.

The single authorized Huber-1.345 stage likewise obtained raw convergence but
was rejected by the certificate with maximum scaled gradient
`3.6007261482860535`. The fail-closed baseline runner intentionally does not
export uncertified Huber Values. Consequently there was no valid warm-start
handoff for the final unchanged-Cauchy stage; proceeding would require a new
intermediate Values export/consume mechanism or a second Huber execution.
Neither was added. This is the first genuinely new structural workflow blocker,
not a physical-graph regression.

The Base FGO and Huber-stage artifacts are isolated under
`experiments/icra2027/dev/RUNNER_LINEARIZATION_CONTRACT_MIGRATION/gate06/`
(`cauchy_base_v2/` and `huber_warm_stage/`). The earlier preflight-only CLI
artifact is retained separately as `cauchy_base/`.

GT evaluation was not run because no Solver Certificate succeeded. No
parameter, fixture, lag, sensor sigma, Huber/Cauchy scale, LM setting,
certificate threshold, final estimator or IE mathematics was tuned.

Final hash verdict: `EXPECTED_INITIALIZER_LINEARIZATION_MIGRATION`.
