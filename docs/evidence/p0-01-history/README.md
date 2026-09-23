# P0-01 history raw-factor equivalence evidence

Status: implementer evidence, pending independent reviewer and supervisor commit/tag.

## Identity

- Required base golden input: `c7fd4cc79a3a22cebeb46aaa03706a0c65938f39`.
- Worktree HEAD before implementation: `55df801bf81d13e645982fdcb36bfa0055044156` (audit-table documentation only relative to the base).
- Intended commit/tag (created only by the supervisor after reviewer PASS): `fix(p0-01): restore history raw-factor equivalence` / `golden-p0-01-history`.
- Code-and-test diff SHA-256 (`git diff HEAD -- include src test`): `0d74c41bf2ae2b05f21fff4df3635dccb422edae7c6a99014edee035701f8f40`.
- The complete reviewable diff is the Git diff from the eventual P0-01 commit's first parent. No generated build product or temporary log is part of the bundle.

## Frozen failing reproduction

Before the implementation change, the new D03-A assertion was built and run alone:

```text
build_exit=0 test_exit=1
history.response.rows() = 333
supported_rows = 15
```

The reproduction is retained in `HistorySummaryPipeline.BoundaryIsTheSquareRootHistorySummary`. It requires `T_b` to have exactly the state-supported row count, `F_b` to have exactly `nu_perp` rows, and both carriers to be finite. `HistorySummaryPipeline.RowAttributionExplicitXorBoundary` independently counts the unique frozen raw rows.

The first independent review was `FAILED`. Its blocking reproduction was: UWB modes 504, accel 60, gyro 60; expected UWB×accel and UWB×gyro pairs 30240 each, but 30000 each were represented (480 total missing), giving 60624 hypotheses instead of 61104. The cause was a per-common-group local-rank gate. That gate is removed from structural enumeration: all manifest-supported physical pairs remain registered, while the global evidence path classifies numerical nullspaces fail-closed. The retained counterexamples are `LocalCollinearityDoesNotEraseGloballyIndependentPair` and `GlobalDangerousNullspaceIsPreservedAndFailsClosedNumerically`.

## O01 and O03 evidence

- `HistoryFaultSummary.P001CorrelatedRawFactorOracle` retains the independent module-level LLT/SVD identity oracle.
- `HistorySummaryPipeline.ProductionWindowMatchesIndependentFrozenRawFactorOracle` is the production O01 oracle. It begins at frozen factor slots and raw group covariance, independently creates per-(anchor,epoch) raw fault columns, and traverses `buildIntegrityWindow`. Its non-zero-RHS/correlated-covariance result was: `rows=375 old=240 boundary=165 faults=160 rhs_norm=1.508129e+00`, objective error `9.080e-10`, state `9.489e-13`, covariance `6.488e-11`, `T_b` cross `6.436e-13`, total `T_b/F_b` Gram `7.628e-15`, `F_b` Gram `1.271e-14`, constant `1.965e-16`, dof `120`.
- The pipeline checks exact row/covariance ownership, the `Ax-b` RHS path, and exact initialized `T_b/F_b` dimensions. With `fixed_lag_epochs=21`, epochs 22/24/26 observe real backend marginalization counts 1/3/5 and oldest-retained epochs 1/3/5; each crossing repeats the ownership/carrier check.
- `WindowAndHistoryColumnsAgreeOnSharedMaterial` constructs persistent/ramp/IMU coefficients term by term from physical epoch times and carrier basis, then compares every map. Observed: `expected=624 represented=624 evaluated=624 history_mapped=316 termwise_worst=1.940e-20`.
- Expected mode and order-2 pair identities are enumerated from frozen transaction material, manifest/config flags, and the declared physical horizon, independently of `history.column_ids` and the generated registry. The 26-epoch joint-order2 probe observed UWB 504, accel 60, gyro 60; pair expected/represented/evaluated/terminal all 60480; UWB×accel 30240, UWB×gyro 30240; total hypotheses 61104.
- The generator fails closed on any expected→represented→evaluated census mismatch. Fault families, hypotheses and actions were not removed.

## Commands and results

```text
cmake -S . -B /tmp/uwb-p001-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uwb-p001-build --target tests -j2
exit: 0

directed O01/O03/census/counterexamples: exit 0 for all four binaries

cd /tmp/uwb-p001-build
LD_PRELOAD=/tmp/uwb-p001-build/devel/lib/libuwb_imu_pl.so \
  ctest --output-on-failure -j2
exit: 0; 26/26 CTest targets passed; 0 failed; 9.27 s
```

The complete CTest exit code, rather than `catkin_test_results`, is the acceptance result. `LD_PRELOAD` is required only because this isolated build's catkin `env_cached.sh` prepends the older overlay `/home/mint/ws_fusion_uwb/devel/lib` ahead of its own RUNPATH. A diagnostic run without the preload loaded that stale library and exited 8 (2/26 wrapper failures); the two affected targets passed 2/2 under the isolated artifact before the final 26/26 run. This harness failure is not counted as a code pass. The former v5-manifest/v6-CSV mismatch is removed by making the round-two validator fixture use the repository's v6 joint-order2 configuration.
The complete target list is retained in `ctest-results.txt`; directed numeric/census output is in `directed-results.txt`.

## Environment and hashes

- Linux `6.18.33.2-microsoft-standard-WSL2`, x86_64, 18 logical CPUs.
- GCC `9.4.0`; CMake `3.16.3`; build workers `2`; test workers `2`.
- Main library: `86d3c3af00596d241f22ee55e08598e71ab8dfe2ca8686ab95e64eef8b09d87f`.
- Module O01 binary: `d370bf7d0f3a336ef3d6c938d2c3f0255fb56332bd0142b2908c5488cdae68fd`.
- O03/census binary: `791aa94f454d08f8252fdaa656cfb392ae16072d6ea232a8a001450c05e5fb82`.
- Production O01/pipeline binary: `53215cb7688af19819fd91762bc458976f7d00fc524ac0a6e81942fc3ba24d6d`.
- Counterexample binary: `a445abeda21eed81ffe5d35e6d07a5652794f512ccbf833c6d369952a8d802a5`.
- GTSAM: `00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da`.
- GTSAM unstable: `7e54a7f4603b30b755f9f0036034e19c9f753c985c82af83c1551b23a9358c23`.
- TBB: `26115b1a00c49e84d314e3f75eab0c752a2dfb20a61c60897f5656b1db239cba`.
- libstdc++: `26916e7389a561537a0510661fc75538cea236139346f3dbe21bc64ba65605bf`.
- Boost filesystem: `e1ba3a5cf611782b0dd5d7981edd06504edcb867482d2929213ae655a98d0ae2`.
- yaml-cpp: `b8a2d129e9b39d03ad7dade4eaa4eb1327a2562e203c17b50bcd8eb117d1ed7e`.
- metis-gtsam: `0a9c286ec13c815313fbe004d9969c3ae0ff08523f38efb575ca785e98f24d2b`.
- Research configuration: `aa38f4a6ced4ea0a5c07191c5f7392816c77c3edae5c47f2cdae96148fc9294a`.
- v6 joint-order2 configuration: `fc0a453f9fc209a6c28c5176c84b5d0f936e08b189367054cd15d314da0b7ff8`.
- Fault manifest: `270a11fdb319b8beb8a535bc44550df00df499286e61bc21fd42a171d7171991`.
- Round-two protocol: `2ddd6412f21efc5954d871db751859c1736e69ae42371afa7c024766ebf6c7d0`.
- O01/O03 inputs and truth are deterministic fixtures embedded in the hashed test sources; there is no external truth file.

## Explicit NOT_RUN

- Sanitizers/MSan: `NOT_RUN` (not required by the P0-01 row and no sanitizer build was available).
- Hardware/ROS sensor replay: `NOT_RUN` (P0-01 uses deterministic raw-factor and estimator pipeline oracles).
- Performance comparison: `NOT_RUN` (P0 correctness step; before/after performance is forbidden as a substitute for equivalence).
- Long all-attempt campaign, risk ledger and publication journal: `NOT_RUN` (outside P0-01; no claim is made for later P0 rows).

No detector threshold, alert limit, total risk, prior, sensor, fault family, hypothesis/action coverage, or statistical denominator was reduced or changed.
