# P0-02 dual-channel contract evidence

Status: implementer evidence, pending independent reviewer and supervisor commit/tag.

## Identity

- Required immutable base: `golden-p0-01-history^{}` = `e73bb81f54ab3d4b051a924077f222c1157f93b7`.
- Worktree HEAD before and after implementation: `e73bb81f54ab3d4b051a924077f222c1157f93b7` (the implementer did not commit or tag).
- Initial status contained only the user-owned untracked audit report and `doc/review/`; both remain preserved. The P0-02 evidence directory is the only additional untracked path.
- Intended supervisor commit/tag after reviewer PASS: `fix(p0-02): enforce dual-channel candidate contract` / `golden-p0-02-dual-channel`.
- Code-and-test diff SHA-256 (`git diff --binary HEAD -- include src test`): `b4d5ddf27ad86641c7f1e5f57812afea502411a8bc8e93378c876465b39bd6c7`.
- The complete reviewable diff is the eventual P0-02 commit against its first parent. Generated build products and temporary logs are not part of the bundle.

## Frozen failing reproduction

The three initial O02 reproductions were added before the implementation and run as:

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target test_integrity_v2 -j2
/home/mint/ws_fusion_uwb/devel/lib/uwb_imu_pl/test_integrity_v2 \
  --gtest_filter='P002DualChannelContract.*'
build_exit=0; test_exit=1; 0/3 passed
```

Observed failures:

- matrix-free KEEP_ALL had no channel payload and silently retained pooled `passed=true` for the pooled-pass/dual-fail fixture;
- dense candidate splitting inferred history from zero Jacobian rows but omitted the nonzero history constant;
- a valid-looking candidate with no detector payload produced `numerically_valid=true`, `passed=true` and was eligible to enter the finite-PL calculation.

The retained fixtures use the unchanged `p_fa=1e-6`. The history statistic is 25 with one degree of freedom, above `chi2_1=23.928...`, while the pooled statistic is 25 with three degrees of freedom, below its threshold. A separate fixture obtains the same history statistic entirely from `constant_offset=25`.

## First reviewer FAILED and repair

The first independent reviewer returned `FAILED` after confirming that the normal path counted the history constant once, did not silently pool active-v6 candidates, and was connected to production. The blocking counterexamples were that the certificate identity could not be recomputed by consumers from the actual window/action/candidate; a whole certificate plus detector result could be transplanted; threshold/P_FA/continuity-union fields were not rebound in PL; and the initial agreement test shared the production certificate builder instead of providing an independent raw-row oracle.

The repair retains that verdict and adds:

- a consumer-recomputed digest over the frozen-window and numerical-contract fingerprints, complete canonical action and added-block content, actual candidate numerical/covariance representation, every row role, channel statistics, constant, rows/rank/dof and accepted event;
- detector and PL reconstruction against the actual supplied window/action/candidate, plus a detector-contract digest and exact threshold/P_FA/channel-count/continuity union checks;
- an independent retained-row QR, residual split, Gram and PL oracle which calls neither `buildCandidateDetectorCertificate` nor the production dual-channel bound (it uses Boost distributions and an independent small-matrix pseudoinverse);
- fail-closed stale/transplanted-candidate, equal-count row-role swap, threshold, P_FA, union bound, channel count, horizon, statistic, dof, constant, event, numerical identity and digest mutations.

## Second reviewer FAILED and isolated repair

The second independent reviewer confirmed the core implementation but returned
`FAILED` because the detector-field mutations retained a stale outer digest.
Those tests proved digest rejection, but did not independently prove each
consumer semantic guard. The retained repair mutates exactly one detector
semantic field at a time and then recomputes `detectorContractDigest` before
calling PL. Current/history thresholds and statistics, pooled statistic and
threshold, current/history/pooled dof, rows/rank, per-test P_FA, operation bound,
channel count, continuity horizon, history constant, accepted event and
candidate identity all pass the outer digest check and are still rejected as
`INVALID` with non-finite PL. A separate case changes only the stored detector
digest and proves the outer digest guard.

Candidate-payload isolation also changes one `state_increment` value, one dense
covariance value, the actual matrix-free shared-base covariance factor, and the
matrix-free low-rank correction while retaining the old certificate. Each is
rejected by the detector and cannot produce finite certified PL. The fixture
explicitly verifies that this matrix-free candidate uses the
shared-base/correction representation rather than a reference-vector fallback.

## O02 result

`CandidateDetectorCertificate` is created by both dense and matrix-free candidate kernels and carries explicit row roles, current/history statistics, history constant presence/value, current/history row/rank/dof, pooled rank/dof, the accepted-event ID `dual_channel_intersection_v6`, frozen-window/action/numerical identities and the consumer-recomputable certificate digest. Post detection rejects a missing, stale, transplanted or incomplete active-v6 certificate without evaluating pooled acceptance.

The reported operation false-alarm bound applies the two-channel union bound for active-v6 all-in and candidate decisions; the per-channel threshold input remains unchanged.

PL requires the detector certificate's event, numerical identity and constant to match the candidate. Its history Gram is selected from certificate row roles, not reconstructed from optional dense Jacobian fields. `LegacyPooledOffline` remains an explicit enum choice for offline callers; lack of an active-v6 split never selects it implicitly. The active frozen pooled PL shortcut fails closed when a history channel is present.

The directed O02 oracle passed 7/7:

- pooled-pass/dual-fail rejects in dense and matrix-free paths;
- nonzero history constant rejects in dense and matrix-free paths;
- missing payload is `INVALID` and cannot produce finite certified PL;
- all-in, KEEP_ALL and a modified exclusion agree on statistics, dof, accepted event, dense/matrix-free numerical identity and finite PL contract;
- missing history constant rejects.
- the independent raw retained-row QR/Gram/PL oracle agrees for all-in, KEEP_ALL, modified, dense and matrix-free paths;
- every stale/swapped/tampered certificate or detector field listed above is invalid and cannot produce finite certified PL; detector semantic cases recompute their digest first, while a distinct digest-only case exercises the outer guard.

The existing independent dual-channel detector suite also passed 5/5, including the production-window split across epochs 18--22.

## Commands and exit codes

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl -j2
exit: 0

cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target clean
exit: 0

cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target tests -j2
exit: 0

/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_v2 \
  --gtest_filter='P002DualChannelContract.*'
exit: 0; 7/7 passed

/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_dual_channel_detector
exit: 0; 5/5 passed

cd /home/mint/ws_fusion_uwb/build/uwb_imu_pl
ctest --output-on-failure -j2
exit: 0; 26/26 CTest targets passed; 0 failed; 6.63 s
```

An earlier complete-CTest attempt after rebuilding only `test_integrity_v2` exited 8 with 23/26 targets passing: three older test executables still had the pre-change public-struct ABI and failed/segfaulted. This result was not counted as acceptance. Rebuilding the complete `tests` target (exit 0) and rerunning the same CTest command produced the recorded 26/26 pass.

## Environment and hashes

- Linux `6.18.33.2-microsoft-standard-WSL2`, x86_64, 18 logical CPUs.
- GCC `9.4.0`; CMake `3.16.3`; build workers `2`; test workers `2`.
- Main library: `f264a680e26939e51f31ec37c253eb3eaf21a8cba5c9169925873124d6d257bd`.
- O02 binary: `c31273dbda13a2e3aabdb2eb79c00d6bf757f3b579b4fc3b9117c8e1666285c5`.
- Dual-channel binary: `138264eebae90c02475de2078e7e8218589857e9d26552ff76b9569b72230280`.
- GTSAM: `00d83aa618194aefc4b2011e1d29bd9aba107a8b5e0ee8e946eb2455351219da`.
- GTSAM unstable: `7e54a7f4603b30b755f9f0036034e19c9f753c985c82af83c1551b23a9358c23`.
- TBB: `26115b1a00c49e84d314e3f75eab0c752a2dfb20a61c60897f5656b1db239cba`.
- TBB malloc: `14147fcfadccacb4ddf94738143a3fa999a065a303dda898da8ab057cfb483b4`.
- libstdc++: `26916e7389a561537a0510661fc75538cea236139346f3dbe21bc64ba65605bf`.
- Boost filesystem: `e1ba3a5cf611782b0dd5d7981edd06504edcb867482d2929213ae655a98d0ae2`.
- yaml-cpp: `b8a2d129e9b39d03ad7dade4eaa4eb1327a2562e203c17b50bcd8eb117d1ed7e`.
- metis-gtsam: `0a9c286ec13c815313fbe004d9969c3ae0ff08523f38efb575ca785e98f24d2b`.
- Research configuration: `aa38f4a6ced4ea0a5c07191c5f7392816c77c3edae5c47f2cdae96148fc9294a`.
- Fault manifest: `270a11fdb319b8beb8a535bc44550df00df499286e61bc21fd42a171d7171991`.
- O02 inputs and expected decisions are deterministic fixtures embedded in the hashed test source; there is no external truth file.

## Explicit NOT_RUN

- Sanitizers/TSan/MSan: `NOT_RUN` (not required by the P0-02 row; no sanitizer build was available).
- Hardware/ROS sensor replay: `NOT_RUN` (O02 is a deterministic detector/PL wiring oracle).
- Performance comparison: `NOT_RUN` (P0 correctness step; no performance claim is made).
- Long all-attempt campaign and deployment calibration: `NOT_RUN` (outside P0-02; no claim is made for P0-03 through P0-07).

No detector threshold, alert limit, total risk, prior, sensor, fault family, hypothesis/action coverage, statistical denominator, exact fallback or timeout policy was changed.
