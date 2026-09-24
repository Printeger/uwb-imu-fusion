# P0-05 transaction, reference, and final-packet evidence

Status: `GOLDEN`; the eighth independent reviewer and supervisor both `PASS`.
All seven independent `FAILED` reviews are retained
in `reviewer-first-failed.md`, `reviewer-second-failed.md`, and
`reviewer-third-failed.md` plus `reviewer-fourth-failed.md`; every repair attempt is retained in
`attempt-journal.tsv`. The fifth evidence-only and sixth low-frequency O08
failures are retained in `reviewer-fifth-failed.md` and
`reviewer-sixth-failed.md`; the raw-covariance and exact-hit proof gaps are
retained in `reviewer-seventh-failed.md`.

## Frozen identity and scope

- Base and current uncommitted `HEAD`: annotated `golden-p0-04-risk^{}` =
  `d2e848fa0fe739357681f44ea456d01d560bdc76`.
- Scope is only P0-05 / R05 / D07-A / D07-B / D11-A / D11-B /
  O04 / O08 / O10. The master verification table was not edited.
- The user-owned untracked Chinese audit and `doc/review/` were read and not
  modified or included in this evidence bundle.
- No detector threshold, alert limit, risk total/allocation, production prior,
  fault family, hypothesis/action coverage, denominator, or timeout definition
  was reduced or relaxed. The formal qualification gate remains unchanged.
  The sixth repair adds only explicit test-fixture state priors to make the O08
  precondition deterministically full rank. The seventh repair adds only a
  pre-symmetrization covariance assertion and exact one-shot fault-hit identity.

## Complete implementation diff and source identity

`complete-diff.patch` is the complete binary-capable P0-05 implementation diff
against `golden-p0-04-risk^{}`. It contains every current implementation,
message, build, tool, and test change under `CMakeLists.txt`, `apps/`,
`include/`, `msg/`, `src/`, `test/`, and `tools/`. It deliberately excludes
this evidence directory, the unchanged master table, and the user-owned
untracked `doc/` inputs, so it cannot recursively contain itself.

It was generated from repository root with:

```bash
tracked=$(git diff --name-only golden-p0-04-risk -- CMakeLists.txt apps include msg src test tools)
git diff --binary golden-p0-04-risk -- ${tracked} > docs/evidence/p0-05-publication/complete-diff.patch
for f in include/uwb_imu_pl/publication/final_output_packet.hpp \
         src/uwb_imu_pl/publication/final_output_packet.cpp \
         test/p005_golden_abi_client.cpp \
         test/p005_ros_certified_publisher.cpp \
         test/test_p0_05_publication.cpp; do
  git diff --binary --no-index /dev/null "$f" >> \
    docs/evidence/p0-05-publication/complete-diff.patch || test $? -eq 1
done
```

At generation time `HEAD` and the peeled base were both
`d2e848fa0fe739357681f44ea456d01d560bdc76`; the implementation was
uncommitted and the evidence directory plus the two user `doc/` inputs were
untracked. `source-hashes.sha256` binds the patch and every implementation file
listed by it. The manifest intentionally excludes itself. The patch is checked
with `git apply --reverse --check` against this exact worktree.

## Retained ROS capture inputs and truth

`artifacts/ros-simulator/` retains the actual `resolved_config.yaml` and
`run_manifest.json` from `/tmp/p005_ros_repro`, whose `integrity.csv` produced
the checked-in `ros-integrity-committed.csv`. It also retains the resolved fault
manifest, invoked launch source as the byte-identical
`realtime_integrity_sim.launch.txt`, and generated `ground_truth.csv` and
`fault_truth.csv`. `artifact-hashes.sha256` verifies all of them using stable
repository-relative paths. The `.txt` suffix is deliberate: retaining a second
`.launch` inside the ROS package makes `roslaunch uwb_imu_pl
realtime_integrity_sim.launch` ambiguous even when the bytes are identical.

There was no prerecorded simulator input file: the launch generated live ROS
IMU/UWB messages. The controlled certified publisher likewise used no YAML,
run manifest, or replay input; its deterministic frozen factors are in the
hashed source `test/p005_ros_certified_publisher.cpp`. These `N/A` items and the
physical-sensor `NOT_RUN` item are explicit in `artifact-status.tsv`; none is
claimed as hashed input.

## Red reproduction and first review

The new P0-05 target was added before the implementation. Its first build
failed because `uwb_imu_pl/publication/final_output_packet.hpp` did not exist.
After the first implementation pass, the directed target reported 3/6 failed;
the failures exposed incorrect absolute update-count and exact floating-point
assertions in the independent harness. After correcting those harness errors,
the production contracts were exercised directly. The first complete CTest
run additionally caught a real invalid-reference case: a non-finite PL had
been marked bindable. The fix leaves such a reference unbound and publishes an
explicit unavailable result rather than throwing before commit.

See `first-failing-results.txt` and `reviewer-first-failed.md`. In particular,
the first review correctly rejected an apparently green run that removed the
round-2 executable, a fabricated proof id that was still described as bound,
incomplete O08 snapshots, pre-callback deadline sampling, self-referential O04
checks, missing raw ROS evidence, and public-structure ABI drift. Those results
remain part of the record; they are not presented as acceptance evidence.

## Contract repair

- Commit certification is now a versioned sidecar. It must be minted from an
  extant P0-03 candidate certificate, its exact frozen transaction/window, and
  its registered protection proof, and is consumed exactly once before backend
  mutation. Candidate/reference mean is derived from the frozen nominal state,
  protected-state map, and candidate increment; it is never accepted from a
  caller field. Missing, fake, stale, replayed, tampered, mismatched, infinite,
  and overflowing evidence fail before mutation. The golden `commitEpoch` ABI
  remains available but explicitly returns an unprotected receipt.
- The backend mutation guard is armed before the update and remains active
  through state query, marginal query, factor-ledger and slot binding,
  metadata/history pruning, complete receipt construction, and staged metadata
  publication. Any post-mutation exception clears the pending transaction,
  poisons the backend, and throws a terminal receipt explicitly marked
  committed-unprotected. It cannot be discarded or reused.
- A mature multi-epoch fixture first exercises real fixed-lag pruning and a
  committed kinematic bridge. Every one of the eight post-update fault points
  then independently snapshots and
  compares every ledger field/lifecycle/ID, slot and prune identity, complete
  committed catalogs, full pose/position/velocity/bias and covariance,
  state/covariance histories, every IMU sample/boundary/cursor, every health
  record and bridge audit,
  graph/order/linpoint/epoch versions, active transaction, pending flag,
  backend-update count, and poison state. Each produces one terminal receipt.
  Its independent raw 15D covariance must be finite and satisfy the documented
  `1e-12 * max(1, ||C||_F)` antisymmetry bound before symmetrization, then be
  strictly positive definite. Every injection uses a distinct monotonic nonce;
  exact point/nonce/hit/consumed identity must agree in both terminal receipt
  and out-of-object sidecar before that point enters the exact-eight reached set.
- Ledger, history, factor slots, graph/order/linpoint versions, current state,
  covariance, IMU cursor, bridge counters, and health state are staged and
  published only with a completed receipt. Validation and container copies are
  performed before the mutation boundary.
- The O04 positive oracle constructs a separate raw GTSAM nonlinear graph and
  values, runs batch LM and independent Marginals, and compares full Pose3
  Logmap, velocity, both biases, time and every 15x15 covariance coefficient
  with the genuinely certified backend commit. A frozen candidate reference
  binds mean, time, configured world frame, body
  origin, numerical proof identity, and component PL. The commit receipt binds
  the nonlinear committed mean and applies the independent componentwise
  triangle transfer `L_commit = L_ref + abs(p_commit - p_ref)`. The schema-v2
  publication proof recomputes that transfer and the original P0-03 proof.
- `FinalOutputPacket` uses a V2 sidecar and candidate/receipt protocol. Every
  actual ROS publish, CSV write and throwing serialization operation runs in
  the measured pre-freeze call/return interval. Non-atomic ROS/CSV fan-out is
  conservatively and permanently non-authoritative, unprotected and
  completion-uncertified; all mirrors and the returned sidecar share exactly
  one semantic packet digest even after a later sink fails. Its terminal
  timing still records the actual return/outcome. Post-freeze receipt commit is
  a no-op for ROS. Only a separately declared non-throwing atomic receipt sink
  can activate authority/protection. Slow actual return, second ROS sink
  failure, and CSV/logger failure have independent production-adapter probes.
- Golden common public ABI is restored byte-for-byte: `PublicationDiagnostics`
  is size 424 with `certificate_id` at 416, and `IntegrityOutput` is 4128 with
  `bridge_audit` at 3864 under the project `-march=native` ABI. Golden V1
  publication layout remains exactly `88/80`; V2 remains `256/248`.
  Committed-reference mean, actual nonlinear mean and triangle transfer live in
  separate V2 sidecars/APIs. A golden-header client passes across the current
  DSO by value and through raw arrays and `std::vector`.
- Odometry keeps pose in the world frame but rotates linear twist and its 3x3
  covariance into `child_frame_id` with `R_world_body^T`; the compound packet
  retains a separately named world-frame velocity. State age and stale state
  are explicit.

## Acceptance

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target clean
exit 0

cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl -j2
exit 0; full all-target build, including integrity_round2_scenario

cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target tests -j2
exit 0

P0-05 directed oracle: 13/13 PASS
Genuine P0-03 proof/reference oracle: 1/1 PASS
Golden-header old client: compile/link/run PASS
Selected inherited contracts: 6/6 CTest targets PASS
Former full-suite failure reproduction after complete rebuild: 6/6 PASS

cd /home/mint/ws_fusion_uwb/build/uwb_imu_pl
ctest --output-on-failure -j2
exit 0; 28/28 targets PASS; 92.35 s (final post-fourth-review rerun)

Sixth-review repair rerun: exit 0; 28/28 targets PASS; 91.52 s. The repaired
O08 test passed 500 same-process repetitions and 800 independent cold-process
runs at eight workers; each invocation asserts exact arrival at all eight
post-mutation boundaries after an independent finite positive-definite 15D
batch-marginal precondition.

Seventh-review repair rerun: exit 0; 28/28 targets PASS; 91.36 s. The P0-05
target passed 13/13 and reported worst raw covariance antisymmetry norm `0`
against tolerance `1e-12`; valid O08 stress passed 500 same-process and 400
independent cold-process runs at eight workers. Exact reached points and unique
nonces were 8/8 in every run.

Eighth independent review: PASS; normal clean/full/tests, directed 13/13,
complete CTest 28/28 in 91.85 s, selected regressions 6/6, round2 7/7,
golden-header ABI, both ROS captures, complete-diff replay and all five hash
manifests passed. Supervisor acceptance independently repeated the normal
clean/full/tests sequence and complete CTest 28/28 in 91.55 s, directed 13/13,
selected inherited gates 5/5, the round2 gate, golden-header cross-DSO ABI,
complete-diff/hash/loaded-library checks and final worktree scope checks.

ROS simulator/subscriber smoke:
- real compound / IntegrityStatus / Odometry same-attempt capture PASS
- exact canonical capture SHA-256:
  fe4f71380021ed75bbc7e16edf29175edd2941150a407571acd462589579ab45
- matching CSV row transaction 1 and stable file hashes recorded
- checked-in simultaneous capture program and reproduction scripts PASS
- genuine certified commit capture transaction 1: SUCCESS_KEEP_ALL, finite V2
  reference transfer, actual compound/status/odometry/CSV match,
  batch_committed=true, publication_protected=false (formal gate unchanged)
- compound, legacy topics and CSV are all explicitly
  authoritative=false/protected=false with the same digest; production has no
  atomic authoritative sink
- certified canonical SHA-256:
  ff507093a44a7318b90e4006851b039f2761f5c1277e315bad4bdd1be5a29899
- certified CSV SHA-256:
  99107d38a844f604cb83790590039d83cc524496e1561fe7e496830586ada39d

git diff --check
exit 0
```

See `directed-results.txt`, `abi-results.txt`, `ctest-results.txt`,
`ros-smoke-results.txt`, `fourth-repair-probes.txt`, `attempt-journal.tsv`, and
`boundary-replay.json`; the latest focused record is
`seventh-repair-probes.txt`.

## Explicit NOT_RUN

- Sanitizers/TSan/MSan: `NOT_RUN` (not configured; O08 mutation injection and
  the complete suite were run).
- Physical UWB/IMU hardware campaign: `NOT_RUN` (not required by P0-05; the
  checked-in ROS simulator, real publishers/subscribers, and logger were run).
- Before/after performance statistics: `NOT_RUN` (P0 correctness row; no
  performance claim).
- Formal deployment qualification: `NOT_RUN` (unchanged external gate;
  unavailable/uncertified cases remain fail-closed).
- Prerecorded simulator input: `N/A` (live messages were generated by the
  retained launch/config; generated truth outputs are retained and hashed).
- Controlled certified-publisher config/manifest/replay input: `N/A` (the
  directed oracle constructs deterministic factors in its hashed source).

`integrity_round2_scenario` remained present throughout the authoritative
normal clean/full-build/tests/CTest sequence. Its top-level status now fails
when either `protected_available=false` or `risk_budget_closed=false`, so the
round-2 harness passes without hiding any binary. An older rejected attempt
left `/tmp/p005_optional_integrity_round2_scenario`; no acceptance command
depends on that copy and it was not used or modified by this repair.
