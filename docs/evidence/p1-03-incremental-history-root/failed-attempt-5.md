# P1-03 failed attempt 5: bounded active history tree

Status: `BLOCKED`; no implementation commit or tag is authorized from this
attempt.

Baseline: `golden-p1-02-shared-dual-numerics`
(`435d682440f34fdeec22c237e4490b18c412cc74`). P0-07 remains
`VALIDATION_PAUSED`.

## Scope checked

This attempt first audited the attempt-4 hierarchy for stale history-root
versions before changing it again. The cache owns one replace-in-place
`Entry`, removes groups that are absent from the current request, and reports
only the current request plus current root aggregate as retained state. No
cross-epoch list of roots or requests exists in the R09 implementation.

The user-authorized effective-rank carrier contract remains unchanged. This
attempt did not add numerical noise, restore the legacy value 120, change a
threshold, reduce any hypothesis/action coverage, change a risk budget, or
change a statistical denominator.

## Safe diagnostic reproduction

Command shape (the shell imposed an 8 GiB virtual-memory ceiling):

```text
ulimit -v 8388608
env LD_LIBRARY_PATH=/home/mint/ws_fusion_uwb/devel/lib \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  UWB_IMU_PL_CANDIDATE_WORKERS=4 UWB_IMU_PL_BENCHMARK_SEED=20260928 \
  taskset -c 0-3 \
  /home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark \
  config/fde_joint_order2.yaml /tmp/p103-diagnose.rn3tew/run 35
```

Observed `/proc` RSS samples and the concurrently flushed history diagnostic:

| completed attempt | RSS (KiB) | fault columns | boundary rows | emitted rows | effective `nu_perp` |
|---:|---:|---:|---:|---:|---:|
| 10 | 689,008 | 0 | 15 | 15 | 0 |
| 25 | 2,969,696 | 220 | 352 | 96 | 81 |
| 34 | 5,281,896 | 220 | 475 | 96 | 81 |

The run completed 35/35. Its final root counters were 35 requests, 29
incremental path updates, 6 full-tree builds, 475 retained rows and 3,833,472
reported retained bytes. Thus the active history dimensions and owned root
state had plateaued while process RSS continued to grow by roughly 150 MiB per
epoch. This falsifies the hypothesis that stale versions in the hierarchical
history root caused the attempt-4 15 GiB failure.

An independently rebuilt `golden-p1-02-shared-dual-numerics` detached
worktree was then run for the same 35 epochs with the same config, seed, CPU
affinity and worker count. It completed 35/35 and reached 6,252,380 KiB peak
RSS. Intermediate samples were 694,460 KiB at attempt 11, 1,851,804 KiB at
attempt 20 and 3,621,336 KiB at attempt 26. The approximately 195 MiB/epoch
slope over attempts 11--26 matches the candidate's approximately 191
MiB/epoch slope over attempts 10--34. The golden history dimensions also
plateaued at 220 fault columns and 475 boundary rows. Therefore the linear
growth predates P1-03; the candidate reduces the absolute footprint but does
not make the process bounded.

## Located out-of-scope owners

Read-only source tracing found pre-existing global proof registries outside
R09:

- `src/uwb_imu_pl/integrity/hypothesis_evidence.cpp` has
  `FrozenProofRegistry::proofs`; `storePlCertificate` inserts a detailed proof
  containing matrices for each changing entry identity, and no erase path
  exists.
- `src/uwb_imu_pl/integrity/protection_level_v2.cpp` has
  `ProtectionProofRegistry::by_result` and `by_identity`; every bound result
  copies the full sidecar, including hypotheses and detailed proof matrices,
  into both maps, and neither map has a lifecycle eviction path.
- the same registry retains publication packet maps without an eviction path.

Those objects are produced for candidate/hypothesis proofs each epoch and
explain why process RSS grows even after history-root dimensions are fixed.
Bounding or consuming them changes the P0-03/P0-05 proof-sidecar lifecycle and
is not an R09 history-root implementation change. Benchmark-only clearing,
allocator trimming, disabling detailed proofs, or lowering coverage would
hide the problem and is therefore rejected.

## Lifecycle audit

There is no existing safe attempt/transaction terminal for these registries:

- A frozen hypothesis proof is inserted while constructing each hypothesis
  entry. It is read again while sealing the shared context and while building
  candidate protection proofs. The public `frozenHypothesisPlProof` lookup
  accepts an entry with no transaction/owner token and has no consume or
  expiry operation. Consequently an attempt-end erase would change the
  observable lifetime of that public proof API.
- A protection proof is copied into both `by_result` and `by_identity` before
  its immediate validation. It is subsequently read by action-proof capture,
  commit-evidence minting, destructive commit-token consumption, transferred
  publication-packet binding, the publication gate, and final-packet
  construction. Only `commit_tokens` has a deliberate consume-and-erase path.
- Publication packets are read during the publication gate and final packet,
  but their public lookup and validation APIs also define no expiry. The
  returned output retains only a string packet ID, so deleting the backing
  packet at attempt completion would make later validation fail.
- Exception and discard paths clean only the bounded action-proof snapshot;
  they do not identify or remove the frozen/protection proofs already emitted
  before the exception.

No present public or private lifecycle hook can therefore reclaim these maps
without changing proof availability. A sound bounded design needs explicit
ownership: an attempt-scoped proof arena retained through final-packet
construction, immutable proof material embedded in/owned by that final
packet, or a documented consume/lease contract for later verification. It
also needs exception cleanup and concurrency-safe reference ownership.

The smallest likely implementation surface is
`hypothesis_evidence.{hpp,cpp}`, `protection_level_v2.{hpp,cpp}`,
`integrity_monitor.cpp`, and `final_output_packet.cpp`, plus the P0-03/P0-05
proof, publication, exception and ABI tests. Risks are stale-ID acceptance,
premature erasure before commit/publication, concurrent candidate lookup
races, breaking external delayed validation, and accidental loss of the
independent evidence bundle. This is a separate proof-lifecycle change, not a
history-root storage repair.

## Stop decision

The required 120/120 bounded-RSS long soak cannot pass by changing only the
active history-root ownership: that state is already bounded and is orders of
magnitude smaller than the observed process growth. Repairing the located
global proof registries requires an explicit scope/contract decision and
fresh validation of proof retrieval, commit, publication and the P0 gates.
Per the no-scope-expansion rule, no such change was made. Complete gates,
formal performance reruns, independent reviewer acceptance, commit and golden
tag are `NOT_RUN` for this attempt.
