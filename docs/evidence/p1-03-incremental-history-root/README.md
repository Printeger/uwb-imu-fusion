# P1-03 incremental history root evidence

Status: `PASS`; the fresh evidence-only independent review and supervisor
acceptance completed after the implementation and long-run reviews. Baseline is annotated tag
`golden-p1-02-shared-dual-numerics`, commit
`435d682440f34fdeec22c237e4490b18c412cc74`.

## Authorized scope

This implementation maintains the history carrier through a hierarchical
orthogonal root with exact full-rebuild fallback.  Per the explicit user
authorization, `nu_perp` is the certified effective numerical rank after
orthogonal compression; no noise is injected to preserve the legacy value.

To make long-run RSS bounded without changing P0 contracts, the implementation
adds an explicit per-attempt proof arena, immutable lease, generation-safe weak
index, RAII close, and a self-contained final proof bundle.  The old P0
ABI/API remains callable.  Production benchmark and realtime paths use the
scoped overloads; legacy unscoped overloads remain unchanged.

No detector threshold, alert limit, risk, prior, sensor/fault-family coverage,
hypothesis/action census, dof formula beyond the authorized effective-rank
correction, commit atomicity, publication authority, or fail-closed terminal
meaning was changed.  The implementation does not use RSS-triggered proof
discard, a fixed-size proof LRU, `malloc_trim`, or benchmark-specific behavior.

## Verification result

- P103 proof-arena lifecycle: 7/7 PASS. This includes scoped
  mint -> close -> consume rejection, 128 close/consume races with one
  linearized winner, pre-consume exception revocation, delayed proof readers,
  invalid non-empty bundle fail-closed behavior, and 512 scoped committed
  sidecars without growth of the legacy registry.
- P0-03 / O05: `test_integrity_v2` 106/106 PASS.
- P0-05: directed publication target 13/13 PASS; golden-header/current-DSO
  client PASS with layouts 424/416, 4128/3864, 200/184, 296/292 and 88/80.
- P1-02 golden-header/current-DSO history client PASS with layout 176/160 and
  an effective rank of 2 for its frozen fixture.
- O01: history summary 14/14 and production pipeline 12/12 PASS.  The 30-step
  production audit recorded 23 incremental paths, six exact hits, one full
  tree rebuild, 30/30 oracle checks and zero mismatch; maximum oracle residual
  was `1.508e-12`.
- O03: action search 24/24 PASS.
- O09: deterministic event 2/2, repeated execution 1/1 and worker
  barrier/scratch 1/1 PASS.
- O12 directed authoritative replay: 2/2 PASS.
- Complete CTest after the reviewer repair: 34/34 PASS in 1138.57 seconds.
  This includes process oracle (36.51 s), all 174 isolated leaves (574.59 s),
  evidence runner (38.74 s), atomic-failure matrix (273.06 s), and ROS
  rostest.
- `git diff --check` and comparator/summary Python compilation PASS.

## Mandatory section 2.4 comparison

Three final 45-attempt runs used the same config, seed, CPU affinity, worker
count and numerical-thread limits as the retained P1-02 golden runs.  All
135/135 attempts completed; none was excluded.  All three strict comparisons
PASS across the 13 output files.  Each comparison identifies the same 24 rows
as the complete authorized rank chain; rank closure and strictly-decreasing
effective dimensions both PASS.  Coverage, every action/terminal state, risk,
PL, winner, commit and final packet compare unchanged outside that explicit
contract delta.

The repaired final candidate aggregates are core/analysis p50 3042.48 ms,
p95 4903.396 ms, p99 5755.307 ms and max 5791.43 ms;
arrival-to-publish p50 3043.02 ms, p95 4903.695 ms, p99 5755.907 ms and max
5791.80 ms. Deadline misses remain 132/135. Peak RSS is 536820 KiB.

Relative to P1-02, median latency improves, but tail latency regresses (the
P1-02 p95/p99/max were 4146.284/4494.577/4768.32 ms).  This is reported
without claiming a tail-latency speedup.  The decisive P1-03 resource result is
bounded proof/history lifetime: RSS falls from the P1-02 9529764 KiB peak to
approximately 0.54 GiB while preserving complete work.

## Bounded RSS soak

After the reviewer repair and rebuild, the three 45-attempt runs peaked at
524472, 526800 and 536820 KiB. The final 120-attempt run completed and
committed 120/120 with zero reject/error in 5:54.94 and peaked at 558512 KiB.
The 120-attempt peak is only 4.0% above the maximum 45-attempt peak rather
than growing linearly with attempt count.

The final 120 run recorded 29 incremental path updates and 91 exact full-tree
fallbacks, 0 numerical contract mismatches, 475 retained rows and 3893504
retained bytes.  No proof was discarded based on RSS.

The exact 3x45 and 120-attempt candidate outputs used by these claims are now
retained under `archived-raw/`; they were copied byte-for-byte from the
validation host and were not regenerated or edited during the evidence-only
repair.  Generated `run_manifest.json` files intentionally retain their
original execution paths as immutable provenance.  All checksum manifests
that cover repository sources or archived run data use repository-relative
paths and can be checked from the repository root.  See `archive-index.md` for
the path mapping and verification commands.

This directory includes the full source diff, source/binary/loaded-library/
config/input/truth/output hashes, complete command/exit record, all-attempt
performance journal, strict comparison JSON and explicit `NOT_RUN` list.
The second independent review accepted the implementation and all rerun gates
but returned `FAILED` only for the Section 2.5 manifest omissions recorded in
`command-record.txt`; its observation is not represented as an invented raw
log.  A fresh evidence-only independent review then verified every checksum,
the byte-for-byte raw archive, unchanged source/binary inputs, and the complete
Section 2.5 bundle.  `complete-diff.patch` is the reviewed candidate patch
captured before the supervisor-only final status flip in the master table.
