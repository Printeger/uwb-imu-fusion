# P0-07 first independent reviewer — FAILED

Verdict: **FAILED**, not BLOCKED.  This review was performed against the
first P0-07 implementation on top of `golden-p0-06-actions`.  The failure is
retained as part of the all-attempt record and is not superseded or rewritten
by later repair results.

## 1. Public ABI and old-profile compatibility failure

Two `std::string` members were appended to the public `ImuNoiseConfig`.
Observed layout changed from `sizeof(ImuNoiseConfig) == 80` to `144` and from
`sizeof(IntegrityConfig) == 2544` to `2608`, while same-name by-reference and
by-value DSO APIs remained exported.  A golden-header/current-DSO ASan canary
crashed, so source-level member-order commentary and the FDE-V1-only symbol
check did not establish compatibility.  The loader also made the two new YAML
keys mandatory; a golden/old profile missing them returned exit code 2 instead
of loading fail-closed.

Required repair: retain the `golden-p0-06-actions` public layout, put new
qualification state behind an internal sidecar or explicitly versioned
boundary, accept old profiles without the new declarations, and add a real
golden-header/current-DSO config construction/load/by-value/destruction canary.

## 2. N01 qualification gate was self-assertable

The first implementation treated an arbitrary non-empty calibration ID plus
two selected strings as sufficient for `formal_eligible=true`.  No artifact
was opened, parsed, hashed, allowlisted, or bound to the applicable model and
configuration digest.  This allowed a profile author to self-declare hardware
qualification without hardware evidence.

Required repair: missing/unknown/free-form IDs, arbitrary declarations,
missing artifact, malformed schema, content/hash mismatch, allowlist mismatch,
and model/config/semantics/overbound binding mismatch must all fail closed.
The repository has no real hardware calibration artifact, so shipped profiles
must remain explicitly unqualified; tests may prove negative cases but must
not use a seam or fabricated artifact to claim a qualification PASS.

## 3. D12 did not close the same-replay three-layer chain

The test hand-authored exclusion actions and passed the independent oracle's
already-whitened `H/z` into `FrozenSquareRootContext`.  It did not drive the
same frozen raw measurement replay through production
`buildIntegrityWindow`/history-summary construction, production hypothesis and
action generation, FDE/candidate/selection/commit, and publication.  Thus it
was a dense-kernel comparison, not the required three-layer
raw-oracle → corrected exhaustive → production pipeline closure.

Required repair: for one replay across multiple lag crossings, compare raw row
identity/ownership, objective, state/full covariance, constant/dof, `T_b/F_b`,
Gram/protected response, expected/represented/evaluated census, every uncapped
action, winner or identical fail-closed refusal, risk/commit/final packet, and
post-mutation exception behavior.  The independent oracle must begin from its
own unique raw factors/measurements/covariance and must not consume a matrix
produced by the builder under test.

## 4. Section 2.5 / O12 evidence was incomplete

The bundle did not contain independently restorable frozen input, truth, and
output snapshots with separate manifests.  Its journal was command-level, not
one row per input attempt, and lacked terminal state, detector/census/coverage/
risk/work counters, `core_compute`, `analysis_completion`,
`arrival_to_publish`, deadline miss, complete-work, and RSS for every attempt.
It also lacked a mechanically derived O12 p50/p95/p99/max, miss-rate,
complete-work-rate, and RSS summary.  Therefore O12 could not be marked PASS,
even as a correctness smoke.

Required repair: add replayable snapshots and per-domain hash manifests; keep
all failed and slow attempts in the denominator; emit the small O12 summary
while explicitly stating that it is a correctness smoke and not a P1
performance or 40 ms acceptance claim.  Any unavailable HIL/calibration/
sanitizer item remains `NOT_RUN`, never PASS.

## Preserved execution facts

The first bundle's own command journal records the stale-executable crash,
failed command invocations, and the sanitizer run in which assertions passed
but LeakSanitizer reported 12,312 bytes in three `libtbb.so.2` exit-time
allocations.  Those facts remain in `attempt-journal.tsv`; the leak-enabled run
is not relabelled PASS.  Repair acceptance requires new evidence in addition
to, not instead of, this FAILED review.
