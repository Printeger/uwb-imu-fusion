# P0-07 second independent reviewer — FAILED

Verdict: **FAILED**, not BLOCKED.  This review was performed after the first
P0-07 repair on top of `golden-p0-06-actions`.  It is a permanent part of the
all-attempt record and is not superseded or rewritten by later repair.

## A. N01 did not guard every public production exit

`RealtimeIntegrityPipeline::processUwbBatch()` failed closed, but the public
`IntegrityMonitor::evaluateSnapshot()` and `evaluateConditional()` paths did
not apply N01.  `runPublicationGate()` also defaulted
`platform_certification_enabled=true`; the pre-existing snapshot test asserted
formal eligibility without an authenticated artifact.

Required repair: put an unavoidable model-qualification gate before every
monitor, pipeline and publication exit.  With no authenticated artifact in
this repository, each path must return `formal_eligible=false`,
`Availability::Unavailable`, `protected_output=false`, and an explicit reason.
Preserve the golden public ABI and old-profile loading.  No environment/test
seam or arbitrary profile string may bypass the gate.  Add negative tests for
every public entry point.

## B. D12 still combined two different fixtures

The independent nine-by-three dense fixture and hand-authored actions were not
the raw replay used by the separate 24-epoch production pipeline test.  The
production replay passed its detector on every input, so it never exercised a
real multi-action recovery search.  This was not the required one-replay
raw-oracle -> corrected exhaustive -> complete production closure.

Required repair: use one frozen raw IMU/UWB replay.  The independent oracle
must independently construct measurement factors, row identities and
covariance without consuming production `H`, `z`, or history-builder output.
The same input must drive real prepare/window/history construction over
multiple lag crossings, production hypothesis generation, detector/evidence,
uncapped production action generation, candidate/FDE/selection/commit and
publication.  It must contain a real alarm/recovery case, evaluate more than
one generated action, compare every action, and establish a winner or the same
honest refusal.  Bind row ownership, objective, state/full covariance,
constant/dof, `T_b/F_b`, Gram, protected response, PL, risk, census, winner,
commit and final packet, including a replay-family post-mutation exception.
If N01 ultimately refuses publication, compare the full candidate/selection
result before applying N01.  Retain independent high-precision/SVD checks and
order invariance.  Hand-authored actions are not a substitute for production
generation.

## C. Section 2.5 / O12 evidence was not mechanically reproducible

The snapshots, attempt journal and summary were manually assembled from two
fixtures.  The acceptance test did not read and validate them or
deterministically regenerate them byte-for-byte.  The journal copied one
elapsed duration into all three timing columns, rather than sampling three
different semantic boundaries.

Required repair: add a version-controlled replay/evidence generator or a
directed-test `--evidence-output-dir` mode that runs the same replay and emits
frozen input, truth, output, one row per attempt, and a mechanically derived
summary.  Acceptance must validate the frozen snapshots or deterministically
regenerate and compare them byte-for-byte.  Every attempt must retain terminal
state, detector result, expected/represented/evaluated census, coverage, risk
ledger and unknown state, work counters, real `core_compute`,
`analysis_completion`, and `arrival_to_publish` boundaries, deadline miss,
complete-work and RSS.  Compute p50/p95/p99/max, miss rate, complete-work rate
and RSS from all 24 attempts, including failures and slow frames.  Record the
rerun command and exit status.  A missing product timing may be sampled in the
test driver only when the boundary and its position are explicit.

## D. Sanitizer history must remain honest

The earlier leak-enabled run's 12,312-byte LeakSanitizer failure from three
`libtbb.so.2` exit-time allocations remains a historical failure; its raw
stack detail is limited to what was captured.  The repair must add a new raw
leak-enabled run without rewriting that fact.  ASan/UBSan results and honest
TSan/MSan `NOT_RUN` reasons must remain explicit.

No sigma/noise value, detector threshold, alert limit, risk allocation, prior,
window, coverage set, action/hypothesis set, or statistical denominator may be
changed to obtain a pass.  P1 optimization is out of scope.
