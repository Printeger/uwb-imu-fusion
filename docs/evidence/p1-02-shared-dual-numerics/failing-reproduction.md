# P1-02 failing reproductions and bounded repair

The new `P102SharedDualNumerics` fixture was added before enabling active-dual
frozen consumption.  Against the old path it failed because
`computeFrozenAllIn` returned `frozen pooled hypothesis context cannot certify
active dual-channel PL`; the reference `computeShared` result was valid.

After the first implementation, a three-epoch baseline/candidate strict
precheck found a real discrete mismatch on attempt 2:

- baseline terminal FDE status: `RISK_BUDGET_INVALID`;
- candidate terminal FDE status: `NO_VALID_CANDIDATE`;
- candidate reason: hypothesis 61 had an invalid rank `0/0` frozen entry.

The inferred mode-activity bitmap had not reproduced the exact
`projectPostActionModes` candidate census.  A second activity-only correction
still produced the same mismatch, so development did not continue with
per-case patches.  The scheme was changed once to a conservative gate: the
original exact projector proves the activity/census and is retained for
fallback; frozen blocks are used only when its fingerprint and every retained
certificate match.  Otherwise the complete original path runs.

The post-switch three-epoch precheck passed for every semantic field.  The
formal 3x45 comparisons also passed.  The only differences were four explicit
work/cache counters retained in `ignored-performance-fields.json`.

## Independent-review failure and ABI stop-loss switch

The first independent reviewer rejected the implementation even though its
directed tests passed.  Two separately frozen equal-content windows had
different control blocks, but a context made from A could be offered with B:
the consumer checked only IDs and 64-bit fingerprints.  Copies with stale or
tampered dual matrices/census/IDs also had no whole-payload seal.

The repair first added owner fields to `FrozenHypothesisNumerics`.  A stale
O12 executable then crashed with exit 139 after loading the rebuilt DSO.  This
was not dismissed as an infrastructure failure: the golden public layout is
320 bytes with `reason` at offset 264, and extending it was ABI-unsafe.  Work
stopped before full CTest and switched designs.

The accepted repair restores the golden struct exactly and puts all R08 data
in a separately owned `FrozenHypothesisDualNumerics` sidecar.  Its consumer
requires the same admission control block, seal object and payload address,
the same base-context owner and address, a valid candidate/action/certificate
chain, and a recomputed digest over dual blocks, response/factor/covariance
matrices, mode activity and census.  Legacy/no-admission calls cannot use it;
any structural or proof miss uses the established exact path.

The expanded P102 fixture now rejects equal-content/different-seal transplant,
forced hash collision, stale matrix, altered census/ID, missing blocks,
abnormal dimensions and modified candidates while accepting same-owner reuse.
A golden-header/current-DSO lifecycle client constructs the base context in
the current DSO, reads both early and tail fields through the old layout, and
destroys it successfully (exit 0).  The sidecar three-epoch strict precheck,
all directed gates, complete CTest and formal 3x45 were then rerun from zero.
