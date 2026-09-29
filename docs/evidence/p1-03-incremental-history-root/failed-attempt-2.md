# P1-03 failed attempt 2: backend-owned factor-group closure

Status: `FAILED`; no commit, tag, or P1-04 transition is authorized from this
attempt.

Baseline: `golden-p1-02-shared-dual-numerics`
(`435d682440f34fdeec22c237e4490b18c412cc74`). P0-07 remains
`VALIDATION_PAUSED`.

## Design switch from attempt 1

This attempt removed the P1-03 estimator-address registry and placed the
candidate state inside `FixedLagBackend`, which preserves the estimator's
golden public object layout. It also enabled GTSAM detailed update receipts so
the backend exposes per-key relinearization state in addition to new factor
slots, removed slots, marked keys, marginalized keys, ordering, whitening and
version receipts.

The cache key is the immutable factor-group UID. A pointer is only one value in
the entry-validation fingerprint; it is not a registry or cache key. A group
hit additionally requires exact factor identity and exact values of every
dependent pose, velocity and bias key. Raw-row reuse is independently checked
by group/row UID and by byte-exact coefficients after state and fault columns
are aligned by semantic UID. Unknown value types force a miss and full
rebuild.

## Focused result and mandatory stop

Commands:

```text
test_history_fault_summary --gtest_filter=HistoryFaultSummary.P103*
test_history_summary_pipeline \
  --gtest_filter=HistorySummaryPipeline.P103ProductionClosureAudit
```

Results:

- module mutation/owner/tamper cases: 2/2 PASS;
- production 30-epoch audit: 1/1 PASS as an instrumentation test;
- requests: 30;
- exact whole-root hits: 0;
- suffix root appends: 0;
- root downdates: 0 (not implemented or claimed);
- aggregate full root rebuilds: 30;
- invalidations: 29;
- epochs containing byte-proved unchanged groups: 26;
- unchanged raw groups: 402; changed groups: 76;
- factor-group linearization cache: 439 hits, 51 misses;
- retained rows: 467; retained bytes: 2,002,496.

This is a real production factor-linearization hit, but it is **not** an R09
incremental history-root hit: every request still executes the complete
two-stage Householder summary. Therefore the required production condition is
not satisfied. The strict short precheck, O01/O03/O05/O09/O12, complete CTest,
formal 3x45 comparison, long soak and RSS gate are `NOT_RUN` under the stated
stop rule.

## Architectural conclusion

Factor-group caching alone cannot satisfy R09. Sliding fixed-lag epochs change
the state/fault column frame and both remove and add groups. A conforming next
design must maintain an orthogonal hierarchical/block root at backend commit,
carry raw-row and fault-column provenance through each node, and recompute only
the paths intersecting the changed-key/removed-slot closure. It must also
preserve the residual-space certificate (`F_b`, `d_perp`, constant, rank and
dof) under group removal. Treating proved unchanged leaves as an incremental
root update would be false, so this attempt stops here rather than entering a
validation loop.
