# P1-03 failed attempt 3: hierarchical orthogonal block root

Status: `FAILED`; no commit, tag, or P1-04 transition is authorized from this
attempt.

Baseline: `golden-p1-02-shared-dual-numerics`
(`435d682440f34fdeec22c237e4490b18c412cc74`). P0-07 remains
`VALIDATION_PAUSED`.

## Required design switch

This attempt replaced the full aggregate QR with a deterministic treap whose
leaves are immutable factor-group/raw-row blocks. Internal nodes align sparse
state and fault columns by semantic UID, stack two child roots, and apply
Householder transformations. Once the state part has full rank, the residual
`[F | d]` block is orthogonally compressed; the original raw-row count is
retained separately. No normal equations, address-keyed registry, global
registry, or thread-local cache is used. Ordering, whitening, recovery, or
owner changes rebuild the tree; add, remove, marginalization, and
relinearization update only affected treap paths.

The compact residual is mathematically equivalent to the full residual basis:
it preserves the objective, `F^T F`, `F^T d`, `d^T d`, state information,
rank, and an explicit raw-row/dof count. It does not preserve the arbitrary
element-by-element orthogonal basis of the monolithic Householder rebuild.

## Independent feasibility gate: PASS

The required independent 30-step mixed sequence included add,
relinearization, remove, marginalization, reorder, and recovery operations.
Every step was compared with `buildHistoryFaultSummary` for objective,
`R^T R`, `R^T T`, `R^T d`, Omega, xi, kappa, rank, and dof.

- requests: 30;
- incremental path updates: 19;
- add paths: 4;
- remove paths: 5;
- relinearize paths: 10;
- internal nodes recomputed: 293;
- full-tree rebuilds: 11 (the deliberate reorder/recovery cases);
- oracle mismatches: 0.

The focused test `HistoryFaultSummary.P103*` passed 2/2.

## Production root gate: arithmetic PASS, downstream certificate FAIL

The 30-epoch production probe, with a test-only full monolithic oracle on
every epoch, reported:

```text
requests=30 paths=23 exact=6 tree_full=1
add=39 remove=0 relinearize=63 internal=562
oracle=30/0 oracle_max=1.508e-12
```

Thus the aggregate root condition itself passed: nonzero production path
updates occurred and aggregate full-tree rebuilds were `1 < 30`.

The mandatory stop condition fired immediately in the first full downstream
O01 pipeline test:

```text
HistorySummaryPipeline.ProductionWindowMatchesIndependentFrozenRawFactorOracle
window.history_summary.nu_perp = 112
raw.old_state.rows() - raw_rank = 120
```

All printed continuous errors in that same test were small
(`objective=8.989e-10`, `state=9.609e-13`, `covariance=6.488e-11`,
`Tcross=1.267e-12`, `TFgram=7.907e-15`, `Fgram=1.319e-14`,
`constant=1.179e-15`), but the discrete detector dof differed by eight.

## Blocker and stop rationale

The hierarchical node removes mathematically redundant residual zero rows and
restores them as exact zero rows at the public boundary. The current production
carrier then discards a perpendicular row unless `F_b` or `d_perp` contains an
element that is *exactly* nonzero. The monolithic QR leaves roundoff-sized
nonzero values in eight such rows, so the golden carrier counts them; the
mathematically equivalent compact root does not. Consequently the published
detector certificate changes its statistical denominator from 120 to 112.

Making the padded rows artificially nonzero would be denominator gaming.
Changing the carrier's row-selection/dof contract is outside R09 and would be
a discrete golden-contract change. Reconstructing the monolithic residual
basis would require retaining/replaying the full orthogonal history and would
remove the compact-root benefit this attempt was required to prove.

Per the one-shot feasibility/production rule, no fourth adjustment was made.
O03/O05/O09/O12, complete CTest, strict golden comparisons, 3x45 performance,
soak/RSS, ABI, and publication checks are `NOT_RUN` after this failure.

