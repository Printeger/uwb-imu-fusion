# ADR 0002: Integrity V2 numerical and transaction contract

Status: accepted for research implementation, Gate J evidence pending.

The estimator freezes `graph_version`, ordering, noise-model, and linearization
versions in every `EpochTransaction`. Preparation may consume neither the IMU
queue nor mutate the backend. A finalized epoch performs zero backend updates
when discarded and exactly one add/remove update when committed. Any version
mismatch, repeated finalize, or exception whose backend mutation cannot be
proved absent fails closed.

The integrity window is represented by a trusted prefix covariance at the
oldest retained epoch, followed by every explicitly selected factor group in
the last 20 intervals and all current pending groups. This representation is
also the dense-oracle contract: the prefix appears once, and no historical row
may also be hidden in that prefix. Fixed lag must be zero or strictly exceed
`window_epochs + recovery_margin_epochs`.

The base information matrix is factorized once. Candidate exclusion uses
Cholesky solves and Woodbury block downdates/updates; candidate iSAM2 clones and
explicit matrix inverses are prohibited. A candidate is invalid on loss of
SPD, rank, finite values, condition-number limit, residual DOF, frozen-version
identity, or the configured linearization-step bound.

Dense-oracle acceptance tolerances are relative `1e-9` for well-conditioned
systems and `1e-7` for moderate-conditioned systems. Values within one decade
of a configured rank/condition gate use a fixed `1e-6` relative comparison and
must be tagged as a near-gate case in evidence. These tolerances apply to state
increment, covariance, squared detector statistic, and PL.

Correlated UWB exclusion replaces the complete all-in factor with a factor
built from the retained measurements and their principal covariance. Removing
already-whitened rows from the all-in factor is not an accepted implementation.

All releases produced by this ADR remain `IMPLEMENTED_UNVERIFIED` and
`formal_eligible=false` until the v4 manifest contains non-empty independent
risk, noise-overbound, and bridge calibration IDs plus completed independent
equation/code review.


P0–P2 implementation clarification (development evidence only): replacements
add their alternative whitened blocks before removing the old blocks. All
changes target the same final normal system, information RHS and residual
energy. A failed low-rank intermediate triggers a final-Jacobian reference
check; it is not by itself a final-candidate rejection. Changed candidates
that reach full checking use exact final-Jacobian SVD rank and condition
values. KEEP_ALL reuses the exact frozen base result. Neither base-condition
copying for changed candidates nor a Cholesky diagonal ratio is a pass gate.

The unchanged Euclidean step limit is 0.25. An early rejection requires a
conservative QR solution-error bound proving that the final solution exceeds
it; inconclusive or near-boundary cases use the SVD reference. Triangular
solves and covariance queries are streamed rather than constructing a full
inverse online. Cache entries are immutable before workers start, scoped to
one frozen base, and compared against complete block content and versions.
A post-detector failure skips bridge/fault-map/PL work, while the diagnostic
attachment retains the separate numerical-validity result. No hypotheses or
actions are pruned by coverage before kernel execution.

## D round status (2026-09-22)

The numerical contract held through the D acceptance runs: NUM-01..04, PRV-01..06
and the Gate-D numerics suites all PASS in `validation-report.json`; the v16
diagnostics export the frozen-window certificate fields unchanged.  Gate J
evidence (independent formula review, real-data calibration) is still pending,
so `formal_eligible` remains false.  No contract value was changed in D.
