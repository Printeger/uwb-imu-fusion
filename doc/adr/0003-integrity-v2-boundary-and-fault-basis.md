# ADR 0003: Integrity V2 boundary ownership and window-effective fault bases

- Status: Accepted for development validation
- Date: 2026-09-16
- Formal status: development-only; this ADR does not declare Gate D eligibility

## Context

The 20-interval detector contains 21 navigation states.  The former builder
condensed every factor committed at the first state epoch into the boundary
prior, while the hypothesis generator still treated that epoch's UWB group as
an explicit recoverable measurement.  The resulting hypotheses had no rows and
made PL unavailable from epoch 21 onward.

The UWB ramp model also has two physical parameters (bias and slope).  At its
latest onset it has only one timestamp, so its raw map is `[1, 0]`.  Requiring a
two-dimensional numerical Gram rank rejects a redundant parameterization even
though the zero column has no current measurement or protected-state effect.

## Decision

1. A frozen factor inventory is the sole authority for explicit/boundary
   ownership.  A complete group is explicit exactly when all of its keys are in
   the integrity-window state layout.  Thus UWB on the first state stays
   explicit, while an IMU transition involving the preceding state remains in
   the boundary input.  No factor slot may have both or neither disposition.
2. Hypotheses and exclusion actions may reference only explicit recoverable
   groups from that inventory.  Suspect information already absorbed into an
   unrecoverable boundary remains fail-closed history contamination.
3. Physical fault dimension, identity, and risk allocation never change.  For
   each frozen window, a single physical mode may receive an effective
   numerical basis obtained from the SVD of its *raw whitened measurement map*,
   before parity projection.  Only raw-map null directions whose measurement
   and protected-state responses satisfy the ADR 0002 numerical tolerance may
   be removed.
4. Dependencies between different physical modes, or directions that become
   null only after state/parity projection, are not compressed.  They continue
   through the fault-Gram monitorability gate and fail closed when appropriate.
5. The effective basis is rebuilt for every frozen window and is shared by
   all-in evidence and post-FDE PL.  A slope direction suppressed at one
   timestamp therefore reappears when later timestamps make it observable.
6. After an exclusion/replacement action, a physical mode whose complete map
   into the candidate measurement set is certified zero is absent from that
   candidate's residual-fault PL census.  Its physical identity and allocated
   risk remain in the frozen audit.  This is not hypothesis pruning: modes with
   any nonzero candidate-measurement or protected-state response remain and
   pass through the ordinary monitorability gates.

## Consequences

The default ramp model can produce finite PL at its latest onset without
changing its physical risk semantics.  Boundary UWB measurements are
recoverable and replaceable exactly once.  The inventory and basis certificate
are included in frozen-content identity and development diagnostics.  Formal
eligibility remains unchanged pending campaign evidence.
