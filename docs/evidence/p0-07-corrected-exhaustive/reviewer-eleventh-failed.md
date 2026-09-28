# P0-07 eleventh independent review — FAILED

Verdict: **FAILED**, not BLOCKED. Do not update the master table, create a
commit/tag, or open P1.

## Decisive findings

1. The `FIXED` branch does not prove typed validation. It reads the expected
   reason from the TSV and deliberately throws that reason, so the claimed
   invariant is self-asserted rather than exercised.
2. The `DERIVED` branch evaluates a separately invented compact model but does
   not compare its protocol field-by-field with the production result from the
   same replay.
3. The action-by-hypothesis branch does not rebuild each candidate after the
   action's actual removed and replacement blocks have been applied.
4. The winner is assigned as the constant `0` instead of being read from the
   production replay and independently selected from the complete oracle
   candidates.

These are evidence-architecture defects. They do not establish D12's required
independent raw-factor and exhaustive FDE oracle, even though the ordinary
product gates remain green.

## Required change of approach

Replace the monolithic leaf self-test with three process-separated programs:

- an offline oracle that reads only frozen raw input, truth and public
  configuration/manifest and emits a versioned expected protocol;
- a production-linked replay probe that emits the actual protocol from the
  same replay and real audit/FDE results; and
- a comparison-only process that reports the first field mismatch and never
  computes expected values.

The offline oracle must not link or load the production library, and the three
programs must not share computation helpers. Retain all 174 schema leaves, but
make every `FIXED` mutation trigger a real typed validator error, every
`DERIVED` mutation rerun both sides and compare their protocols, and every
`OBSERVED` mutation compare same-run production observations rather than a
constant. Per-action candidates, action-by-hypothesis Gram/protected response,
risk, eligibility, refusal and winner must all be reconstructed or observed,
not inferred from a gate state.

No threshold, risk budget, prior, denominator, hypothesis/action coverage, ABI
or P1 scope change is authorized.
