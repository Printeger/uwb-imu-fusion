# P0-04 third independent review — FAILED

Reviewer role: read-only independent reviewer; no worktree modifications.

Verdict: **FAILED**. The step was not eligible for the master-table update,
commit, or `golden-p0-04-risk` tag.

## Blocking finding

`risk_budget_audit.cpp:289` `no_omissions` and line 323
`exact_no_envelope` unconditionally overrode the sidecar
bound/known/validated fields.  The independent `/tmp` probe linked directly
against the then-current `risk_budget_audit.o` and exited 0 with:

```text
invalid_dormant_omitted close=1 validated=1 formal=1 omitted=0:VALIDATED
invalid_dormant_envelope close=1 validated=1 formal=1 envelope=0:VALIDATED
qualified_zero close=1 validated=1 formal=1 omitted=0:VALIDATED envelope=0:VALIDATED
```

For the first two cases, `event_bound=.25`, `event_bound_known=false`, and
`event_bound_validated=true`.  The omitted case also had
`omitted_scope_complete=true` with an empty omitted set; the envelope case had
`envelope_online=false` and `envelope_leaf_count=0`.  Thus contradictory
nonzero dormant evidence was rewritten as a validated zero.

The reviewer did not provide the exact probe compilation command; the three
lines above are the retained verbatim probe output.

## Positive evidence that did not clear the blocker

Negative/NaN/+Inf/overflow/rejected-hypothesis, exact-quad,
singleton/forgery, winner/final-consumer, ABI, evidence-history, and forbidden
contract-change checks passed.  The reviewer did not rerun clean CTest after
the deterministic counterexample.

## Required repair

A dormant zero proof may accept only one consistent canonical representation:
a finite non-negative zero bound with known/validated flags satisfying the
explicit contract.  A nonzero, negative, NaN, either infinity,
`validated && !known`, or any other contradiction must leave the term UNKNOWN
or invalid, with complete closure and formal eligibility false.  Exercise the
full Boolean Cartesian product for omitted, envelope, and selection terms,
including `-Inf`, before assigning a fresh reviewer.
