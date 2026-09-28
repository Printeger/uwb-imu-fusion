# P0-07 tenth independent review — FAILED

Verdict: **FAILED**, not BLOCKED.  Do not update the master table, create a
commit/tag, or open P1.

## Decisive finding: the 174-leaf semantic gate is still ineffective

`deriveOracleSemanticArtifacts` is a bucketed field fingerprint rather than a
semantic derivation:

- lines 3334–3349 serialize epoch/period/sample/anchor/fault booleans but do
  not construct raw `H/z/C`;
- lines 3363–3412 concatenate owner/group/row/count fields but do not construct
  or compare real raw/served `H/z/C` and covariance;
- lines 3436–3452 append detector fields and only compute the `p_fa` quantile;
  they do not evaluate profiles/plausibility for all 504 modes;
- lines 3454–3461 manufacture the plausible set `{1,2,3}` from family count
  rather than derive it from the actual replay;
- lines 3477–3479 merely append max-cardinality/identity/duplicate fields;
- lines 3482–3502 derive only total/action-count arithmetic and directly
  append the other risk, alert-limit, policy, prior, `p_md`, and selection
  fields; they do not produce the seven candidate ledgers, eligibility, or
  refusal;
- lines 3504–3514 merely append the missing-provenance fields.

Consequently lines 3607–3665 prove only that a mutation changes a string in
the selected bucket.  The 116 rejected leaves use one generic mutation and a
catch-all exception; no explicit per-path fixed invariant, reason, or targeted
mutation is declared.  Incidental rejection such as duplicate anchors does
not prove that a field is fixed by contract.  The eight observed fields keep
the expected map byte-identical, but the validator at lines 3552–3564 compares
against hard-coded plausible/action/winner/note values rather than an
`ActualObservation` captured from this run's real replay.

## Minimum repair

- Route every semantic mutation through a typed oracle result shared with the
  main acceptance: real raw/served `H/z/C` and ownership, 504 maps/profiles and
  plausibility decisions, actions/replacements, per-action × hypothesis PL and
  risk ledgers, eligibility/selection/refusal, and tolerance-driven numeric
  comparisons.
- Declare an explicit path-to-case table.  Every fixed leaf needs its own
  invariant/reason and targeted mutation, with the specific error reason
  asserted rather than any exception accepted.
- Feed the eight observed validators an `ActualObservation` captured from the
  same real main replay.  Expected typed artifacts must remain unchanged and
  each mutation must be rejected with the corresponding mismatch reason.
- Generate the 174-row TSV from those explicit cases and typed results.

## Accepted and independently rerun portions

The frozen row-ownership sidecar passed static and executable review: keyed
mutex/weak immutable ownership, constructor/destructor cleanup, prepare-time
freeze, audit without live-ledger reads, copy/post-commit/address-reuse and
concurrent commit/audit behavior, P0-05 auxiliary-row coverage, and ABI design
showed no blocking defect.

Independent gates passed: clean build and tests target; directed 8/8; complete
CTest 32/32 in 343.53 seconds; runner and atomic-failure matrix; diff and hash
checks; and HEAD/status checks.  Because the semantic gate had already failed,
the reviewer did not rerun sanitizer or ABI gates.  No contract, threshold,
risk, coverage, denominator, or P1 change was accepted.
