# P1-04 complete streamed recovery search evidence

Status: `PASS`; independent reviewer and supervisor acceptance are complete.
The comparison baseline is annotated tag
`golden-p1-03-incremental-history-root`, commit
`bdc39560b4c74c5b7a7e4dc766d3994290f31f32`.

## Scope and result

The production recovery path now treats `max_candidate_count` as the maximum
resident heavy-action batch, not as a total-search cap.  It first freezes and
validates the complete generated census, evaluates deterministic action-ID
ranges batch by batch, reduces into the unchanged FDE selection/risk contract,
and persists a terminal rejection-funnel state for every evaluated
representative.  Exact semantic duplicates remain explicit omission records
with the pre-existing exact-dedup proof.  The legacy V1 capped API remains
unchanged and fail-closed for old callers.

No plausible threshold, action-generation or selection ordering, hypothesis or
action coverage, risk budget, statistical denominator, dof, commit authority,
publication authority, or fail-closed meaning changed.  The strict P1-03
comparison allows no numerical/output semantic delta.

The post-review ABI repair restores `PipelineTestDependencySeamsV1` to the
golden P1-03 four-field layout (`sizeof=128`, `alignof=8`).  The P1-04-only
`proof_recompute_action` test callback is now in the additive
`PipelineTestDependencySeamsV2`, installed by a separate V2 setter and held in
the pre-existing out-of-object sidecar.  The old V1 setter symbol, API and
behavior remain available, and `RealtimeIntegrityPipeline` gains no member.

## Honest memory bound

This change does **not** claim total memory independent of action count.  The
peak model is:

`irreducible O(A) raw/exact-dedup/audit/selection state + B * heavy workspace + one full winner`

where `A` is the complete action census and `B` is the configured resident
batch capacity.  Production no longer retains the previous full-array
`EvaluatedAction` heavy workspaces or a second full-action kernel copy.  Dense
candidate covariance, factors, reference vectors, retained Jacobian/residual
and row-role state are released after each batch; only the selected full proof
workspace survives for commit/publication.  The proof-audit mode deliberately
retains its self-contained proof output because that output is the requested
audit product.

The focused 130-action fixture proves capacities 7 and 31 produce identical
candidate audit, production refusal and action dispositions, with action 130
executing the real kernel/post/PL path as the sole pre-risk reference action.
Because those production clones intentionally share numerical proof identity,
a separate 65-action scoped ownership fixture uses distinct real candidate,
post and PL proofs and observes exact proof high-waters 7 and 31, batch close
between ranges, and one final proof in the main arena. It also asserts
`stream_full_action_heavy_copy=0` and that every representative has evaluation
or an omission proof.  The 45-attempt production profile happens to contain
one action per attempt, so its diagnostic high-water is one; it is retained as
the mandatory same-config performance/equivalence gate, not as the stress
proof.

## Verification

- P104 focused stream/rejection tests: 6/6 PASS; exception funnel 1/1 PASS;
  scoped proof ownership 1/1 PASS.
- O03 `test_integrity_v2`: 107/107 PASS.
- O07 complete action-search target: focused 30/30 PASS, 79.04 s (and
  79.15 s in the complete CTest run).  This includes the
  130-action streamed-vs-uncapped oracle and exact winner/refusal/disposition,
  risk and ordering comparison.
- O12 directed authoritative replay: 2/2 PASS, 39.86 s.
- Frozen round2 stage-schema tooling: PASS after funnel storage was moved into
  the existing `candidate_audit` stage reason.
- P0 evidence runner: PASS, 38.68 s; the candidate semantic hash is unchanged.
- Golden-P1-03-header/current-DSO and current-header/current-DSO V1 lifecycle
  canaries: PASS. Both report `128/8`, construct a real `shared_ptr<V1>`,
  install all four callbacks, process the production no-winner path
  (`NO_VALID_CANDIDATE`), clear/reset and destroy normally.
- Complete CTest after the V1/V2 ABI repair: 34/34 PASS, 1139.01 s.
  Long gates include leaf protocol 569.23 s, evidence runner 38.62 s and
  atomic-failure matrix 271.56 s.
- `git diff --check` and Python comparator compilation: PASS.

## Mandatory section 2.4 gate

Three final 45-attempt runs used the same config, seed, CPU affinity, worker
count and numerical-thread limits as the retained P1-03 golden runs.  All
135/135 terminal attempts are included, all completed their work, and none is
excluded.  Strict comparisons are PASS/PASS/PASS with zero mismatch in every
frozen numerical/output field.  Action census, candidate terminal states,
risk, PL, winner/refusal, commit and final packets are unchanged.  The
rejection funnel is additive evidence in the existing diagnostic stage file.

The final ABI-repair rerun is retained separately under
`archived-raw/abi-repair-3x45`. Aggregate timing over all 135 attempts:

- core/analysis-completion: p50 3142.20 ms, p95 5068.379 ms,
  p99 5942.9354 ms, max 6007.24 ms;
- arrival-to-publish: p50 3142.83 ms, p95 5069.121 ms,
  p99 5943.3068 ms, max 6007.89 ms;
- complete-work rate 135/135; deadline misses 132/135; no exclusions;
- RSS peaks 553024/539392/542012 KiB, aggregate maximum 553024 KiB;
- numerical and history work counters are identical across all three runs,
  including 178 candidate reference SVDs, 135 candidate inner LLTs, zero
  numerical-contract mismatches, 45 history requests and 16 full rebuilds.

The latest golden aggregate was p50/p95/p99/max
3042.48/4903.396/5755.307/5791.43 ms and peak RSS 536820 KiB.  Therefore this
evidence makes no latency or RSS improvement claim; P1-04's accepted result is
complete search with bounded heavy residency and unchanged semantics.

## Evidence notes

The first implementation attempt incorrectly replaced the frozen candidate
terminal reason with funnel attribution; O12 rejected it.  A second attempt
appended attribution to candidate reason and the evidence runner rejected the
changed semantic hash.  A third attempt introduced new stage names and the
round2 fixed-stage validator rejected the schema.  The final design preserves
candidate records and the fixed stage set, placing the complete additive
funnel in the existing `candidate_audit` stage reason.

The first two-pass proof-owner repair also used the short-lived action arena as
the input source for frozen hypothesis proofs. The mandatory 45-attempt strict
comparison caught the resulting PL/FDE mismatch immediately. The final
additive overload separates the immutable frozen-proof input arena from the
batch action-proof output arena; focused gates, all three strict comparisons
and the complete CTest were rerun after that repair.

The following review found that `proof_recompute_action` had been appended
directly to the frozen public V1 seam, changing its size from 128 to 160 bytes.
That implementation was rejected.  The V1/V2 migration above was followed by
fresh ABI lifecycle canaries, focused O03/O07/O12, complete CTest and all three
section 2.4 runs; none of the earlier gate output is substituted for this final
repair evidence.

Raw 3x45 directories, `/usr/bin/time -v` records, strict comparator outputs,
focused logs, complete CTest log, source diff and checksum manifests are
retained here.  See `NOT_RUN.md` for deliberately omitted gates.
