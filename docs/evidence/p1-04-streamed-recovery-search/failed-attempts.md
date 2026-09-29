# P1-04 failed attempts retained as evidence

1. Funnel attribution overwrote `CandidateDiagnostics::skip_reason`.  O12
   exposed the frozen-terminal mismatch.  Repaired by preserving the original
   candidate terminal fields.
2. Funnel attribution was appended to `CandidateAuditRecord::reason`.  The
   P0 evidence runner rejected the changed `reason_hash`.  Repaired by keeping
   candidate audit records byte-semantically unchanged.
3. Funnel attribution used new diagnostic stage names.  Complete CTest was
   33/34; `test_round2_tools` failed with `attempt 1: incomplete stage set`.
   Repaired by using the existing `candidate_audit` stage reason.  Focused
   schema/evidence gates and the final full CTest were then rerun and passed.
4. The first two-pass proof-owner implementation reused the short-lived batch
   output arena as the frozen-hypothesis proof input arena.  The first
   mandatory section 2.4 comparison failed because normal action PL evaluation
   could not resolve its frozen proof.  Repaired with an additive split-arena
   overload: immutable frozen proofs are read from the attempt arena while
   action proofs are written to the batch arena.  All focused gates, all three
   section 2.4 comparisons and the complete CTest were rerun after this repair.
5. `proof_recompute_action` was initially appended to the public
   `PipelineTestDependencySeamsV1`.  That changed the golden layout from four
   fields/128 bytes to five fields/160 bytes and made a golden-header
   `shared_ptr<V1>` unsafe when the current DSO inspected the nonexistent tail.
   Repaired by restoring V1 exactly and moving the callback to additive V2
   with an independent setter and sidecar ownership.  A real golden-header
   lifecycle/process canary and every required gate were rerun after repair.

No failed approach changed thresholds, risk, coverage, denominator or dof.
