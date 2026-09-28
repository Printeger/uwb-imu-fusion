# P0-07 process-isolated oracle architecture — BLOCKED

Date: 2026-09-26

This is the result of the mandated architecture change after the eleventh
review. It is not a PASS claim and does not authorize a commit, tag, master
table update, or P1 work.

## Read-only/public API result

A temporary production-linked probe was built and run against the same frozen
24-epoch replay. The probe read the existing public window, evidence and final
output objects; it did not fill the winner from recipe metadata.

Observed actual fields were:

```text
window rows=341 columns=165 rank=165 dof=176 raw-owner-rows=506
all-in statistic=1280.3572059456483 threshold=279.97380275397535 passed=false
hypotheses=504 plausible=[57,60,61,62,63]
generated/evaluated actions=7
selected_action_id=0 formal_eligible=false protected_output=false committed=false
```

The probe test passed in 12.594 seconds. `ldd` showed that it loaded
`libuwb_imu_pl.so`, `libgtsam.so.4`, `libgtsam_unstable.so.4`, and
`libyaml-cpp.so.0.6`. These observations establish that a separate production
process can obtain the window-level and action-level actual protocol.

## Exact unavailable actual fields

The existing production pipeline does not export the per-action × hypothesis
proof needed by D12 and by the changed architecture:

- `IntegrityOutput::candidate_audit` contains per-action rank/dof/statistic,
  post result, aggregate HPL/VPL, risk allocation, selection and terminal
  reason, but no per-hypothesis candidate Gram, protected response, slopes or
  nullspace class.
- `IntegrityOutput::hypothesis_audit` contains all-in hypothesis summaries,
  not the proof after each action's removed and replacement blocks.
- `ProtectionLevelV2ProofV1` contains the required Gram/protected response,
  but `protectionLevelV2Proof()` can retrieve it only for a
  `ProtectionLevelV2Result` held by the caller. The pipeline does not retain or
  expose each internal candidate result/proof after it builds
  `candidate_audit`.

The relevant public definitions are in:

```text
include/uwb_imu_pl/common/types.hpp: CandidateAuditRecord, IntegrityOutput
include/uwb_imu_pl/integrity/protection_level_v2.hpp: ProtectionLevelV2ProofV1
include/uwb_imu_pl/integrity/protection_level_v2.hpp: protectionLevelV2Proof
```

## Why probe-side recomputation is not an acceptable substitute

A probe can call `DenseCandidateOracle`, `JointWindowDetector`,
`ProtectionLevelV2::compute`, and `protectionLevelV2Proof` again for the same
window and action. That produces a second production computation. It does not
read the proof used by the pipeline's actual optimized FDE attempt and
therefore cannot establish that the compared Gram/response/risk/terminal data
came from the same production execution. Treating the second computation as
the actual protocol would recreate the self-reference problem rejected in the
eleventh review.

## Required dependency

Completion of the A/B/C architecture requires an additive production audit or
immutable sidecar which freezes, for every evaluated action and every required
hypothesis, the exact served proof identity and the following actual fields:

```text
action id; hypothesis id; candidate row-set/proof identity;
Gram; protected response; protected slopes; nullspace class;
per-hypothesis PL/risk contribution; action eligibility/refusal;
selection identity binding those proofs to the final selected_action_id.
```

Adding such a production API/sidecar is beyond the implementer instruction for
this replacement attempt. Without it, the required same-run expected/actual
field-by-field comparison cannot be honestly implemented. P0-07 is therefore
BLOCKED at this architecture boundary rather than patched with another
self-proving test.
