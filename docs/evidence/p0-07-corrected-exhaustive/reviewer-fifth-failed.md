# P0-07 fifth independent reviewer — FAILED

Verdict: **FAILED**, not BLOCKED. This review covered the fourth P0-07 repair
on top of `golden-p0-06-actions`. It is retained permanently and is not
superseded by a later repair. The master table was not updated, no commit or
tag was created, and P1 remains closed.

## Preserved independent execution results

- clean Release build PASS in 320.60s and explicit tests target PASS in
  219.22s;
- seven selected gates PASS in 321.33s and complete CTest 32/32 PASS in
  333.52s;
- two fresh runner invocations and the then-current authority failure matrix
  PASS;
- fresh ASan/UBSan 7/7 PASS in 149.45s and LSan subset 2/2 PASS;
- golden-header/current-DSO ABI sizes/alignments 80/8 and 2544/8, member
  offsets, by-value calls and exports PASS;
- manifests, hashes, JSON, complete patch apply/byte/reverse and diff checks
  PASS;
- N01 public exits, real pipeline exception chain, honest `NotAttempted`
  publication outcome and old-profile/config compatibility PASS;
- no contract value, coverage, denominator or P1 change was found.

Those executions do not close the independent-reference defects below.

## 1. The oracle manifest stores outcomes instead of a derivation recipe

`oracle-manifest-v1.json` directly stores plausible ordinals
`[57,60,61,62,63]`, seven operations, their terminal states and the expected
selection. The parser merely loads these constants. It does not derive them
from the frozen raw replay, profile statistic, configured threshold and fault
contract, and it does not mechanically validate the row/group/mode/action
rules, uniqueness, risk sum or internal consistency.

Required repair: make the manifest a mechanically auditable recipe containing
the complete row/owner/factor/group/replacement rules, raw/config/fault
contract and detector/profile/risk/action-generation parameters. The parser
must validate its schema, uniqueness, row counts, group ownership,
high-precision risk closure and operation-identity rules. Plausible ordinals,
the seven operations, terminals and winner may be retained only as observed
fields; they must not be authority for expected results.

## 2. Expected raw shape and placement still depend on production output

`completeRawReference` still reads production `window.H.cols()`,
`window.detector_first_epoch`, `window.blocks` and
`window.history_summary.column_ids` to determine parts of the expected shape,
placement and comparison. The row-owner ledger is not mechanically parsed,
and explicit-block raw `H/z/C` are not completely rebuilt from frozen
factor/noise inputs independently.

Required repair: derive expected state shape, column order, row offsets,
history separator and raw/whitened `H`, `z`, `C`, `R`, `d`, `T` and `F` only
from the manifest, frozen raw/config and frozen nonlinear factors/values.
Production fields may be comparison targets only.

## 3. Candidate reconstruction consumes production candidate payload

`independentDenseActionReferences` directly uses `window.blocks` and each
production action's `added_blocks`. It independently recomputes only rank,
DOF, statistic, post-test and manifest-provided coverage.

Required repair: rebuild every candidate added block from the frozen source
factor/history replacement recipe. Enumerate plausible modes and single/union
operations from the independent mode/native-map/profile recipe, perform exact
operation deduplication and derive coverage without consuming production
models, evidence or actions as expected inputs.

## 4. PL, risk and selection are not independently computed

`DenseActionReference` contains no independent PL, risk ledger, eligibility or
selection tuple. The test instead asserts production invalid/infinite PL and a
manifest risk constant. The README claim that these quantities were
independently recomputed is therefore inaccurate.

Required repair: independently recompute each candidate's terminal, rank,
DOF, statistic, post result, candidate fault profiles, finite/infinite PL
class or value, complete risk ledger, eligibility, selection tuple and final
winner/refusal. If N01 makes a risk term unknown, the independent rule must
say so explicitly rather than merely asserting production infinity.

## 5. MissingProvenance is absent from the main replay

The main replay reports zero missing-provenance modes and zero such actions.
The separate test hand-constructs an attractive candidate and only proves the
FDE manager refuses a bypass; it is not a complete generator/action/FDE oracle.

Required repair: make MissingProvenance arise naturally from the recipe/raw
fixture and independently predict its nonzero mode/action terminal, or add a
separate fixture which completely drives generator, action generation and FDE
instead of hand-authoring a candidate.

## 6. Existing content-addressed bundles are trusted without revalidation

When a content-addressed target already exists, the runner discards the stage
without validating every existing file against its manifest and intended
semantic content. The failure matrix does not fully cover same-content and
different-content concurrency, a corrupt existing identity, an old-pointer
reader, or post-commit cleanup.

Required repair: validate every file, manifest hash and semantic identity of
an existing target before it can be selected. Corrupt content under the same
identity must be rejected without moving the pointer. Extend the matrix with
same-content concurrency, different-content concurrency, corrupt-existing
identity, old-pointer reader and post-commit cleanup while preserving an
always-valid `current.json` authority.

No sigma/noise value, detector threshold, alert limit, risk allocation, prior,
window, hypothesis/action coverage or statistical denominator may be changed
to obtain a pass. P1 remains out of scope.
