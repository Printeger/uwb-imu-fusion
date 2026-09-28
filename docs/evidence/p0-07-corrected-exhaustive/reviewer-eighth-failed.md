# P0-07 eighth independent reviewer — FAILED

Verdict: **FAILED, not BLOCKED**. Do not update the master table, commit, or
create `golden-p0-07-corrected-exhaustive`; P1 remains closed.

## Blocking findings

1. The strict schema still did not consume every legal recipe field.
   `loadOracleManifest()` rejected unknown and missing keys, but the following
   fields were only present in key sets, stored without driving a rule, or not
   read at all:

   - `root.authority`;
   - `history_raw.uwb_only_epochs.last`;
   - `history_raw.imu_and_uwb_epochs.uwb_rows_each`;
   - `history_raw.imu_only_epochs.first`;
   - `history_raw.imu_only_epochs.rows_each`;
   - `served_window.explicit_uwb_epochs.rows_each`;
   - `served_window.explicit_imu_epochs.rows_each`;
   - `fault_contract.window_boundary_onset_rule`;
   - `missing_provenance_fixture.recipe`;
   - `missing_provenance_fixture.expected_recoverability_rule`;
   - `tolerance_policy.classifications_always_compared`, which was only
     checked for non-emptiness instead of an exact set with every member bound
     to a comparison;
   - the four `observed_non_authoritative` fields, whose key names alone were
     checked without type, range, or consistency validation against the
     observed actual result. They may remain non-authoritative, but the strict
     parser must still consume and validate them.

   Consequently, a mutation of several legal fields could neither fail nor
   change the independently expected result. This did not satisfy the claim
   that all recipe fields were parsed and consumed.

2. The evidence hash manifest was not closed. From the repository root,
   `sha256sum -c docs/evidence/p0-07-corrected-exhaustive/artifact-hashes.sha256`
   failed for 11 entries:

   - `README.md`;
   - `abi-results.txt`;
   - `acceptance-summary.json`;
   - `attempt-journal.tsv`;
   - `binary-hashes.sha256`;
   - `complete-diff.patch`;
   - `loaded-library-hashes.sha256`;
   - `sanitizer-results.txt`;
   - `source-hashes.sha256`;
   - `test-results.txt`;
   - `verification.txt`.

   This contradicted the README and verification claims that the artifact
   manifest passed. The other independently checked manifests passed: source
   21/21, binary, loaded-library, config/input, and truth; every JSON file
   parsed.

3. This review's fresh LeakSanitizer gate failed. The freshly compiled full
   ASan+UBSan seven-case binary first passed normally. The subsequent
   leak-enabled N01/config-ABI two-case invocation of the same binary entered
   a startup loop of stackless `AddressSanitizer:DEADLYSIGNAL` messages,
   emitted a large repeated stream, and remained alive until safely
   terminated. This attempt must be retained as FAIL and cannot be overwritten
   by a retry.

## Preserved independent execution results

- clean Release build PASS in 5m17.9s;
- tests target rebuilt from zero PASS in 3m37.6s;
- directed P0-07 suite 7/7 PASS in 17.16s;
- runner/authority gates 3/3 PASS in 147.63s, including the 114.25s authority
  matrix;
- complete CTest 32/32 PASS in 341.42s;
- fresh ASan+UBSan 7/7 PASS in 169.78s with no ASan/UBSan finding;
- complete patch apply, 64-file byte comparison, and reverse check PASS from a
  fresh `golden-p0-06-actions` archive;
- golden-header/current-DSO ABI sizes/alignments were `ImuNoiseConfig` 80/8
  and `IntegrityConfig` 2544/8; every offset and load/by-value/destruct check
  passed;
- the export census found exactly three `FdeManager::decide` overloads and
  exactly one non-virtual read-only `auditRawRowOwnershipV1` export, with no
  estimator vtable/layout change;
- independent hypothesis/risk input came from `oracleHypotheses(recipe)`;
  production hypotheses were used only for actual-side field comparisons;
- the expected 506-row ledger was first constructed from the recipe and frozen
  replay; the actual ledger came from the transaction, real factor inventory,
  and `FactorLedger`, and every row ID/group/index/covariance placement matched;
- the sole current frozen authority was `authoritative-replay` at epoch 23 with
  504 modes and seven actions; old epoch-10 material was explicitly legacy and
  the temporary `replay-bundle/` directory was empty;
- all checked N01 public exits remained `formal_eligible=false`,
  `Unavailable`, and protected-output disabled; the pipeline exception,
  atomic authority, and `MissingProvenance` gates passed;
- no threshold, risk budget, prior, `p_md`, alert limit, hypothesis/action
  coverage, statistical denominator, or P1-scope change was found;
- the reviewer did not modify, generate, stage, commit, or tag the worktree;
  user documentation remained untouched.

## Minimum repair

- Parse every legal leaf above into a typed field and use it in a derivation or
  strict cross-consistency check; remove any unneeded field from the schema.
- Add a version-controlled per-leaf mutation test: mutating every field must
  explicitly fail parsing/semantics or observably change the independent
  expected result; unknown and missing keys must fail.
- Finalize every README/summary/journal/test/sanitizer/ABI/verification/patch
  artifact first, generate `artifact-hashes.sha256` last, and mechanically
  verify it from the repository root.
- Retain this review's LeakSanitizer failure. Freshly compile and rerun the
  complete ASan+UBSan suite plus a bounded LSan subset; acceptance requires at
  least one new fresh-binary LSan run to exit 0 without a finding.

No sigma/noise value, detector threshold, alert limit, risk allocation, prior,
window, hypothesis/action coverage, or statistical denominator may change to
obtain a pass. P1 remains out of scope.
