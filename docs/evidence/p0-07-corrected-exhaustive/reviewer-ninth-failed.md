# P0-07 ninth independent review — FAILED

The ninth independent reviewer concluded **FAILED**, not BLOCKED.  The master
table must not be updated, no commit or tag may be created, and P1 remains
closed.

## Blocking findings

1. `test/test_p0_07_corrected_exhaustive.cpp:3420-3444` mutates every leaf but
   only parses the result and compares `oracleExpectedRecipeFingerprint`.
   `oracleExpectedRecipeFingerprint` at lines 1271-1363 unconditionally hashes
   every typed member, so merely storing a field in `OracleManifest` and in the
   generic hash is counted as an expected semantic change.  For example,
   changing `observed_non_authoritative.generated_action_count` from 7 to 8
   parses and changes only that hash.  The mutation test does not rerun the
   actual seven-action comparison.  Although the normal chain compares the
   observation around lines 3858-3860 and 4260, the mutation gate does not call
   it.  Consequently the claimed `174/174` semantic coverage in the evidence
   summary is not established.

2. The reviewer's freshly compiled ASan+UBSan full eight-case binary (SHA-256
   prefix `4c032891`) entered a stackless `AddressSanitizer:DEADLYSIGNAL` loop
   at startup and was safely terminated after four seconds.  This is a failed
   first run and cannot be replaced by a retry.  The same binary's first
   bounded LSan N01/config-ABI subset passed 2/2 in 103 ms.

3. `auditRawRowOwnershipV1` is non-virtual and additive, but its implementation
   around `incremental_estimator.cpp:3205` reads live
   `factor_ledger_.groups_` without synchronization.  Commit can replace the
   ledger around line 2843, and there is neither a mutex nor a concurrency
   test proving that the diagnostic is safe during commit.  Row/covariance
   provenance must be frozen into the prepare transaction (or an ABI-safe
   immutable sidecar) and the audit must read only that frozen certificate; an
   executable serial-use guard is an alternative only if the API is explicitly
   constrained to it.

## Gates that passed

- clean build: 5m17.7s; tests target from zero: 3m40.5s;
- directed suite: 8/8 in 17.17s;
- runner/authority suite: 3/3 in 147.71s (authority case 114.27s);
- complete CTest: 32/32 in 339.82s;
- bounded first LSan subset: 2/2;
- ABI/layout/export: `ImuNoiseConfig` 80/8, `IntegrityConfig` 2544/8,
  offsets/by-value/destruction, three decision exports and one diagnostic
  export;
- source, binary, library, config, truth, JSON, and artifact manifests 46/46;
- complete 65-file patch apply/byte/reverse, SHA-256 prefix `79e8...`;
- no detector threshold, risk, prior, `p_md`, coverage, or P1 scope change;
  status/tag/user documents remained correct.

One manifest command was first run from the wrong working directory and
failed; it then passed completely from the repository root.  This was a
command error, not a product result.

## Minimum repair required

- replace the generic all-field fingerprint gate with an explicit
  path-to-semantic-sink classification: construction mutations must rerun the
  independent derivation and change a concrete canonical artifact; fixed
  declarations/contracts must be strictly rejected; mutations of the four
  observed, non-authoritative fields must leave independent expected artifacts
  byte-identical and the actual-observation validator must reject the mismatch;
- retain the sanitizer failure, then obtain a fresh binary whose first full
  8-case ASan+UBSan run and first bounded LSan run both pass;
- freeze row ownership/covariance provenance during prepare and prove immutable
  copy/post-commit/address-reuse behavior plus concurrent commit/audit safety
  (or an explicit single-use guard).
