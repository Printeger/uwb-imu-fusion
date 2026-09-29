# P1-01 second independent review — FAILED

Verdict: **FAILED**, not BLOCKED. Do not commit.

The sealed candidate validator did not bind the candidate to the same
`FrozenWindowAdmission` owner.  In `rank_update_kernel.cpp`,
`sameWindowOwnershipOrCanonicalContent()` accepted a different payload address
by comparing only the two 64-bit content fingerprints, and
`validateCandidateDetectorCertificateImpl()` used that helper.  Consequently
a candidate produced by independent seal A could be accepted under seal B; a
64-bit collision could also admit different content.  The complete canonical
equality helper existed but was not connected to this real consumer.

The existing collision test called the helper directly.  It did not transplant
a candidate into the sealed validator, and the address-reuse-named test did not
force actual reuse.

Minimum repair:

- an admission-path candidate must reference exactly
  `&admission.window()` and the same owner;
- the sealed path must never fall back to bare fingerprint equality;
- if cross-seal equivalence is ever required, both seals must be present and a
  hash match must be followed by complete canonical equality;
- add end-to-end same-content/different-owner and forced-collision/different-
  content transplant tests, asserting rejection with no extra content scan;
- cover production move-source isolation and real owner/address-reuse stress.

All other independent gates passed: P101 9/9, O03 24/24 including the 54-cell
grid, O05 94/94, O09 deterministic/repeated/worker tests, complete CTest 34/34
in 1119.16 seconds, hashes/diff/patch/config/ABI, and the 135/135 performance
denominator.  Each candidate run had 45 scans for 45 windows and zero
descriptor linear scans.  P0-07 remained paused with no golden tag.
