# Third independent-review verdict: FAILED

The third reviewer preserved A/B/D/E/G as passing and rejected P0-03 on the
following remaining items:

1. The fallback root validator had to recompute the canonical QR/SVD state,
   parity and statistic from the actual raw `H/z`; changing the stored spectral
   state by `+0.125`, synchronizing its derivatives and rehashing still had to
   fail.
2. Candidate PL, the P0-02 conversion and the publication gate needed real
   proof consumers.  The sidecar had to carry recomputable per-hypothesis and
   dual-channel W inputs/eigensystems/results rather than only IDs or counts.
3. O05 had to exercise the production candidate's final-H exact fallback and
   independently cover rank, condition and discrete decisions without using a
   production PL/certificate, `DenseCandidateOracle`, or normal covariance as
   its reference.
4. ABI compatibility required the exact golden public struct layouts, not
   fields inserted in the middle or an unsupported append-only claim.  A
   client compiled with golden headers had to link to and run against the
   current library, covering the affected layouts and legacy 8-argument
   symbol.

This verdict is retained as failure history.  The subsequent repair does not
represent it as a passing review.
