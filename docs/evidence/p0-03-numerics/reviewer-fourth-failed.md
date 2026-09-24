# Fourth independent-review verdict: FAILED

The reviewer preserved all previously accepted P0-03 areas except three
consumer gaps:

1. The fallback root validator did not independently validate the stored
   spectral vectors and inverse-squared singular values as the actual
   covariance/all-RHS operator derived from raw `H`.
2. Dual, frozen-hypothesis and PL sidecar consumers compared only subsets of
   their served fields or relied on a digest.  They had to independently
   recompute every numerical/class/result field, nominal covariance and tail,
   bridge contributions, per-hypothesis components and final PL outputs.
3. Publication lookup still searched by served PL and detector certificate.
   The selected candidate proof identity had to be carried unchanged through a
   versioned P0-02/final packet and used for exact publication lookup.  Two
   candidates with identical PL/certificate values had to reject crossed
   packet/proof pairs.

This failed verdict and its probes remain part of the all-attempt record.
