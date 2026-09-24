# Second independent reviewer verdict: FAILED

The second reviewer preserved PASS for A, B, E and G, and required the
following P0-03 repairs before another review:

- C: bind the actual served frozen state/parity/statistic and QR/SVD fallback;
  retain complete per-hypothesis Gram/G/eigensystem/error/nullspace/class/result
  proof; propagate dual-channel W proof through PL, P0-02 and publication; add
  real consumer recomputation and tamper failure.
- D: route `evaluateMapped()` through the same square-root-certified overload
  and assert shared/batch-disabled, frozen and coverage operator/proof/fallback
  counters.
- F: replace the nominally independent reference with a direct raw-H QR/SVD
  oracle covering state, covariance, all RHS, Gamma/t/G/J/slopes/PL, candidate
  paths, fallback and discrete sweeps; do not invoke production certificates,
  `ProtectionLevelV2`, `DenseCandidateOracle`, or normal-equation covariance
  inside the oracle.
- ABI: restore the old eight-argument `classifyDetectionResponse` mangled symbol
  while retaining the new policy input in a separate overload; audit public
  proof payload additions for append-only/schema-versioned compatibility.

This file preserves the blocking verdict; closure evidence is in README.md,
`directed-results.txt`, `legacy-path-results.txt`, `abi-symbols.txt` and the
complete diff.
