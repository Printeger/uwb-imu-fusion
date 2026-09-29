# Explicit NOT_RUN list

- Hardware-in-loop and field qualification: NOT_RUN; P1-01 is an offline
  identity/indexing optimization and does not claim hardware qualification.
- ASan/UBSan/TSan: NOT_RUN; these are not row-required gates and are not
  represented as PASS.
- Cross-compiler or cross-architecture ABI qualification: NOT_RUN.  The local
  golden-header/current-DSO canary and the four frozen layouts passed, but no
  claim is made for a different compiler or architecture.
- Independent review: NOT_RUN by this implementer; bundle status remains
  `PENDING_INDEPENDENT_REVIEW`.
