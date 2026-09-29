# P1-03 failed attempt 4: authorized effective-rank carrier contract

Status: `FAILED`; no commit or tag is authorized from this attempt.

Baseline: `golden-p1-02-shared-dual-numerics`
(`435d682440f34fdeec22c237e4490b18c412cc74`). P0-07 remains
`VALIDATION_PAUSED`.

## Authorized contract and implementation

The user explicitly authorized replacing the legacy raw-row `nu_perp` with
the certified effective numerical rank of the orthogonally compressed
`[F_b | d_perp]` carrier. The implementation uses the existing configured
rank tolerance and an SVD certificate with threshold

`rank_tolerance * sigma_max + gamma_n * ||carrier||_F`, where
`gamma_n = n * epsilon / (1 - n * epsilon)`.

The certificate distinguishes carrier storage rows, raw residual dof and
effective residual dimension, and retains singular values, scale, roundoff
bound, threshold, discarded norm and proof identity. Monolithic corrected-full
and hierarchical incremental paths use the same contract. No perturbation,
noise, epsilon row or expected-rank constant was introduced.

The first ABI-changing version was rejected. The repaired implementation
restores the golden `HistoryFaultSummary` layout (176 bytes, `d_perp` offset
160) and places the detailed rank certificate in an additive explicit API and
private production sidecar. The golden-header/current-DSO lifecycle client
constructed, read and destroyed the summary successfully.

## Correctness results before the stop

- O01 production: raw residual dof 120, certified effective `nu_perp` 81.
- 30-epoch incremental/corrected-full oracle: 30/30 oracle checks, zero
  mismatch, 23 incremental path updates and one initial full-tree rebuild in
  the focused fixture; maximum relative error `1.508e-12`.
- O03: 24/24 PASS; O05: 99/99 PASS; O09: 2/2 deterministic,
  1/1 repeated and 1/1 worker PASS; directed O12: 2/2 PASS.
- O12 independent causal proof: rows 341 -> 334, dof 176 -> 169,
  `nu_perp` 88 -> 81, unchanged-formula threshold 279.974 -> 271.190;
  mode 57 statistic 273.844 therefore changes only its plausibility, and the
  action census changes 7 -> 6. All 504 modes are proved and risk total, alert
  limits, prior and coverage remain unchanged.
- The process-isolated oracle checked 6 corrected actions x 504 modes = 3024
  records. The complete 174-leaf protocol retained DERIVED 13, FIXED 153 and
  OBSERVED 8 coverage. The old P0 evidence bundle was not modified.
- Complete CTest before the final read-only benchmark-counter addition:
  34/34 PASS, exit 0, 1168.13 seconds.

## Formal 3x45 result

The final candidate output root is
`/tmp/p1-03-candidate-final-3x45.wVazM2`; the retained golden output root is
`/tmp/p1-02-sidecar-formal-3x45.pCnV75`.

All three candidate commands exited zero. All 135/135 attempts completed,
zero were excluded and 132 missed the deadline. Three strict dual-reference
comparisons passed. The only golden semantic differences were the explicit
rank/dof-derived allowlist on attempts 22-45; action/census/terminal/risk/PL/
winner/commit/final-packet fields had no additional mismatch.

Each candidate run reported the same root work: 45 requests, 29 incremental
path updates, 16 full-tree builds, zero full-oracle mismatch, 475 retained
rows and 3,927,808 retained bytes. This separately demonstrates root-path
reuse and is not a speedup claim from reduced effective dof.

The end-to-end latency did not improve and must not be presented as a speedup:

| all-attempt metric | golden P1-02 | P1-03 candidate |
|---|---:|---:|
| core/analysis p50 (ms) | 3274.54 | 3771.94 |
| core/analysis p95 (ms) | 4146.284 | 5521.506 |
| core/analysis p99 (ms) | 4494.577 | 6408.647 |
| core/analysis max (ms) | 4768.32 | 6432.38 |
| arrival p50 (ms) | 3274.77 | 3772.24 |
| arrival p95 (ms) | 4146.708 | 5521.848 |
| arrival p99 (ms) | 4494.870 | 6409.093 |
| arrival max (ms) | 4768.64 | 6432.97 |
| peak RSS (KiB) | 9,529,764 | 7,842,980 |

## Mandatory long-soak failure

The 120-epoch run is retained at `/tmp/p1-03-long-soak.9Jd88G`. It completed
only 89 attempts and was killed by signal 9 after 6:28.30. GNU time recorded a
peak RSS of 15,122,228 KiB. `soak.time` begins with
`Command terminated by signal 9`; its trailing `Exit status: 0` is not treated
as success. The buffered stdout never reached the final summary/root-counter
lines, which independently confirms incomplete work.

This violates the required bounded long-run RSS and complete-work gates.
Further fixes and additional long tasks were stopped at this point. Formal
acceptance, post-instrumentation complete CTest, evidence finalization,
independent reviewer acceptance, commit and golden tag are all `NOT_RUN` or
not authorized after this failure.
