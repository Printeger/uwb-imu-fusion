# Recoverability golden fixtures

`golden_cases.json` is strict JSON intended for later C++/sparse-score comparison.
Each case stores the already-whitened `F` and `G`, reference `N` and `R`, the full
expected result (status, ranks, spectra, `eta`, `s`, thresholds and audits), and an
independent analytic expectation used by the Python tests.

Regenerate deterministically from the repository root:

```bash
python3 -B tools/paper/reference_recoverability.py \
  --generate-fixtures test/fixtures/recoverability/golden_cases.json
```

The committed unit test compares a fresh in-memory generation with the parsed JSON.
Case identity, status, dimensions, JSON types, keys, list lengths, and ordering are
strict; floating-point leaves use the bundle's predeclared
`comparison_atol=1e-10` and `comparison_rtol=1e-7`. Run the same comparison across
BLAS implementations with:

```bash
python3 -B tools/paper/reference_recoverability.py \
  --compare-fixtures REFERENCE.json CANDIDATE.json
```

Byte-identical regeneration is a separate same-environment reproducibility check,
not a cross-environment numerical-correctness requirement. These comparison
tolerances are numerical test tolerances only. They do not define or lock
`tau_eta`, `tau_s`, `tau_gamma`, or any formal experiment parameter.

An infinite `s` is serialized as `s_m: null` with `s_is_infinite: true`; no JSON
`NaN`/`Infinity` token is used. The near-degenerate case records a rank-tolerance
sweep but carries the aggregate verdict
`TOLERANCE_SENSITIVE_NOT_GATE_ELIGIBLE`, so increasing the rank tolerance does not
turn that fixture into gate evidence.

The saved roundoff model uses binary64 `gamma_k=ku/(1-ku)` bounds. `N-R` is scaled
from the spectra of `N` and `R` rather than the cancellation result `N-R`, and has
no fixed absolute PSD floor. The `R` rank/finite-`s` decision also includes the
squared projector forward-error bound. This model is for these small dense NumPy
fixtures only; non-finite intermediates or an inapplicable bound produce
`NUMERICAL_FAILURE`.
