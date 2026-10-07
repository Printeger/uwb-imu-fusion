# P1-06 focused repair v10

Status: `FROZEN / NONREALTIME_ACCEPTANCE_BLOCKED`.

This is a focused diagnostic pass only. It does not supersede final-v8 or
repair-v9, and it does not claim the revised Goal is complete. In particular,
the final-v8 quality failures and all three strict section 2.4 failures remain
failures. No ignore set, model, noise, prior, threshold, risk allocation, dof,
coverage, or denominator was changed.

## History-root fallback

The invalid-root handling now records a structured reason and resets the tree
in both branches. The valid full-row-oracle branch increments the persistent
`full_oracle_mismatches` counter (also printed as
`history_root_fallback_count`), returns the certified full-row summary
verbatim, and resets the suspect tree. The invalid full-row-oracle branch
records `tree_invalid=1,full_oracle_valid=0,tree_reset=1,fail_closed=1`, clears
retained rows/bytes, returns invalid, and forces the next finite request to
cold-rebuild.

Focused result:

- tree invalid + full oracle invalid -> fail closed + reset: `PASS` by the
  deterministic non-finite mutation in `fallback-mutations.log`;
- tree invalid + full oracle valid -> full oracle + reset: `NOT_RUN` on the
  current source identity. The old repair-v9 520-attempt observation is stale,
  and this pass deliberately did not replay it. The production branch is
  explicit in source but source inspection is not relabelled as mutation
  evidence;
- fallback-epoch summary/Gram/response/PL/action equivalence: `NOT_RUN`.
  Returning `rebuilt.summary` verbatim establishes summary construction, but
  no current-identity empirical downstream oracle was produced;
- golden P1-05 header/client against the current DSO: `PASS`; see
  `abi-canary.log`. No public header was changed in this repair.

Thus the history fallback is hardened, but its requested acceptance package is
not complete and must not be marked PASS.

## Complete-graph observability and Jacobians

The earlier repair-v9 claim used only a single-epoch UWB attitude block. The
new oracle builds the complete frozen 3D trajectory graph with eight-anchor
UWB, nonzero lever arm, gravity-aware `CombinedImuFactor`, bias evolution, and
pose/velocity/bias initial priors. Its whitened Jacobian results are:

| Epoch window | rows x cols | rank | minimum singular value |
|---|---:|---:|---:|
| 4 | 107 x 75 | 75 | 0.8810243269978374 |
| 8 | 199 x 135 | 135 | 0.6249913635965442 |
| 12 | 291 x 195 | 195 | 0.5143124822948862 |

All three are full column rank. A common right rotation about the body lever
axis leaves the UWB objective exactly unchanged (`0 -> 0`), but raises the IMU
objective from `1.423942e-6` to `0.0671507` and the initial-prior objective
from `0` to `0.125`. Therefore the single-epoch UWB null direction is not a
structural nullspace of the complete model.

Independent central differences at a perturbed trajectory agree with the
analytic whitened graph directional derivative for every factor class:

| Factor class | relative error |
|---|---:|
| initial pose/velocity/bias priors | 1.394e-11 |
| CombinedImuFactor + bias + gravity | 4.085e-10 |
| UWB + lever arm | 9.414e-13 |
| complete graph | 3.115e-10 |

The dedicated nonzero-lever UWB Jacobian central-difference test also passes.
See `full-graph-oracle.log`.

## Solver discrimination

The bounded existing 50-epoch fixture compares the production incremental
state and marginal against an independent full batch LM optimization. It
passes with pose tangent delta `5.804e-16` and relative marginal error
`1.054e-9`; see `production-vs-batch.log`.

The requested KKT/batch-MAP check at an actual final-v8 failed seed/window is
`NOT_RUN`. Those evidence directories retain output state/quality summaries
but not a serialized nonlinear factor graph, production linearization point,
or marginal prior sufficient to reconstruct the exact failed window. Reaching
that state from the seed requires a new 1200/12000-attempt replay, excluded by
this focused no-long-run pass. The bounded result rules out a basic short-run
solver/Jacobian error, but cannot distinguish long-run fixed-lag
marginalization bias from expected statistical quality under the fixed model.

## Freeze decision

No structural impossibility was proved; repair-v9's structural-nullspace
claim is disproved. Conversely, no implementation-level mathematical defect
was isolated on the focused complete graph, Jacobian, ABI, or bounded
production-vs-batch checks. Making further changes without that evidence would
be tuning the fixed contract. Therefore no full gate rerun was started.

Section 2.4 remains `FAIL` from final-v8 (3/3 strict comparisons); v10 is
`NOT_RUN`. The complete campaign, 38x20 matrix, full CTest, sanitizer subset,
and long quality runs are `NOT_RUN`. This pass is frozen for independent
review, not eligible for a golden tag.
