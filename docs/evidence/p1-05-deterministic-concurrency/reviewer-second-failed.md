# P1-05 second independent review — FAILED

Verdict: `FAILED`, not `BLOCKED`. The reviewer did not modify the worktree.

The independent eight-attempt production replay had exact semantic results for
one, two and four workers, but the statistical-cache work was schedule
dependent. The final cumulative `(hits, misses)` values were respectively
`(182775, 33)`, `(182772, 36)` and `(182758, 50)`; a repeated four-worker run
also disagreed with the archived value. Core decomposition counters such as
`candidate_reference_svd=4` and `candidate_inner_llt=3` remained exact.

The old comparator could not prove R11 because it listed
`statistical_cache_hits`, `statistical_cache_misses`, `cache_entries` and
`hypothesis_parallel_blocks` together as performance ignores. The first three
are algorithm-work counters and must be exact for the same canonical task set.
The last is a worker-configuration diagnostic and is expected to be `0/2/4`
for the one/two/four-worker runs rather than falsely reported as invariant.

Static inspection identified two repairable defects in
`StatisticalBoundsCache`: a noncentral exact-key miss was solved outside the
cache mutex, allowing concurrent duplicate solves before publication; and
`invalid_inputs`, `non_converged` and `policy_mismatches` had lock-free writes.
The reviewer required same-key single-flight (or unique computation under the
lock), synchronized failure counters, explicit same/different-key and failure
boundary tests, exact one/two/four-worker algorithm counters, and repeated
four-worker stability. `reviewer-second-failure-reproduction.json` preserves
the strict comparator's failure against the pre-repair archived runs.
