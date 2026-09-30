# P1-05 deterministic concurrency and timing evidence

Status: `GOLDEN`; the final fresh independent reviewer and supervisor accepted
the second repair.  The immutable comparison baseline is annotated tag
`golden-p1-04-streamed-recovery-search`, commit
`fcc66dfa3bd3ef3f71860207ae313b7b74df550c`.

## Scope and contract

R11 is implemented as one flat task space per streamed recovery batch.  The
caller first prepares immutable candidate roots in canonical candidate order,
then schedules the Cartesian `(candidate slot, hypothesis slot)` leaves with
fixed index `candidate_slot * H + hypothesis_slot`.  Workers write only their
fixed leaf slots.  A barrier precedes deterministic candidate/hypothesis-index
reduction.  There is no per-candidate outer scheduler and no nested use of the
same worker pool.

The first independent review was `FAILED`. It found and rejected an unused
per-candidate worker-pool overload and showed that the original scratch limit
only cleared retained storage instead of failing closed. The repair removes
that overload and the corresponding private worker branch. `runFlat` now
checks retained scratch after every task, frees an oversized scratch object,
records the violation at that task index, completes every task and the common
barrier, then rethrows the lowest-index task failure. Ordinary task exceptions
and scratch violations use the same deterministic ordering. The repaired test
forces all four workers over the limit concurrently, proves every slot reaches
terminal, checks ordinary-exception competition and cleanup, and reuses the
same pool afterward. See `reviewer-first-failed.md`.

The second independent review was also `FAILED`. Although semantic outputs
matched, an exact-key statistical-cache miss could be solved concurrently by
multiple workers before publication. Consequently cache hits/misses depended
on scheduling, and the old comparator hid those work counters beside the
legitimately worker-dependent `hypothesis_parallel_blocks` diagnostic. The
repair makes lookup, the unique noncentral solve and publication one
mutex-protected operation, and protects every failure statistic with the same
mutex. No cache key, numerical policy or returned bound changed.
`compare_p1_05_workers.py` now compares every algorithm-work counter exactly
and reports only wall timing plus `hypothesis_parallel_blocks` as permitted
diagnostics. See `reviewer-second-failed.md` and the preserved pre-repair
failure JSON.

The P1-02 frozen shared-dual root remains the numerical source: candidate
identity, shared unique-mode responses and fixed pair blocks are prepared once
per candidate and consumed read-only by the leaves.  P1-04 batch streaming and
its heavy-residency boundary are unchanged.  Exceptions are retained by fixed
task index, all slots are driven terminal, and the lowest-index exception is
re-thrown after the barrier.  Same-pool nested invocation fails closed.

One attempt-scoped steady-clock timer now covers watchdog admission through
the ordinary or exceptional terminal path, and emits exactly one `core_total`
sample.  Deadline misses and exceptions remain in the statistical denominator.
No sensor, fault family, hypothesis or action coverage, threshold, alert limit,
risk, prior, dof, selection order, denominator, commit/publication authority or
fail-closed meaning changed.

## Verification

- P105 flat concurrency plus P102 frozen-shared-dual and timer exception
  focus: 8/8 PASS. The six-test P105 suite passed 100/100 repeated invocations.
  New cache gates prove one same-key solve (`1 miss/63 hits`), 24 distinct-key
  solves (`24 misses`), exact failure-boundary accounting, exact
  `12 misses/84 hits` for one/two/four workers, and repeated four-worker
  stability.
- O08/O10 publication regression: 13/13 PASS.
- O12 directed authoritative replay: 2/2 PASS, 11.09 s.
- One/two/four-worker and repeated-four-worker strict production comparisons:
  PASS/PASS/PASS. Final eight-attempt cache counts are exactly
  `hits=182775`, `misses=33`, `entries=50`; core SVD/LLT counts are exact.
  `hypothesis_parallel_blocks=0/2/4/4` is explicitly a configuration
  diagnostic, not invariant algorithm work.
- Complete CTest: 34/34 PASS, 485.49 s. Long gates include leaf protocol
  202.16 s, evidence runner 9.77 s and atomic-failure matrix 69.81 s.
- ThreadSanitizer: `NOT_RUN`; the installed GCC 9.4 toolchain cannot link a
  trivial TSan probe because `libtsan_preinit.o` is absent.  See `NOT_RUN.md`.
- Final fresh independent review: PASS.  The reviewer independently repeated
  the complete 34/34 CTest suite, focused gates, worker comparisons, ABI/diff
  audit and TSan probe; see `reviewer-final-pass.md`.

## Mandatory section 2.4 gate

Three final 45-attempt runs use the same configuration, seed, CPU affinity,
four-worker setting and numerical-thread limits as the P1-04 evidence.  All
135/135 attempts completed and remain in the denominator; none is excluded.
Strict comparisons against the matching golden runs are PASS/PASS/PASS with
zero mismatch in every frozen numerical/output field.

Aggregate measurements over all 135 attempts:

- core/analysis-completion p50/p95/p99/max:
  4080.56/6184.544/7048.426/7305.0 ms;
- arrival-to-publish p50/p95/p99/max:
  4081.04/6184.976/7048.9694/7305.49 ms;
- complete-work rate 135/135; deadline misses 132/135; exclusions 0;
- run RSS peaks 1045620/1104620/1108000 KiB; aggregate maximum 1108000 KiB;
- frozen numerical/history counters are identical to the golden comparison,
  including 178 candidate reference SVDs, 135 candidate inner LLTs, zero
  numerical-contract mismatches, 45 history requests and 16 full rebuilds.
- every final run has exact cumulative statistical-cache work:
  `5418131 hits`, `78 misses`, `100 entries`; strict run1/run2/run3 repeat
  comparisons pass without ignoring these fields.

The P1-04 baseline aggregate was core
3142.20/5068.379/5942.9354/6007.24 ms, arrival
3142.83/5069.121/5943.3068/6007.89 ms, and peak RSS 553024 KiB.  P1-05 is
therefore a latency and RSS regression on this workload.  This bundle makes no
speed or memory improvement claim.  RSS remains bounded by the P1-04 streamed
batch plus a fixed worker count and per-worker scratch bound, but its observed
peak is roughly twice the golden peak.  Reviewer/supervisor must evaluate this
honest result against R11; it is not hidden by exclusions, early rejection or
reduced work.

## Evidence layout

`archived-raw/candidate-3x45` contains every final attempt journal and output;
`archived-raw/workers-1-2-4` retains one/two/four and repeated-four-worker
comparison inputs.
The three `equivalence-runN.json` files are the strict golden comparisons;
`workers-1v2-equivalence.json`, `workers-1v4-equivalence.json` and
`workers-4-repeat-equivalence.json` are the exact-work comparisons.
`all-attempt-performance.json` is the no-exclusion
aggregate.  Focused and complete test logs, command/environment records,
complete source diff and checksum manifests accompany this README.
