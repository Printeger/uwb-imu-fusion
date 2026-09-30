# P1-05 first independent review — FAILED

Verdict: `FAILED`, not `BLOCKED`. The reviewer did not modify the worktree.

Two repairable defects prevented acceptance:

1. The additive per-candidate
   `computeFrozenAllIn(... CandidateWorkerPool*, ...)` overload had no caller
   anywhere in the repository. Production uses `computeSharedFlatBatch`; the
   overload and the corresponding private worker branch were unused and out
   of scope.
2. `CandidateWorkerPool::runFlat` did not fail closed when
   `scratch_limit_bytes` was exceeded. It merely cleared retained scratch
   after the barrier, and its test explicitly allowed an approximately 8 MiB
   allocation under a 1 KiB limit to return successfully.

The reviewer required removal of the unused overload/branch and a deterministic
scratch-limit exception ordered by lowest task index, while preserving the
barrier, cleanup, all terminal slots, ordinary-exception competition and later
pool reuse. A fresh reviewer must rerun the focused O08/O09/O10/O12 gates,
one/two/four-worker equivalence, complete CTest and all three Section 2.4 runs.

The review otherwise confirmed that production used one flat Cartesian batch,
fixed slots, canonical reduction, same-pool nested rejection and one attempt
timer; it also confirmed the original evidence honestly reported latency and
RSS regression and that TSan was explicitly `NOT_RUN` because the toolchain
could not link `libtsan_preinit.o`.
