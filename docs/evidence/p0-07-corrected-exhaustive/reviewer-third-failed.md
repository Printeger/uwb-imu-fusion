# P0-07 third independent reviewer — FAILED

Verdict: **FAILED**, not BLOCKED. This review covered the second P0-07 repair
on top of `golden-p0-06-actions`. It is retained permanently and is not
superseded by a later repair.

## Preserved independent execution results

- clean Release build PASS in 5m20.0s; explicit tests target PASS;
- directed/config/snapshot/publication/P0-06 CTest gates PASS (P0-06 88.43s,
  P0-07 16.07s); complete CTest 30/30 PASS in 208.96s;
- two fresh runner invocations exit 0; frozen input/truth/actions were
  byte-identical, and 24 attempts matched after timing/RSS exclusion;
- ASan+UBSan 6/6 PASS in 124.81s with no report; LSan subset 2/2, exit 0;
- golden-header/current-DSO ABI sizes/alignments 80/8 and 2544/8, offsets,
  load, by-value use and destruction PASS;
- manifests/JSON/hashes/patch apply-byte-reverse/diff checks PASS;
- no tag/master-table update or contract-value change; user docs untouched.

Those executions do not close the independent-reference defects below.

## 1. Partial raw oracle and mixed whitening coordinates

`verifyRawUwbOracle` independently constructs only the alarm epoch's current
UWB block. It does not construct the complete prior/IMU/UWB/history raw window
or compare complete row IDs/ownership, raw and whitened `H/z/C`, objective,
state/full covariance, constant/dof, `T_b/F_b`, and every mode/hypothesis
Gram/profile/protected response/PL. Its SVD-derived quantities are checked only
for finiteness/non-negativity. For correlated covariance, `H/z` use the GTSAM
upper information root while the fault matrix uses lower `L^-1`; the README
also incorrectly claims a lower canonical coordinate. Retaining production's
GTSAM upper root is reasonable, but `H/z/F` and the evidence must consistently
use that same coordinate.

## 2. No independent uncapped action/FDE reference

The six kernel actions have no independent reference. Builder/pipeline tests
compare counts/detector fields, while `alarm-actions.tsv` is production
`candidate_audit` self-report. The 9x3 dense fixture is a different replay.
There is no independent raw-truth/manifest enumeration and dense recomputation
of candidate terminal, post detector/statistic/dof/coverage, PL,
risk/eligibility/selection tuple, and winner/refusal. `MissingProvenance` also
lacks an independently expected terminal, although static inspection shows it
fails closed without deleting the mode/action.

## 3. Exception and publication chain was bypassed

Epoch 25 directly calls the estimator commit seam. The catch block hand-builds
an `IntegrityOutput` and fixed timing before finalization instead of exercising
the pipeline catch/`lastAttemptOutput`/packet/logger/capture chain. The normal
packet likewise fabricates timestamps 1..5 and `publish_outcome=Success` even
though no publisher exists. A later wall sample is called
`arrival_to_publish`. The repair must use the real pipeline seam and honest
`NotAttempted`/unpublished boundary, binding the same replay family, receipt,
backend update count, poison state and final semantic digest.

## 4. Runner is self-referential, non-atomic, and non-deterministic

The runner validates production self-report and hard-codes truth in the same
script. The frozen test checks shallow schema/counts. After timing/RSS removal,
`terminal_packet_digest` still differs between two runs. Several output files
are overwritten directly, so late parse/I/O/validation failure can leave a
partial bundle. Versioned input and independent truth/reference must be tool
inputs, complete semantic validation must precede atomic replacement, failure
must preserve the prior bundle, and a stable semantic digest must be separated
from timing metadata and match across runs.

## 5. Row ownership remains self-referential

The current ownership check only proves measurement IDs in production
`recoverable_history` are unique. It does not independently enumerate the
expected ledger and compare every production window row.

## Accepted N01 portion

N01 is acceptable: snapshot/conditional public entries and pipeline normal,
watchdog and exception finishes all close; ABI and old profiles pass; and the
gate does not hide the epoch-10 six-of-six kernel action execution. This does
not cure the D12/O12 failures above.

Required repair is a same-replay complete raw oracle, independent uncapped
action/FDE reference, real pipeline exception/publication chain, atomic
reference-driven runner, stable semantic digest, and accurately scoped docs.
