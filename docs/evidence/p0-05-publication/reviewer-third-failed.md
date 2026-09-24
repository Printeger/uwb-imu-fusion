# P0-05 third independent review — FAILED

The third read-only review failed P0-05 while preserving the prior passing
regressions.  It reported:

- normal build and complete CTest 28/28 passed; V1/V2 publication-packet ABI,
  round-2, yaw/covariance, P0 regressions and hashes passed;
- the publication callback timed only `prepare()`, while the actual ROS/log
  `commit()` fan-out ran after terminal freeze.  A slow actual publish return
  (`call=140`, `return=1170`, deadline `100`) incorrectly remained protected,
  and a second-sink exception could leave the first sink protected;
- O08 still used first-epoch/vacuous catalog, prune, health and bridge state,
  and both snapshots came from `commitBoundaryAudit()` rather than an
  independent traversal of the underlying state;
- O04 performed a genuine token commit, but its reference repeated the
  production nominal-plus-map formula and its covariance comparison reused
  the audit path instead of an independent nonlinear batch/SO(3) oracle;
- ROS evidence did not carry or compare a common terminal digest, complete PL,
  formal/risk/reasons, attempt/time/frame/reference fields across compound,
  receipt, legacy mirrors and CSV; its adapter was not the production
  publication protocol.

The reviewer prohibited updating the master table, committing, or tagging
until these P0-05-only findings were repaired.
