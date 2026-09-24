# P0-03 unified numerical certificate evidence

Status: implementer repair evidence after four independent-review `FAILED`
verdicts; all reported C/D/F/ABI blockers are repaired and the bundle is
pending a new independent reviewer and supervisor acceptance.

## Frozen identity and scope

- Base and current uncommitted `HEAD`:
  `golden-p0-02-dual-channel^{}` =
  `6b8c83b45eab6a69ca6364da48cb01df61e098af`.
- Only P0-03 implementation, tests, and this evidence bundle changed. The
  untracked user-owned Chinese audit and `doc/review/` inputs were read but not
  edited. The implementer did not edit the master table, commit, or tag.
- Read before repair: master §§2.1--2.5, P0-03, D08, O05, §5; original-review
  R04/D08/O05 and numerical-equivalence rules; Chinese audit numerical/PL/
  reproduction/evidence sections; previous golden and all inherited diff.
- `complete-diff.patch` is the complete binary-capable `include/src/test` diff
  against the frozen base. `hashes.sha256` binds it and every retained evidence
  file except the hash manifest itself.

## Preserved failure history

`reviewer-first-failed.md`, `reviewer-second-failed.md`, and
`reviewer-third-failed.md`, and `reviewer-fourth-failed.md` record all reviewer
verdicts and their blocking
probe classes. The original red reproduction is retained in
`first-failing-results.txt`: indefinite and asymmetric Grams incorrectly produced
finite valid bounds before the fix (`test_exit=1`, four failed assertions).
`attempt-journal.tsv` also retains the repair-time 76/84 and 81/84 diagnostic
runs; neither is represented as an acceptance run.

## Repair and reviewer-block closure

- A: arbitrary `-1e-16` eigenvalues, scale-relative asymmetry, and
  `Gram=0/Gker=1e-16` fail closed. There is no `max(1,norm)` rank/PSD floor.
  An unbounded nullspace is harmless only when the protected remainder is
  bitwise structural zero.
- B: raw-factor singular gates and Gram eigenvalue gates use the same
  quantity-aware rule (`tau*sigma_max` versus its square). The frozen sweep
  `1e-12..1e-4`, exact `1e-10` boundary, and `diag(1,1e-6)` probe agree.
- C: the frozen window proof independently recomputes from raw `H/z` and binds
  the actual served state increment, parity,
  statistic, canonical spectral state, QR/SVD path and fallback result. Every
  frozen hypothesis stores and validates its complete Gram/G/eigensystem/error/
  nullspace/class/result proof. The PL proof hashes the complete inputs and
  outputs. A versioned sidecar carries the complete hypotheses and every
  dual-channel W input/eigensystem/result into the candidate consumer, P0-02
  conversion and publication identity; each consumer recomputes it. Different
  matrices or paths change the proof; tampered window, hypothesis, W and PL
  payloads fail, including synchronized fallback-state derivatives plus rehash.
  The fallback covariance is compared as the canonical operator
  `V diag(1/sigma^2) V'`, invariant to SVD signs and degenerate-subspace bases.
  Dual and hypothesis consumers independently reconstruct every served
  numerical/class field. The PL consumer reconstructs nominal covariance/tail,
  bridge gains and bounds, every hypothesis component, final PL/HPL/VPL, risk,
  availability and reason. The P0-02 result carries a versioned publication
  packet with the selected candidate identity; lookup is packet/proof-specific,
  never a scan by equal PL/certificate values.
- D: shared, batch-disabled/legacy mapped, matrix-free, frozen and coverage
  routes consume the same square-root-certified operator and strict
  Gram/response policy. Operator, proof and fallback counters are asserted.
  The explicit `UWB_IMU_PL_DISABLE_HYPOTHESIS_SHARED/BATCH` run passed.
- E: exact fallback remains a direct final-Jacobian Jacobi SVD. There is no full
  `n x n` long-double normal-equation solve; long double is used only for small
  scalar accumulation/residual verification. Numerical path and served result
  are part of the candidate/PL proof.
- F: O05 uses a direct raw-H SVD reference which calls no production
  certificate, `ProtectionLevelV2`, `DenseCandidateOracle`, or normal-equation
  covariance. It checks state and multi-RHS covariance/fault/bridge solves,
  `Gamma/t/G/J`, slopes and PL over KEEP_ALL/modified, dense/matrix-free,
  rank/condition/fallback and discrete boundary cases.
- ABI: all affected pre-existing public structures retain their exact golden
  sizes and tail offsets; proof payloads live in new versioned sidecar types.
  The original eight-argument `classifyDetectionResponse` symbol and the new
  explicit nine-argument overload are both exported. A client compiled against
  `golden-p0-02-dual-channel` headers links to and runs against the current
  library. `abi-layout-old.txt`, `abi-layout-current.txt`, and
  `abi-symbols.txt` freeze this check.
- G: this bundle includes the complete diff, all-attempt journal, environment,
  census/coverage/risk/work-counter summary, frozen boundary replay and hashes.

The constructive factor-Gram entry point does not weaken strict PSD handling:
it accepts `F'F` only when every supplied actual-Gram entry exactly matches a
fresh product from that same `F`. Supplying an indefinite/tampered Gram with a
benign factor is rejected. Arbitrary Gram inputs continue through the strict
symmetric eigenvalue-interval certificate.

## Acceptance commands and results

```text
cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_pl --target tests -j2
exit 0

P003 directed root/PSD/rank/proof/multi-RHS probes: 5/5 PASS
P003 indefinite/asymmetric dual-channel probe: 1/1 PASS
Numerical/direct/PL/legacy/coverage directed set: 31/31 PASS
Publication proof consumer/wiring set: 6/6 + 3/3 PASS
Shared+batch disabled production/legacy directed set: 5/5 PASS

cd /home/mint/ws_fusion_uwb/build/uwb_imu_pl
ctest --output-on-failure
exit 0; 26/26 targets PASS; 103.94 s (after final clean rebuild)

git diff --check
exit 0
```

Full output is in `directed-results.txt`, `legacy-path-results.txt`, and
`ctest-results.txt`.

## Environment and exclusions

The compiler/dependency/library/config/input hashes and worker settings are in
`environment.txt` and `hashes.sha256`. Deterministic fixtures embedded in the
hashed tests are the P0-03 input/truth; `boundary-replay.json` freezes the
reviewer boundaries and expected discrete decisions.

- Sanitizers/TSan/MSan: `NOT_RUN` (no sanitizer build available).
- Hardware/ROS sensor replay: `NOT_RUN` (not an O05 numerical dependency).
- Before/after performance: `NOT_RUN` (P0 correctness step; no speed claim).
- Deployment calibration/long rare-event campaign: `NOT_RUN` (outside P0-03).

No detector threshold, alert limit, total risk, prior, sensor/fault family,
hypothesis/action coverage, statistical denominator, timeout, or exact fallback
was reduced or disabled.
