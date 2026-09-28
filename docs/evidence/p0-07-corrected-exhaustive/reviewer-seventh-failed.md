# P0-07 seventh independent reviewer — FAILED

Verdict: **FAILED, not BLOCKED**. Do not update the master table, commit, or
create `golden-p0-07-corrected-exhaustive`; P1 remains closed.

## Blocking findings

1. `loadOracleManifest` did not parse or execute `mode_order`,
   `hypothesis_rule`, `double_fault_families`, detector `distribution`, risk
   `allocation_rule`, or `model_calibration`. `verifyRecipeConfigBinding`
   covered only part of the resolved configuration and did not compare every
   `config.fault_manifest` field. It also omitted the UWB prior and `p_md`,
   FDE candidate cap, and both terminal policies.
2. `independentDenseActionReferences` accepted production hypotheses and used
   their `p_md_allocation`, `hmi_allocation`, and
   `prior_probability_bound` directly in the dual bound, PL, and risk
   calculations. Production hypotheses therefore were not merely the actual
   side of a comparison.
3. The owner recipe checked only its own uniqueness and count. It did not bind
   every expected prior and IMU row identity to the production owner ledger.
   `completeRawReference` still enumerated `tx.recoverable_history` to decide
   the expected record/metadata inventory.
4. O05 and O11 linked a nonexistent root `boundary-replay.json`. O09 claimed a
   current unconditional 7/7 sanitizer PASS while README/summary retained the
   sixth-repair full sanitizer failure.

## Independent execution facts

- Clean Release build: PASS.
- Explicit tests target: PASS.
- P0-07 directed suite: 7/7 PASS in 17.26 s.
- Complete CTest: 32/32 PASS in 344.60 s.
- Authority checks: PASS.
- Golden-header/current-DSO ABI: `ImuNoiseConfig` 80/8,
  `IntegrityConfig` 2544/8, by-value load/destruct and all three decide exports:
  PASS.
- Hash and 62-file patch apply/byte/reverse checks: PASS.
- Fresh ASan same-raw-replay case: PASS in 166.105 s.
- The first full fresh sanitizer startup produced a stackless
  `AddressSanitizer:DEADLYSIGNAL` failure. It remains a failure. A bounded
  retry using the exact same binary then passed 7/7 in 174.347 s with no
  sanitizer finding. The reviewer judged startup/resource instability more
  likely, but did not erase or relabel the first failure.
- Fresh LeakSanitizer N01/config-ABI subset: 2/2 PASS.
- External hardware calibration/HIL, TSan, and MSan: `NOT_RUN`.

The minimum repair is limited to strict consumed-field/config/manifest
binding, a fully independent hypothesis/risk input path, per-row actual owner
binding, and internally consistent evidence references. It must not alter
thresholds, risk budgets, noise values, coverage, or statistical denominators.
