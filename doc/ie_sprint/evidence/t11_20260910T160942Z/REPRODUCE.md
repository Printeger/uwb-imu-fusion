# T11 reproduction and evidence boundary

The science matrix is closed: six fresh prefix tickets and two independent U13 tickets were consumed once. Running `t11_prefix.py all` against this manifest rejects existing output/tickets; do not treat verification as authorization for new science runs.

Actual build: `cmake --build /home/mint/ws_fusion_uwb/build/uwb_imu_fgo --target a19_r08_pipeline -j2` (both builds exit 0; build1.log/build2.log).

Actual serial entry: `python3 tools/paper/t11_prefix.py all`. Each invocation is recorded under runs/<case>/execution/COMMAND.json, RESULT.json and file_access.trace. Prefix preparation had already physically copied <=cutoff raw bytes before launching the C++ process. The C++ automatic alarm is 900 seconds and each final child has its own 900-second limit; the outer 2730-second process-tree guard is not a shared 900-second budget.

Verification commands (all actually executed):

- `python3 test/test_t11_prefix.py` — 5 tests passed in final_reporting_tests.
- `python3 test/test_reference_recoverability.py` — 22 tests passed.
- `python3 tools/paper/t11_verify.py u13` — exact comparisons; only 20102 complete PASS; overall NOT_PASSED.
- `python3 tools/paper/t11_verify.py fixed` — saved-row semantics and matrix-order checks.
- `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/paper/t11_golden_audit.py` — six sparse/SVD matches.
- `python3 tools/paper/t11_evaluate.py` — frozen reporting, truth admission rejected; no truth opened.

PRE_EVALUATION_FREEZE.json is the original immutable payload hash list. EXECUTION_BINDING.json binds initial code/runner/dependencies, REPORTING_*_BINDING.json records reporting-only corrections without estimator reruns. U13_COMPARATOR_TIMING_FIX.json preserves the initially omitted timing-field comparison failure; fixed_audit_initial preserves the NumPy bool serialization error. Neither changed scientific values or tolerances.

RETENTION.json maps post-verification exact-byte duplicate U13 payloads to relative references to original H0 payloads. Independent runs preceded storage deduplication; original frozen hashes still resolve unchanged. No old frozen package was modified. Curated files are tracked, detailed local runs remain ignored by git. T11_CLOSEOUT.md has the ten-item report. T12 is NOT_RUN.
