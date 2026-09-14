# ICRA v6 clean backbone audit

Current scope: R0 → R1 → R2 → R2.5, ISAS Walk1/2/3 clean DEVELOPMENT only.
B3 and R3–R11 are NOT_RUN. No physical NLOS or held-out claim is supported.
Historical experiments and the existing working-tree initialization fix are preserved.

The protocol and generated audit reports distinguish unaligned coordinate mismatch
from independently calibrated raw-frame ATE. All comparisons use common GT samples,
scale-fixed SE(3) alignment, and the same body/IMU–tracker proxy assumption.
No fitted transform is sent to an estimator.

Commands and fingerprints will be recorded in VERSION.yaml and each isolated run.
Large raw runs remain local; compact audits, configurations and metric sources are
the reviewable delivery. No numerical paper assets are hand edited.

R2.5 is complete with retained failures: see [B1 result](audits/B1_calibration_result.md)
and [nine comparison rows](metrics/B1_calibration.csv). Three fresh B1 runs only:
Walk1/2 reached the existing preliminary LM iteration limit; Walk3 ATE worsened
by 0.007532 m (+4.226%). Author offsets were explicitly authorized for input-parity
analysis, with independent calibration provenance still unverified. B2/B3 remain NOT_RUN.
Actual execution: `python3 experiments/icra2027/run_b1_calibration.py`; the single-run
lock prevents accidental repetition. Existing R1/R2 metrics and reports are preserved.

Prior R0–R2 result: R0 local reproduction passed; R1 aligned-only audit completed;
R2 calibration/density audits completed but B1/B2 scientific comparison is blocked.
See [report](audits/backbone_parity_report.md) and [verification](audits/verification.json).
The unresolved independent calibration procedure is required by roadmap R2, not
an inferred approval requirement. The optional `--author-offsets-dev` capability
was not exercised or authorized in R0–R2; R2.5 subsequently authorized and used it,
without granting independent calibration evidence.

Reproduce in the existing workspace (commands create fresh isolated run folders):

```bash
python3 experiments/icra2027/fingerprint.py
python3 experiments/icra2027/run_r0.py
python3 experiments/icra2027/run_backbones.py
python3 experiments/icra2027/evaluate_clean.py --b0 experiments/icra2027/runs/B0_CURRENT-20260912T160609Z-5581d98e
python3 experiments/icra2027/audit_bag_metadata.py
python3 experiments/icra2027/make_report.py
python3 -m unittest discover -s experiments/icra2027 -p 'test_*.py' -v
python3 experiments/icra2027/verify_delivery.py
```

Replace the evaluator's `--b0` path with the new run path to evaluate a reproduction.
The shown path identifies this actual execution. R0 actual run is
`runs/R0-20260912T160419Z-e803adca`; its command, exit0, elapsed time and nested backend
run statuses are in `run_metadata.json`, `command.log`, and the nested batch.
The same information for B0 is in its run folder. Evaluation log is
`logs/evaluate_clean.log`; compact metrics remain outside ignored large runs.

Fingerprint: `bc4b4a9f250e33549999c3bc20f618221f516e98a380b52cf4fbeeabad1575ce`.
`VERSION.yaml` binds the original commit plus dirty source hashes, runner/libraries,
input hashes and starting configs. Later harness/config versions are additionally
recorded in each run. Source diff is preserved in `configs/source_worktree.patch`.
For a clean checkout, restore the recorded dependencies/external input paths and
apply that patch before rebuilding; patch applicability was checked against the
frozen commit in an isolated index, but an independent clean-checkout build was
NOT_RUN. Do not label the current local smoke as that separate acceptance gate.
