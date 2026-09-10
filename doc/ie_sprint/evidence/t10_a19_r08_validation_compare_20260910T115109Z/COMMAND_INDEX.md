# Command and identity index

- Pre-registration: `pre_registration/scope.json`, `pre_registration/material_hashes.txt`, `pre_registration/git_status_before.txt`, and `doc/ie_sprint/T10_A19_R08_PROTOCOL.md`.
- Build/tests/negative/prepare commands and exits: `engineering/*.log`, `engineering/*.exit_code`, `engineering/*.resources.txt`, `ENGINEERING_GATE.json`, and `ENGINEERING_GATE_V2.json`.
- Runtime identities: `engineering/runtime_identities.sha256`, `engineering/fix_v2_identities.sha256`, `source_frozen.sha256`, `source_frozen_v2.sha256`, `a19_r08_pipeline`, and `a19_r08_pipeline_v2`.
- Every scientific invocation: `attempts/<base>_<scenario>/command.json`; ticket issue/consumption, stdout/stderr, `/usr/bin/time` resources and `strace` are adjacent. Wrapper terminal records are `RUN_RESULT.json`; the terminal-output recovery record for turn01/step2 is `RUN_RESULT_RECOVERED.json`.
- Durable budgets and fixed ordering: `RUN_MATRIX*.json`, `BUDGET_LEDGER*.json`, `MATRIX_RUN*.{stdout,stderr,exit_code,resources.txt}`, and `SYSTEM_CRASH_RECOVERY_AUDIT.json`.
- Freeze and truth boundary: `PRE_EVALUATION_FREEZE.json`, `PRE_EVALUATION_FREEZE_SCOPE_CLARIFICATION.json`, `freeze_cells/*.json`, and `TRUTH_ACCESS_AUDIT.json`.
- Independent evaluator commands/exits: `evaluation_logs/*.command.json`, stdout/stderr beside each record, `EVALUATION_RUNS.json`, and `evaluation/*.json`.
- Tables and conclusions: `INPUT_RESULTS.csv`, `GROUP_RESULTS.csv`, `PAIRED_RESULTS.csv`, `EVALUATION_SUMMARY.json`, `RESULTS.md`, `FINAL_CONSISTENCY.json`, `COMPLETION.json`, and `VERIFICATION.md`.
