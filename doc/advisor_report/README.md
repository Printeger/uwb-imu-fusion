# Advisor report pipeline

`main.pdf` is a 16:9, 26-slide main talk plus 7 backup slides. Numerical
content, plots, and LaTeX macros are generated from the same validated summary;
the deck never treats the legacy batch estimator or real-data results as
evidence for the current incremental algorithm.

The committed protocol is `config/advisor_report_experiments.yaml`. The full
campaign is intentionally separate from the frozen Week-4 acceptance:

```bash
python3 tools/run_advisor_report.py all --threads 4
```

For a reduced end-to-end reproducibility check, use:

```bash
python3 tools/run_advisor_report.py all --smoke --threads 4
```

For the advisor update used here (complete isolated/persistent PL challenges,
smoke-sized baseline comparisons, and the dedicated 1000-epoch figure-eight),
use:

```bash
python3 tools/run_advisor_report.py all --profile focused-pl --threads 4
```

Every artifact and result slide carries its execution profile; baseline smoke
results are not a substitute for their preregistered sample sizes. The
`FOCUSED_PL` profile does execute all 640 isolated-fault seeds and all 80
persistent-fault seeds. The artifact directory is bound
to the Git SHA and protocol SHA-256 and contains resolved configs, commands,
environment, raw rows, compact summaries, copied figures, the PDF, and a
verified checksum manifest. No command reads or writes Week-4 acceptance except
for checksum-verified, read-only reuse during analysis.
