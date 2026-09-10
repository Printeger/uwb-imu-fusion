# Evidence retained in Git

Git contains historical tracked evidence and the compact T10 closeout summaries, decision, verification records, run manifest, and retention policy. Full local run directories, raw inputs, per-case final numerical payloads, source snapshots, and cleanup ledgers remain on the originating workspace and are ignored. Rebuildable binaries, archives, traces, and matrix dumps are not published.

A fresh clone can inspect the reported results and run source-level regression tests. It cannot independently replay `verify_t10_closeout.py` or the per-case evaluator without the retained local payloads and dependencies. Paths and hashes in the frozen manifests describe that original workspace; publishing summaries does not certify that the full evidence bundle is present in Git. See `../T10_CLOSEOUT.md` for the scientific limitations and `t10_closeout_20260910T141607Z/REPRODUCE.md` for local replay guidance.

The original 20101 step2 input subset required by `test/test_t10_closeout.py` is also tracked as a regression fixture (raw UWB/IMU, input/context manifests, generation manifest and independent truth sidecars). Other inputs remain local.
