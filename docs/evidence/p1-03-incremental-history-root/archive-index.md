# P1-03 archived raw evidence index

This index was created during an evidence-only repair after the second
independent review.  No build, test, benchmark, soak or comparator was rerun.
The existing candidate files were copied byte-for-byte; their contents and
hashes were not edited.

## Archived runs

- `archived-raw/candidate-3x45/run1`, `run2`, and `run3`: the three 45-attempt
  candidate runs used by the retained Section 2.4 comparisons.
- `archived-raw/candidate-3x45/run1.stdout` through `run3.stdout` and matching
  `.time` files: original command output and resource measurements.
- `archived-raw/candidate-120/run`: the final 120-attempt bounded-RSS soak.
- `archived-raw/candidate-120/run.stdout` and `run.time`: original soak output
  and resource measurement.

The generated run manifests retain the producer's original absolute command
and output paths.  Those strings are immutable provenance, not live evidence
references.  The checksum manifests reference only the archived copies.

`equivalence-run1.json` through `equivalence-run3.json` and
`all-attempt-performance.json` are the retained comparison and aggregate
records.  The second independent reviewer produced three fresh comparator
results; the observed files were byte-identical to the three retained
equivalence JSON files.  No standalone reviewer report was provided, so the
reviewer observation is explicitly identified as such in `command-record.txt`.

## Checksum working directories

From the repository root:

```text
sha256sum -c docs/evidence/p1-03-incremental-history-root/lifecycle-source-hashes.sha256
sha256sum -c docs/evidence/p1-03-incremental-history-root/source-hashes.sha256
sha256sum -c docs/evidence/p1-03-incremental-history-root/input-config-truth-hashes.sha256
sha256sum -c docs/evidence/p1-03-incremental-history-root/output-hashes.sha256
sha256sum -c docs/evidence/p1-03-incremental-history-root/artifact-hashes.sha256
```

`binary-hashes.sha256` and `loaded-library-hashes.txt` preserve validation-host
binary provenance required by Section 2.5.  Build products are intentionally
not committed as evidence artifacts.  The two ephemeral ABI-client executable
digests were `57c907065ba307b8978d8d2e37026d51d609d0590c218f11a335e8d90fe28919`
and `08b653eb407f5f3123e4e0c00718c6d961a65a8779681ccbd873da1d5c97112d`;
their archived stdout is `abi-history.log` and `abi-p005.log`.

Run the artifact manifest from the repository root.  It deliberately hashes
the archive index, all top-level evidence records and every archived raw file,
but excludes itself to avoid a recursive digest.

