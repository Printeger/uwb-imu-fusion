# Refactor Gate 02 Report — Measurement Semantics Decoupling

Status: `PASS / MEASUREMENT_SEMANTICS_DECOUPLED / GATE_03_READY`

Scope: Prompt 2 in `doc/v3/v3.md`. This gate changes input and measurement
planning semantics only. It does not change IE detection, grouping, refit,
recoverability, inference, solver mathematics, thresholds, or scientific locks.
No experiment matrix or scientific evaluation was run.

## Previous behavior

`BuildPaperInputPlan` previously stored source facts and estimator decisions in
one `ObservationRecord`. The fields `valid`, `planned`, `keyframe_id`, and
`nominal_sigma` therefore mixed source validity, model usability, state
association, selection, and factor noise.

For a planned observation, the paper input path computed effective range noise
from the time since the previously planned observation on the same tag-anchor
link:

```text
gap = max(0, current_time - previous_planned_same_link_time)
sigma_eff = sqrt(sigma_range^2 + (max_velocity * gap / 3)^2)
```

The first planned observation on a link received no gap term. Consequently,
changing `keyframe.step` changed both the selected measurement set and the sigma
of observations common to two plans. Gate 01 characterization tests froze this
behavior before it was removed from the paper path.

## New data model

`ObservationRecord` is now the raw source ledger row. It contains stable
identity and source provenance (`obs_id`, ledger/source ordinals, source frame
and time), raw payload (`tag_id`, `anchor_id`, range and RSSI), source validity
with a reason, and the diagnostic `suspected_nlos` label. It contains no state
key, selection flag, or estimator noise.

`MeasurementPlanEntry` is a separate row keyed back to the ledger by both
`obs_id` and `observation_index`. It contains:

- `estimator_usable` and its reason;
- `selected` and its reason;
- the associated `keyframe_id` when selected;
- fixed `sensor_sigma`.

`PaperInputPlan` now records explicit state-density and association policies and
separate `ledger_hash`, `state_plan_hash`, and `measurement_plan_hash` values.
Its composite identity is versioned as `paper_input_v3`.

## Exact validity and selection semantics

Source-valid means the loader/source-protocol row is valid and source time,
range, first-path RSSI, and receive RSSI are finite. A source-invalid row remains
in the ledger with a stable reason.

Estimator-usable means source-valid plus all of the following:

1. the tag and anchor can be resolved by the configured estimator model;
2. the raw range is within the configured inclusive minimum and maximum range.

Estimator selection is separate from both definitions. The state timeline is
constructed from usable source frames according to the configured
`keyframe.step` and minimum-time policy. An estimator-usable observation is
selected only when its exact source frame belongs to that timeline; otherwise
the measurement plan retains it with an explicit non-selection reason. Thus
`state_step > 1` is an explicit measurement-subsampling policy, not source
invalidity. With `state_step = 1`, every estimator-usable UWB observation is
selected.

`suspected_nlos` remains diagnostic and does not make an observation invalid or
unusable. This gate deliberately does not invent the statistically-independent
or duplicate/stale-payload policy reserved by Contract C.

## Sensor sigma before and after

Before this gate, the paper path used the selected-gap-adaptive formula above.
After this gate:

```text
sensor_sigma(observation) = Config::sigma_range
```

This value is assigned for every ledger row and is invariant to state density,
selection order, and same-link gaps. The current configuration has one UWB
range sigma; `Anchor::prior_sigma` is an anchor-position prior and is not
reinterpreted as range-sensor noise.

The graph input type now carries an explicit `UwbNoiseSemantics` marker:

- `FIXED_SENSOR_SIGMA_V2` uses the materialized sensor sigma unchanged;
- `LEGACY_SELECTED_GAP_ADAPTIVE_V1` preserves the previous graph-builder
  calculation for legacy/development callers.

This keeps old behavior available and named without allowing it to leak into
the new paper path.

## Compatibility and semantic mapping

The old-to-new semantic mapping is:

| Previous field/identity | Gate 02 representation |
|---|---|
| `ObservationRecord::valid` | raw `source_valid` plus downstream `estimator_usable` |
| `validity_reason` | `source_validity_reason` or `estimator_usability_reason` |
| `planned` | `MeasurementPlanEntry::selected` |
| `keyframe_id` | `MeasurementPlanEntry::keyframe_id` |
| `nominal_sigma` | `MeasurementPlanEntry::sensor_sigma` |
| one combined plan hash | ledger, state-plan, and measurement-plan hashes plus a versioned composite hash |

The existing `observations.csv` schema is retained at the artifact boundary:
its historical `valid` column maps to estimator usability, `planned` maps to
measurement selection, and `nominal_sigma` carries fixed sensor sigma.
`AllPlannedObservationMask` remains as a compatibility alias for the explicitly
named selected-observation mask. Existing result directories are untouched and
are not reinterpreted. The versioned composite hash prevents new plans from
silently sharing legacy cache identity.

Removing estimator fields from `ObservationRecord` required mechanical mapping
updates in Stage 1, refit, inference, scoring, PL, paper-runner, preflight, and
forensic-tool call sites and fixtures. Those sites now obtain estimator facts
through `MeasurementForObservation`. No formulas, gates, thresholds, factor
definitions, or optimizer settings were changed in those components.

## Verification

Production package build:

```text
catkin build uwb_imu_fgo --no-deps --summarize
exit: 0
result: all requested packages succeeded
log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/build.make.197.log
```

Package test suite:

```text
catkin test uwb_imu_fgo --no-deps --summarize
exit: 0
result: 480 tests, 0 errors, 0 failures, 0 skipped
result log: /home/mint/ws_fusion_uwb/logs/uwb_imu_fgo/test.results.035.log
```

Gate 02 adds direct regression coverage for:

- raw ledger rows, ordering, hashes, and stable `obs_id`s across state steps;
- sensor sigma invariance across state steps;
- explicit state/measurement-plan change without source-ledger mutation;
- `state_step=1` selection of every estimator-usable UWB observation;
- explicit retention of the legacy adaptive-noise mode.

The focused Gate 02 invocation ran all five tests above: `5 tests, 5 passed`.

The pre-existing duplicate-payload characterization remains unchanged: distinct
source rows still receive distinct `obs_id`s and the statistical independence
policy remains future work. No test failure was waived, and no scientific run
was used to tune or validate the refactor.

## Gate decision

Gate 02 acceptance criteria are satisfied. Prompt 3 may proceed only after a new
explicit user instruction. Its work must not be inferred as part of this gate.
