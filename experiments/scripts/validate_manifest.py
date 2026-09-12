#!/usr/bin/env python3
"""Static validation for the five-family dataset inventory.

The validator never edits the manifest or source data, never launches an
estimator, and never performs network access.  Optional path/hash checks only
read bytes.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / 'experiments/datasets/dataset_manifest.schema.json'
DEFAULT = ROOT / 'experiments/datasets/dataset_manifest.json'
FAMILIES = {'HUEC', 'MILUV', 'own_vicon', 'SFUISE', 'starloc'}
ABSOLUTE_TYPES = {'ABSOLUTE_TOA', 'ABSOLUTE_TWR', 'ABSOLUTE_RANGE'}
FACT_FIELDS = {
    'tag_ids', 'anchor_ids', 'anchor_coordinates_source', 'imu_source',
    'gt_source', 'gt_reference_point', 'tag_imu_extrinsics', 'gt_extrinsics',
    'time_units', 'time_synchronization', 'range_calibration',
    'physical_obstruction', 'license',
}
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def read_json(path):
    def bad_constant(value):
        raise ValueError(f'nonfinite JSON constant: {value}')
    return json.loads(path.read_text(), object_pairs_hook=unique_object,
                      parse_constant=bad_constant)


def finite(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(finite(item) for item in value)
    if isinstance(value, dict):
        return all(finite(item) for item in value.values())
    return True


def safe_data_path(value, family):
    path = Path(value)
    return (not path.is_absolute() and '..' not in path.parts and
            len(path.parts) >= 3 and path.parts[:2] == ('data', family))


def file_hash(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def validate(document, root=ROOT, check_paths=False, check_hashes=False):
    from jsonschema import Draft7Validator

    schema = read_json(SCHEMA)
    Draft7Validator.check_schema(schema)
    errors = [f'{"/".join(map(str, item.absolute_path))}: {item.message}'
              for item in Draft7Validator(schema).iter_errors(document)]
    warnings = []
    if errors:
        return errors, warnings

    root = root.resolve()
    identities = set()
    raw_paths = {}
    raw_hashes = {}
    split_groups = {}
    family_counts = Counter()
    hash_cache = {}

    policy = document['measurement_policy']
    rejected_normalized = {item.upper().replace('-', '_') for item in policy['rejected']}
    if not {'TDOA', 'TIME_DIFFERENCE_OF_ARRIVAL', 'RANGE_DIFFERENCE'} <= rejected_normalized:
        errors.append('measurement_policy: TDoA/range-difference rejection is incomplete')

    artifact = document['inventory_artifact']
    artifact_path = Path(artifact['path'])
    if (artifact_path.is_absolute() or '..' in artifact_path.parts or
            artifact_path.parts[:2] != ('experiments', 'datasets')):
        errors.append('inventory_artifact: path must stay under experiments/datasets')
    artifact_resolved = (root / artifact_path).resolve()
    if check_paths or check_hashes:
        if not artifact_resolved.is_file():
            errors.append(f'inventory_artifact: missing file: {artifact_path}')
        elif check_hashes:
            actual = file_hash(artifact_resolved)
            if actual != artifact['sha256']:
                errors.append('inventory_artifact: SHA-256 mismatch')

    for recording in document['recordings']:
        family = recording['dataset_family']
        rid = recording['recording_id']
        identity = (family, rid)
        family_counts[family] += 1
        if identity in identities:
            errors.append(f'{family}/{rid}: duplicate recording identity')
        identities.add(identity)

        measurement_type = recording['uwb_measurement_type']
        if measurement_type not in ABSOLUTE_TYPES:
            errors.append(f'{family}/{rid}: non-absolute or unknown UWB type rejected')

        for name in FACT_FIELDS:
            value = recording[name]
            unknown = value['status'] in ('UNKNOWN', 'NOT_APPLICABLE')
            if unknown != (value['value'] is None):
                errors.append(f'{family}/{rid}/{name}: status/value inconsistent')
            if not finite(value['value']):
                errors.append(f'{family}/{rid}/{name}: nonfinite numeric value')

        for name in ('tag_ids', 'anchor_ids'):
            value = recording[name]['value']
            if value is not None:
                if (not isinstance(value, list) or not value or
                        any(not isinstance(item, str) or not item for item in value) or
                        len(value) != len(set(value))):
                    errors.append(f'{family}/{rid}/{name}: expected nonempty unique string IDs')

        units = recording['time_units']['value']
        if units is not None:
            if (not isinstance(units, dict) or set(units) != {'uwb', 'imu', 'gt'} or
                    any(value not in {'s', 'ms', 'us', 'ns'} for value in units.values())):
                errors.append(f'{family}/{rid}/time_units: invalid or incomplete unit map')

        role = recording['role']
        used = recording['used_during_method_development']
        group = recording['split_group_id']
        if used is True and role != 'DEV':
            errors.append(f'{family}/{rid}: development-used recording must be DEV')
        if role == 'DEV' and used is not True:
            errors.append(f'{family}/{rid}: DEV role requires confirmed development use')
        if used is None and role != 'UNASSIGNED':
            errors.append(f'{family}/{rid}: unknown development exposure requires UNASSIGNED role')
        if role in ('VAL', 'TEST') and used is not False:
            errors.append(f'{family}/{rid}: VAL/TEST requires confirmed no development use')
        if (role == 'UNASSIGNED') != (group is None):
            errors.append(f'{family}/{rid}: split_group_id must be null exactly when role is UNASSIGNED')
        if group is not None:
            prior = split_groups.setdefault(group, (role, used))
            if prior != (role, used):
                errors.append(f'{family}/{rid}: split group crosses role/development boundary')

        validation = recording['local_validation']
        issues = validation['issues']
        status_with_issues = recording['metadata_status'].endswith('_WITH_RECORDED_ISSUES')
        outcome_with_issues = validation['outcome'] == 'PASS_WITH_RECORDED_ISSUES'
        if bool(issues) != status_with_issues or bool(issues) != outcome_with_issues:
            errors.append(f'{family}/{rid}: local validation issue/status mismatch')
        if not validation['artifact_pointer'].startswith(artifact['path'] + '#'):
            errors.append(f'{family}/{rid}: local validation pointer does not bind inventory artifact')

        raw = recording['raw_data_path']
        if not safe_data_path(raw, family):
            errors.append(f'{family}/{rid}: raw path must be repository-relative data/{family}/...')
        raw_resolved = (root / raw).resolve()
        for key, seen, label in ((str(raw_resolved), raw_paths, 'raw path'),
                                 (recording['raw_sha256'], raw_hashes, 'raw hash')):
            if key in seen:
                errors.append(f'{family}/{rid}: duplicate {label} alias of {seen[key]}')
            seen[key] = f'{family}/{rid}'

        file_keys = set()
        listed_primary = False
        for item in recording['recording_files']:
            path = item['path']
            key = (item['role'], path)
            if key in file_keys:
                errors.append(f'{family}/{rid}: duplicate recording file role/path')
            file_keys.add(key)
            if not safe_data_path(path, family):
                errors.append(f'{family}/{rid}: support path escapes data/{family}: {path}')
            if path == raw:
                listed_primary = True
                if item['sha256'] != recording['raw_sha256']:
                    errors.append(f'{family}/{rid}: primary path hash differs from raw_sha256')
            resolved = (root / path).resolve()
            if check_paths or check_hashes:
                if not resolved.is_file():
                    errors.append(f'{family}/{rid}: missing file or broken symlink: {path}')
                elif check_hashes:
                    actual = hash_cache.get(str(resolved))
                    if actual is None:
                        actual = file_hash(resolved)
                        hash_cache[str(resolved)] = actual
                    if actual != item['sha256']:
                        errors.append(f'{family}/{rid}: SHA-256 mismatch: {path}')
        if not listed_primary:
            errors.append(f'{family}/{rid}: raw_data_path absent from recording_files')

        if issues:
            warnings.append(f'{family}/{rid}: LOCAL_PASS_WITH_RECORDED_ISSUES; {"; ".join(issues)}')
        if recording['unresolved_items']:
            warnings.append(
                f'{family}/{rid}: NOT_ADMITTED; {len(recording["unresolved_items"])} unresolved item(s)')

    missing = FAMILIES - set(family_counts)
    extra = set(family_counts) - FAMILIES
    if missing:
        errors.append(f'missing dataset families: {sorted(missing)}')
    if extra:
        errors.append(f'unexpected dataset families: {sorted(extra)}')
    return errors, warnings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path, nargs='?', default=DEFAULT)
    parser.add_argument('--repo-root', type=Path, default=ROOT)
    parser.add_argument('--check-paths', action='store_true',
                        help='Read-only existence and symlink-target checks')
    parser.add_argument('--check-hashes', action='store_true',
                        help='Read-only SHA-256 checks; implies path checks')
    args = parser.parse_args()
    try:
        document = read_json(args.manifest)
        errors, warnings = validate(document, args.repo_root,
                                    args.check_paths, args.check_hashes)
    except (OSError, ValueError, ImportError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    for warning in warnings:
        print(f'WARNING: {warning}')
    for error in errors:
        print(f'ERROR: {error}', file=sys.stderr)
    counts = Counter(item['dataset_family'] for item in document.get('recordings', []))
    family_text = ','.join(f'{name}:{counts[name]}' for name in sorted(counts))
    print(f'STATIC_VALIDATION_{"FAIL" if errors else "PASS"}; '
          f'errors={len(errors)}; warnings={len(warnings)}; '
          f'recordings={sum(counts.values())}; families={family_text}; '
          f'FIVE_FAMILY_MANIFEST_VALIDATION={"FAIL" if errors else "PASS"}; '
          'EXPERIMENT_ADMISSION=NOT_GRANTED; ESTIMATOR=NOT_RUN')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main())
