#!/usr/bin/env python3
"""Engineering rejection tests for the static manifest validator."""
import copy
from collections import Counter
import tempfile
import unittest
from pathlib import Path

from validate_manifest import DEFAULT, read_json, validate


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self.doc = read_json(DEFAULT)

    def test_current_inventory(self):
        self.assertEqual(validate(self.doc)[0], [])
        counts = Counter(item['dataset_family'] for item in self.doc['recordings'])
        self.assertEqual(counts, {'HUEC': 8, 'MILUV': 3, 'own_vicon': 3,
                                  'SFUISE': 3, 'starloc': 22})

    def test_measurement_rejection(self):
        for value in ('TDoA', 'TDOA', 'tdoa', 'TDOA_RANGE_DIFFERENCE', 'UNKNOWN'):
            with self.subTest(value=value):
                document = copy.deepcopy(self.doc)
                document['recordings'][0]['uwb_measurement_type'] = value
                self.assertTrue(validate(document)[0])

    def test_all_absolute_types(self):
        for value in ('ABSOLUTE_TOA', 'ABSOLUTE_TWR', 'ABSOLUTE_RANGE'):
            document = copy.deepcopy(self.doc)
            document['recordings'][0]['uwb_measurement_type'] = value
            self.assertFalse(validate(document)[0])

    def test_every_recording_field_is_required(self):
        for key in self.doc['recordings'][0]:
            with self.subTest(key=key):
                document = copy.deepcopy(self.doc)
                del document['recordings'][0][key]
                self.assertTrue(validate(document)[0])

    def test_all_families_are_required(self):
        document = copy.deepcopy(self.doc)
        document['recordings'] = [item for item in document['recordings']
                                  if item['dataset_family'] != 'HUEC']
        self.assertTrue(any('missing dataset families' in item
                            for item in validate(document)[0]))

    def test_development_recording_cannot_be_test(self):
        recording = next(item for item in self.doc['recordings']
                         if item['role'] == 'DEV')
        recording['role'] = 'TEST'
        self.assertTrue(validate(self.doc)[0])

    def test_unknown_exposure_remains_unassigned(self):
        recording = next(item for item in self.doc['recordings']
                         if item['used_during_method_development'] is None)
        recording['role'] = 'TEST'
        recording['split_group_id'] = 'invented-test'
        self.assertTrue(validate(self.doc)[0])

    def test_duplicate_identity_and_raw_alias_are_rejected(self):
        self.doc['recordings'].append(copy.deepcopy(self.doc['recordings'][0]))
        errors = validate(self.doc)[0]
        self.assertTrue(any('duplicate recording identity' in item for item in errors))
        self.assertTrue(any('duplicate raw path' in item for item in errors))

    def test_unknown_value_cannot_be_zero_or_identity(self):
        recording = next(item for item in self.doc['recordings']
                         if item['tag_imu_extrinsics']['status'] == 'UNKNOWN')
        recording['tag_imu_extrinsics']['value'] = {'translation_m': [0, 0, 0]}
        self.assertTrue(validate(self.doc)[0])

    def test_invalid_units_and_nonfinite_are_rejected(self):
        recording = self.doc['recordings'][0]
        recording['time_units']['value']['uwb'] = 'ticks'
        self.assertTrue(validate(self.doc)[0])
        self.setUp()
        recording = next(item for item in self.doc['recordings']
                         if item['tag_imu_extrinsics']['value'] is not None)
        recording['tag_imu_extrinsics']['value']['tag_levers_m']['10'][0] = float('inf')
        self.assertTrue(validate(self.doc)[0])

    def test_local_issue_status_must_agree(self):
        recording = next(item for item in self.doc['recordings']
                         if item['local_validation']['issues'])
        recording['local_validation']['issues'] = []
        self.assertTrue(validate(self.doc)[0])

    def test_primary_file_hash_binding(self):
        self.doc['recordings'][0]['recording_files'][0]['sha256'] = '0' * 64
        self.assertTrue(validate(self.doc)[0])

    def test_path_and_hash_failures_are_reported(self):
        recording = self.doc['recordings'][0]
        recording['raw_data_path'] = 'data/HUEC/missing.bag'
        recording['recording_files'][0]['path'] = 'data/HUEC/missing.bag'
        errors = validate(self.doc, check_paths=True)[0]
        self.assertTrue(any('missing file or broken symlink' in item for item in errors))

    def test_incomplete_tdoa_rejection_policy(self):
        self.doc['measurement_policy']['rejected'] = ['TDOA']
        self.assertTrue(validate(self.doc)[0])

    def test_bad_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'bad.json'
            for text in ('{"x":1,"x":2}', '{"x":NaN}', '{'):
                path.write_text(text)
                with self.assertRaises(ValueError):
                    read_json(path)


if __name__ == '__main__':
    unittest.main()
