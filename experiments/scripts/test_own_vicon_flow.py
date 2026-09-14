#!/usr/bin/env python3
import unittest
import numpy as np

import run_own_vicon_flow as own


class OwnViconFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.origin, cls.imu, cls.uwb = own.extract_measurements()

    def test_full_source_cardinality_and_ids(self):
        self.assertEqual(len(self.imu), 14680)
        self.assertEqual(len(self.uwb), 14479)
        self.assertEqual(len({row['obs_id'] for row in self.uwb}), len(self.uwb))
        self.assertEqual(len({row['source_message_index'] for row in self.uwb}), 3669)

    def test_units_and_common_start(self):
        norms = np.linalg.norm([[row['acc_x_mps2'],row['acc_y_mps2'],row['acc_z_mps2']]
                                for row in self.imu[:200]], axis=1)
        self.assertGreater(float(np.median(norms)), 9.0)
        self.assertLess(float(np.median(norms)), 10.5)
        gyro = np.abs([[row['gyro_x_radps'],row['gyro_y_radps'],row['gyro_z_radps']]
                       for row in self.imu])
        self.assertLess(float(np.percentile(gyro, 99.9)), 20.)
        self.assertAlmostEqual(min(float(self.imu[0]['sensor_time_s']),
                                   float(self.uwb[0]['sensor_time_s'])), 0., places=12)

    def test_anchor_geometry(self):
        anchors, spans = own.anchor_geometry()
        self.assertEqual(set(anchors), {1,2,3,4})
        self.assertLess(max(spans.values()), .03)

    def test_cache_identity_uses_original_grouping(self):
        anchors, _ = own.anchor_geometry()
        manifest, _, _ = own.cache_payloads(self.origin, self.imu, self.uwb, anchors)
        self.assertEqual(manifest['uwb_observation_count'], 14479)
        self.assertEqual(manifest['uwb_message_count'], 3669)
        self.assertEqual(manifest['recording_time_origin_s'], 0.)


if __name__ == '__main__':
    unittest.main()
