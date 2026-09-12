#!/usr/bin/env python3
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
import pandas as pd
import tempfile
from pathlib import Path
from audit_obstacle_ranges import Audit
from audit_obstacle_ranges import geometric_range, interpolate, longest_episode, time_stats, canonical, summarize, starloc_anchor_ids, validate_published_geometry, static_rows


class AuditTests(unittest.TestCase):
    def test_range_sign_and_nonzero_lever(self):
        q = Rotation.from_euler('z', 90, degrees=True)
        antenna = np.array([1., 2., 0.]) + q.apply([1., 0., 0.])
        self.assertAlmostEqual(float(geometric_range(antenna, [1, 6, 0])), 3.)
        d = canonical([0], [10], [1], [3.7], 'fixture')
        d['gt_range_m'] = 3.; d['error_m'] = d.raw_m-d.gt_range_m
        self.assertAlmostEqual(summarize(d)['raw_range_error_median_m'], .7)

    def test_strict_episode_breaks(self):
        x = longest_episode([0, .5, 1, 1.5, 2, 4, 4.5], [.6, .7, np.nan, .8, .9, 1, .8], .5)
        self.assertEqual(x['start_s'], 0.)
        self.assertEqual(x['count'], 2)
        self.assertEqual(x['duration_s'], .5)
        self.assertEqual(longest_episode([0, 1, 2], [.6, .5, .6], .5)['count'], 1)
        self.assertEqual(longest_episode([0, 1, 2], [.6, .6, .6], .5)['duration_s'], 2)

    def test_unmatched_is_not_zero_error(self):
        d = canonical([0, 1], [10, 10], [1, 1], [3, 4], 'fixture')
        d['error_m'] = np.nan
        s = summarize(d)
        self.assertIsNone(s['raw_range_error_rmse_m'])
        self.assertEqual(s['unavailable_error_count'], 2)
        self.assertIsNone(s['positive_gt_0.5_fraction'])

    def test_interpolation_and_gap(self):
        p, _, status = interpolate([0, .025, .1, .2, .3], [0, .05, .2], [[0,0,0],[1,0,0],[4,0,0]])
        self.assertEqual(list(status), ['AVAILABLE', 'AVAILABLE', 'GT_GAP_EXCEEDED', 'AVAILABLE', 'OUTSIDE_GT_SUPPORT'])
        self.assertAlmostEqual(p[1, 0], .5)
        with self.assertRaises(ValueError): interpolate([0], [0,0], [[0,0,0]]*2)

    def test_invalid_gt_endpoint_retained(self):
        _, _, status = interpolate([.01], [0,.02], [[0,0,0],[np.nan,0,0]])
        self.assertEqual(status[0], 'INVALID_GT_ENDPOINT')

    def test_time_audit_is_not_packet_loss(self):
        s = time_stats([0, .1, .1, .05, 2.05])
        self.assertEqual(s['duplicate_steps'], 1)
        self.assertEqual(s['backward_steps'], 1)
        self.assertEqual(s['gaps_gt_1s_count'], 1)

    def test_radio_marker_mapping_rejects_wrong_geometry(self):
        self.assertEqual(starloc_anchor_ids(['4','10','12'], 'v1'), ['12','13','6'])
        self.assertEqual(starloc_anchor_ids(['4','10','12'], 'v2'), ['4','10','12'])
        validate_published_geometry([1.,2.], [1.,2.])
        with self.assertRaises(ValueError): validate_published_geometry([1.,2.], [1.,3.])

    def test_static_footer_and_text_id(self):
        u = pd.DataFrame({'timestamp': ['1', '2', 'Distance Mean', 'bad'],
                          'Distance': [3., 4., np.nan, 5.], 'anchor_id': ['12', '12anchor_id: 12', np.nan, 'unknown']})
        rows, footer = static_rows(u)
        self.assertEqual(footer, 1)
        self.assertEqual(rows.source_row.tolist(), [0,1,3])
        self.assertEqual(rows.anchor_id.tolist(), ['12','12','UNPARSED_SOURCE_ID'])
        self.assertTrue(np.isnan(rows.timestamp.iloc[-1]))

    def test_all_invalid_recording_retained(self):
        with tempfile.TemporaryDirectory() as tmp:
            audit = Audit(Path(tmp))
            df = canonical([np.nan], ['UNKNOWN'], ['UNKNOWN'], [np.nan], 'fixture')
            audit.finish('HUEC_static', 'bad', 'LOS', 'UNAVAILABLE', df, 'fixture')
            self.assertEqual(audit.recordings[0]['sample_count'], 1)
            self.assertEqual(audit.recordings[0]['valid_range_count'], 0)
            self.assertEqual(audit.recordings[0]['status'], 'UNAVAILABLE_GT')
            self.assertIsNone(audit.links[0]['leading_gap_s'])


if __name__ == '__main__': unittest.main()
