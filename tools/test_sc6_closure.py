"""Independent Stanford boundary, receipt gating and statistical-denominator tests."""
import math
import unittest
from sc6_closure import partition, quantile, stanford, spatial, current, metric_stats, score


def receipt(**change):
    r = dict(epoch='1', updates='1', qualified='1', stamp_ns='50000000',
             version='2', generation='1', parent='0', source_prefix='149',
             hpe='1', vpe='1', hpl='2', vpl='2', timely='1', delivery_ms='10',
             status='VALID', mutations='1')
    r.update(change)
    return r


class StanfordContract(unittest.TestCase):
    def test_equalities_and_unavailable_underbound(self):
        cases = [(5, 5, 5, 'nominal_admissible'), (5, 4, 5, 'MI_candidate'),
                 (5.01, 5, 5, 'HMI_candidate'), (9, 8, 5, 'AL_unavailable'),
                 (0, 0, 5, 'nominal_admissible'), (4, 5, 5, 'nominal_admissible')]
        for e, p, a, expected in cases:
            self.assertEqual(partition(e, p, a), expected)
        for e, p, a in [(math.nan, 1, 5), (1, math.inf, 5), (-1, 2, 5), (1, 2, 0)]:
            with self.assertRaises(ValueError):
                partition(e, p, a)

    def test_missing_and_qualified_late_receipts_are_not_fake_points(self):
        rows = [receipt(), receipt(hpe='9', hpl='8'),
                receipt(delivery_ms='51', timely='0'), receipt(qualified='0'),
                receipt(stamp_ns='0'), receipt(hpe='nan'),
                receipt(updates='0', status='SOURCE_GAP_OR_ABORT')]
        c = stanford(rows, 10, 'H', 5)
        self.assertEqual(c['plot_points'], 2)
        self.assertEqual(c['excluded_from_plot'], 8)
        self.assertEqual(c['spatial_qualified'], 3)
        self.assertEqual(c['expired'], 1)
        self.assertEqual(c['AL_unavailable'], 1)
        self.assertEqual(c['underbound'], 1)
        self.assertEqual(c['spatial_underbound'], 1)
        self.assertEqual(c['attempted'], 6)
        self.assertEqual(c['timely_numerical_availability'], .1)
        self.assertFalse(current(rows[2]))
        self.assertTrue(spatial(rows[2]))

    def test_scale_conversion_and_mi_hmi_denominators(self):
        rows = [receipt(hpe='5', hpl='4'), receipt(hpe='6', hpl='5'),
                receipt(hpe='9', hpl='8'), receipt(hpe='5', hpl='5')]
        c = stanford(rows, 4, 'H', 5)
        self.assertEqual([c[k] for k in ['MI_candidate', 'HMI_candidate', 'AL_unavailable', 'nominal_admissible']], [1, 1, 1, 1])
        self.assertEqual(c['underbound'], 3)
        # Millimetre conversion must preserve regions when ALL units change.
        for r in rows:
            self.assertEqual(partition(float(r['hpe'])*1000, float(r['hpl'])*1000, 5000),
                             partition(float(r['hpe']), float(r['hpl']), 5))

    def test_nearest_rank_preserves_slow_admitted_samples(self):
        values = list(range(1, 21))
        self.assertEqual(quantile(values, .95), 19)
        self.assertEqual(quantile(values, .99), 20)
        self.assertEqual(quantile([1]*18+[100, 200], .95), 100)
        self.assertIsNone(quantile([], .95))

    def test_no_output_has_no_metrics_or_stanford_points(self):
        c = stanford([], 12, 'V', 3)
        self.assertEqual(c['excluded_from_plot'], 12)
        self.assertEqual(c['plot_points'], 0)
        self.assertEqual(c['nominal_admissible'], 0)
        self.assertIsNone(metric_stats([])['ape']['rmse'])
        self.assertEqual(metric_stats([])['ape']['n'], 0)

    def test_version_and_source_mismatch_never_protects(self):
        for field, val in [('version', '3'), ('generation', '0'), ('parent', '1'),
                           ('source_prefix', '148'), ('vpl', 'inf')]:
            self.assertFalse(spatial(receipt(**{field: val})))

    def test_actual_decimal_refusal_epoch_has_no_synthetic_point(self):
        row = receipt(epoch='1.000000', updates='0', mutations='0', qualified='0',
                      timely='0', status='ABORT', core_ms='10', hpe='nan', vpe='nan',
                      hpl='nan', vpl='nan')
        result = score([row], {'faults': []}, 3)
        self.assertEqual(result['attempted'], 1)
        self.assertEqual(result['unknown_missing_service'], 2)
        self.assertEqual(result['unattempted'], 0)
        self.assertEqual(result['stanford']['5/5']['H']['plot_points'], 0)


if __name__ == '__main__':
    unittest.main()
