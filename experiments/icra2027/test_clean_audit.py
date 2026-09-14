import unittest

import numpy as np
from evaluate_clean import associate, score


def pose(t, p):
    return (float(t), np.array(p, dtype=float), np.array([0., 0., 0., 1.]))


class CleanAuditTest(unittest.TestCase):
    def test_known_rigid_transform_is_evaluator_only(self):
        points = [[0, 0, 0], [1, 0, 0], [0, 2, 0], [1, 2, 1]]
        rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        translation = np.array([3., 4., 2.])
        est = [pose(i, p) for i, p in enumerate(points)]
        gt = [pose(i, rotation @ np.array(p) + translation) for i, p in enumerate(points)]
        before = [e[1].copy() for e in est]
        m, _, fit = score(associate(est, gt, 0., 3.), [0., 1., 2., 3.])
        self.assertLess(m['ATE_RMSE_aligned'], 1e-12)
        self.assertGreater(m['unaligned_coordinate_RMSE_m'], 1.)
        self.assertIsNone(m['ATE_RMSE_raw'])
        np.testing.assert_allclose(fit['rotation'], rotation, atol=1e-12)
        for p, e in zip(before, est):
            np.testing.assert_array_equal(p, e[1])

    def test_nearest_duplicate_uses_closest_then_earliest(self):
        gt = [pose(1., [0, 0, 0])]
        estimates = [pose(.99, [1, 0, 0]), pose(.999, [2, 0, 0]), pose(1.005, [3, 0, 0])]
        selected = associate(estimates, gt, 0., 2.)
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[1.][0][0], .999)

    def test_no_matching_and_degenerate_are_unavailable(self):
        self.assertEqual(associate([pose(0, [0, 0, 0])], [pose(1, [0, 0, 0])], 0, 2), {})
        with self.assertRaises(ValueError):
            score({}, [])
        mapping = {float(i): (pose(i, [i, 0, 0]), pose(i, [i, 0, 0])) for i in range(3)}
        with self.assertRaises(ValueError):
            score(mapping, [0., 1., 2.])

    def test_common_gt_intersection_preserves_exact_identity(self):
        gt = [pose(i, [i % 2, i // 2, 0]) for i in range(4)]
        dense = associate(gt, gt, 0, 3)
        sparse = associate(gt[1:], gt, 0, 3)
        common = sorted(set(dense) & set(sparse))
        a, _, _ = score(dense, common)
        b, _, _ = score(sparse, common)
        self.assertEqual(a['common_gt_sha256'], b['common_gt_sha256'])
        self.assertEqual(a['n_gt_samples'], 3)


if __name__ == '__main__':
    unittest.main()
