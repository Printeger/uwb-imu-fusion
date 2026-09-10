#!/usr/bin/env python3

import sys
import unittest

from check_paper_loader_identity import raw_time_abs_tolerance, raw_times_match


class RawTimeComparisonTest(unittest.TestCase):
    def test_epoch_double_roundoff_is_accepted(self):
        epoch_time = 1664959673.1234567
        adjacent = epoch_time + sys.float_info.epsilon * abs(epoch_time)
        self.assertLessEqual(
            abs(adjacent - epoch_time),
            raw_time_abs_tolerance(epoch_time, adjacent),
        )
        self.assertTrue(raw_times_match(epoch_time, adjacent))

    def test_half_second_epoch_mutation_is_rejected(self):
        epoch_time = 1664959673.1234567
        self.assertFalse(raw_times_match(epoch_time, epoch_time + 0.5))


if __name__ == "__main__":
    unittest.main()
