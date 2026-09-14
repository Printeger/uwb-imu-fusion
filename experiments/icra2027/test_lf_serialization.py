import unittest
from formal_r6_lf import normalize


class LFSerializationTests(unittest.TestCase):
    def test_preserves_numeric_spelling_and_empty_final_column(self):
        raw=b'obs_id,sensor_time_s,range,reason\r\n7,1664959684.9745398,1.00000000000001,\r\n'
        self.assertEqual(normalize(raw),raw.replace(b'\r\n',b'\n'))
        self.assertIn(b'1.00000000000001,\n',normalize(raw))

    def test_idempotent(self):
        self.assertEqual(normalize(b'a,b\n1,2\n'),b'a,b\n1,2\n')

    def test_bare_cr_rejected(self):
        with self.assertRaises(ValueError):normalize(b'a,b\n1,2\rbroken\n')


if __name__=='__main__':unittest.main()
