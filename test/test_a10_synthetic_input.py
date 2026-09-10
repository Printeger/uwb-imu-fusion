#!/usr/bin/env python3
"""Adversarial generator checks, no estimator processes or held-out data."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/paper'))
import generate_synthetic_input as gen
import check_synthetic_input as checker


class SyntheticInputTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='a10-adversarial-')
        self.root=Path(self.temp.name)/'input'
        gen.generate(self.root,10101)

    def tearDown(self):
        self.temp.cleanup()

    def rehash(self,case):
        raw=self.root/'raw'/case
        mp=raw/'input_manifest.json'
        m=json.loads(mp.read_text())
        m['uwb_sha256']=gen.file_sha(raw/'uwb_observations.csv')
        m['cache_id']=gen.cache_id(m)
        gen.write_json(mp,m)
        gp=self.root/'generation_manifest.json'
        g=json.loads(gp.read_text())
        g['scenarios'][case]['cache_id']=m['cache_id']
        for p in (mp,raw/'uwb_observations.csv'):
            g['payload_sha256'][str(p.relative_to(self.root))]=gen.file_sha(p)
        gen.write_json(gp,g)

    def change_raw(self,column,value):
        path=self.root/'raw/step/uwb_observations.csv'
        data=checker.rows(path)
        data[0][column]=value
        gen.write_csv(path,list(data[0]),[list(r.values()) for r in data])
        self.rehash('step')

    def test_measurement_corruption_with_resealed_hashes(self):
        self.change_raw('observed_range_m','123')
        with self.assertRaisesRegex(AssertionError,'raw UWB'):
            checker.verify(self.root)

    def test_time_corruption_with_resealed_hashes(self):
        self.change_raw('sensor_time_s','.01')
        with self.assertRaises(AssertionError):
            checker.verify(self.root)

    def test_obs_id_corruption_with_resealed_hashes(self):
        self.change_raw('obs_id','123')
        with self.assertRaises(AssertionError):
            checker.verify(self.root)

    def test_invalid_seed_and_overwrite_do_not_mutate(self):
        old=gen.file_sha(self.root/'generation_manifest.json')
        for seed in (0,-1,True,1.5):
            with self.assertRaises(ValueError):
                gen.generate(Path(self.temp.name)/'invalid',seed)
        with self.assertRaises(FileExistsError):
            gen.generate(self.root,10101)
        self.assertEqual(old,gen.file_sha(self.root/'generation_manifest.json'))

    def test_exact_noise_stream_identity_and_units(self):
        import numpy as np
        seed=10101
        t=checker.rows(self.root/'evaluation/los_range_truth.csv')
        uwb=checker.matrix(t,['range_noise_m']).ravel()
        imu=checker.rows(self.root/'evaluation/imu_latent.csv')
        for stream,actual,sigma,shape in [
                (0,uwb,.05,(41,8)),
                (1,checker.matrix(imu,['noise_ax','noise_ay','noise_az']),.002/np.sqrt(.005),(1601,3)),
                (2,checker.matrix(imu,['noise_gx','noise_gy','noise_gz']),.0002/np.sqrt(.005),(1601,3))]:
            expected=np.random.Generator(np.random.PCG64(np.random.SeedSequence([seed,stream]))).normal(0.,sigma,shape)
            np.testing.assert_array_equal(actual.ravel(),expected.ravel())
            self.assertLess(abs(float(np.mean(actual))),5*sigma/np.sqrt(actual.size))
            self.assertTrue(.7 < float(np.std(actual))/sigma < 1.3)


if __name__=='__main__':
    unittest.main()
