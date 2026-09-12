#!/usr/bin/env python3
import csv
import io
import tempfile
from pathlib import Path
import unittest
import canonical_injection as c
import run_walk1_smoke as h
from estimator_isolation import measurement_files
import yaml

class InjectionTests(unittest.TestCase):
    def test_boundaries_and_only_range_changes(self):
        header='obs_id,tag_id,anchor_id,sensor_time_s,observed_range_m,source_valid\n'
        times=[c.SPEC['onset_s']-.01,c.SPEC['onset_s'],c.SPEC['offset_s']-.01,c.SPEC['offset_s']]
        lines=[f'{i},27956,20276,{t:.17g},3.5,0\n' for i,t in enumerate(times)]
        payload=(header+''.join(lines)).encode();changed,affected=c.transform(payload)
        self.assertEqual([r['obs_id'] for r in affected],['1','2'])
        before=list(csv.DictReader(io.StringIO(payload.decode())));after=list(csv.DictReader(io.StringIO(changed.decode())))
        for a,b in zip(before,after):
            self.assertEqual(float(b.pop('observed_range_m'))-float(a.pop('observed_range_m')),1. if a['obs_id'] in ['1','2'] else 0.)
            self.assertEqual(a,b)

    def test_window_uses_full_alignment_not_window_refit(self):
        import evaluate_controlled_smoke as e
        import evaluate_runs as ev
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);run=root/'run';run.mkdir();gt=root/'gt.tum'
            truth=[];estimate=[];origin=c.SPEC['onset_s']-8
            for i in range(30):
                t=origin+i+.02;x=.2*i;y=.01*i*i
                delta=1. if c.SPEC['onset_s']<=t<c.SPEC['offset_s'] else 0.
                truth.append(f'{t:.17g} {x} {y} 0 0 0 0 1\n')
                estimate.append(f'{t:.17g} {x+delta+4} {y-2} 0 0 0 0 1\n')
            gt.write_text(''.join(truth));(run/'trajectory.tum').write_text(''.join(estimate))
            m=e.window_metrics(run/'trajectory.tum',ev.load_tum(gt))
            original=ev.trajectory_metrics(run,{**h.PROTOCOL,'ground_truth':str(gt)})
            self.assertAlmostEqual(m['ATE_RMSE_m'],original['aligned_ATE_rmse_m']['value'])
            self.assertEqual(m['window_count'],10)
            self.assertGreater(m['NLOS_window_RMSE_m'],.1)

    def test_real_provenance_and_payload_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);child=c.generate(root/'measurement',root/'private')
            cfg=yaml.safe_load(h.CONFIG.read_text());cfg['dataset']['cache_manifest']=str(child)
            config=root/'config.yaml';config.write_text(yaml.safe_dump(cfg))
            files=measurement_files(config)
            self.assertNotIn(root/'private/injection_truth.json',files)
            self.assertEqual(h.sha(root/'measurement/imu.csv'),h.sha(c.PARENT.parent/'imu.csv'))
            self.assertNotEqual(h.read(child)['cache_id'],h.read(c.PARENT)['cache_id'])
            p=root/'measurement/uwb_observations.csv';p.write_bytes(p.read_bytes().replace(b'27956',b'27957',1))
            with self.assertRaises(ValueError):c.validate_child(child)

if __name__=='__main__':unittest.main()
