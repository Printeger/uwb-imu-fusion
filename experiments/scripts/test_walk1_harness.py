#!/usr/bin/env python3
import copy
from pathlib import Path
import tempfile
import unittest
import yaml
import run_walk1_smoke as h
import evaluate_runs as ev


class HarnessTests(unittest.TestCase):
    def test_statuses(self):
        for value in ['OK','NO_CANDIDATES']:
            self.assertEqual(h.classify({'status':value,'exit_code':0},'COMPLETE'),'success')
        self.assertEqual(h.classify({'status':'FALLBACK_OK','valid_estimate_exported':True,'exit_code':0},''),'fallback')
        self.assertEqual(h.classify({'status':'FAILED','exit_code':1},''),'failure')
        self.assertEqual(h.classify({},'PARENT_CACHE_UNAVAILABLE'),'failure')

    def test_complete_identity(self):
        a={'yaml':{'sigma':0.15},'method':'robust_cauchy','env':{'scale':2.3849},'parent':'abc'}
        for k,v in [('method','all_range'),('env',{'scale':3}),('parent','def'),('yaml',{'sigma':0.2})]:
            b=copy.deepcopy(a);b[k]=v
            self.assertNotEqual(h.digest(a),h.digest(b))
        self.assertEqual(h.digest(a),h.digest(dict(reversed(list(a.items())))))

    def test_existing_evaluator_rigid_transform(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);gt=root/'gt.tum';run=root/'run';run.mkdir()
            truth=[];estimate=[]
            for i in range(10):
                t=i*0.5;x=t;y=0.1*t*t
                truth.append(f'{t} {x} {y} 0 0 0 0 1\n')
                estimate.append(f'{t} {x+4} {y-2} 1 0 0 0 1\n')
            gt.write_text(''.join(truth));(run/'trajectory.tum').write_text(''.join(estimate))
            m=ev.trajectory_metrics(run,{**h.PROTOCOL,'ground_truth':str(gt)})
            self.assertLess(m['aligned_ATE_rmse_m']['value'],1e-10)
            self.assertLess(m['RPE_rmse_m']['value'],1e-10)
            self.assertEqual(m['matched_count']['value'],10)

    def test_missing_trajectory_is_unavailable(self):
        with tempfile.TemporaryDirectory() as tmp:
            m=ev.trajectory_metrics(Path(tmp),{})
            self.assertIsNone(m['aligned_ATE_rmse_m']['value'])

    def test_frozen_provider_path(self):
        import run_experiments as scheduler
        cell={'mode':'suppress_all','execution_type':'FINAL_TRAJECTORY',
              'path':'PL_BIDIRECTIONAL_CUSUM','producer_id':'shared','thresholds':{}}
        scheduler.validate_mode_execution_path(cell)
        self.assertEqual(cell['cache_namespace'],'PL_BIDIRECTIONAL_CUSUM')
        cell['cache_namespace']='AUTO_DISCOVERY'
        with self.assertRaises(ValueError):
            scheduler.validate_mode_execution_path(cell)
        config=yaml.safe_load(h.CONFIG.read_text())
        for key,value in scheduler.PL_BIDIRECTIONAL_LOCKED_OVERRIDE.items():
            self.assertEqual(config['nlos'][key],value)


if __name__=='__main__':unittest.main()
