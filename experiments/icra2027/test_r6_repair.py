"""Infrastructure regression tests on sealed fixtures; no estimator/metric computation."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import yaml
import run_r6_case as runner
import formal_treatment as adapter
from formal_native import HERE, read, sha
from test_formal_adapter import TreatmentTests


class RepairTests(unittest.TestCase):
    def test_scientific_vs_infrastructure_classification(self):
        self.assertTrue(runner.science_failure({'backend':{'reason':'PRELIMINARY_LM_FAILED:CONDITIONAL_LM_MAX_ITERATIONS'}})[0])
        self.assertFalse(runner.science_failure({'backend':{'reason':'PARENT_CACHE_UNAVAILABLE'}})[0])
        self.assertFalse(runner.science_failure({'backend':{'reason':'missing path'}})[0])
        self.assertTrue(runner.science_failure({'backend':{'solver_status':'MAX_REFIT_ITERATIONS'}})[0])

    def test_resume_rejects_mutated_payload(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'payload').write_text('frozen');runner.seal(p);runner.verify_seal(p)
            (p/'payload').write_text('mutated')
            with self.assertRaises(AssertionError):runner.verify_seal(p)

    def test_real_export_and_three_derived_inputs(self):
        case=next(c for c in runner.load()['cases'] if c['case_id']=='Walk1_A20276')
        native=read(HERE/'runs/R6-LF-v2/results.json')[case['case_id']]
        p=Path(native['AUX_PRODUCER']['run']);d=read(p/'production_detector_status.json')
        parent={'support_hash':d['support_hash'],'detector_hash':sha(p/'production_detector_status.json'),'parent_hash':'fixture-parent'}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'out';out.mkdir()
            with patch.object(runner,'HERE',root),patch.object(adapter,'HERE',root):
                t,key=runner.materialize_treatment(case,native,out,parent)
                self.assertTrue(t['recover_available']);self.assertGreater(sum(r['recovery_accepted'] for r in t['rows']),0)
                paths=[adapter.make_bag(key,case['sequence'],Path(case['input_manifest']),t,m) for m in ('SF_NATIVE','SF_REJECT','SF_RECOVER')]
                proofs=[read(p.parent/(p.stem+'_input.json')) for p in paths]
                self.assertEqual(len({p['imu_payload_hash'] for p in proofs}),1)
                self.assertEqual(proofs[0]['retained_obs_count']-proofs[1]['retained_obs_count'],t['candidate_count'])
                self.assertEqual(proofs[0]['retained_obs_count'],proofs[2]['retained_obs_count'])
                doc=yaml.safe_load((root/'exports/treatments'/f"{case['case_id']}.yaml").read_text())
                h=doc.pop('treatment_hash');self.assertEqual(h,runner.digest(doc))
                self.assertEqual(set(doc['accepted_obs_ids']),{r['obs_id'] for r in t['rows'] if r['recovery_accepted']})
                self.assertEqual(set(doc['rejected_obs_ids']),{r['obs_id'] for r in t['rows'] if r['candidate']})

    def test_failed_parent_still_allows_frozen_rejection_support(self):
        case=next(c for c in runner.load()['cases'] if c['case_id']=='Walk1_A10548')
        native=read(HERE/'runs/R6-LF-v2/results.json')[case['case_id']]
        p=Path(native['AUX_PRODUCER']['run']);d=read(p/'production_detector_status.json')
        parent={'support_hash':d['support_hash'],'detector_hash':sha(p/'production_detector_status.json'),'parent_hash':'fixture-parent'}
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);out=root/'out';out.mkdir()
            with patch.object(runner,'HERE',root),patch.object(adapter,'HERE',root):
                t,key=runner.materialize_treatment(case,native,out,parent)
                self.assertTrue(t['support_valid']);self.assertFalse(t['recover_available'])
                self.assertGreater(t['candidate_count'],0)
                self.assertTrue(all(r['applied_correction']==0 for r in t['rows']))
                self.assertIsNone(adapter.make_bag(key,case['sequence'],Path(case['input_manifest']),t,'SF_RECOVER'))
                result=runner.blocked_result(case,'FGO_REJECT',out/'blocked',native['AUX_PRODUCER'])
                self.assertEqual(result['cell']['status'],runner.BLOCKED)
                self.assertFalse(result['backend']['executed'])

if __name__=='__main__':unittest.main()
