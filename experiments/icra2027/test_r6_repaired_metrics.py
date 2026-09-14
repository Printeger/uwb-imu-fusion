import copy
from pathlib import Path
import unittest
from unittest.mock import patch
import formal_assets_lf as original
import formal_assets_repaired as repaired


class FrozenCriteriaTests(unittest.TestCase):
    def test_all_outcomes_and_pair_threshold_match_original(self):
        cases=[{'case_id':str(i)} for i in range(15)]
        detect=[dict(case_id=str(i),status='completed',candidate_count=1,event_detected=1,
                     precision=.5,recall=1.,F1=2/3,offline_onset_error=0.,offline_offset_error=0.) for i in range(15)]
        runs=[dict(case_id=str(i),status='completed',fallback=0,method_id='FGO_REJECT',failure_reason='') for i in range(15)]
        for native,sf,valid in ((True,True,15),(True,False,15),(False,True,15),(False,False,15),(True,True,11),(True,True,12)):
            pairs=[]
            for i in range(15):
                p={'delta_SF_recover_native':-.1}
                for prefix,good in [('FGO',native),('SF',sf)]:
                    for metric in ('ATE','fault'):p['delta_'+prefix+'_RR_'+metric]=(-.1 if good else .1) if i<valid else None
                pairs.append(p)
            captured=[]
            for module in (original,repaired):
                with patch.object(module,'write',side_effect=lambda p,d:captured.append(copy.deepcopy(d))),patch.object(Path,'write_text'):
                    module.go_nogo({'cases':cases},runs,pairs,detect,[])
            a,b=captured
            a['RECOVERY_CLAIM']={'NATIVE_ONLY':'PARTIAL_NATIVE','MEASUREMENT_LAYER':'PARTIAL_TRANSFER'}.get(a['RECOVERY_CLAIM'],a['RECOVERY_CLAIM'])
            self.assertEqual(a,b)
            if valid==11:self.assertEqual(b['RECOVERY_CLAIM'],'NOT_SUPPORTED')

if __name__=='__main__':unittest.main()
