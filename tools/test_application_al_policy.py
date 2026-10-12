#!/usr/bin/env python3
import copy
import unittest
from pathlib import Path
import yaml
from application_al_policy import validate,evaluate
TEMPLATE=Path(__file__).resolve().parents[1]/'config/application_al_policy.yaml'


def example():
    """Arithmetic fixture only. None of these values configures the project."""
    p=yaml.safe_load(TEMPLATE.read_text())
    for section in ['application','geometry_and_motion','time_and_safe_action','chosen_limits']:
        for key,value in p[section].items():
            if value is None: p[section][key]='ILLUSTRATIVE_TEST_REFERENCE_ONLY'
    a=p['application'];a.update(hazard_risk_limit=1e-6,risk_time_basis='specified_finite_mission',mission_horizon_s=2.5,required_protection_frame='world',required_protection_point='uwb_tag_world',horizontal_is_safety_critical=True,vertical_is_safety_critical=False,expected_bound_scope='healthy-IMU-single-UWB-episode-50-v1',expected_source_law_id='raw-source-rw-v1')
    p['geometry_and_motion'].update(clearance_for_protected_point_m=.25,clearance_direction_or_region='horizontal_ball',speed_bound_mps=0.,acceleration_bound_mps2=0.,angular_rate_bound_radps=0.)
    p['time_and_safe_action'].update(time_to_alert_requirement_s=1.,measurement_age_bound_s=.01,processing_delay_bound_s=.01,communications_delay_bound_s=.01,safe_action_delay_bound_s=.01,delay_and_motion_already_in_PL=False)
    p['chosen_limits'].update(horizontal_alert_limit_m=.099,vertical_alert_limit_m=None,selected_before_evaluation=True)
    for name,item in p['nonlocalization_allowances'].items():
        if isinstance(item,dict): item.update(bound_m=.03,direction='horizontal_ball',time_horizon_s=2.5,evidence_kind='deterministic_bound',event_failure_bound=0.,source_reference='ILLUSTRATIVE_'+name,already_in_position_PL=False)
    p['nonlocalization_allowances']['joint_event_or_conservative_union_reference']='ILLUSTRATIVE_DETERMINISTIC_BOUNDS_ONLY'
    return p


class Policy(unittest.TestCase):
    def test_template_missing_not_safe(self):
        r=validate(yaml.safe_load(TEMPLATE.read_text()))
        self.assertEqual(r['status'],'APPLICATION_AL_UNQUALIFIED');self.assertTrue(r['missing'])
    def test_illustrative_budget_and_immutable_evaluation(self):
        p=example();before=copy.deepcopy(p);r=validate(p)
        self.assertEqual(r['status'],'APPLICATION_AL_QUALIFIED_CONDITIONAL');self.assertAlmostEqual(r['localization_budget_upper_m'],.1)
        row=dict(finite_bound_qualified=True,source_law_id='raw-source-rw-v1',bound_scope='healthy-IMU-single-UWB-episode-50-v1',protection_frame='world',proof_binding_status='BOUND_TO_ACTUAL_RECEIPT',timeliness_status='VERIFIED',bound_risk_upper=4e-7,risk_horizon_s=2.5,hpl=.09)
        original=copy.deepcopy(row);self.assertTrue(evaluate(p,row)['application_available']);self.assertEqual(row,original);self.assertEqual(p,before)
        for field,value in [('finite_bound_qualified',False),('timeliness_status','UNKNOWN'),('risk_horizon_s',5.),('hpl',.11),('bound_risk_upper',2e-6),('source_law_id','other')]:
            with self.subTest(field=field):
                changed=dict(row,**{field:value});self.assertFalse(evaluate(p,changed)['application_available'])
    def test_negative_zero_and_units(self):
        for clearance in [.15,.1,0]:
            p=example();p['geometry_and_motion']['clearance_for_protected_point_m']=clearance
            self.assertNotEqual(validate(p)['status'],'APPLICATION_AL_QUALIFIED_CONDITIONAL')
        p=example();p['chosen_limits']['horizontal_alert_limit_m']=.1
        self.assertTrue(validate(p)['errors'])  # outward budget refuses rounding overshoot
        p=example();p['nonlocalization_allowances']['geometry_extrinsic']['bound_m']='3cm'
        self.assertTrue(validate(p)['errors'])
        p=example();p['geometry_and_motion']['speed_bound_mps']=1e308;p['time_and_safe_action']['safe_action_delay_bound_s']=1e308
        self.assertEqual(validate(p)['status'],'APPLICATION_AL_UNQUALIFIED')
    def test_direction_time_rmse_doublecount_tta_selection(self):
        for field,value in [('direction','vertical'),('time_horizon_s',5),('evidence_kind','RMSE'),('already_in_position_PL',True)]:
            p=example();p['nonlocalization_allowances']['geometry_extrinsic'][field]=value
            self.assertTrue(validate(p)['errors'])
        p=example();p['time_and_safe_action']['processing_delay_bound_s']=2
        self.assertTrue(validate(p)['errors'])
        p=example();p['chosen_limits']['selected_before_evaluation']=False
        self.assertTrue(validate(p)['errors'])

if __name__=='__main__': unittest.main()
