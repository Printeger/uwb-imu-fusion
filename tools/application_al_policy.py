#!/usr/bin/env python3
"""Pure application budget validation and immutable-result AL evaluation."""
import argparse
import json
import math
from pathlib import Path
import yaml
import numpy as np


def number(value, positive=False):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value) and (value>0 if positive else value>=0)


def adjacent(value,towards):
    return float(np.nextafter(value,towards))


def upper_sum(values):
    try: value=math.fsum(values)
    except OverflowError: return math.inf
    return adjacent(value,math.inf) if value else 0.


def validate(policy):
    missing=[];errors=[]
    if policy.get("schema")!="hybrid-application-al-policy-proposal-v1": errors.append("unsupported policy schema")
    required={
        'application':['name','operating_mode','hazard_description','risk_requirement_source','hazard_risk_limit','risk_time_basis','mission_horizon_s','required_protection_frame','required_protection_point','horizontal_is_safety_critical','vertical_is_safety_critical','user_approved_requirement_reference','expected_bound_scope','expected_source_law_id','vertical_noncritical_rationale_reference'],
        'geometry_and_motion':['obstacle_or_geofence_reference','clearance_for_protected_point_m','clearance_direction_or_region','body_or_payload_geometry_reference','tag_to_safety_point_transform_reference','speed_bound_mps','acceleration_bound_mps2','angular_rate_bound_radps','admissible_motion_domain_reference'],
        'time_and_safe_action':['time_to_alert_requirement_s','measurement_age_bound_s','processing_delay_bound_s','communications_delay_bound_s','safe_action','actuation_or_braking_model_reference','safe_action_delay_bound_s','delay_and_motion_already_in_PL','evidence_reference'],
        'chosen_limits':['horizontal_alert_limit_m','selection_rationale_reference','selected_before_evaluation'],
    }
    for section,fields in required.items():
        for field in fields:
            if policy.get(section,{}).get(field) in (None,''): missing.append(section+'.'+field)
    allowances=policy.get('nonlocalization_allowances',{})
    for name in ['geometry_extrinsic','map_environment','control_tracking','delay_safe_action','other']:
        for field in ['bound_m','direction','time_horizon_s','evidence_kind','event_failure_bound','source_reference','already_in_position_PL']:
            if allowances.get(name,{}).get(field) in (None,''): missing.append(f'nonlocalization_allowances.{name}.{field}')
    if not allowances.get('joint_event_or_conservative_union_reference'): missing.append('nonlocalization_allowances.joint_event_or_conservative_union_reference')
    result={'status':'APPLICATION_AL_UNQUALIFIED','missing':missing,'errors':errors,'localization_budget_upper_m':None,'allows_production_publication':False}
    if missing: return result
    app=policy['application'];geo=policy['geometry_and_motion'];time=policy['time_and_safe_action'];chosen=policy['chosen_limits']
    if app['risk_time_basis']!='specified_finite_mission': errors.append('unsupported risk time basis')
    if not number(app['hazard_risk_limit'],True) or app['hazard_risk_limit']>=1: errors.append('invalid hazard risk limit')
    if not number(app['mission_horizon_s'],True): errors.append('invalid mission horizon')
    # Scalar template implements a conservative horizontal ball budget. A
    # critical vertical requirement needs a separate directional budget.
    if app['horizontal_is_safety_critical'] is not True or app['vertical_is_safety_critical'] is not False or geo['clearance_direction_or_region']!='horizontal_ball': errors.append('separate directional budget required')
    if app['required_protection_frame']!='world' or app['required_protection_point']!='uwb_tag_world': errors.append('unsupported protected point/frame mapping')
    for field in ['clearance_for_protected_point_m','speed_bound_mps','acceleration_bound_mps2','angular_rate_bound_radps']:
        if not number(geo[field]): errors.append('invalid units/value: '+field)
    for field in ['time_to_alert_requirement_s','measurement_age_bound_s','processing_delay_bound_s','communications_delay_bound_s','safe_action_delay_bound_s']:
        if not number(time[field],field=='time_to_alert_requirement_s'): errors.append('invalid units/value: '+field)
    if chosen['selected_before_evaluation'] is not True: errors.append('AL must be selected before evaluation')
    if not number(chosen['horizontal_alert_limit_m'],True): errors.append('invalid horizontal AL')
    if not isinstance(time['delay_and_motion_already_in_PL'],bool): errors.append('unknown delay inclusion')
    if errors: return result
    delay=upper_sum([time[k] for k in ['measurement_age_bound_s','processing_delay_bound_s','communications_delay_bound_s','safe_action_delay_bound_s']])
    if not math.isfinite(delay):
        errors.append('nonfinite delay arithmetic');return result
    if delay>time['time_to_alert_requirement_s']: errors.append('TTA budget exceeded')
    speed_part=adjacent(geo['speed_bound_mps']*delay,math.inf)
    delay_squared=adjacent(delay*delay,math.inf)
    accel_part=adjacent(adjacent(.5*geo['acceleration_bound_mps2'],math.inf)*delay_squared,math.inf)
    delay_motion=upper_sum([speed_part,accel_part])
    deduction=[];risks=[];seen=set();included=set(policy.get('pl_included_sources',[]))
    for name in ['geometry_extrinsic','map_environment','control_tracking','delay_safe_action','other']:
        item=allowances[name]
        if not number(item['bound_m']) or not number(item['event_failure_bound']) or item['event_failure_bound']>=1: errors.append(name+': invalid bound/risk units');continue
        if item['direction']!=geo['clearance_direction_or_region'] or item['time_horizon_s']!=app['mission_horizon_s']: errors.append(name+': direction/time mismatch')
        if item['evidence_kind'] not in ['deterministic_bound','probabilistic_bound']: errors.append(name+': RMSE or unsupported evidence')
        if item['evidence_kind']=='deterministic_bound' and item['event_failure_bound']!=0: errors.append(name+': deterministic event must have zero failure bound')
        if not isinstance(item['already_in_position_PL'],bool): errors.append(name+': unknown PL inclusion')
        source=item['source_reference']
        if source in seen: errors.append(name+': duplicate allowance source')
        seen.add(source)
        if item['already_in_position_PL']:
            if source not in included: errors.append(name+': unverified inclusion in PL')
        else:
            deduction.append(item['bound_m']);risks.append(item['event_failure_bound'])
    delay_item=allowances['delay_safe_action']
    if time['delay_and_motion_already_in_PL']!=delay_item['already_in_position_PL']: errors.append('delay double-count/inclusion mismatch')
    if not time['delay_and_motion_already_in_PL'] and delay_item['bound_m']<delay_motion: errors.append('delay allowance does not cover bounded motion')
    union=upper_sum(risks)
    if not math.isfinite(upper_sum(deduction)) or not math.isfinite(delay_motion):
        errors.append('nonfinite geometric arithmetic');return result
    budget=adjacent(geo['clearance_for_protected_point_m']-upper_sum(deduction),-math.inf)
    result.update(localization_budget_upper_m=budget,nonlocalization_failure_upper=union,total_delay_s=delay)
    if budget<=0:
        result['status']='INFEASIBLE';return result
    if chosen['horizontal_alert_limit_m']>budget: errors.append('chosen AL exceeds geometric budget')
    if union>=app['hazard_risk_limit']: errors.append('nonlocalization risk exhausts mission budget')
    if not errors: result['status']='APPLICATION_AL_QUALIFIED_CONDITIONAL'
    return result


def evaluate(policy, row):
    validation=validate(policy)
    ok=validation['status']=='APPLICATION_AL_QUALIFIED_CONDITIONAL'
    if ok:
        app=policy['application'];chosen=policy['chosen_limits']
        ok=(row.get('finite_bound_qualified') in (True,'1') and row.get('source_law_id')==app['expected_source_law_id'] and row.get('bound_scope')==app['expected_bound_scope'] and row.get('protection_frame')==app['required_protection_frame'] and row.get('proof_binding_status')=='BOUND_TO_ACTUAL_RECEIPT' and row.get('timeliness_status')=='VERIFIED' and number(row.get('bound_risk_upper')) and upper_sum([row['bound_risk_upper'],validation['nonlocalization_failure_upper']])<=app['hazard_risk_limit'] and number(row.get('risk_horizon_s'),True) and row['risk_horizon_s']==app['mission_horizon_s'] and number(row.get('hpl')) and row['hpl']<=chosen['horizontal_alert_limit_m'] and set(policy.get('pl_included_sources',[])).issubset(set(row.get('pl_included_sources',[]))))
    return dict(validation,application_available=bool(ok))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('policy',type=Path)
    args=parser.parse_args()
    print(json.dumps(validate(yaml.safe_load(args.policy.read_text())),indent=2))

if __name__=='__main__': main()
