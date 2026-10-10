#!/usr/bin/env python3
"""Read-only provenance/absorption research; never supplies production priors."""
import argparse
import csv
import hashlib
import json
import math
import re
import unittest
from pathlib import Path

import yaml


def occupancy_bound(initial_probability, onset_rate_per_second, horizon_seconds):
    # Union + Markov: expected episode count bounds any new onset. No
    # independence between adjacent epochs or different sensors is assumed.
    if not (0 <= initial_probability <= 1 and onset_rate_per_second >= 0
            and horizon_seconds >= 0):
        raise ValueError('invalid probability/rate/horizon')
    return min(1., initial_probability + onset_rate_per_second * horizon_seconds)


class PriorMathChecks(unittest.TestCase):
    def test_correlated_joint(self):
        p, q = 1e-4, 1e-5
        # Nested events attain the Fréchet upper bound; product is unjustified.
        self.assertEqual(min(p, q), q)
        self.assertGreater(q, p*q)

    def test_rate_is_not_state_probability(self):
        rate = 1e-4/3600.
        self.assertAlmostEqual(occupancy_bound(.002, rate, .05), .002+rate*.05)
        self.assertGreater(occupancy_bound(.002, rate, .05), rate*.05)

    def test_duration_and_history_extend_influence(self):
        rate, duration, retention, window = .001, 2., 3., .5
        self.assertAlmostEqual(occupancy_bound(rate*(duration+retention), rate, window), .0055)
        self.assertGreater(rate*(duration+retention+window), rate*window)

    def test_zero_faults_do_not_prove_zero_prior(self):
        # Only for independently sampled Bernoulli trials, not streaming rows.
        upper = -math.expm1(math.log(.05)/35.)
        self.assertGreater(upper, .08)
        self.assertGreater(-math.log(.05)/35., 0.)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--results', type=Path)
    parser.add_argument('--report', type=Path)
    args=parser.parse_args()
    if args.self_test:
        run=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromTestCase(PriorMathChecks))
        if not run.wasSuccessful(): raise SystemExit(1)
    if args.report is None: return
    if args.results is None: parser.error('--results is required with --report')
    root=Path(__file__).resolve().parents[1]
    config=root/'config/fde_imu_order1.yaml'
    manifest=root/'config/integrity_fault_manifest.yaml'
    cfg=yaml.safe_load(config.read_text())
    declaration=yaml.safe_load(manifest.read_text())
    rows=list(csv.DictReader((args.results/'imu_absorption.csv').open()))
    observed=[r for r in rows if r['rhs']=='observed']
    axis=[r for r in rows if r['rhs']=='unit_axis_0']
    finite_log=args.results/'imu_finite.log'
    match=re.search(r'\[FDE-IMU-FINITE\] (.*)',finite_log.read_text())
    if match is None: raise RuntimeError('finite injection test evidence missing')
    data=json.loads(args.report.read_text())
    section={
        'functional_status':'PARTIAL_BLOCKED',
        'production_contract_changed':False,
        'persistent_constant_research_status':'USER_ACCEPTED_STOPPED; no repeated merge experiments',
        'prior_status':'UNQUALIFIED',
        'introduction_commit':'1c547f31981854c03d68f557c7fd5179fd35c417',
        'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),
        'manifest_sha256':hashlib.sha256(manifest.read_bytes()).hexdigest(),
        'priors':{
            'uwb':cfg['fault_models']['uwb']['prior_probability_bound'],
            'accel':cfg['fault_models']['imu']['accel_prior_probability_bound'],
            'gyro':cfg['fault_models']['imu']['gyro_prior_probability_bound']},
        'independent_priors':cfg['fault_models']['combinations']['assume_independent_priors'],
        'event_declarations':[{k:e[k] for k in ('event_id','source_id','prior_bound_source','prior_time_basis','physical_time_support','evidence_status')} for e in declaration['events']],
        'actual_generator_joint_rule':'min(member_prior_bounds); no product; upper bound on intersection, not exact probability',
        'hardware_probability_evidence':None,
        'literature_transfer':'NOT_JUSTIFIED: GNSS SIS and vision keypoint priors are not UWB/IMU calibration',
        'imu_snapshot':str(args.results/'consistent_imu_recovery/replay/attempt-7.bin'),
        'imu_snapshot_sha256':hashlib.sha256((args.results/'consistent_imu_recovery/replay/attempt-7.bin').read_bytes()).hexdigest(),
        'absorption_csv_sha256':hashlib.sha256((args.results/'imu_absorption.csv').read_bytes()).hexdigest(),
        'absorption_verified_rows':len(rows),
        'normal_equation_relative_max':max(float(r['normal_equation_relative']) for r in rows),
        'absorption_observed':observed,
        'absorption_unit_full_interval_accel_x':axis,
        'counterfactuals_are_not_production_models':True,
        'finite_injection_stationary_fixture':{k:float(v) for k,v in re.findall(r'(\w+)=([\d.eE+-]+)',match.group(1))},
        'diagnosis':'Raw injection reaches factor; weak parity sensitivity dominated by free velocity, not bias. Step gate correctly refuses large linear correction. Raw-batch support still differs from declared full-interval model.',
        'approval_required_proposals':[
            'Evidence-bound prior semantics and qualification artifact; no replacement number proposed.',
            'Raw-sample episode convolution across adjacent preintegrations/history; separate from probability contract.',
            'Independent motion/velocity constraint only with a declared valid physical envelope; no velocity freezing or threshold lowering.'],
        'new_tests':{'prior_math_checks':4,'finite_injection_gtests':1,'frozen_snapshots':1},
        'campaigns_run':0,
    }
    data['prior_imu_followup']=section
    args.report.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')


if __name__=='__main__': main()
