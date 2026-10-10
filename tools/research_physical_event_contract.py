#!/usr/bin/env python3
"""Offline probability-contract research. Never calls a selector or publisher.

Consume two times of one diagnostic capture; retain production priors, alpha,
beta, AL and candidate PL. Fraction arithmetic supplies independent exact
ledger and event-union certificates, without a new probability solver.
"""
import argparse
import csv
from collections import Counter, defaultdict
from decimal import Decimal, localcontext
from fractions import Fraction as F
import hashlib
import json
import math
import platform
from pathlib import Path
import unittest
import numpy as np


def exact(text):
    """Interpret the exported, round-tripping binary64, not a decimal prior."""
    return F.from_float(float(text))


def display(value):
    with localcontext() as context:
        context.prec = 90
        decimal = str(Decimal(value.numerator) / Decimal(value.denominator))
    nearest = float(value)
    upper = nearest if F.from_float(nearest) >= value else float(np.nextafter(nearest, math.inf))
    return dict(exact_fraction=str(value), decimal=decimal,
                rounded_up=upper, rounded_up_hex=upper.hex())


def read(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def coefficient(row):
    prior, allocation, beta = map(exact, (row['prior_bound'], row['hmi_allocation'], row['p_md_allocation']))
    if not (prior > 0 and allocation >= 0 and 0 < beta < 1):
        raise ValueError('Invalid leaf probabilities')
    alpha = min(F(1, 2), allocation / prior)
    # These fixed snapshots have no cap slack. A capped allocation requires
    # retaining its reserve rather than pretending allocation == pi*alpha.
    if alpha * prior != allocation:
        raise ValueError('Tail-cap reserve needs separate accounting')
    return alpha, beta, max(alpha, beta)


def mass_preserving_groups(rows, key):
    """Union bound with no new source/aggregate prior evidence.

    Bound union mass by sum of original priors and epsilon by max of uniform
    conditional leaf certificates. Grouping can increase cost when epsilons
    differ; it does not justify replacing sum(prior) by max(prior).
    """
    groups = defaultdict(list)
    for row in rows:
        groups[key(row)].append(row)
    result = []
    for group, leaves in sorted(groups.items()):
        mass = sum((exact(r['prior_bound']) for r in leaves), F())
        epsilon = max(coefficient(r)[2] for r in leaves)
        result.append(dict(group=group, leaf_ids=[int(r['hypothesis_id']) for r in leaves],
                           mass_upper=display(mass), epsilon=display(epsilon),
                           charge=display(mass * epsilon)))
    return result


def snapshot(directory, epoch):
    attempt = read(directory / 'diagnostic_attempts.csv')[epoch - 1]
    window = attempt['window_id']
    leaves = [r for r in read(directory / 'hypotheses.csv') if r['window_id'] == window]
    if len(leaves) != int(attempt['hypothesis_count']):
        raise ValueError('Incomplete leaf census')
    terms = {}
    for term in attempt['risk_ledger_terms'].split(';'):
        name, rest = term.split('=', 1)
        number, status = rest.split(':', 1)
        terms[name] = dict(value=number, status=status)
    allocation = sum((exact(r['hmi_allocation']) for r in leaves), F())
    miss = sum((exact(r['prior_bound']) * max(F(), coefficient(r)[1] - coefficient(r)[0])
                for r in leaves), F())
    # The nominal term export has already rounded three axis products upward.
    # Recover the configured input to independently sum the unrounded product.
    import yaml
    config = yaml.safe_load((directory / 'resolved_config.yaml').read_text())
    constant = F.from_float(config['risk']['nominal_axis_tail']) * 3 + exact(terms['p_nm']['value'])
    others = sum((exact(term['value']) for name, term in terms.items()
                  if name not in ('nominal', 'p_nm', 'hypotheses', 'hypotheses_miss_channel')
                  and term['value'] != 'UNKNOWN'), F())
    known = constant + allocation + miss + others
    cpp = float(attempt['risk_ledger_charged_total'])
    if display(known)['rounded_up'] != cpp:
        raise ValueError('Independent exact ledger disagrees with C++ once-rounded total')
    by_source = mass_preserving_groups(leaves, lambda r: r['physical_source_ids'])
    grouped_charge = sum((F(x['charge']['exact_fraction']) for x in by_source), F())
    if grouped_charge != allocation + miss:
        raise ValueError('This comparison expects equal within-event coefficients')
    per_shape = Counter(r['fault_kind'] for r in leaves)
    last_onset = defaultdict(list)
    for row in leaves:
        if int(row['onset_epoch']) == epoch:
            last_onset[row['physical_source_ids']].append(row)
    fields=('slope_x','slope_y','slope_z','boundary_direction_gram','noncentrality_boundary')
    aliases=[]
    for source, records in sorted(last_onset.items()):
        if len(records)==3 and all(tuple(r[k] for k in fields)==tuple(records[0][k] for k in fields)
                                  for r in records):
            aliases.append(dict(source=source,leaf_ids=[int(r['hypothesis_id']) for r in records],
                                claim='Equal numerical audit at the last onset; NOT a probability-event identity certificate.'))
    first_source=by_source[0]['group']
    first_leaves=[r for r in leaves if r['physical_source_ids']==first_source]
    first_onset=min(int(r['onset_epoch']) for r in first_leaves)
    def cover_ids(onset,kinds):
        return [int(r['hypothesis_id']) for r in first_leaves
                if int(r['onset_epoch'])==onset and r['fault_kind'] in kinds]
    ep='ANCHOR_BIAS_EPOCH_INDEPENDENT'; persistent='ANCHOR_BIAS_PERSISTENT_CONSTANT'; ramp='ANCHOR_BIAS_RAMP'
    budget = F.from_float(config['risk']['p_hmi_total'])
    nonpersistent_floor = sum((exact(r['prior_bound']) * coefficient(r)[1] for r in leaves
                              if r['fault_kind'] != 'ANCHOR_BIAS_PERSISTENT_CONSTANT'), F())
    # Preserve actual output per-action bounds. No arithmetic reallocation or
    # new PL proof is fabricated from the all-in sensitivity audit.
    candidates = [r for r in read(directory / 'candidates.csv') if r['window_id'] == window]
    candidate_bounds = [{k: r[k] for k in ('action_id', 'hpl_m', 'vpl_m', 'post_detector_passed', 'covers_plausible_set')}
                        for r in candidates if math.isfinite(float(r['hpl_m']))]
    gates = [';'.join(x for x in r['reason'].split(';') if x.startswith(('gate:', 'ledger:')))
             for r in read(directory / 'diagnostic_stages.csv')
             if r['input_attempt_id'] == str(epoch) and 'gate:risk:' in r['reason']]
    return dict(epoch=epoch, window=window, leaf_count=len(leaves), leaf_shapes=dict(per_shape),
        raw_ledger=terms, allocation=display(allocation), miss_excess=display(miss),
        nominal_plus_p_nm=display(constant), other_known=display(others),
        exact_known=display(known), cpp_known=cpp, budget=display(budget),
        known_overspend=display(max(F(), known - budget)),
        physical_source_groups=by_source,
        first_source_numerical_cover_mapping=[dict(
            leaf_id=int(r['hypothesis_id']), source=r['physical_source_ids'],
            onset_epoch=int(r['onset_epoch']), numerical_shape=r['fault_kind'],
            raw_support='EXACT_ONSET_OCCURRENCE' if r['fault_kind']==ep else 'ONSET_THROUGH_PROTECTED_EPOCH',
            status='Numerical cover description only; NOT an exclusive physical event or a prior constraint.')
            for r in first_leaves],
        concrete_physical_cell_cover_examples=[
            dict(source=first_source, onset_epoch=first_onset, last_affected_occurrence=first_onset,
                 physical_law='constant; episode ended before later observations',
                 cover_ids=cover_ids(first_onset,{ep})),
            dict(source=first_source, onset_epoch=first_onset, last_affected_occurrence=epoch,
                 physical_law='constant; episode affects through protected measurement',
                 cover_ids=cover_ids(first_onset,{persistent,ramp})),
            dict(source=first_source, onset_epoch=first_onset, last_affected_occurrence=epoch,
                 physical_law='affine nonzero slope; episode affects through protected measurement',
                 cover_ids=cover_ids(first_onset,{ramp})),
            dict(source=first_source, onset_epoch=epoch, last_affected_occurrence=epoch,
                 physical_law='one observed occurrence; any declared constant/affine law or duration compatible with that occurrence',
                 cover_ids=cover_ids(epoch,{ep,persistent,ramp}))],
        last_onset_numerical_aliases=aliases,
        mass_preserving_partition_known=display(constant + grouped_charge + others),
        unknown_terms=[k for k, v in terms.items() if v['value'] == 'UNKNOWN'],
        current_candidate_bounds=candidate_bounds, selector_trace=gates,
        pl_comparison='UNCHANGED: old per-leaf alpha/beta, every numerical witness, common-reference and selection requirements retained. No new PL certificate minted.',
        old_availability='UNAVAILABLE', new_availability='UNAVAILABLE',
        protected_output=False, new_contract_implemented=False,
        persistent_only_best_case_floor=display(constant + nonpersistent_floor + others),
        persistent_only_can_close_even_if_other_persistent_charges_zero=(constant + nonpersistent_floor + others <= budget))


def counterexamples():
    pi, beta = exact('0.0001'), exact('0.001')
    # Disjoint physical onset/law atoms each obey every provided per-leaf
    # bound. The nominal atom completes the probability space exactly.
    mass = 168 * pi
    # Alternative space: 21 numerical labels really are the SAME event. This
    # is a valid positive for the theorem, not evidence for the real dataset.
    duplicate_old, duplicate_new = 21 * pi * beta, pi * beta
    # Unknown correlation: P(U)=p, P(I)=q and P(U & I)=z.
    p, q = pi, exact('0.00001')
    z_range = (max(F(), p + q - 1), min(p, q))
    return dict(
        disjoint_168_atoms=dict(each_prior=display(pi), fault_union_mass=display(mass),
            nominal_mass=display(1 - mass), conditional_miss=display(beta),
            miss_charge=display(mass * beta),
            falsely_merged_eight_source_charge=display(8 * pi * beta),
            conclusion='All current per-leaf priors permit this partition. A source-union prior of 1e-4 is an additional assumption, not an implication of those priors.'),
        certified_identical_event_21_labels=dict(old=display(duplicate_old), new=display(duplicate_new),
            prior_changed=False, prerequisite='All 21 labels are proven covers of one identical physical event, whose union prior is already bounded by the SAME original 1e-4.',
            applicable_to_epoch7=False),
        correlated_pair=dict(p=display(p), q=display(q), joint_bounds=[display(x) for x in z_range],
            perfect_correlation_joint=display(q), independent_product=display(p*q),
            product_understates_possible_joint_by=float(q/(p*q)),
            old_marginal_plus_joint_charge=display((p + q + min(p,q))*beta),
            worst_partition_union_charge=display((p+q)*beta),
            prerequisite='Bounds must be authenticated marginals of the same primitive events, not unrelated exclusive-mode upper bounds.'),
        onset_witness=dict(fault=[1,1,1], latest_only_span=[0,0,1], uncovered_residual_squared=2,
            conclusion='Retaining only one onset loses earlier persistent faults.'),
        duration_witness=dict(fault=[0,1,1,0], result='Outside the currently enumerated one-epoch or onset-to-end constant/affine families; must stay in omitted/escape unless scope is explicitly extended.'),
        merged_full_span=dict(detector=[1,1], protected=[1,-1], individual_slopes=[1,1],
            dangerous_null_vector=[1,-1], detector_response=0, protected_response=2,
            conclusion='Union of two single-fault spaces is not their concatenated span; concatenation admits a new dangerous combined fault. Keep union witnesses and their max bounds.'),
        sequence_reuse=dict(single_persistent_event_prior=display(pi), per_epoch_conditional_miss=display(beta),
            three_disjoint_conditional_miss_sets=display(3*beta), mission_charge=display(pi*3*beta),
            conclusion='A per-protected-epoch bound cannot be charged once across multiple published epochs. Sequence risk requires its own conditional union bound.'),
        adaptive_output_groups=dict(conditional_failure_per_group=display(beta),
            two_disjoint_failure_sets=display(2*beta), incorrectly_merged_max=display(beta),
            conclusion='A shared physical source does not prove a shared output failure event. Without common-reference evidence, sum group bounds inside each physical event.'))


class DeterministicContracts(unittest.TestCase):
    def test_disjoint_priors_do_not_imply_union_prior(self):
        pi = exact('0.0001')
        atoms = [pi] * 168 + [1 - 168*pi]
        self.assertEqual(sum(atoms), 1)
        self.assertTrue(all(0 <= x <= 1 for x in atoms))
        self.assertGreater(sum(atoms[:21]), pi)

    def test_one_physical_event_can_have_many_covers(self):
        pi, epsilon = exact('0.0001'), exact('0.001')
        self.assertEqual(pi * max([epsilon]*21), pi*epsilon)
        self.assertGreater(sum([pi*epsilon]*21), pi*epsilon)

    def test_grouping_preserves_mass_and_may_worsen_unequal_epsilons(self):
        rows=[dict(prior_bound='0.0001', hmi_allocation='0.00000001', p_md_allocation=b,
                   hypothesis_id=str(i)) for i,b in enumerate(['0.001','0.002'],1)]
        group=mass_preserving_groups(rows,lambda r:'source')[0]
        old=sum((exact(r['prior_bound'])*coefficient(r)[2] for r in rows),F())
        self.assertGreater(F(group['charge']['exact_fraction']), old)

    def test_joint_marginals_do_not_prove_independence(self):
        p,q=exact('0.0001'),exact('0.00001')
        z=q
        self.assertEqual((p-z)+(q-z)+z+(1-p-q+z),1)
        self.assertGreater(z,p*q)

    def test_omitted_duration_and_onset_are_not_covered(self):
        target=np.array([0.,1.,1.,0.])
        for onset in range(4):
            epoch=np.eye(4)[:,onset:onset+1]
            constant=np.array([float(i>=onset) for i in range(4)])
            ramp=np.array([max(0.,float(i-onset)) for i in range(4)])
            for basis in [epoch, constant[:,None],np.column_stack([constant,ramp])]:
                self.assertGreater(np.linalg.norm(target-basis@np.linalg.lstsq(basis,target,rcond=None)[0]),.1)

    def test_union_is_not_concatenated_span(self):
        self.assertEqual(1*1+1*(-1),0)
        self.assertEqual(1*1-1*(-1),2)

    def test_repeated_outputs_need_a_conditional_union_bound(self):
        pi,beta=exact('0.0001'),exact('0.001')
        self.assertLess(3*beta,1)
        self.assertGreater(pi*3*beta,pi*beta)

    def test_adaptive_output_groups_do_not_share_a_failure_event(self):
        beta=exact('0.001')
        # Disjoint noise intervals [0,beta), [beta,2*beta) are each at the
        # original bound. Selecting their respective outputs covers both.
        self.assertLessEqual(2*beta,1)
        self.assertGreater(2*beta,max(beta,beta))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',type=Path)
    parser.add_argument('--report',type=Path)
    parser.add_argument('--self-test',action='store_true')
    args=parser.parse_args()
    if args.self_test:
        result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DeterministicContracts))
        if not result.wasSuccessful():raise SystemExit(1)
    if args.snapshot:
        if not args.report:parser.error('--snapshot requires --report')
        data=json.loads(args.report.read_text())
        existing=data.get('event_contract_research',{})
        study=dict(status='RESEARCH_ONLY_REVIEW_REQUIRED',production_contract_changed=False,
            approval='Research authorized; implementation/reinterpretation of priors not authorized.',
            source=str(args.snapshot), source_hashes={name:digest(args.snapshot/name) for name in
                ('hypotheses.csv','diagnostic_attempts.csv','diagnostic_stages.csv','candidates.csv','resolved_config.yaml')},
            snapshots=[snapshot(args.snapshot,e) for e in (4,7)], counterexamples=counterexamples(),
            physical_source_union_prior='UNKNOWN: existing per-leaf bound does not identify or bound the union event.',
            conclusion='No defensible net budget or availability gain on epoch7 from currently supplied priors; event grouping with conserved mass gives the same 4.69e-5 known total.',
            imu_diagnostic='Independent blocker retained: weak parity response, unchanged step gate, new-samples-only versus whole-interval support mismatch.',
            proof_domain='Frozen linear Gaussian / event-conditional uniform certificates, with complete fault coverage and same accepted output reference as prerequisites. No hardware or publication qualification.',
            deterministic_checks=8)
        study['research_script_sha256']=digest(Path(__file__))
        study['runtime']=dict(python=platform.python_version(),numpy=np.__version__)
        study['selection_boundary']='The existing action groups are retained. A lower physical tail charge cannot be subtracted from the old 9.9e-6 selection reserve without a new quantitative union certificate. No hypothetical lower-prior availability is claimed.'
        if 'capture' in existing:study['capture']=existing['capture']
        data['event_contract_research']=study
        args.report.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
        print('Exact old/new mass-preserving ledger checks passed at epochs 4 and 7; no production changes.')


if __name__=='__main__':main()
