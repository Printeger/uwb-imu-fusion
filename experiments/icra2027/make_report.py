#!/usr/bin/env python3
"""Generate review tables/plots from measured clean-audit CSVs, not handwritten numbers."""
import json
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml

from evaluate_clean import HERE, rows, save


def number(row, key):
    value = row.get(key, '')
    return f'{float(value):.6f}' if value else 'NA'


def main():
    data = rows(HERE / 'metrics/backbone_parity.csv')
    density = rows(HERE / 'audits/range_density_audit.csv')
    offsets = []
    for n in (1, 2, 3):
        cfg = yaml.safe_load((HERE / f'configs/sfuse/config_test_isas-walk{n}.yaml').read_text())
        for aid, offset in zip([7475, 9524, 10548, 15155, 20276], cfg['toa_offset']):
            offsets.append({'sequence': f'Walk{n}', 'anchor_id': aid, 'official_offset_m': offset,
                            'native_fixed_beta_m': -offset, 'ordering': 'ASCENDING_UINT16_ANCHOR_ID',
                            'official_residual': 'h-z-offset', 'native_residual': 'h+beta-z',
                            'application': 'ONCE_IN_FACTOR_NOT_RAW_DATA',
                            'sign_units_order_status': 'SOURCE_VERIFIED',
                            'independent_calibration_status': 'UNVERIFIED',
                            'test_gt_independence_status': 'UNVERIFIED',
                            'source_commit': '75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d',
                            'source': 'sfuise/config/config_test_isas-walk%d.yaml;SplineFusion.cpp:223;Residuals.h:121' % n})
    save(HERE / 'audits/range_calibration_audit.csv', offsets)
    text = ['# R0–R2 clean backbone parity report', '',
            'R0 local reproduction passed. R1 completed under the aligned-only/proxy boundary. '
            'R2 density and calibration-source audits completed; the B1/B2 scientific matrix is incomplete '
            'unless separately authorized author-config development results are present below. B3 is NOT_RUN.', '',
            '## Common-GT trajectory results', '',
            'All errors are metres. Raw coordinate mismatch is not calibrated raw-frame ATE. '
            'Each sequence uses the same exact GT samples across available methods.', '',
            '| Sequence | Variant | Status | N | Unaligned diagnostic | Aligned ATE | P95 | RPE 1 s proxy |',
            '|---|---|---|---:|---:|---:|---:|---:|']
    for row in data:
        text.append('| ' + ' | '.join([row['sequence'], row['variant'], row['status'],
                                      row.get('n_gt_samples', ''), number(row, 'unaligned_coordinate_RMSE_m'),
                                      number(row, 'ATE_RMSE_aligned'), number(row, 'ATE_P95'), number(row, 'RPE_1s')]) + ' |')
    text += ['', '## Range utilization', '',
             '| Sequence | Variant | Raw | Valid | Planned = final used | States | Valid dropped by step | Additional exact-time ranges |',
             '|---|---|---:|---:|---:|---:|---:|---:|']
    for row in density:
        if row['anchor_id'] == 'ALL':
            counts = json.loads(row['drop_reasons'])
            text.append('| ' + ' | '.join([row['sequence'], row['variant'], row['raw_range_count'], row['valid_range_count'],
                                          row['used_range_count'], row['state_count'], str(counts.get('KEYFRAME_STEP_4', 0)),
                                          row['additional_valid_at_existing_state']]) + ' |')
    text += ['', '## Attribution and B3 decision boundary', '',
             '- Frame: translation-only and rigid fits explain most of the unaligned mismatch in both methods. '
             'No independent frame/reference-point correction was found or applied; alignment is evaluation, '
             'so this is not a measured estimator improvement.',
             '- Static calibration: official order, units and sign are verified. The values first appear in '
             'the author initial commit `2ae389f`; repository docs and the author paper do not disclose an '
             'independent calibration recording/procedure or demonstrate independence from Walk GT. '
             'Existing native `fixed_beta_by_link` already supports the equivalent factor constant. '
             'Without admission, B1 gains are NA, not zero.',
             '- Density: all valid observations at the existing states already enter the final graph once. '
             'No additional valid observation has an exact timestamp at an existing state. Native states '
             'are about 0.251 s apart (roughly 4 Hz); the cache has roughly 16 Hz message frames, '
             'with five range slots per message. Aggregate per-anchor ranges/s must not be confused with state Hz.',
             '- B2 cannot be a distinct density-only variant with this exact-timestamp unary-factor graph. '
             'Changing step 4→1 also adds navigation states and changes adaptive nominal sigma via per-link dt. '
             'Assigning dropped observations to old states introduces an unvalidated time approximation. '
             'Neither was silently implemented. B2 is NOT_RUN, not a measured zero-gain experiment.',
             '- The missing-range count makes state-rate/density coupling a plausible engineering limitation, '
             'but does not prove it causes the remaining ATE gap. First resolve B1 admission and measure its '
             'gain; only then decide on a separately specified B3 experiment. No B3 setting is selected here.',
             '', '## Evidence limits', '',
             'Fresh science: R0 original Walk1 clean pipeline (5 backend tasks) and B0 Walk1/2/3. '
             'SF_NATIVE reuses the three previously sealed unmodified official runs, with source bag hash '
             'verified and metrics recomputed. Runtime columns retain different native versus 1x ROS playback semantics. '
             'R0 was reproduced in the current environment; an independent clean-checkout rebuild remains NOT_RUN.',
             '', 'No new core estimator, detector, recovery, SFUISE, initialization, source data or GT edits. '
             'No injection, simulation, other datasets, claim upgrade, commit or push. '
             'An evaluator engineering attempt used the wrong CSV column name `final_used`; actual '
             'native schema is `final_use`. That attempt exited 1; correcting the reader and rerunning '
             'evaluation did not rerun scientific estimation.',
             '', 'Author sources: [SFUISE paper](https://arxiv.org/abs/2301.09033), '
             '[official code](https://github.com/KIT-ISAS/SFUISE/tree/75bf5a32f1a8e5c3046a5bd1a1ddf659fa996f7d).']
    (HERE / 'audits/backbone_parity_report.md').write_text('\n'.join(text) + '\n')
    latex = ['% Generated by make_report.py from metrics/backbone_parity.csv; DEVELOPMENT only.',
             '\\begin{tabular}{lllr}', '\\hline', 'Sequence & Variant & Status & Aligned ATE (m) \\\\', '\\hline']
    for row in data:
        latex.append(' & '.join([row['sequence'], row['variant'].replace('_', '\\_'),
                                 row['status'].replace('_', '\\_'), number(row, 'ATE_RMSE_aligned')]) + ' \\\\')
    latex += ['\\hline', '\\end{tabular}']
    (HERE / 'tables/latex/backbone_parity.tex').write_text('\n'.join(latex) + '\n')
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.3))
    for n, ax in enumerate(axes, 1):
        selected = [r for r in data if r['sequence'] == f'Walk{n}' and r.get('ATE_RMSE_aligned')]
        ax.bar([r['variant'] for r in selected], [float(r['ATE_RMSE_aligned']) for r in selected])
        ax.set_title(f'Walk{n}: common GT')
        ax.set_ylabel('Aligned ATE RMSE (m)')
        ax.tick_params(axis='x', labelrotation=20)
    fig.suptitle('Clean development audit; tracker/body proxy; missing variants are NOT_RUN')
    fig.tight_layout()
    fig.savefig(HERE / 'figures/pdf/backbone_parity.pdf')
    fig.savefig(HERE / 'figures/png/backbone_parity.png', dpi=160)
    plt.close(fig)
    print('Generated calibration audit, parity report, CSV-backed LaTeX and vector PDF.')


if __name__ == '__main__':
    main()
