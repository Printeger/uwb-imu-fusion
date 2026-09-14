#!/usr/bin/env python3
from pathlib import Path
from formal_native import HERE,verify,read,write,save
from formal_treatment import export,make_bag,run_sf
from formal_evaluate import clean_equivalence


def main():
    lock=verify();assert read(HERE/'runs/R3/stage_status.json')['status']=='PASS'
    native=read(HERE/'runs/R3/results.json');allresults={};eq=[]
    stage='R4-runtime-fixed' if (HERE/'runs/R4/results.json').exists() else 'R4'
    for n in (1,2,3):
        sequence=f'Walk{n}';key=f'clean{n}';manifest=Path(lock['inputs'][n-1]['manifest'])
        treatment=export(key,manifest,native[sequence]);write(HERE/'exports/corrected_ranges'/f'{key}_treatment.json',treatment)
        results={};bags={}
        for method in ('SF_NATIVE','SF_REJECT','SF_RECOVER'):
            if stage=='R4-runtime-fixed':
                old=read(HERE/'runs/R4/results.json')[sequence][method]
                assert old['status']=='failed'
                if old['identity']['input_hash']:
                    assert "bwrap: Can't create file" in (Path(old['run'])/'worker.log').read_text()
                    bags[method]=HERE/'exports/sfuse_inputs'/key/(method+'.bag')
                else:bags[method]=None
            else:bags[method]=make_bag(key,sequence,manifest,treatment,method)
            results[method]=run_sf(key,sequence,bags[method],method,stage)
            print(sequence,method,results[method]['status'],flush=True)
        assert len({r['identity']['config_hash'] for r in results.values()})==1
        eq+=clean_equivalence(sequence,results,bags,treatment)
        allresults[sequence]=results;write(HERE/'runs'/stage/'results.json',allresults)
        save(HERE/'metrics/clean_sf_equivalence.csv',eq)
    bad=[r for r in eq if r['status'] in ('FAIL_TOLERANCE','failed')]
    # A scientific solver failure is retained, not an adapter input-integrity failure.
    status='FAIL' if bad else 'PASS'
    write(HERE/'runs'/stage/'stage_status.json',{'status':status,'equivalence':eq})
    (HERE/'audits/sfuse_adapter_report.md').write_text(
        '# R4 SFUISE adapter\n\nR4 '+status+'\n\n'
        'Each observation is mapped by original message/range index and stable obs_id. Only sensor topics are exported. '
        'Original IMU bytes, message/header timestamps, nonrange fields and all untreated ranges are independently checked after bag roundtrip. '
        'ROS float32 conversion error is bounded and recorded. Rejected observations retain an identity/treatment ledger.\n\n'
        'SF_REJECT removes detector support. SF_RECOVER subtracts only final accepted fixed-full compensation; all nonaccepted observations remain raw. '
        'Unavailable offline inference is explicitly failed, not replaced by oracle support or a different method.\n\n'
        'All three SF variants use exactly one frozen official config per sequence, checked by SHA256. '
        'Each process has an empty filesystem/network namespace with sensor bag, official config, binaries/runtime and its private output only. '
        'No GT, injection manifest, source bag containing GT, or detector oracle is mounted. Official algorithm unchanged.\n\n'
        'Clean no-candidate equivalence is conditional; detected clean sequences are not assumed equivalent. '
        'Tolerance fixed before execution: 0.001 m position, 0.001 rad orientation, 0.001 m aligned ATE. '
        'Semantic serialized payload equality (including topic and timestamp), not bag-container bytes, defines input equality.\n\n'
        'Evidence: `../metrics/clean_sf_equivalence.csv`, `../exports/sfuse_inputs/*/*_input.json`, '
        '`../runs/R4/*/*/isolation.json`, logs, commands, exit codes and `formal_result.json`.\n')
    return 0 if status=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
