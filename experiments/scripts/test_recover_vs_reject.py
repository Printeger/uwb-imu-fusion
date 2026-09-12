#!/usr/bin/env python3
"""Engineering fixtures only; these never count as scientific recording tasks."""
import copy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
import run_recover_vs_reject as r
import evaluate_recover_vs_reject as e


def geometry():
    return dict(status='ADMITTED_INDEPENDENT_APPROXIMATE',imu=dict(acceleration_origin='IMU',
        angular_velocity_origin='IMU',acceleration_semantics='SPECIFIC_FORCE',
        upstream_processing_source='ENGINEERING_FIXTURE_NOT_REAL_DATA',transform_source='ANALYTIC_FIXTURE',
        R_imu_acc=np.eye(3).tolist(),R_imu_gyro=np.eye(3).tolist()))


def measurements():
    u=[dict(source_row=i,time_s=str(.123456789+i*.037),range=str(3.456789123+i),from_id='1',to_id=str(4+i)) for i in range(3)]
    im=[dict(source_row=i,time_s=str(.11+i*.01),**{k:str(j+.1*i) for j,k in enumerate(r.IMU_COLUMNS[1:])}) for i in range(3)]
    return u,im


class Preparation(unittest.TestCase):
    def test_whitelist_gt_change_does_not_change_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'uwb.csv'
            p.write_text('time_s,range,from_id,to_id,gt_range,x,range_calib\n0.1,3.14159,1,7,900,20,80\n')
            first=r.projection(p,r.UWB_COLUMNS)
            p.write_text('time_s,range,from_id,to_id,gt_range,x,range_calib\n0.1,3.14159,1,7,-222,-30,70\n')
            second=r.projection(p,r.UWB_COLUMNS)
            self.assertEqual(first,second)
            _,im=measurements()
            self.assertEqual(r.cache_payloads('fixture',first,im,geometry()),r.cache_payloads('fixture',second,im,geometry()))
            self.assertNotIn('gt_range',json.dumps(first))

    def test_ids_rows_async_and_cache_v2(self):
        u,im=measurements();u[1]['from_id']='2'
        m,payload,_=r.cache_payloads('fixture',u,im,geometry())
        import csv
        rows=list(csv.DictReader(io.StringIO(payload.decode())))
        self.assertEqual([x['source_message_index'] for x in rows],['0','2'])
        self.assertEqual([x['sensor_time_s'] for x in rows],[u[0]['time_s'],u[2]['time_s']])
        self.assertEqual([int(x['obs_id']) for x in rows],[r.stable_id('fixture',0),r.stable_id('fixture',2)])
        self.assertEqual(m['uwb_message_count'],2)
        self.assertEqual(m['schema'],'nlos_measurement_cache_v2')
        self.assertEqual(m['cache_id'],r.cache_id(m))
        self.assertTrue(all(x['fp_rssi_dbm']==x['rx_rssi_dbm']=='0' for x in rows))
        self.assertNotEqual(r.stable_id('another',0),r.stable_id('fixture',0))

    def test_cpp_stable_id_golden(self):
        # Independently compile the exact C++ expression/uint64 overflow rule.
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'golden.cpp';binary=Path(tmp)/'golden'
            src.write_text('#include <iostream>\n#include <cstdint>\n#include <string>\nint main(){std::uint64_t h=1469598103934665603ULL;for(unsigned char c:std::string("fixture|source_message=12|source_range=0")){h^=c;h*=1099511628211ULL;}std::cout<<h;}')
            subprocess.run(['g++',str(src),'-o',str(binary)],check=True)
            self.assertEqual(int(subprocess.check_output([str(binary)])),r.stable_id('fixture',12))

    def test_semantics_guard_and_units(self):
        u,im=measurements()
        for key in ('acceleration_origin','acceleration_semantics','upstream_processing_source'):
            g=geometry();g['imu'][key]=''
            with self.assertRaisesRegex(ValueError,'BLOCKED'):r.cache_payloads('fixture',u,im,g)
        g=geometry();g['imu']['R_imu_acc']=[[0,-1,0],[1,0,0],[0,0,1]]
        _,_,raw=r.cache_payloads('fixture',u,im,g)
        import csv
        first=next(csv.DictReader(io.StringIO(raw.decode())))
        self.assertEqual(float(first['acc_x_mps2']),-1.)
        self.assertEqual(float(first['gyro_x_radps']),3.)
        self.assertEqual(first['has_orientation'],'0')

    def test_transform_composition_and_antenna(self):
        a=np.eye(4);a[:3,3]=[1,2,3]
        b=np.eye(4);b[:3,3]=[.1,.2,.3]
        b[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        lever=r.compose_lever(a,b,[2,3,4])
        np.testing.assert_allclose(lever,[.8,-.9,.7])
        tum=np.array([[0,1.1,2.2,3.3,0,0,np.sqrt(.5),np.sqrt(.5)]])
        _,pos=e.tag_trajectory(tum,lever);np.testing.assert_allclose(pos,[[2,3,4]])
        b[0,0]=3
        with self.assertRaisesRegex(ValueError,'ROTATION'):r.compose_lever(a,b,[2,3,4])

    def test_reject_pair_mismatches(self):
        a={k:'unique-'+k for k in r.PAIR_KEYS};r.require_pair(a,dict(a))
        for key in r.PAIR_KEYS:
            b=dict(a);b[key]='changed'
            with self.assertRaisesRegex(ValueError,key):r.require_pair(a,b)
            b=dict(a);del b[key]
            with self.assertRaisesRegex(ValueError,key):r.require_pair(a,b)

    def test_unknown_real_geometry_no_override(self):
        for row in r.read(r.GEOMETRY)['recordings'].values():
            with self.assertRaisesRegex(ValueError,'BLOCKED'):r.require_imu_semantics(row)

    def test_mount_gt_and_source_invisible(self):
        from estimator_isolation import sandbox_command
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);output=root/'output';output.mkdir()
            gt=root/'gt.csv';gt.write_text('private GT')
            source=root/'source.csv';source.write_text('measurement,GT')
            measurement=root/'measurement.csv';measurement.write_text('measurement only')
            command=['/usr/bin/python3','-c',
                'from pathlib import Path; import sys; assert Path(sys.argv[1]).is_file(); assert not Path(sys.argv[2]).exists(); assert not Path(sys.argv[3]).exists()',
                str(measurement),str(gt),str(source)]
            subprocess.run(sandbox_command(command,[measurement],output),check=True)


class Evaluation(unittest.TestCase):
    def test_all_events_gap_missing_and_overlap_union(self):
        rows=[]
        for aid,shift in ((4,0),(5,1)):
            for i in range(15):
                rows.append(dict(tag_id=1,anchor_id=aid,source_row=i,time_s=i*.5+shift,
                                 error_m=.6 if i!=6 else float('nan')))
        ev=e.events(rows)
        self.assertEqual(len(ev),4) # two events per link, not only the longest
        self.assertEqual(e.union_intervals([(x['start_s'],x['end_s']) for x in ev]),[[0,8]])
        rows[0]['error_m']=.5
        self.assertEqual(e.events(rows)[0]['start_s'],.5)
        broken=[dict(tag_id=1,anchor_id=4,source_row=i,time_s=t,error_m=.6)
                for i,t in enumerate([0,.5,1.,3.,3.5])]
        self.assertEqual(e.events(broken),[])

    def test_gt_gap_invalid_no_extrapolation(self):
        t=np.array([0,.02,.04,.12]);p=np.repeat(t[:,None],3,axis=1)
        out=e.interpolate_gt(t,p,[-.01,.01,.04,.08,.13])
        np.testing.assert_allclose(out[1],[.01]*3)
        self.assertTrue(np.isnan(out[[0,3,4]]).all())
        p[1]=np.nan
        self.assertTrue(np.isnan(e.interpolate_gt(t,p,[.01,.03])).all())

    def test_grid_nearest_and_source_interval(self):
        np.testing.assert_allclose(e.grid(.007,.312),[.007,.107,.207,.307])
        times=[-.001,.107,.232];pos=np.ones((3,3))
        out=e.nearest_estimate(times,pos,[.007,.107,.207],[.007,.312])
        self.assertTrue(np.isnan(out[[0,2]]).all());self.assertTrue(np.isfinite(out[1]).all())

    def test_rr_common_coverage_no_window_realign_and_sign(self):
        t=np.arange(100)/10.;gt=np.column_stack([np.cos(t),np.sin(t),t])
        reject=gt.copy();recover=gt.copy();recover[40:60,0]+=1
        windows=[(4,5.9),(4.5,5.5)]
        m=e.matched_metrics({'suppress':reject,'recover':recover},gt,t,windows)
        self.assertEqual(m['recover']['window_samples'],20)
        self.assertGreater(m['recover']['Window_RMSE_m'],.5)
        diff=e.paired_difference(m['suppress'],m['recover']);self.assertGreater(diff['delta_RR_m'],0)
        self.assertEqual(diff['interpretation'],'RECOVER_WORSE')
        sf=np.full_like(gt,np.nan)
        four=e.matched_metrics({'suppress':reject,'recover':recover,'cauchy':reject,'sf':sf},gt,t,windows)
        self.assertEqual(four['sf']['evaluated_samples'],0)
        self.assertEqual(m['recover']['evaluated_samples'],100)
        self.assertEqual(e.paired_difference({}, {})['delta_RR_m'],None)
        self.assertEqual(e.paired_difference(m['suppress'],m['recover'],True)['interpretation'],'FALLBACK_NOT_RECOVERY_BENEFIT')

    def test_range_sign_recovered_subset_suppression_fallback(self):
        raw={1:3.,2:4.};h={1:2.,2:3.};offset={1:.8}
        m=e.range_metrics([1,2],raw,h,offset,{1})
        self.assertAlmostEqual(m['range_before_RMSE_m'],1)
        self.assertAlmostEqual(m['range_after_RMSE_m'],np.sqrt(.52))
        self.assertEqual(m['recovered_range_count'],1)
        self.assertAlmostEqual(m['recovered_after_RMSE_m'],.2)
        fallback=e.range_metrics([1,2],raw,h,offset,{1},True)
        self.assertEqual(fallback['range_before_RMSE_m'],fallback['range_after_RMSE_m'])
        self.assertEqual(fallback['recovered_range_count'],0)
        with self.assertRaises(ValueError):e.range_metrics([1,2],raw,h,{2:1},{1})
        self.assertIsNone(e.range_metrics([1,2],raw,{},offset,{1})['range_before_RMSE_m'])

    def test_empty_zero_accepted_recovery_fallback_reporting(self):
        self.assertEqual(e.outcome_metrics(0,0,0,0,False)['recovery_status'],'NO_CANDIDATES')
        self.assertEqual(e.outcome_metrics(10,1,0,0,False)['recovery_status'],'ZERO_ACCEPTED')
        self.assertEqual(e.outcome_metrics(10,1,1,1,False)['recovery_status'],'RECOVERED')
        self.assertEqual(e.outcome_metrics(10,1,1,0,True)['recovery_status'],'FALLBACK')
        with self.assertRaises(ValueError):e.outcome_metrics(10,1,1,1,True)


class Accounting(unittest.TestCase):
    def test_producer_failure_continues_independent_once(self):
        calls=[]
        def call(row,task):
            calls.append(task);return dict(status='FAILURE' if task=='producer' else 'SUCCESS',exit_code=1 if task=='producer' else 0)
        rows=r.serial_tasks([dict(recording='fixture',status='ADMITTED_INDEPENDENT_APPROXIMATE')],call)
        self.assertEqual(calls,['producer','robust_cauchy','SFUISE-ToA'])
        self.assertEqual([x.get('reason') for x in rows[1:3]],['PRODUCER_FAILED']*2)

    def test_empty_support_is_success_no_rerun(self):
        calls=[]
        def call(row,task):
            calls.append(task);return dict(status='SUCCESS',candidate_segments=0,exit_code=0)
        rows=r.serial_tasks([dict(recording='fixture',status='ADMITTED_INDEPENDENT_APPROXIMATE')],call)
        self.assertEqual(calls,list(r.TASKS));self.assertEqual(len(rows),5)

    def test_blocked_all_fifteen_not_zero_results(self):
        recordings=[dict(recording=n,status='BLOCKED_GEOMETRY_OR_IMU') for n in r.RECORDINGS]
        rows=r.serial_tasks(recordings,lambda *_:self.fail('must not launch'))
        self.assertEqual(len(rows),15)
        self.assertTrue(all(v['exit_code'] is None and v['status']=='NOT_RUN' for v in rows))

    def test_launch_failure_is_recorded(self):
        with tempfile.TemporaryDirectory() as tmp:
            row=r.science_process(['/nonexistent/rr-test-program'],Path(tmp)/'failure')
            self.assertEqual(row['exit_code'],127)
            self.assertEqual(row['status'],'FAILURE')

    def test_timeout_process_tree_and_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            command=[sys.executable,'-c','import subprocess,time; subprocess.Popen(["sleep","30"]); time.sleep(30)']
            row=r.science_process(command,d/'timeout',timeout=.1)
            self.assertEqual(row['exit_code'],124);self.assertEqual(row['status'],'TIMEOUT')
            failed=r.science_process([sys.executable,'-c','raise SystemExit(3)'],d/'failed',timeout=1)
            self.assertEqual(failed['exit_code'],3)
            with self.assertRaises(FileExistsError):r.science_process(command,d/'timeout',timeout=.1)

    def test_ros_serialization_roundtrip(self):
        # Wire format limits are explicit: RTLSRange.range is float32, IMU float64,
        # ROS time is nanoseconds. This is not a real STAR-loc/SFUISE admission.
        from sensor_msgs.msg import Imu
        from isas_msgs.msg import RTLSStick,RTLSRange
        import rospy
        u,im=measurements()
        for row in u:
            m=RTLSStick();m.header.stamp=rospy.Time.from_sec(float(row['time_s']));m.id=1
            rr=RTLSRange();rr.id=int(row['to_id']);rr.range=float(row['range']);rr.ra=1;m.ranges=[rr]
            data=io.BytesIO();m.serialize(data);n=RTLSStick();n.deserialize(data.getvalue())
            self.assertLessEqual(abs(n.header.stamp.to_sec()-float(row['time_s'])),1e-9)
            self.assertEqual(n.id,1);self.assertEqual(len(n.ranges),1);self.assertEqual(n.ranges[0].id,int(row['to_id']))
            self.assertEqual(n.ranges[0].range,float(np.float32(float(row['range']))))
        for row in im:
            m=Imu();m.header.stamp=rospy.Time.from_sec(float(row['time_s']))
            for a in 'xyz':
                setattr(m.linear_acceleration,a,float(row['linear_acceleration_'+a]))
                setattr(m.angular_velocity,a,float(row['angular_velocity_'+a]))
            data=io.BytesIO();m.serialize(data);n=Imu();n.deserialize(data.getvalue())
            for a in 'xyz':
                self.assertEqual(getattr(n.linear_acceleration,a),float(row['linear_acceleration_'+a]))
                self.assertEqual(getattr(n.angular_velocity,a),float(row['angular_velocity_'+a]))

if __name__=='__main__':unittest.main(verbosity=2)
