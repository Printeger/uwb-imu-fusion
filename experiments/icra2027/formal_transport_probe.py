#!/usr/bin/env python3
"""Read-only, post-run sensor transport validation. No GT/truth inputs."""
import hashlib
import io
from pathlib import Path
import rosbag


def validate(source,received):
    def payload(path,imu_topic,toa_topic):
        imu=[];toa=[];starts=[]
        with rosbag.Bag(str(path)) as bag:
            for topic,m,t in bag.read_messages():
                if topic==imu_topic:
                    imu.append((m.header.stamp.to_nsec(),m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,
                                m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z))
                elif topic==toa_topic:
                    # roscpp assigns publication header.seq; sensor stamp/frame and every payload field stay invariant.
                    m.header.seq=0
                    stream=io.BytesIO();m.serialize(stream)
                    toa.append((m.header.stamp.to_nsec(),stream.getvalue()))
                elif topic=='/SplineFusion/start_time':starts.append(m.data)
        return imu,toa,starts
    a,b,_=payload(source,'/waveshare_sense_hat_b','/rtls_flares')
    x,y,start=payload(received,'/EstimationInterface/imu_ds','/EstimationInterface/toa_ds')
    return {'imu_equal':a==x,'toa_equal':b==y,'source_imu_count':len(a),'received_imu_count':len(x),
            'source_toa_count':len(b),'received_toa_count':len(y),'start_times':sorted(set(start)),
            'header_seq_semantics':'ROS publication counter ignored; sensor timestamp/frame and all payload fields checked',
            'source_sensor_hash':hashlib.sha256(repr((a,b)).encode()).hexdigest(),
            'received_sensor_hash':hashlib.sha256(repr((x,y)).encode()).hexdigest(),
            'status':'PASS' if a==x and b==y else 'FAIL'}
