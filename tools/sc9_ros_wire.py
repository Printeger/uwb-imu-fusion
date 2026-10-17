#!/usr/bin/env python3
"""SC9 lossless ROS producer or independent actual sink (same-host clocks)."""
import argparse
import csv
import json
import time
from pathlib import Path


def publish(args):
    import rospy
    from uwb_imu_pl.msg import CanonicalSensorEvent, CanonicalRange
    # The producer may preload its immutable recording; the algorithm gets only
    # arrivals released at the original bag receive offsets, independently.
    events = [json.loads(line) for line in Path(args.events).open()]
    if not events:
        raise ValueError('empty recording')
    rospy.init_node('sc9_wire_producer', anonymous=True)
    publisher = rospy.Publisher('/sc9/input', CanonicalSensorEvent, queue_size=512, tcp_nodelay=True)
    deadline = time.monotonic() + 30
    while publisher.get_num_connections() != 1:
        if time.monotonic() > deadline:
            raise RuntimeError('expected exactly one algorithm subscriber')
        time.sleep(.01)
    origin = time.monotonic_ns() + 1_000_000_000
    recording = events[0]['received_ns']
    with Path(args.output).open('w') as log:
        writer = csv.writer(log)
        writer.writerow(['event_sequence', 'scheduled_ns', 'released_ns', 'lateness_ns'])
        for event in events:
            m = CanonicalSensorEvent()
            m.kind = m.IMU if event['kind'] == 'imu' else m.RANGE
            for name in ('event_sequence', 'stamp_ns', 'received_ns'):
                setattr(m, name, event[name])
            if m.kind == m.IMU:
                for name in ('measurement_id', 'quality_flags', 'accel', 'gyro'):
                    setattr(m, name, event[name])
            else:
                m.batch_id = event['batch_id']
                for row in event['ranges']:
                    r = CanonicalRange()
                    for name in r.__slots__:
                        setattr(r, name, row[name])
                    m.ranges.append(r)
            scheduled = origin + event['received_ns'] - recording
            m.replay_origin_steady_ns = origin
            m.recording_origin_ns = recording
            while True:
                remaining = scheduled - time.monotonic_ns()
                if remaining <= 0:
                    break
                time.sleep(remaining / 1e9)
            m.release_steady_ns = time.monotonic_ns()
            publisher.publish(m)
            writer.writerow([m.event_sequence, scheduled, m.release_steady_ns, m.release_steady_ns-scheduled])
        end = CanonicalSensorEvent()
        end.kind = end.END
        end.event_sequence = events[-1]['event_sequence'] + 1
        end.replay_origin_steady_ns = origin
        end.recording_origin_ns = recording
        end.release_steady_ns = time.monotonic_ns()
        publisher.publish(end)
        time.sleep(1)


def sink(args):
    import rospy
    from uwb_imu_pl.msg import UnifiedRollingOutput
    rospy.init_node('sc9_wire_actual_sink', anonymous=True)
    header = Path(args.header).open().readline()
    range_times = {}
    batch_stamps = {}
    for line in Path(args.events).open():
        event = json.loads(line)
        if event['kind'] == 'range':
            batch_stamps[event['batch_id']] = event['stamp_ns']
            stamps = [r['stamp_ns'] for r in event['ranges'] if r['valid']]
            range_times[event['batch_id']] = (min(stamps), max(stamps)) if stamps else None
    finished = []
    errors = []
    received_epochs = []
    with Path(args.output).open('w') as result, Path(args.timing).open('w') as timing:
        result.write(header)
        def receive(m):
            arrived = time.monotonic_ns()
            if m.finished:
                finished.append(dict(error=m.error, summary=m.record, events=m.input_events,
                                     queue_high_water=m.input_queue_high_water, sink_ns=arrived))
                return
            try:
                if m.epoch != len(received_epochs) + 1:
                    raise ValueError('sink result sequence gap or duplicate')
                received_epochs.append(m.epoch)
                if not m.record.endswith('\n'):
                    raise ValueError('incomplete canonical result')
                result.write(m.record)
                row = dict(epoch=m.epoch, batch_id=m.batch_id, sample_ns=m.sample_ns,
                           latest_original_used_ns=m.latest_original_used_ns,
                           callback_ns=m.callback_steady_ns, worker_start_ns=m.worker_start_steady_ns,
                           publish_ns=m.publish_steady_ns, sink_ns=arrived, core_ms=m.core_ms,
                           queue_high_water=m.input_queue_high_water, events=m.input_events,
                           measurement_to_sink_ms=(arrived-m.replay_origin_steady_ns-
                               (batch_stamps[m.batch_id]-m.recording_origin_ns))/1e6,
                           latest_used_sensor_to_sink_ms=(arrived-m.replay_origin_steady_ns-
                               (m.latest_original_used_ns-m.recording_origin_ns))/1e6,
                           callback_to_sink_ms=(arrived-m.callback_steady_ns)/1e6,
                           worker_queue_ms=(m.worker_start_steady_ns-m.callback_steady_ns)/1e6)
                stamps = range_times.get(m.batch_id)
                if stamps:
                    for label, stamp in zip(('earliest_range_to_sink_ms','latest_range_to_sink_ms'), stamps):
                        row[label] = (arrived-m.replay_origin_steady_ns-(stamp-m.recording_origin_ns))/1e6
                row['model_epoch_to_sink_ms'] = (arrived-m.replay_origin_steady_ns-(m.sample_ns-m.recording_origin_ns))/1e6
                timing.write(json.dumps(row, separators=(',', ':'))+'\n')
            except Exception as error:
                errors.append(str(error))
        sub = rospy.Subscriber('/sc9/result', UnifiedRollingOutput, receive, queue_size=1024, tcp_nodelay=True)
        deadline = time.monotonic() + args.timeout
        while not finished and not errors and not rospy.is_shutdown():
            if time.monotonic() > deadline:
                errors.append('sink timeout before END')
                break
            time.sleep(.02)
        Path(args.status).write_text(json.dumps(dict(finished=finished, errors=errors, received_epochs=len(received_epochs)), indent=2)+'\n')
        if errors or not finished or finished[0]['error']:
            raise RuntimeError('actual sink failed; see status artifact')


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest='command', required=True)
    producer = commands.add_parser('publish')
    producer.add_argument('--events', required=True)
    producer.add_argument('--output', required=True)
    consumer = commands.add_parser('sink')
    consumer.add_argument('--header', required=True)
    consumer.add_argument('--events', required=True)
    consumer.add_argument('--output', required=True)
    consumer.add_argument('--timing', required=True)
    consumer.add_argument('--status', required=True)
    consumer.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    (publish if args.command == 'publish' else sink)(args)


if __name__ == '__main__':
    main()
