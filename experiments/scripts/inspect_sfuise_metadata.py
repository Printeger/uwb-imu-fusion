#!/usr/bin/env python3
"""Read only ISAS Walk1/2/3 metadata; print JSON, never run an estimator."""
import hashlib
import json
from pathlib import Path
import rosbag

ROOT = Path(__file__).resolve().parents[2]


def inspect():
    result = {}
    for n in (1, 2, 3):
        path = ROOT / f'data/SFUISE/ISAS-Walk{n}.bag'
        tags, anchors, frames, counts = set(), set(), {}, {}
        times, coordinates = {}, {}
        with rosbag.Bag(str(path), 'r') as bag:
            topics = bag.get_type_and_topic_info().topics
            for topic, msg, _ in bag.read_messages():
                counts[topic] = counts.get(topic, 0) + 1
                if hasattr(msg, 'header'):
                    times.setdefault(topic, []).append(msg.header.stamp.to_nsec())
                    frames.setdefault(topic, set()).add(msg.header.frame_id)
                if topic == '/rtls_flares':
                    tags.add(str(msg.id))
                    anchors.update(str(r.id) for r in msg.ranges)
                if topic == '/anchor_list' and not coordinates:
                    coordinates = {str(a.id): [a.position.x, a.position.y, a.position.z]
                                   for a in msg.anchor}
                if topic == '/vive/transform/tracker_1_ref':
                    frames.setdefault('gt_child', set()).add(msg.child_frame_id)
        result[f'ISAS-Walk{n}'] = {
            'raw_data_path': str(path.relative_to(ROOT)),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'tag_ids': sorted(tags), 'anchor_ids': sorted(anchors),
            'anchor_coordinates_first_message_m': coordinates,
            'frames': {k: sorted(v) for k, v in frames.items()},
            'topics': {k: {'message_type': v.msg_type, 'count': counts[k]}
                       for k, v in topics.items()},
            'header_time_ns': {k: {'min': min(v), 'max': max(v),
                                  'backward_steps': sum(b < a for a, b in zip(v, v[1:]))}
                               for k, v in times.items()},
            'scope': 'METADATA_ONLY_NO_ESTIMATOR_NO_GT_POSE_EXPORT'
        }
    return result


if __name__ == '__main__':
    print(json.dumps(inspect(), indent=2, sort_keys=True))
