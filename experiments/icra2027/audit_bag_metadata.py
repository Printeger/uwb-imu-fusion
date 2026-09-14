#!/usr/bin/env python3
"""Read only anchor/IMU/GT headers for R1 provenance; never create estimator inputs."""
import json

import numpy as np
import rosbag
import yaml
from fingerprint import ROOT, HERE, sha


def main():
    result = []
    for n in (1, 2, 3):
        cfg = yaml.safe_load((HERE / f'configs/backbone/Walk{n}_B0_CURRENT.yaml').read_text())
        bag_path = ROOT / f'data/SFUISE/ISAS-Walk{n}.bag'
        anchors, headers, seen = {}, {}, 0
        with rosbag.Bag(str(bag_path)) as bag:
            for topic, msg, stamp in bag.read_messages(topics=['/anchor_list', '/waveshare_sense_hat_b',
                                                              '/rtls_flares', '/vive/transform/tracker_1_ref']):
                if topic not in headers and hasattr(msg, 'header'):
                    headers[topic] = {'frame_id': msg.header.frame_id,
                                      'header_time_s': msg.header.stamp.to_sec(),
                                      'bag_time_s': stamp.to_sec(),
                                      'message_type': msg._type}
                if topic == '/anchor_list' and seen < 20:
                    seen += 1
                    for anchor in msg.anchor:
                        anchors.setdefault(int(anchor.id), []).append([anchor.position.x, anchor.position.y, anchor.position.z])
                if seen == 20 and len(headers) == 4:
                    break
        entries = []
        for anchor in cfg['anchors']:
            samples = np.asarray(anchors[anchor['id']])
            entries.append({'anchor_id': anchor['id'], 'source_first': samples[0].tolist(),
                            'source_mean_first20': samples.mean(axis=0).tolist(),
                            'native_config_position': anchor['pos'],
                            'config_mean_difference_m': float(np.linalg.norm(samples.mean(axis=0)-anchor['pos']))})
        result.append({'sequence': f'Walk{n}', 'raw_bag_sha256': sha(bag_path), 'headers': headers,
                       'anchors': entries, 'purpose': 'EVALUATOR_ONLY_PROVENANCE_NO_ESTIMATOR_WRITEBACK'})
    (HERE / 'audits/bag_metadata.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps([{'sequence': r['sequence'], 'headers': r['headers'],
                       'max_anchor_config_difference_m': max(a['config_mean_difference_m'] for a in r['anchors'])}
                      for r in result], indent=2))


if __name__ == '__main__':
    main()
