#!/usr/bin/env python3
"""Save the live map-frame robot pose for the SLAM-to-AMCL handoff."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from tf2_ros import Buffer, TransformListener

parser = argparse.ArgumentParser()
parser.add_argument('--output', default='part1_rosorin/results/runtime/localization_initial_pose.json')
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('capture_mapping_pose')
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + 10
try:
    while not buffer.can_transform('map', 'base_footprint', rclpy.time.Time()):
        if time.monotonic() > deadline:
            raise TimeoutError('No map -> base_footprint transform; is SLAM running?')
        rclpy.spin_once(node, timeout_sec=0.1)
    transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    p, q = transform.transform.translation, transform.transform.rotation
    result = {'x': p.x, 'y': p.y, 'z': p.z,
              'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
finally:
    node.destroy_node()
    rclpy.shutdown()
