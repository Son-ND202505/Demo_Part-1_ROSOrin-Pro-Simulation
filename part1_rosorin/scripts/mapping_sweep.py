#!/usr/bin/env python3
"""Operator-style slow rotation to expose the room to the vendor SLAM pipeline."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from verify_control_sensors import Probe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='part1_rosorin/results/runtime/mapping_sweep.json')
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    total = 0.0
    result = {'status': 'FAIL', 'purpose': 'SLAM observation sweep'}
    try:
        node.spin_until(lambda: node.pose is not None and node.scan is not None)
        result['initial_odom_pose'] = list(node.pose)
        previous = node.pose[2]
        start = time.monotonic()
        command = Twist()
        command.angular.z = 0.25
        while total < 2 * math.pi + 0.3:
            if time.monotonic() - start > 120:
                raise TimeoutError('Mapping rotation did not finish within 120 wall seconds')
            node.publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
            yaw = node.pose[2]
            total += math.atan2(math.sin(yaw - previous), math.cos(yaw - previous))
            previous = yaw
        node.command(0.0, 0.0, 0.5)
        result.update(status='PASS', rotated_radians=total,
                      wall_seconds=time.monotonic() - start, final_odom_pose=node.pose)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        for _ in range(10):
            node.publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.05)
        Path(args.output).write_text(
            json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
