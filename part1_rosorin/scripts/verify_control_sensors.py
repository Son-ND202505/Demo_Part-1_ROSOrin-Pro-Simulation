#!/usr/bin/env python3
"""Measure actual simulation motion, scan data and TF for Phase C.

Publishes small velocity commands only; it does not run SLAM or navigation.
"""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Probe(Node):
    def __init__(self):
        super().__init__('part1_control_sensor_check', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.pose = None
        self.scan = None
        self.odom_count = 0
        self.scan_count = 0
        self.first_scan_stamp = None
        self.last_scan_stamp = None
        self.odom_frames = set()
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = [p.x, p.y, yaw]
        self.odom_count += 1
        self.odom_frames.add((msg.header.frame_id, msg.child_frame_id))

    def on_scan(self, msg):
        self.scan = msg
        self.scan_count += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_scan_stamp is None:
            self.first_scan_stamp = stamp
        self.last_scan_stamp = stamp

    def spin_until(self, predicate, wall_timeout=40):
        deadline = time.monotonic() + wall_timeout
        while not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError('Timed out waiting for live simulation data/time')
            rclpy.spin_once(self, timeout_sec=0.05)

    def command(self, x, yaw, seconds):
        msg = Twist()
        msg.linear.x, msg.angular.z = x, yaw
        start_sim = self.get_clock().now().nanoseconds * 1e-9
        start_wall = time.monotonic()
        while self.get_clock().now().nanoseconds * 1e-9 - start_sim < seconds:
            if time.monotonic() - start_wall > 45:
                raise TimeoutError('Simulation did not advance within 45 seconds')
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.get_clock().now().nanoseconds * 1e-9 - start_sim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='part1_rosorin/results/runtime/control_sensors.json')
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    result = {'status': 'FAIL', 'tests': [], 'errors': []}
    try:
        node.spin_until(lambda: node.pose is not None and node.scan is not None
                        and node.get_clock().now().nanoseconds > 0)
        node.command(0.0, 0.0, 0.3)
        for name, x, yaw in [('forward', 0.15, 0.0), ('backward', -0.15, 0.0),
                             ('turn_left', 0.0, 0.3), ('turn_right', 0.0, -0.3)]:
            initial = list(node.pose)
            elapsed = node.command(x, yaw, 1.5)
            node.command(0.0, 0.0, 0.3)
            final = list(node.pose)
            dx, dy = final[0] - initial[0], final[1] - initial[1]
            forward = dx * math.cos(initial[2]) + dy * math.sin(initial[2])
            rotation = math.atan2(math.sin(final[2] - initial[2]),
                                  math.cos(final[2] - initial[2]))
            passed = ((forward * math.copysign(1, x) > 0.03) if x else
                      (rotation * math.copysign(1, yaw) > 0.08))
            test = {'name': name, 'cmd_linear_x': x, 'cmd_angular_z': yaw,
                    'sim_command_seconds': elapsed, 'initial_pose': initial,
                    'final_pose': final, 'forward_displacement_m': forward,
                    'yaw_change_rad': rotation, 'pass': passed}
            result['tests'].append(test)
            print(json.dumps(test), flush=True)
            if not passed:
                raise RuntimeError(f'{name} did not produce sufficient signed movement')
        scan = node.scan
        finite = [v for v in scan.ranges if math.isfinite(v)
                  and scan.range_min <= v <= scan.range_max]
        span = node.last_scan_stamp - node.first_scan_stamp
        result['lidar'] = {'frame_id': scan.header.frame_id,
                           'last_stamp_sim_seconds': node.last_scan_stamp,
                           'samples': len(scan.ranges), 'finite_valid_samples': len(finite),
                           'min_valid_m': min(finite) if finite else None,
                           'max_valid_m': max(finite) if finite else None,
                           'messages_received': node.scan_count,
                           'rate_per_sim_second': (node.scan_count - 1) / span if span > 0 else None}
        if not finite or span <= 0:
            raise RuntimeError('LiDAR lacks valid ranges or advancing timestamps')
        result['odom_frames'] = sorted(node.odom_frames)
        result['odom_messages_received'] = node.odom_count
        result['tf'] = {}
        for target in ('base_footprint', 'base_link', scan.header.frame_id):
            node.spin_until(lambda: node.tf.can_transform('odom', target, rclpy.time.Time()))
            transform = node.tf.lookup_transform('odom', target, rclpy.time.Time())
            t = transform.transform.translation
            result['tf'][f'odom->{target}'] = [t.x, t.y, t.z]
        result['status'] = 'PASS'
    except Exception as error:
        result['errors'].append(str(error))
        print(f'FAIL: {error}', flush=True)
    finally:
        node.publisher.publish(Twist())
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.05)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(f"Verification {result['status']}: {args.output}", flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
