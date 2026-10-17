"""Bounded AMCL localization using live LiDAR/map consistency, without old poses.

No simulator ground truth and no motion commands are used. Global localization
can remain ambiguous in symmetric rooms; timeout is a failure, never a success.
"""
import math
import time

import numpy as np
from scipy.ndimage import distance_transform_edt
import rclpy
from tf2_ros import TransformException
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from global_scan_match import find_pose


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class LocalizationGuard:
    def __init__(self, node, tf):
        self.node, self.tf = node, tf
        self.map = self.grid = self.distances = self.scan = self.estimate = None
        self.estimate_at = self.scan_at = 0.0
        self.updates = 0
        self.metrics = {}
        self.scan_seed = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.subs = [
            node.create_subscription(OccupancyGrid, '/map', self.on_map, latched),
            node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, latched),
            node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
        ]
        self.update_client = node.create_client(Empty, '/request_nomotion_update')
        self.global_client = node.create_client(Empty, '/reinitialize_global_localization')

    def on_map(self, msg):
        self.map = msg
        self.grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.distances = distance_transform_edt(self.grid < 65) * msg.info.resolution

    def on_pose(self, msg):
        self.estimate, self.estimate_at = msg, time.monotonic()
        self.updates += 1

    def on_scan(self, msg):
        self.scan, self.scan_at = msg, time.monotonic()

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)

    def call(self, client, request, deadline):
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Service unavailable: {client.srv_name}')
            self.spin()
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError(f'Service timeout: {client.srv_name}')
            self.spin()
        if future.exception():
            raise future.exception()
        return future.result()

    def wait_active(self, name, deadline):
        self.node.get_logger().info(f'Waiting for active lifecycle node: {name}')
        client = self.node.create_client(GetState, f'/{name}/get_state')
        try:
            while time.monotonic() < deadline:
                if self.call(client, GetState.Request(), deadline).current_state.label == 'active':
                    return
                self.spin()
            raise TimeoutError(f'{name} did not become active')
        finally:
            self.node.destroy_client(client)

    def cells(self, x, y):
        origin = self.map.info.origin
        angle = yaw(origin.orientation)
        dx, dy = x - origin.position.x, y - origin.position.y
        col = np.floor((np.cos(angle)*dx + np.sin(angle)*dy) / self.map.info.resolution).astype(int)
        row = np.floor((-np.sin(angle)*dx + np.cos(angle)*dy) / self.map.info.resolution).astype(int)
        return row, col

    def check(self):
        self.metrics = {'ready': False}
        if self.map is None or self.scan is None or self.estimate is None:
            return False
        now = time.monotonic()
        if now - self.scan_at > 2 or now - self.estimate_at > 3:
            return False
        scan = self.scan
        stamp = rclpy.time.Time.from_msg(scan.header.stamp)
        # Reject a paused/stale simulated clock and transforms unrelated to this scan.
        age = (self.node.get_clock().now() - stamp).nanoseconds / 1e9
        if age < -0.2 or age > 2:
            return False
        try:
            transform = self.tf.lookup_transform('map', scan.header.frame_id, stamp).transform
            robot = self.tf.lookup_transform('map', 'base_footprint', stamp).transform
        except Exception:
            return False
        r, c = self.cells(robot.translation.x, robot.translation.y)
        if not (0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]):
            return False
        if self.grid[r, c] < 0 or self.grid[r, c] >= 65:
            return False
        ranges = np.asarray(scan.ranges)
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        indices = np.flatnonzero(valid)[::2]
        if len(indices) < 20:
            return False
        angles = scan.angle_min + indices * scan.angle_increment + yaw(transform.rotation)
        x = transform.translation.x + ranges[indices] * np.cos(angles)
        y = transform.translation.y + ranges[indices] * np.sin(angles)
        rows, cols = self.cells(x, y)
        inside = (rows >= 0) & (rows < self.grid.shape[0]) & (cols >= 0) & (cols < self.grid.shape[1])
        matched = np.zeros(len(indices), dtype=bool)
        matched[inside] = self.distances[rows[inside], cols[inside]] <= .20
        ratio = float(matched.mean())
        covariance = np.asarray(self.estimate.pose.covariance).reshape(6, 6)
        if not np.isfinite(covariance).all():
            return False
        xy_std = math.sqrt(max(0., float(np.linalg.eigvalsh(covariance[:2, :2]).max())))
        yaw_std = math.sqrt(max(0., float(covariance[5, 5])))
        ready = (math.isfinite(xy_std) and math.isfinite(yaw_std)
                 and xy_std <= .35 and yaw_std <= .35 and ratio >= .65)
        self.metrics = {'ready': ready, 'xy_std_m': xy_std, 'yaw_std_rad': yaw_std,
                        'scan_match_ratio': ratio, 'matched_beams': int(matched.sum())}
        return ready

    def ensure(self, mode='auto', timeout=90., initial_pose=None):
        deadline = time.monotonic() + timeout
        self.wait_active('amcl', deadline)
        if initial_pose is not None:
            self.node.setInitialPose(initial_pose)
        started = time.monotonic()
        last_reset = None
        last_update = last_log = 0.
        stable = 0
        last_count = self.updates
        reset_count = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if (mode == 'global' and last_reset is None) or (
                mode in ('auto', 'global') and initial_pose is None
                and now - (last_reset or started) >= (20 if last_reset else 5)
            ):
                self.node.get_logger().info('AMCL: searching the full map (no saved initial pose).')
                self.call(self.global_client, Empty.Request(), deadline)
                if self.map is not None and self.scan is not None and now - self.scan_at < 2:
                    try:
                        laser = self.tf.lookup_transform('base_footprint', self.scan.header.frame_id,
                                                         rclpy.time.Time()).transform
                        o = self.map.info.origin
                        seed = find_pose(self.grid, self.distances, self.map.info.resolution,
                                         (o.position.x, o.position.y, yaw(o.orientation)), self.scan,
                                         (laser.translation.x, laser.translation.y, yaw(laser.rotation)),
                                         min(deadline, time.monotonic() + 15))
                        if seed:
                            msg = PoseStamped()
                            msg.header.frame_id = 'map'
                            msg.header.stamp = self.node.get_clock().now().to_msg()
                            msg.pose.position.x, msg.pose.position.y = seed['x'], seed['y']
                            msg.pose.orientation.z = math.sin(seed['yaw']/2)
                            msg.pose.orientation.w = math.cos(seed['yaw']/2)
                            self.node.setInitialPose(msg)
                            self.scan_seed = seed
                            self.node.get_logger().info(f'Global LiDAR/map pose candidate: {seed}')
                    except (TransformException, LookupError, RuntimeError) as exc:
                        self.node.get_logger().warning(f'Scan candidate unavailable: {exc}')
                last_reset = time.monotonic()
                self.estimate = None
                stable = 0
                reset_count += 1
            if now - last_update >= .5:
                self.call(self.update_client, Empty.Request(), deadline)
                last_update = now
            self.spin()
            if self.updates != last_count:
                stable = stable + 1 if self.check() else 0
                last_count = self.updates
            if now - last_log >= 5:
                self.node.get_logger().info(f'Localization check: {self.metrics}')
                last_log = now
            if stable >= 5:
                # Fresh startup: global costmap/planner activation needs AMCL's
                # map->odom first. Waiting for Nav2 before localization deadlocks.
                self.wait_active('bt_navigator', deadline)
                return {**self.metrics, 'global_resets': reset_count,
                        'scan_seed': self.scan_seed,
                        'method': 'explicit_pose' if initial_pose else mode}
        raise TimeoutError(f'Localization did not converge; no navigation goal sent. Last checks: {self.metrics}. '
                           'Check map/scan and move to a more distinctive area if the map is ambiguous.')

    def validate_goals(self, goals):
        for x, y, _ in goals:
            r, c = self.cells(x, y)
            if not (0 <= r < self.grid.shape[0] and 0 <= c < self.grid.shape[1]):
                raise ValueError(f'Goal ({x}, {y}) is outside the loaded map')
            if self.grid[r, c] != 0 or self.distances[r, c] < .16:
                raise ValueError(f'Goal ({x}, {y}) is unknown, occupied or too close to a mapped obstacle')
