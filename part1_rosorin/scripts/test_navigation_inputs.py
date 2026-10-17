"""Regression checks for configurable goals and map-frame validation (no ROS graph)."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from navigation_trials import load_goals
from localization_guard import LocalizationGuard
from global_scan_match import find_pose
from scipy.ndimage import distance_transform_edt


class NavigationInputTests(unittest.TestCase):
    def test_auto_reuses_valid_pose_without_initial_pose_or_global_reset(self):
        guard = object.__new__(LocalizationGuard)
        sent = []
        guard.node = NS(get_logger=lambda: NS(info=lambda _: None),
                        setInitialPose=lambda _: self.fail('Must not send an initial pose'))
        def wait_active(name, deadline):
            if name == 'bt_navigator':
                self.assertGreaterEqual(guard.updates, 5,
                                        'Nav2 must not be awaited before AMCL localization')
            sent.append(name)
        guard.wait_active = wait_active
        guard.update_client, guard.global_client = 'nomotion', 'global'
        guard.call = lambda client, *args: sent.append(client)
        guard.updates = 0
        guard.metrics = {'ready': True}
        guard.scan_seed = None
        guard.spin = lambda: setattr(guard, 'updates', guard.updates + 1)
        guard.check = lambda: True
        result = guard.ensure('auto', 1)
        self.assertEqual(result['global_resets'], 0)
        self.assertNotIn('global', sent)
        self.assertLess(sent.index('nomotion'), sent.index('bt_navigator'))

    def test_global_scan_match_recovers_asymmetric_room(self):
        grid = np.zeros((80, 100), dtype=int)
        grid[[0, -1], :] = 100
        grid[:, [0, -1]] = 100
        grid[40:65, 60:65] = 100
        grid[15:20, 25:40] = 100
        resolution = .05
        x, y, heading = 1.1, 1.4, .4
        angles = np.linspace(-math.pi, math.pi, 180, endpoint=False)
        ranges = []
        for a in angles + heading:
            for distance in np.arange(.05, 8, .025):
                r = int((y + distance*math.sin(a))/resolution)
                c = int((x + distance*math.cos(a))/resolution)
                if not (0 <= r < 80 and 0 <= c < 100) or grid[r, c] == 100:
                    ranges.append(float(distance))
                    break
        scan = NS(ranges=ranges, range_min=.03, range_max=8.,
                  angle_min=-math.pi, angle_increment=2*math.pi/180)
        found = find_pose(grid, distance_transform_edt(grid < 65)*resolution,
                          resolution, (0, 0, 0), scan, (0, 0, 0), time.monotonic()+15)
        self.assertIsNotNone(found)
        self.assertLess(math.hypot(found['x']-x, found['y']-y), .2)
        self.assertLess(abs(found['yaw']-heading), .1)

    def test_default_goals_and_degrees(self):
        path = Path(__file__).resolve().parents[1] / 'config/navigation_goals.yaml'
        tasks = load_goals(path, 'both')
        self.assertEqual([len(goals) for _, goals in tasks], [1, 3])
        self.assertAlmostEqual(tasks[0][1][0][2], -math.pi/2)

    def test_override_only_single(self):
        path = Path(__file__).resolve().parents[1] / 'config/navigation_goals.yaml'
        tasks = load_goals(path, 'single', [2, 3, 90])
        self.assertEqual(tasks[0][1][0], (2., 3., math.pi/2))

    def test_reject_empty_and_nonfinite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'goals.yaml'
            path.write_text('waypoints: []\nsingle_goal: {x: .nan, y: 0, yaw_deg: 0}')
            for task in ('single', 'waypoints'):
                with self.assertRaises(ValueError):
                    load_goals(path, task)

    def test_rotated_map_origin_and_invalid_goals(self):
        guard = object.__new__(LocalizationGuard)
        guard.map = NS(info=NS(resolution=1., origin=NS(position=NS(x=10., y=20.),
                        orientation=NS(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4)))))
        guard.grid = np.zeros((4, 4), dtype=int)
        guard.distances = np.ones((4, 4))
        self.assertEqual(guard.cells(8.5, 21.5), (1, 1))
        guard.validate_goals([(8.5, 21.5, 0)])
        guard.grid[1, 1] = -1
        with self.assertRaises(ValueError):
            guard.validate_goals([(8.5, 21.5, 0)])
        guard.grid[1, 1] = 100
        with self.assertRaises(ValueError):
            guard.validate_goals([(8.5, 21.5, 0)])
        with self.assertRaises(ValueError):
            guard.validate_goals([(100., 100., 0)])

    def test_mecanum_drive_geometry_follows_urdf_wheel_joints(self):
        launch_file = Path(__file__).resolve().parents[1] / 'launch/room.launch.py'
        spec = importlib.util.spec_from_file_location('room_launch', launch_file)
        room = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(room)
        urdf = """<robot>
          <link name="front_left_link"><collision><geometry>
            <cylinder radius="0.04" length="0.025"/></geometry></collision></link>
          <joint name="front_left_joint"><origin xyz="0.08 0.09 0.01"/></joint>
          <joint name="front_right_joint"><origin xyz="0.08 -0.09 0.01"/></joint>
          <joint name="rear_left_joint"><origin xyz="-0.08 0.09 0.01"/></joint>
          <gazebo><plugin name="ignition::gazebo::systems::MecanumDrive" filename="x">
            <wheel_separation>0.34</wheel_separation><wheelbase>0.3</wheelbase>
            <wheel_radius>0.05</wheel_radius></plugin></gazebo></robot>"""
        robot = ET.fromstring(urdf)
        room._fit_mecanum_drive(robot)
        plugin = robot.find('gazebo/plugin')
        self.assertAlmostEqual(float(plugin.find('wheel_separation').text), 0.18)
        self.assertAlmostEqual(float(plugin.find('wheelbase').text), 0.16)
        self.assertAlmostEqual(float(plugin.find('wheel_radius').text), 0.04)
        robot.find('gazebo').remove(plugin)
        with self.assertRaises(RuntimeError):
            room._fit_mecanum_drive(robot)


if __name__ == '__main__':
    unittest.main()
