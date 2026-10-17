#!/usr/bin/env python3
"""Derive a repository config from the unchanged vendor Nav2 parameters."""
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[2]
source = root / 'third_party/rosorin_pro/upstream/src/simulations/robot_gazebo/config/nav2_params.yaml'
config = yaml.safe_load(source.read_text())


def sim_time(tree):
    if isinstance(tree, dict):
        for key, value in tree.items():
            if key == 'use_sim_time':
                tree[key] = True
            else:
                sim_time(value)


sim_time(config)
config['amcl']['ros__parameters']['robot_model_type'] = 'nav2_amcl::OmniMotionModel'
# Gazebo odometry is the exact model pose; the vendor motion noise (0.2) only makes
# the particle cloud, and so map->odom, wander by decimetres and degrees.
for key in ('alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5'):
    config['amcl']['ros__parameters'][key] = 0.05
# Represent the chassis instead of the vendor's undersized 7 cm radius.
for name in ('local_costmap', 'global_costmap'):
    config[name][name]['ros__parameters']['robot_radius'] = 0.16
# The vendor DWB controller deadlocks on paths that U-turn around a thin wall (the
# room's counter): its GoalAlign/GoalDist critics pull the robot straight toward the
# last plan pose inside the 4 m local window, which lies behind the wall, while
# BaseObstacle blocks that direction. The robot then spins in place forever
# ("Failed to make progress") although the plan itself keeps clear of every wall.
# Regulated Pure Pursuit follows the plan with a lookahead point instead.
config['controller_server']['ros__parameters']['FollowPath'] = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 0.25,          # velocity_smoother limit is 0.26 m/s
    'lookahead_dist': 0.5,
    'min_lookahead_dist': 0.3,
    'max_lookahead_dist': 0.9,
    'lookahead_time': 1.5,
    'transform_tolerance': 0.2,
    'use_velocity_scaled_lookahead_dist': False,
    'min_approach_linear_velocity': 0.05,
    'approach_velocity_scaling_dist': 0.6,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'use_cost_regulated_linear_velocity_scaling': False,
    'regulated_linear_scaling_min_radius': 0.9,
    'regulated_linear_scaling_min_speed': 0.15,
    'use_rotate_to_heading': True,
    'rotate_to_heading_min_angle': 0.785,
    'rotate_to_heading_angular_vel': 1.0,
    'max_angular_accel': 3.2,
    'allow_reversing': False,
}
# RPP stops as soon as the goal checker is satisfied, so the final error is about
# the tolerance itself. The vendor 0.25 m left only 0.1 m below the 0.35 m trial limit.
config['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance'] = 0.15
output = root / 'part1_rosorin/config/nav2.yaml'
output.write_text('# Derived from vendor nav2_params.yaml; see prepare_nav_config.py.\n' +
                  yaml.safe_dump(config, sort_keys=False))
print(output.relative_to(root))
