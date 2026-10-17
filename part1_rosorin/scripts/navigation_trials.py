#!/usr/bin/env python3
"""Run stock Nav2 actions and record their actual outcomes and robot poses."""
import argparse
import json
import math
from pathlib import Path
import time
from datetime import datetime
import yaml
import fcntl

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener
from localization_guard import LocalizationGuard


def load_goals(path, task, override=None):
    config = yaml.safe_load(Path(path).read_text())
    def convert(value):
        values = [float(value[key]) for key in ('x', 'y', 'yaw_deg')]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Goal coordinates must be finite numbers')
        return (values[0], values[1], math.radians(values[2]))
    tasks = []
    if task in ('single', 'both'):
        target = dict(zip(('x', 'y', 'yaw_deg'), override)) if override else config['single_goal']
        tasks.append(('single_goal', [convert(target)]))
    if task in ('waypoints', 'both'):
        points = config['waypoints']
        if not isinstance(points, list) or not points:
            raise ValueError('waypoints must be a non-empty list')
        tasks.append(('multi_point', [convert(p) for p in points]))
    return tasks


def pose(nav, x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = nav.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--initial-pose', help='Optional known pose JSON; never read automatically')
    parser.add_argument('--output', help='Default: a new timestamped result; previous evidence is preserved')
    parser.add_argument('--goals', default='part1_rosorin/config/navigation_goals.yaml')
    parser.add_argument('--task', choices=('single', 'waypoints', 'both'), default='both')
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DEG'),
                        help='Override single goal in metres/degrees; use with --task single')
    parser.add_argument('--localization', choices=('auto', 'global', 'current'), default='auto',
                        help='auto: reuse verified AMCL, otherwise global search; global: force search; current: no reset')
    parser.add_argument('--localization-timeout', type=float, default=90.)
    parser.add_argument('--localize-only', action='store_true', help='Verify/localize without sending navigation goals')
    args = parser.parse_args()
    if args.goal and args.task != 'single':
        parser.error('--goal requires --task single')
    if args.initial_pose and args.localization == 'global':
        parser.error('--initial-pose cannot be combined with --localization global')
    if not math.isfinite(args.localization_timeout) or args.localization_timeout <= 0:
        parser.error('--localization-timeout must be finite and positive')
    try:
        tasks = [] if args.localize_only else load_goals(args.goals, args.task, args.goal)
    except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    output = Path(args.output or ('part1_rosorin/results/runtime/navigation_trials_' +
                                 datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json'))
    output.parent.mkdir(parents=True, exist_ok=True)
    lock_path = Path(__file__).resolve().parents[1] / 'results/runtime/.navigation.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock = lock_path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        parser.error('Another trials/localize process is already running; stop it first')
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    tf = Buffer()
    listener = TransformListener(tf, nav)
    guard = LocalizationGuard(nav, tf)
    result = {'status': 'FAIL', 'trials': [], 'initial_pose_source': args.initial_pose,
              'goals_source': args.goals, 'localization_mode': args.localization}

    def current():
        try:
            t = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            p, q = t.transform.translation, t.transform.rotation
            return [p.x, p.y, 2 * math.atan2(q.z, q.w)]
        except Exception:
            return None

    try:
        initial = None
        if args.initial_pose:
            data = json.loads(Path(args.initial_pose).read_text())
            initial = pose(nav, data['x'], data['y'], 2 * math.atan2(data['qz'], data['qw']))
        result['localization'] = guard.ensure(args.localization, args.localization_timeout, initial)
        for name, goals in tasks:
            guard.validate_goals(goals)
            messages = [pose(nav, *g) for g in goals]
            start = time.monotonic()
            start_sim = nav.get_clock().now().nanoseconds * 1e-9
            trial = {'name': name, 'goals': goals, 'waypoint_transitions': []}
            if name == 'single_goal':
                if not nav.nav_to_pose_client.wait_for_server(timeout_sec=10.):
                    raise TimeoutError('NavigateToPose server unavailable')
                accepted = nav.goToPose(messages[0])
            else:
                if not nav.follow_waypoints_client.wait_for_server(timeout_sec=10.):
                    raise TimeoutError('FollowWaypoints server unavailable')
                accepted = nav.followWaypoints(messages)
            if not accepted:
                raise RuntimeError(f'{name}: goal rejected')
            last_waypoint = -1
            while not nav.isTaskComplete():
                feedback = nav.getFeedback()
                if feedback is not None and hasattr(feedback, 'current_waypoint'):
                    index = feedback.current_waypoint
                    if index != last_waypoint:
                        trial['waypoint_transitions'].append(
                            {'current_waypoint': index, 'pose': current()})
                        last_waypoint = index
                if time.monotonic() - start > 150:
                    nav.cancelTask()
                    raise TimeoutError(f'{name}: exceeded 150 wall seconds')
            action_result = nav.getResult()
            final = current()
            error = math.hypot(final[0] - goals[-1][0], final[1] - goals[-1][1]) if final else None
            trial.update(action_result=action_result.name, final_pose=final,
                         final_position_error_m=error, wall_seconds=time.monotonic() - start,
                         simulation_seconds=nav.get_clock().now().nanoseconds * 1e-9 - start_sim)
            if name == 'multi_point' and nav.result_future is not None:
                trial['missed_waypoints'] = list(nav.result_future.result().result.missed_waypoints)
            trial['pass'] = (action_result == TaskResult.SUCCEEDED and error is not None
                             and error <= 0.35 and not trial.get('missed_waypoints'))
            result['trials'].append(trial)
            print(json.dumps(trial), flush=True)
            if not trial['pass']:
                raise RuntimeError(f'{name} did not meet action/pose checks')
        result['status'] = 'PASS'
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc)
        if nav.result_future is not None and not nav.result_future.done():
            nav.cancelTask()
    finally:
        output.write_text(json.dumps(result, indent=2) + '\n')
        nav.destroy_node()
        rclpy.shutdown()
        lock.close()
    print(json.dumps(result), flush=True)
    print(f'Result saved: {output}', flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
