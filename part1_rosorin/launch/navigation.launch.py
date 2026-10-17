"""Standard Nav2 bringup with vendor-derived parameters and simulation bridge."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    part1 = Path(__file__).resolve().parents[1]
    bringup = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(part1 / 'maps/room_slam.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': str(part1 / 'config/nav2.yaml'),
                'use_sim_time': 'true', 'autostart': 'true',
                'use_composition': 'False', 'slam': 'False',
            }.items(),
        ),
        # Nav2's smoothed /cmd_vel reaches the same vendor Gazebo drive topic.
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='navigation_velocity_bridge',
             arguments=['/controller/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
             remappings=[('/controller/cmd_vel', '/cmd_vel')], output='screen'),
    ])
