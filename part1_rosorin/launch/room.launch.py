"""Run the vendor room/model with its Gazebo MecanumDrive and sensor plugins."""

import os
import xml.etree.ElementTree as ET

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _fit_mecanum_drive(robot):
    """Make the MecanumDrive plugin kinematics match the wheel joints in the URDF.

    The vendor plugin block declares wheel_separation 0.34, wheelbase 0.30 and
    wheel_radius 0.05, but the wheel joints it drives sit at (+-0.08, +-0.09) with
    0.04 m wheels. Measured open loop: forward commands reach only 0.8x and
    rotation commands 1.6x, so Nav2's DWB overshoots every turn, flips direction
    and never drives forward (endless in-place spinning, "Failed to make progress").
    """
    joints = {j.get('name'): j for j in robot.findall('joint')}
    links = {l.get('name'): l for l in robot.findall('link')}

    def position(name):
        return [float(v) for v in joints[name].find('origin').get('xyz').split()]

    front_left, front_right = position('front_left_joint'), position('front_right_joint')
    rear_left = position('rear_left_joint')
    radius = float(links['front_left_link'].find('collision/geometry/cylinder').get('radius'))
    fitted = {'wheel_separation': front_left[1] - front_right[1],
              'wheelbase': front_left[0] - rear_left[0],
              'wheel_radius': radius}
    for plugin in robot.iter('plugin'):
        if plugin.get('name', '').endswith('MecanumDrive'):
            for tag, value in fitted.items():
                plugin.find(tag).text = f'{value:.4f}'
            return
    raise RuntimeError('MecanumDrive plugin not found in the robot description')


def _setup(context):
    share = get_package_share_directory('robot_gazebo')
    gazebo_share = get_package_share_directory('ros_ign_gazebo')
    sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    world = os.path.join(share, 'worlds', 'robocup_home.sdf')
    # Set only the camera selection needed by the vendor Xacro, not all .typerc.
    os.environ['DEPTH_CAMERA_TYPE'] = LaunchConfiguration('camera_type').perform(context)
    description = xacro.process_file(
        os.path.join(share, 'urdf', 'robot.gazebo.xacro'),
        mappings={'sim_ign': 'true'},
    ).toxml()
    robot = ET.fromstring(description)
    # Vendor URDF has no ros2_control hardware declaration. The independent
    # MecanumDrive and JointStatePublisher systems already control/publish joints.
    for gazebo in robot.findall('gazebo'):
        for plugin in list(gazebo.findall('plugin')):
            if 'ROS2ControlPlugin' in plugin.get('name', ''):
                gazebo.remove(plugin)
    _fit_mecanum_drive(robot)
    description = ET.tostring(robot, encoding='unicode')

    def vendor_launch(filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items(),
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'ign_gazebo.launch.py')
            ),
            launch_arguments={'ign_args': ' -r ' + world}.items(),
        ),
        vendor_launch('spawn_objects.launch.py', {}),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': sim_time}],
            output='screen',
        ),
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-world', 'robot_world', '-topic', 'robot_description',
                       '-name', 'robot', '-x', '0', '-y', '0', '-z', '0'],
            parameters=[{'use_sim_time': sim_time}], output='screen',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            arguments=[
                '/controller/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
            remappings=[('/odom/tf', '/tf')],
            parameters=[{'use_sim_time': sim_time}], output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('camera_type', default_value='aurora'),
        OpaqueFunction(function=_setup),
    ])
