# Based on:
# https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
# https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/tb3_simulation_launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = FindPackageShare('assessment')
    turtlebot3_gazebo_package_dir = get_package_share_directory('turtlebot3_gazebo')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pose = {'x': LaunchConfiguration('x_pose', default='0.0'),
            'y': LaunchConfiguration('y_pose', default='0.0'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll',   default='0.0'),
            'P': LaunchConfiguration('pitch',  default='0.0'),
            'Y': LaunchConfiguration('yaw',    default='0.0')}
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='waffle_pi',
        description='Name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=PathJoinSubstitution([package_dir, 'models', 'waffle_pi', 'model.sdf']),
        description='Full path to robot SDF file to spawn the robot in Gazebo')

    urdf = os.path.join(turtlebot3_gazebo_package_dir, 'urdf', 'turtlebot3_waffle_pi' + '.urdf')

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=remappings)

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
