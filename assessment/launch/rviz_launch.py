# Based on:
# https://raw.githubusercontent.com/ros-planning/navigation2/humble/nav2_bringup/launch/rviz_launch.py
# https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_bringup/launch/rviz2.launch.py

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def rviz_action(context : LaunchContext):

    namespace = context.launch_configurations['ros_namespace']
    rviz_config_file = context.launch_configurations['rviz_config']

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})
    
    namespaced_rviz_config_file = ReplaceString(
            condition=LaunchConfigurationEquals('ros_namespace', '/robot1'),
            source_file=namespaced_rviz_config_file,
            replacements={'<window_x>': '0',
                          '<window_y>': '720'})
    
    namespaced_rviz_config_file = ReplaceString(
            condition=LaunchConfigurationEquals('ros_namespace', '/robot2'),
            source_file=namespaced_rviz_config_file,
            replacements={'<window_x>': '1280',
                          '<window_y>': '720'})
    
    namespaced_rviz_config_file = ReplaceString(
            condition=LaunchConfigurationEquals('ros_namespace', '/robot3'),
            source_file=namespaced_rviz_config_file,
            replacements={'<window_x>': '1280',
                          '<window_y>': '0'})

    start_namespaced_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', namespaced_rviz_config_file],
        output='screen')

    exit_event_handler_namespaced = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    return [start_namespaced_rviz_cmd, exit_event_handler_namespaced]


def generate_launch_description():
    package_name = 'assessment'

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'namespaced.rviz']),
        description='Full path to the RViz config file to use')
    
    rviz_cmd = OpaqueFunction(function=rviz_action)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(rviz_cmd)

    return ld
