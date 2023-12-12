import os
from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetUseSimTime, SetRemap, PushRosNamespace, RosTimer

def generate_launch_description():

    num_robots = 1
    random_seed = 0
    experiment_duration = 300.0
    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced_nav2.rviz'])
    map = PathJoinSubstitution([FindPackageShare('assessment'), 'maps', 'assessment_world.yaml'])
    params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])

    yaml_path = os.path.join(get_package_share_directory('assessment'), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': str(num_robots),
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'sensor_noise': 'false',
                          'use_rviz': 'true',
                          'rviz_config': rviz_config,
                          'obstacles': 'true',
                          'item_manager': 'true',
                          'random_seed': str(random_seed),
                          'use_nav2': 'True',
                          'map': map,
                          'params_file': params,
                          # 'extra_gazebo_args': '--verbose',
                          }.items()
    )

    robot_controller_cmd = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                output='screen',
                parameters=[initial_poses[robot_name]]),

            # Node(
            #     package='turtlebot3_gazebo',
            #     executable='turtlebot3_drive',
            #     output='screen'),

        ])

        robot_controller_cmd.append(group)

    data_logger_cmd = Node(
        package='solution',
        executable='data_logger',
        output='screen',
        arguments=['--filename', 'data_log_' + str(random_seed) + '.csv'])

    timeout_cmd = RosTimer(                                         
            period = experiment_duration,
            actions = [                                                       
                Shutdown(reason="Experiment timeout reached")     
            ],
        )

    ld = LaunchDescription()

    ld.add_action(SetUseSimTime(True))

    ld.add_action(assessment_cmd)

    for cmd in robot_controller_cmd:
        ld.add_action(cmd)

    ld.add_action(data_logger_cmd)

    ld.add_action(timeout_cmd)

    return ld
