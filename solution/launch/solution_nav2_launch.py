from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced_nav2.rviz'])
    map = PathJoinSubstitution([FindPackageShare('assessment'), 'maps', 'assessment_world.yaml'])
    params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': '1',
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'sensor_noise': 'false',
                          'use_rviz': 'true',
                          'rviz_config': rviz_config,
                          'obstacles': 'true',
                          'item_manager': 'true',
                          'random_seed': '0',
                          'use_nav2': 'True',
                          'map': map,
                          'params': params,
                          # 'extra_gazebo_args': '--verbose',
                          }.items()
    )

    robot_controller_cmd = Node(
        package='solution',
        executable='robot_controller',
        output='screen',
        namespace='robot1')

    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)

    return ld
