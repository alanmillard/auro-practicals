# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

package_name = 'assessment'
launch_file_dir = PathJoinSubstitution([FindPackageShare(package_name), 'launch'])

def group_action(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    visualise_sensors = eval(context.launch_configurations['visualise_sensors'].lower().capitalize())

    if visualise_sensors == True:
        robot_sdf = PathJoinSubstitution([FindPackageShare(package_name), 'models', 'waffle_pi_sensors', 'model.sdf'])
    else:
        robot_sdf = PathJoinSubstitution([FindPackageShare(package_name), 'models', 'waffle_pi', 'model.sdf'])

    robots_list = {}

    match num_robots:
        case 1:
            robots_list["robot1"] = {}
            robots_list["robot1"]["y"] =  0.0
        case 2:
            robots_list["robot1"] = {}
            robots_list["robot1"]["y"] =  1.0

            robots_list["robot2"] = {}
            robots_list["robot2"]["y"] = -1.0
        case _:
            robots_list["robot1"] = {}
            robots_list["robot1"]["y"] =  2.0

            robots_list["robot2"] = {}
            robots_list["robot2"]["y"] =  0.0

            robots_list["robot3"] = {}
            robots_list["robot3"]["y"] = -2.0

    for robot_name, robot in robots_list.items():
        robot["x"] = -3.5
        robot["yaw"] = 0.0

    bringup_cmd_group = []

    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction([
            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_file_dir, 'rviz_launch.py'])),
                condition=IfCondition(context.launch_configurations['use_rviz']),
                launch_arguments={'namespace': TextSubstitution(text=robot_name),
                                  'rviz_config': context.launch_configurations['rviz_config']}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    launch_file_dir,
                    'spawn_robot_launch.py'])),
                launch_arguments={'namespace': robot_name,
                                  'use_nav2': context.launch_configurations['use_nav2'],
                                  'map': context.launch_configurations['map'],
                                  'params_file': context.launch_configurations['params_file'],
                                  'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_name': TextSubstitution(text=robot_name),
                                  'robot_sdf': robot_sdf}.items()),
            Node(
                package=package_name,
                executable='item_sensor',
                output='screen',
                namespace=robot_name),
            Node(
                package=package_name,
                executable='home_zone_sensor',
                output='screen',
                namespace=robot_name),
            Node(
                package=package_name,
                executable='robot_sensor',
                output='screen',
                namespace=robot_name)
        ])
    
        bringup_cmd_group.append(group)

    return bringup_cmd_group

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    num_robots = LaunchConfiguration('num_robots')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    obstacles = LaunchConfiguration('obstacles')
    item_manager = LaunchConfiguration('item_manager')
    random_seed = LaunchConfiguration('random_seed')
    use_nav2 = LaunchConfiguration('use_nav2')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('assessment'), 'maps', 'assessment_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('assessment'), 'params', 'nav2_params_namespaced.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_visualise_sensors_cmd = DeclareLaunchArgument(
        'visualise_sensors',
        default_value='false',
        description='Whether to visualise sensors in Gazebo')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'namespaced.rviz']),
        description='Full path to the RViz config file to use')    
    
    declare_obstacles_cmd = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description='Whether the world contains obstacles')

    declare_item_manager_cmd = DeclareLaunchArgument(
        'item_manager',
        default_value='true',
        description='Whether to start the item manager')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager')
    
    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to use the navigation stack (Nav2)')
    
    world = PathJoinSubstitution([
        FindPackageShare(package_name),
        'worlds',
        'assessment_world.world'
    ])

    world_obstacles = PathJoinSubstitution([
        FindPackageShare(package_name),
        'worlds',
        'assessment_world_obstacles.world'
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        condition=UnlessCondition(obstacles),
        launch_arguments={'world': world}.items()
    )

    gzserver_cmd_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        condition=IfCondition(obstacles),
        launch_arguments={'world': world_obstacles}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
    )

    start_tf_relay_cmd = Node(
        package='tf_relay',
        executable='relay',
        output='screen',
        arguments=['robot', num_robots])

    start_item_manager_cmd = Node(
        package=package_name,
        executable='item_manager',
        output='screen',
        condition=IfCondition(item_manager),
        arguments=['--random_seed', random_seed])

    bringup_cmd_group = OpaqueFunction(function=group_action)
        
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_visualise_sensors_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_obstacles_cmd)
    ld.add_action(declare_item_manager_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzserver_cmd_obstacles)
    ld.add_action(gzclient_cmd)

    ld.add_action(start_tf_relay_cmd)
    ld.add_action(start_item_manager_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
