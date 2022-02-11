import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi

import xacro


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('kalman_filter'))
    world_path = os.path.join(pkg_path, 'worlds', 'neuronbot2_world.world')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    model_pkg_path = os.path.join(get_package_share_directory('sw_neuronbot2_gazebo'))
    gazebo_model_path = os.path.join(model_pkg_path, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else :
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Prepare Robot State Publisher Params
    neuronbot_gazebo_pkg_path = os.path.join(get_package_share_directory('sw_neuronbot2_gazebo'))
    urdf_file = os.path.join(neuronbot_gazebo_pkg_path, 'urdf', 'neuronbot2.urdf')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file],
    )

    #TODO: verbose option with args
    teleport_service = Node(
        package='kalman_filter',
        executable='teleport_service',
        output='screen',
        parameters=[
            {"model_name": "neuronbot2"},
            {"target_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
        ]
    )

    odom_utility = Node(
        package='kalman_filter',
        executable='odom_utility_tools',
        output='screen',
        parameters=[
            {"model_name": "neuronbot2"},
        ]
    )

    laser_ray_localization = Node(
        package='kalman_filter',
        executable='laser_ray_localization',
        output='screen',
        parameters=[
            {"obstacle_front_x_axis": 11.0},
            {"laser_scan_topic": "scan"},
        ]
    )

    move_forward = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/cmd_vel', 'geometry_msgs/msg/Twist', '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
        output='screen'
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        teleport_service,
        odom_utility,
        laser_ray_localization,
        move_forward,
    ])