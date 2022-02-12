from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='kalman_filter',
            node_executable='kalman_1d',
            parameters=[
                {"initial_mu": 5.0},
                {"initial_sig": 1000.0},
                {"motion_sig": 4.0},
                {"measurement_sig": 0.05},
                {"laser_scan_topic": "scan"},
                {"verbose": "False"},
            ],
            output='screen',
            emulate_tty=True
        ),
    ])