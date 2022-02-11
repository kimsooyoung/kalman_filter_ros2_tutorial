from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='bayes_tutorials',
            node_executable='bayes_filter_node',
            parameters=[
                {'kernel': [.025, .95, .025]},
                {'light_sensor_accuracy_rate': 0.95},
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='bayes_tutorials',
            node_executable='simulated_robot',
            parameters=[
                {'kernel': [.025, .95, .025]},
                {'light_sensor_accuracy_rate': 0.95},
            ],
            output='screen',
            emulate_tty=True
        )
    ])