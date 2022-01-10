from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bayes_tutorials',
            executable='simulated_robot',
            parameters=[
                {'kernel': [.05, .9, .05]},
                {'light_sensor_accuracy_rate': 0.95},
            ],
            output='screen',
            emulate_tty=True
        )
    ])