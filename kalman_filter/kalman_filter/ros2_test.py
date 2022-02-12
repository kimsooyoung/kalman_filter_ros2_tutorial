# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy


class LaserSubscriber(Node):

    def __init__(self):
        super().__init__('laser_sub_node')

        qos_depth = 5

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.subscriber = self.create_subscription(
            LaserScan, 'scan', self.sub_callback, QOS_RKL10V
        )
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        self.get_logger().info(f'Distance from Front Object : {msg.ranges[10]}')


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()