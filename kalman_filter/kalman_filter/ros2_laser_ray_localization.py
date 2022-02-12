# !/usr/bin/env/ python3
#
# Copyright 2022 RoadBalance Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

import math
import numpy as np

class LaserPositionNode(Node):

    def __init__(self):
        super().__init__('position_from_laser_ray_publisher')

        self.declare_parameter('obstacle_front_x_axis', 11.0)
        self.declare_parameter('alpha_laser_scan', 0.05)
        self.declare_parameter('laser_scan_topic', 'scan')
        self.declare_parameter('verbose', "True")

        self._obs_dis = self.get_parameter('obstacle_front_x_axis').value

        # laser scan noise parameter, specifies the noise in the laser scan readings
        self._alpha_laser_scan = self.get_parameter('alpha_laser_scan').value
        self._laser_topic = str(self.get_parameter('laser_scan_topic').value)
        self._verbose = eval(self.get_parameter('verbose').value)

        self._obs_dis_pub = self.create_publisher(
            Float64, 'position_from_laser_ray', 1
        )

        qos_depth = 5

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self._laser_sub = self.create_subscription(
            LaserScan, self._laser_topic, self.laser_scan_callback, QOS_RKL10V
        )
        self._laser_sub

        # global variables
        self._robot_x_axis_position = 0
        # pub msg
        self._robot_x_axis_position = Float64()

        # Print node start information
        if self._verbose:
            print("-----------------------------------------------------\n")
            print("Started laser ray localization node")
            print("-----------------------------------------------------\n")

    def laser_scan_callback(self, msg):
        # print("-----------------------------------------------------\n")
        # self.get_logger().info(f'Distance from Front Object : {msg.ranges[10]}')

        # keep the minimum distance reading from 10 rays pointing to the front
        # second min is required to filter out 'inf' values, in that case 12 is used 
        front_laser_ray = min(min(msg.ranges), self._obs_dis)
        # print(min(msg.ranges[354:363]), len(msg.ranges))

        for i, point in enumerate(msg.ranges):
            if point != math.inf:
                if self._verbose:
                    print(i, point)

        if self._verbose:
            self.get_logger().info(f"Distance to object in front (front_laser_ray): {front_laser_ray}")

        # calculate robot position in the world considering the known position of an obstacle in front
        # example: position of obstacle: 10, laser_ray_reading = 8  => robot_x_axis_position = 2
        # This assumes/requires a robot moving straigt and parallel to x-axis, with orientation = [0,0,0,1]
        x_position_in_map = self._obs_dis - front_laser_ray
        
        if self._verbose:
            self.get_logger().info(f"X position in map frame (x_position_in_map): {x_position_in_map}")

        # generate noisy position
        # standard deviation for noise
        sd_trans = self._alpha_laser_scan * (front_laser_ray)
        self._robot_x_axis_position.data =  np.random.normal(x_position_in_map, sd_trans * sd_trans)
        self._obs_dis_pub.publish(self._robot_x_axis_position)

def main(args=None):
    rclpy.init(args=args)

    laser_position_node = LaserPositionNode()

    rclpy.spin(laser_position_node)

    laser_position_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()