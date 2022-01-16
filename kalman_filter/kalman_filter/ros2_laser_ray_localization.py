#! /usr/bin/env python

'''
Publishes a noisy x-axis position of a robot (w.r.t. map frame of reference) each time a new laser scan msg arrives

Node reads in laser scan topic, picks one laser ray pointing to the front of the robot
Calculates the robot's x-axis position in the map substracting the laser measurement
from the position (provided as parameter) of one fixed obstacle in the front.
Publishes the robot's x-axis position in the map coordinates as Float64 value
Assumes robot starts in map coord. 0,0,0 and moves towards positive x-axis
Assumes laser sensor is located in the center of the robot (no transform to robot base)
Author: Roberto Zegers R.
Date: August 2020
'''

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

import math
import numpy as np

class LaserPositionNode(Node):

    def __init__(self):
        super().__init__('position_from_laser_ray_publisher')

        self.declare_parameter('obstacle_front_x_axis', 11.0)
        self.declare_parameter('laser_scan_topic', 'scan')
        self._obs_dis = self.get_parameter('obstacle_front_x_axis').value
        self._laser_topic = self.get_parameter('laser_scan_topic').value

        self._obs_dis_pub = self.create_publisher(
            Float64, '/position_from_laser_ray', 1
        )

        self._laser_sub = self.create_subscription(
            LaserScan, self._laser_topic, self.laser_scan_callback, 10
        )

        # global variables
        self._robot_x_axis_position = 0
        # laser scan noise parameter, specifies the noise in the laser scan readings
        self._alpha_laser_scan = 0.05
        # pub msg
        self._robot_x_axis_position = Float64()

        # Print node start information
        print("-----------------------------------------------------\n")
        print("Started laser ray localization node")
        print("-----------------------------------------------------\n")


    def laser_scan_callback(self, msg):

        # keep the minimum distance reading from 10 rays pointing to the front
        # second min is required to filter out 'inf' values, in that case 12 is used 
        front_laser_ray = min(min(msg.ranges), self._obs_dis)
        # print(min(msg.ranges[354:363]), len(msg.ranges))

        for i, point in enumerate(msg.ranges):
            if point != math.inf:
                print(i, point)

        self.get_logger().info(f"Distance to object in front (front_laser_ray): {front_laser_ray}")

        # calculate robot position in the world considering the known position of an obstacle in front
        # example: position of obstacle: 10, laser_ray_reading = 8  => robot_x_axis_position = 2
        # This assumes/requires a robot moving straigt and parallel to x-axis, with orientation = [0,0,0,1]
        x_position_in_map = self._obs_dis - front_laser_ray
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