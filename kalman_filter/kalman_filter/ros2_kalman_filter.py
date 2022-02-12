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

'''
Sub:
    noisy_odom_x - Float64
    position_from_laser_ray - Float64
Pub:
    filtered_pos - Float64
'''

class KF1D(Node):

    def __init__(self):
        super().__init__('kalman_fileter_1D')

        self.declare_parameter('initial_mu', 5.0)
        self.declare_parameter('initial_sig', 1000.0)
        self.declare_parameter('motion_sig', 4.0)
        self.declare_parameter('measurement_sig', 0.05)

        self.declare_parameter('laser_scan_topic', 'scan')
        self.declare_parameter('verbose', "True")

        # self._obs_dis = self.get_parameter('obstacle_front_x_axis').value

        # # laser scan noise parameter, specifies the noise in the laser scan readings
        self._initial_mu = self.get_parameter('initial_mu').value
        self._initial_sig = self.get_parameter('initial_sig').value

        # initialize motion sigma (the standard deviation of the motions normal distribution)
        self._motion_sig = self.get_parameter('motion_sig').value
        self._measurement_sig = self.get_parameter('measurement_sig').value

        self._verbose = eval(self.get_parameter('verbose').value)

        self._laser_sub = self.create_subscription(
            Float64, 'position_from_laser_ray', self.laser_sub_callback, 5
        )

        self._noisy_odom_sub = self.create_subscription(
            Float64, 'noisy_odom_x', self.odom_sub_callback, 5
        )

        self._filter_pub = self.create_publisher(
            Float64, 'filtered_pos', 1
        )

        # Global variables
        self._last_odom_pos = 0.0
        # keep last measurement from laser ray
        self._last_measurement = 0.0

        ### Add initial state HERE ###
        self._mu = self._initial_mu
        self._sig = self._initial_sig

        # distance since last filter cycle
        self._motion = 0.0

        # Final Filtered Odom Msg
        self._odom_msg = Float64()

    ### Add correct_st`ep function HERE ###
    def correct_step(self, mean1, var1, mean2, var2):
        ''' This function takes in two means and two squared variance terms,
        and returns updated gaussian parameters'''
        # Calculate the new gaussian parameters
        new_mean = (var1 * mean2 + var2 * mean1) / (var1 + var2)
        new_var = 1 / (1 / var1 + 1 / var2)
        return new_mean, new_var

    ### Add predict_step function HERE ###
    def predict_step(self, mean1, var1, mean2, var2):
        ''' This function takes in two means and two squared variance terms,
        and returns updated gaussian parameters'''
        # Calculate the new gaussian parameters
        new_mean = mean2 + mean1
        new_var = var1 + var2
        return new_mean, new_var

    # Callback function to handle new messages received
    def odom_sub_callback(self, data):
        # distance since last filter cycle
        self._motion = data.data - self._last_odom_pos

        ### ADD KALMAN FILTER CYCLE HERE ###
        self._mu, self._sig = self.predict_step(
            self._mu, self._sig, self._motion, self._motion_sig
        )
        self._mu, self._sig = self.correct_step(
            self._mu, self._sig, self._last_measurement, self._measurement_sig
        )
        
        if self._verbose:
            self.get_logger().info(f"predict step: [{self._mu}, {self._sig}]")
            self.get_logger().info(f"correct_step: [{self._mu}, {self._sig}]")

        # keep for next filter cycle
        self._last_odom_pos = data.data

        # Publish filtered position estimate
        self._odom_msg.data = self._mu
        self._filter_pub.publish(self._odom_msg)

    def laser_sub_callback(self, msg):
        # keep position as measured by laser ray
        self._last_measurement = msg.data

def main(args=None):
    rclpy.init(args=args)

    kf_1D_node = KF1D()
    rclpy.spin(kf_1D_node)
    kf_1D_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()