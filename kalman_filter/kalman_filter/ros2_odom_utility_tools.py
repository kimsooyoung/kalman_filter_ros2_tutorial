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

from std_msgs.msg import Float64

from std_srvs.srv import Trigger
from gazebo_msgs.srv import SetEntityState, GetEntityState

import numpy as np


# truth_pub=rospy.Publisher('/ground_truth_x', Float64, queue_size=1)
# noisy_odom_pub = rospy.Publisher('/noisy_odom_x', Float64, queue_size=1)

# get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
# teleport_service = rospy.ServiceProxy('/reset_model_pose', Trigger)

class EntityStateClient(Node):

    def __init__(self, model_name):
        super().__init__('reset_model_client')

        self._entity_state_client = self.create_client(
            GetEntityState, 'get_entity_state'
        )

        while not self._entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' [gazebo/get_entity_state] service not available, waiting again...')

        self._entity_state_req = GetEntityState.Request()
        self._entity_state_req.name = model_name

        self.get_logger().info('==== Entity State Service Client Ready ====')

    def send_request(self):
        
        future = self._entity_state_client.call_async(self._entity_state_req)
        self.get_logger().info('=== Request Sended ===')
        
        return future

class TeleportClient(Node):

    def __init__(self):
        super().__init__('robot_teleport_client')

        self._teleport_client = self.create_client(
            Trigger, 'reset_model_pose'
        )

        while not self._teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' [reset_model_pose] service not available, waiting again...')

        self._trigger_req = Trigger.Request()
        self.get_logger().info('==== Teleport Service Client Ready ====')

    def send_request(self):

        future = self._teleport_client.call_async(self._trigger_req)
        self.get_logger().info('=== Teleport Request Sended ===')

        return future

class OdomUtilNode(Node):

    def __init__(self):
        super().__init__('odometry_util_node')

        self.declare_parameter('model_name', 'neuronbot2')
        self.declare_parameter('alpha', 0.4)
        self._model_name = self.get_parameter('model_name').value
        self._alpha = self.get_parameter('alpha').value

        self._teleport_client = TeleportClient()
        self._entity_state_client = EntityStateClient(self._model_name)

        self._ground_truth_publisher = self.create_publisher(
            Float64, 'ground_truth_x', 1
        )

        self._noisy_odom_publisher = self.create_publisher(
            Float64, 'noisy_odom_x', 1
        )

        self.gt_x = Float64()
        self.noisy_x = Float64()

        self._timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):

        state_client_future = self._entity_state_client.send_request()
        rclpy.spin_until_future_complete(self._entity_state_client, state_client_future)

        if state_client_future.done():
            try:
                state_response = state_client_future.result()
            except Exception:
                raise RuntimeError(
                    'exception while calling entity state service: %r' % state_client_future.exception()
                )
            else:
                ground_truth_x = state_response.state.pose.position.x
            finally:
                self.get_logger().warn('==== Entity Client Execution Done ====')

        self.get_logger().info(f"Got ground truth pose.position.x: {ground_truth_x}" )
        
        ground_truth_delta = 0
        last_ground_truth_x = 0
        noisy_odom = 0

        if ground_truth_x > 10.0:
            
            teleport_client_future = self._teleport_client.send_request()
            rclpy.spin_until_future_complete(self._teleport_client, teleport_client_future)
            
            if teleport_client_future.done():
                try:
                    state_response = teleport_client_future.result()
                except Exception:
                    raise RuntimeError(
                        'exception while calling teleport service: %r' % teleport_client_future.exception()
                    )
                else:
                    self.get_logger().info(f"==== Service Call Done : Result Message : {'Success' if state_response.success == True else 'Fail'} ====")
                finally:
                    self.get_logger().warn('==== Teleport Execution Done ====')
            
            ground_truth_delta = 0
            last_ground_truth_x = 0
            noisy_odom = 0
        
        ground_truth_delta = ground_truth_x - last_ground_truth_x

        sd_trans = self._alpha * (ground_truth_delta)
        noisy_odom += np.random.normal(ground_truth_delta, sd_trans * sd_trans)
        
        self.gt_x.data = ground_truth_x
        self.noisy_x.data = noisy_odom

        self._ground_truth_publisher.publish(self.gt_x)
        self._noisy_odom_publisher.publish(self.noisy_x)

        last_ground_truth_x = ground_truth_x


def main(args=None):
    rclpy.init(args=args)

    odom_util_node = OdomUtilNode()

    rclpy.spin(odom_util_node)

    odom_util_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()