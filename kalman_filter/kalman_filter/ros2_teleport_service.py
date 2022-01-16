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

from std_srvs.srv import Trigger
from gazebo_msgs.srv import SetEntityState

import rclpy
from rclpy.node import Node

class RespawnClient(Node):

    def __init__(self):
        super().__init__('robot_respawn_client')

        self._respawn_client = self.create_client(
            SetEntityState, 'set_entity_state'
        )

        while not self._respawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self._set_entity_req = SetEntityState.Request()
        self.get_logger().info('==== Set Entity Service Client Ready ====')

    def set_srv(self, model_name, target_pose):

        self._set_entity_req.state.name = model_name

        self._set_entity_req.state.pose.position.x = target_pose[0]
        self._set_entity_req.state.pose.position.y = target_pose[1]
        self._set_entity_req.state.pose.position.z = target_pose[2]
        self._set_entity_req.state.pose.orientation.x = target_pose[3]
        self._set_entity_req.state.pose.orientation.y = target_pose[4]
        self._set_entity_req.state.pose.orientation.z = target_pose[5]
        self._set_entity_req.state.pose.orientation.w = target_pose[6]

        return

    def send_request(self, model_name, target_pose):
        
        self.set_srv(model_name, target_pose)
        future = self._respawn_client.call_async(self._set_entity_req)
        
        self.get_logger().info('=== Request Sended ===')
        return future

class TeleportServer(Node):

    def __init__(self):
        super().__init__('robot_teleport_server')

        self._teleport_srv = self.create_service(
            Trigger, '/reset_model_pose', self.trigger_callback
        )

        self._respawn_client = RespawnClient()

        self.declare_parameter('model_name', 'neuronbot2')
        self.declare_parameter('target_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

        self._model_name = self.get_parameter('model_name').value
        self._target_pose = self.get_parameter('target_pose').value

    def __del__(self):
        super().__del__()
        self._respawn_client.destroy_node()

    def trigger_callback(self, request, response):

        future = self._respawn_client.send_request(self._model_name, self._target_pose)

        rclpy.spin_until_future_complete(self._respawn_client, future)

        if future.done():
            try:
                spawn_response = future.result()
            except Exception:
                raise RuntimeError(
                    'exception while calling service: %r' % future.exception()
                )
            else:
                self.get_logger().info(f"==== Service Call Done : Result Message : {'Success' if spawn_response.success == True else 'Fail'} ====")
            finally:
                self.get_logger().info('==== Execution Done ====')
        
        response.success = True
        self.get_logger().warn('Respawn Process Done...')

        return response

def main(args=None):
    rclpy.init(args=args)

    teleport_node = TeleportServer()

    rclpy.spin(teleport_node)

    teleport_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()