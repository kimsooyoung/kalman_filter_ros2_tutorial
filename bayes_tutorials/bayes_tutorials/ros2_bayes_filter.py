import time
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from std_srvs.srv import Empty

from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3

class BayesFilter(Node):
    def __init__(self):
        super().__init__("rviz_marker_publish_node")

        self._movement_sub = self.create_subscription(
            Int32, "movement_data", self.movement_callback, 10
        )

        self._sensor_sub = self.create_subscription(
            Int32, "sensor_data", self.sensor_callback, 10
        )

        self._motion_update = self.create_service(
            Empty, "/request_motion_update", self.motion_srv_callback
        )

        self._measurement_update = self.create_service(
            Empty, "/request_measurement_update", self.measurement_srv_callback
        )

        self._marker_publisher = self.create_publisher(
            Marker, "visualization_marker", 10
        )
        self._timer = self.create_timer(0.02, self.visualize_callback)

        ### Add initial belief HERE ###
        self._belief = [0.1] * 10

        ### Add corridor map HERE ### 
        self._corridor_map = [0, 1, 0, 1, 0, 0, 0, 1, 0, 0]

        self.declare_parameter('kernel', [.05, .9, .05]).value
        self.declare_parameter('light_sensor_accuracy_rate', 0.05).value

        self._kernel = self.get_parameter('kernel').value
        self._light_sensor_accuracy_rate = self.get_parameter('light_sensor_accuracy_rate').value

        self._bayes_filter_iteration = 0

    ### likelihood function ###
    def likelihood(self, z):

        likelihood = [1] * len(self._corridor_map)

        for i, val in enumerate(likelihood):
            if z == self._corridor_map[i]:
                likelihood[i] *= self._light_sensor_accuracy_rate
            else:
                likelihood[i] *= (1 - self._light_sensor_accuracy_rate)

        return likelihood
        
    ### Add normalize function HERE ###
    def normalize(self, input_list):
        
        list_sum = sum(input_list)
        normalized_list = [(i / list_sum) for i in input_list]

        return normalized_list

    ### Add correct_step function HERE ###
    def correct_step(self, likelihood_estimation):
        
        new_belief = []

        for likelihood, belief_val in zip(likelihood_estimation, self._belief):
            new_belief.append(likelihood * belief_val)
        
        self._belief = self.normalize(new_belief)

    ### Add predict_step function HERE ###
    def predict_step(self, offset):
        """Applies a convolution by sliding kernel over the belief"""
        N = len(self._belief)
        kN = len(self._kernel)
        width = int((kN - 1) / 2)
        output = [0] * N
        for i in range(N):
            for k in range (kN):
                index = (i + (width-k) - offset) % N
                output[i] += self._belief[index] * self._kernel[k]
        return output

    def print_belief(self):
        """ Prints current belief to console """
        print("\nProbabilities of the robot presence at every grid cell:")
        for idx, prob_value in enumerate(self._belief):
            print("Grid cell %d probability: %f" % (idx, prob_value))
        print("\n")

        m_values = max(self._belief)
        max_probability_grid_cells = [i for i, j in enumerate(self._belief) if j == m_values]
        print("Current position best estimate %06.4f %%, corresponding to grid cell(s):" % (m_values*100) )
        print(max_probability_grid_cells)
        print("\n")

    # Callback function to handle new movement data received
    def movement_callback(self, movement_data):
        self._bayes_filter_iteration += 1
        print("-----------------------------------------------------\n")
        print("Bayes filter iteration: %d" % self._bayes_filter_iteration)
        print("Movement data received: '%d'" % (movement_data.data))

        ### ADD BAYES FILTER PREDICT STEP HERE ##
        self._belief = self.predict_step(movement_data.data)

        ### Visualize in Rviz ###
        #create_rviz_markers(marker_publisher, belief)
        self.visualize_callback()

    # Callback function to handle new sensor data received
    def sensor_callback(self, sensor_data):
        # global belief
        print("Light sensor data received: '%d'" % (sensor_data.data))

        ### ADD BAYES FILTER CORRECT STEP HERE ###
        likelihood_estimation = self.likelihood(z=sensor_data.data)
        self.correct_step(likelihood_estimation)

        # Print current belief to console
        self.print_belief()

        ### Visualize in Rviz ###
        # create_rviz_markers(marker_publisher, belief)
        self.visualize_callback()

    def motion_srv_callback(self, request, response):
        # global belief
        # Call to Bayes Filter predict step (offset (distance) z = 1)
        distance = 1
        self._belief = self.predict_step(distance)
        # Visualize in Rviz
        # create_rviz_markers(marker_publisher, belief)
        self.visualize_callback()
        # response = EmptyResponse()
        return response

    def measurement_srv_callback(self, request, response):
        # global belief
        # Call to Bayes Filter correct step (sensor reading z = 1)
        likelihood_estimation = self.likelihood(z=1)
        self.correct_step(likelihood_estimation)
        # Visualize in Rviz
        # create_rviz_markers(marker_publisher, belief)
        self.visualize_callback()
        # response = EmptyResponse()
        return response

    def get_text_args(self, idx, prob_value):

        pose = Pose()
        pose.position.x = -4.5 + idx
        pose.position.y = 0.0
        pose.position.z = 0.15 + prob_value * 10
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        scale = Vector3()
        scale.x = 0.01
        scale.y = 0.01
        scale.z = 0.4

        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
        color.a = 0.8

        return pose, scale, color

    def get_column_args(self, idx, prob_value):

        pose = Pose()
        pose.position.x = -4.5 + idx
        pose.position.y = 0.0
        pose.position.z = (prob_value * 10) / 2
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        scale = Vector3()
        scale.x = 0.2
        scale.y = 0.01
        scale.z = prob_value * 10

        color = ColorRGBA()
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
        color.a = 0.8

        return pose, scale, color

    def visualize_callback(self):

        for idx, prob_value in enumerate(self._belief):

            text_pose, text_scale, text_color = self.get_text_args(idx, prob_value)

            text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=idx,
                lifetime=Duration(seconds=0.0).to_msg(),
                pose=text_pose,
                scale=text_scale,
                header=Header(frame_id="map"),
                color=text_color,
                text=str(prob_value)[:5],
            )
            
            self._marker_publisher.publish(text_marker)
            time.sleep(0.01)

            column_pose, column_scale, column_color = self.get_column_args(
                idx, prob_value
            )
            column_marker = Marker(
                type=Marker.CUBE,
                id=10 + idx,
                lifetime=Duration(seconds=0.0).to_msg(),
                pose=column_pose,
                scale=column_scale,
                header=Header(frame_id="map"),
                color=column_color,
            )
            self._marker_publisher.publish(column_marker)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)

    bayes_filter_node = BayesFilter()

    # Print node start information
    print("-----------------------------------------------------\n")
    print("Started Bayes Filter node")
    print("Waiting for sensor data...")
    print("-----------------------------------------------------\n")

    rclpy.spin(bayes_filter_node)

    bayes_filter_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
