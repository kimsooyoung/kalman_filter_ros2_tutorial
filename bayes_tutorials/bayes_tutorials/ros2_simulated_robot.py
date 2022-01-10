import time
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

# for generating movement noise
import numpy as np

# for Rviz visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class SimulatedRobot(Node):

    def __init__(self):
        super().__init__("simulated_robot_node")

        self._movement_sub = self.create_subscription(
            Int32, "movement_cmd", self.movement_cmd_callback, 10
        )

        self._marker_publisher = self.create_publisher(
            Marker, "visualization_marker", 10
        )

        self._movement_data_pub = self.create_publisher(
            Int32, "movement_data", 10
        )

        self._sensor_data_pub = self.create_publisher(
            Int32, "sensor_data", 10
        )

        # Set an initial ground truth robot location (zero-index based)
        self._ground_truth_position = 5
        self._corridor_map = [0,1,0,1,0,0,0,1,0,0]
        
        self.declare_parameter('kernel', [.05, .9, .05])
        self.declare_parameter('light_sensor_accuracy_rate', 0.95)

        self._odometry_noise_profile = self.get_parameter('kernel').value
        self._light_sensor_accuracy_rate = self.get_parameter('light_sensor_accuracy_rate').value

        # errors in last 10 readings
        self._error_counter = 0
        self._flag_3_errors = False
        self._flag_5_errors = False
        
        # counter for accurate odometry and accurate light sensor readings
        self._flawless_cycle = 0
        self._flawless_odometry = False

    @property
    def ground_truth_position(self):
        return self._ground_truth_position

    def movement_cmd_callback(self, data):

        print("Odometry data (noisy): %d" % data.data)

        # Movement noise: real robot movement can be one less, equal, or one more than command
        movement_noise = np.random.choice(
            a=[-1,0,1], 
            p=self._odometry_noise_profile
        )

        ground_truth_movement = data.data + movement_noise

        print("Odometry data (ground truth): %d" % ground_truth_movement)
        
        # the corridor is circular: at the end of the corridor wrap around to the beginning
        self._ground_truth_position = ((self._ground_truth_position + ground_truth_movement) % 10 )
        
        if (int(movement_noise) != 0):
            self._error_counter += 1
            print('\33[33m' + 'Notice the (intentional) inaccurate odometry reading (error occurs randomly)' + '\x1b[0m')
            self._flawless_odometry = False
            self._flawless_cycle = 0
        else:
            self._flawless_odometry = True

        # initialise data variables to sent
        motion_data = Int32()
        light_sensor_data = Int32()

        # Sensor noise: % of time sensor reading is wrong, % of time sensor reading is correct
        sensor_reading_noisy = np.random.choice(
            a=[
                abs((self._corridor_map[self._ground_truth_position])-1),
                self._corridor_map[self._ground_truth_position]
            ], 
            p=[
                1-self._light_sensor_accuracy_rate, self._light_sensor_accuracy_rate
            ])
        print("\n")
        print("Light sensor data (noisy): %d" % sensor_reading_noisy)
        print("Light sensor data (ground truth): %d" % self._corridor_map[self._ground_truth_position])
        
        if (int(sensor_reading_noisy) is not int (self._corridor_map[self._ground_truth_position])):
            self._error_counter += 1
            print('\33[33m' + 'Notice the (intentional) inaccurate light sensor reading (error occurs randomly)' + '\x1b[0m')
            self._flawless_cycle = 0
        else:
            # reset warning counter after 5 cycles with both accurate odometry and accurate light sensor readings
            # flawless light sensor
            if (self._flawless_odometry):
                self._flawless_cycle += 1
                if self._flawless_cycle == 5:
                    # reset warnings
                    self._error_counter = 0
                    self._flawless_cycle = 0
                    self._flag_3_errors = False
                    self._flag_5_errors = False
                    print('\33[32m' + 'Great! With a few correct odometry and light sensor readings the Bayes Filter has a good chance of success.' '\x1b[0m')
    
        if (self._error_counter == 3 or self._error_counter == 4):
            if not self._flag_3_errors:
                self._flag_3_errors = True
                print('\33[33m' + 'Warning: '+ str(self._error_counter) + ' erroneous readings in latest readings' '\x1b[0m')
                print('\33[33m' + 'A Bayes Filter requires a few correct readings in a row to recover' '\x1b[0m')
                print('\33[33m' + 'Otherwise localization will be difficult' '\x1b[0m')
        elif (self._error_counter == 5 or self._error_counter == 6):
            if not self._flag_5_errors:      
                self._flag_5_errors = True
                print('\33[31m' + 'Warning: '+ str(self._error_counter) + ' erroneous readings in latest readings' '\x1b[0m')
                print('\33[31m' + 'A Bayes Filter requires a few correct readings in a row to recover' '\x1b[0m')
                print('\33[31m' + 'Otherwise localization will be very difficult' '\x1b[0m')
        
        # assign noisy movement and noisy measurement value to be send
        motion_data.data = data.data
        light_sensor_data.data = int(sensor_reading_noisy)

        self._movement_data_pub.publish(motion_data)
        time.sleep(0.8)
        self._sensor_data_pub.publish(light_sensor_data)

        print("\n")
        print("Ground truth position of the robot (grid cell):")
        print("[%d] \n" % self._ground_truth_position)
        print("-----------------------------------------------------")

        self.draw_marker()

    def draw_marker(self):
        ### Visualize ground truth position of the robot in Rviz ###
        pose, scale, color = self.get_marker_elem()
        robot_marker = Marker(
                type=Marker.CYLINDER,
                id=99,
                lifetime=Duration(seconds=0.0).to_msg(),
                pose=pose,
                scale=scale,
                header=Header(frame_id='map'),
                color=color
            )

        self._marker_publisher.publish(robot_marker)

    def get_marker_elem(self):

        pose = Pose()
        pose.position.x = -4.5 + self._ground_truth_position
        pose.position.y = -0.5
        pose.position.z = 0.25
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        scale = Vector3()
        scale.x = 0.3
        scale.y = 0.3
        scale.z = 0.5

        color = ColorRGBA()
        color.r = 0.0
        color.g = 0.0
        color.b = 1.0
        color.a = 1.0

        return pose, scale, color

def main(args=None):
    rclpy.init(args=args)

    simulated_robot_node = SimulatedRobot()

    # Print start position
    print("-----------------------------------------------------\n")
    print("Started simulated robot node")
    print("Ground truth start position of the robot (grid cell):")
    print("[%d] \n" % simulated_robot_node.ground_truth_position)
    print("-----------------------------------------------------\n")

    simulated_robot_node.draw_marker()

    rclpy.spin(simulated_robot_node)

    simulated_robot_node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
