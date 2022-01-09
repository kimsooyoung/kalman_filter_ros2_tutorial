import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

# for generating movement noise
import numpy as np

# for Rviz visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

# Set an initial ground truth robot location (zero-index based)
ground_truth_position = 5
corridor_map = [0,1,0,1,0,0,0,1,0,0]
odometry_noise_profile = [0.1, 0.8, 0.1]
light_sensor_accuracy_rate = 0.9
# errors in last 10 readings
error_counter = 0
flag_3_errors = False
flag_5_errors = False
# counter for accurate odometry and accurate light sensor readings
flawless_cycle = 0

# Temp global, for Rviz Markers
ground_truth_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

# Callback function to handle new messages received
def movement_cmd_callback(data):
    global ground_truth_position, error_counter, flag_3_errors, flag_5_errors, flawless_cycle
    print("Odometry data (noisy): %d" % data.data)

    # Movement noise: real robot movement can be one less, equal, or one more than command
    movement_noise = np.random.choice(
        a=[-1,0,1], 
        p=odometry_noise_profile
    )

    ground_truth_movement = data.data + movement_noise

    print("Odometry data (ground truth): %d" % ground_truth_movement)
    # the corridor is circular: at the end of the corridor wrap around to the beginning
    ground_truth_position = ((ground_truth_position + ground_truth_movement) % 10 )
    if (int(movement_noise) is not 0):
        error_counter += 1
        print('\33[33m' + 'Notice the (intentional) inaccurate odometry reading (error occurs randomly)' + '\x1b[0m')
        flawless_odometry = False
        flawless_cycle = 0
    else:
        flawless_odometry = True

    # initialise data variables to sent
    motion_data = Int32()
    light_sensor_data = Int32()

    # Sensor noise: % of time sensor reading is wrong, % of time sensor reading is correct
    sensor_reading_noisy = np.random.choice(
        a=[
            abs((corridor_map[ground_truth_position])-1),
            corridor_map[ground_truth_position]
        ], 
        p=[
            1-light_sensor_accuracy_rate, light_sensor_accuracy_rate
        ])
    print("\n")
    print("Light sensor data (noisy): %d" % sensor_reading_noisy)
    print("Light sensor data (ground truth): %d" % corridor_map[ground_truth_position])
    if (int(sensor_reading_noisy) is not int (corridor_map[ground_truth_position])):
        error_counter += 1
        print('\33[33m' + 'Notice the (intentional) inaccurate light sensor reading (error occurs randomly)' + '\x1b[0m')
        flawless_cycle = 0
    else:
        # reset warning counter after 5 cycles with both accurate odometry and accurate light sensor readings
        # flawless light sensor
        if (flawless_odometry):
            flawless_cycle += 1
            if flawless_cycle == 5:
                # reset warnings
                error_counter = 0
                flawless_cycle = 0
                flag_3_errors = False
                flag_5_errors = False
                print('\33[32m' + 'Great! With a few correct odometry and light sensor readings the Bayes Filter has a good chance of success.' '\x1b[0m')
   
    if (error_counter == 3 or error_counter == 4):
        if not flag_3_errors:
            flag_3_errors = True
            print('\33[33m' + 'Warning: '+ str(error_counter) + ' erroneous readings in latest readings' '\x1b[0m')
            print('\33[33m' + 'A Bayes Filter requires a few correct readings in a row to recover' '\x1b[0m')
            print('\33[33m' + 'Otherwise localization will be difficult' '\x1b[0m')
    elif (error_counter == 5 or error_counter == 6):
        if not flag_5_errors:      
            flag_5_errors = True
            print('\33[31m' + 'Warning: '+ str(error_counter) + ' erroneous readings in latest readings' '\x1b[0m')
            print('\33[31m' + 'A Bayes Filter requires a few correct readings in a row to recover' '\x1b[0m')
            print('\33[31m' + 'Otherwise localization will be very difficult' '\x1b[0m')
    
    # assign noisy movement and noisy measurement value to be send
    motion_data.data = data.data
    light_sensor_data.data = sensor_reading_noisy

    pub_motion.publish(motion_data)
    rospy.sleep(0.8)
    pub_light_sensor.publish(light_sensor_data)

    print("\n")
    print("Ground truth position of the robot (grid cell):")
    print("[%d] \n" % ground_truth_position)
    print("-----------------------------------------------------")

    ### Visualize ground truth position of the robot in Rviz ###
    robot_marker = Marker(
            type=Marker.CYLINDER,
            id=99,
            lifetime=rospy.Duration(0),
            pose=Pose(Point(-4.5+ground_truth_position, -0.5, 0.25), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.3, 0.3, 0.5),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 0.0, 1.0, 1.0))
    ground_truth_publisher.publish(robot_marker)

# Initialize a ROS node named "simulated_robot_node"
rospy.init_node('simulated_robot_node')
# Subscribe to the ROS topic called "movement_cmd"
rospy.Subscriber("movement_cmd", Int32, movement_cmd_callback)
# Publishes to data topic
pub_motion = rospy.Publisher('movement_data', Int32, queue_size=10)
pub_light_sensor = rospy.Publisher('sensor_data', Int32, queue_size=10)

if not rospy.has_param("/odometry_noise_profile"):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (("/odometry_noise_profile"), "[.1, .8, .1]"))
    odometry_noise_profile = rospy.get_param("/odometry_noise_profile", [.1, .8, .1])

# Light sensor accuracy rate, meaning the frequency of correct readings (value between 0.0 and 1.0)
if not rospy.has_param("/light_sensor_accuracy_rate"):
    rospy.logwarn('Parameter [%s] not found, using default: %s' % (("/light_sensor_accuracy_rate"), "0.9"))
    kernel = rospy.get_param("/light_sensor_accuracy_rate", 0.9)

def main():

    # Create a new ROS rate limiting timer
    rate = rospy.Rate(5)

    # Print start position
    print("-----------------------------------------------------\n")
    print("Started simulated robot node")
    print("Ground truth start position of the robot (grid cell):")
    print("[%d] \n" % ground_truth_position)
    print("-----------------------------------------------------\n")

    # Execute indefinitely until ROS tells the node to shut down.
    while not rospy.is_shutdown():

        # Sleep appropriately so as to limit the execution rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass