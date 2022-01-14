#!/usr/bin/env python
"""
Kalman Filter in 1D

Author: Roberto Zegers R.
Date: August 2020
"""

import rospy
from std_msgs.msg import Float64

# Global variables
last_odom_pos = 0.0
# keep last measurement from laser ray
last_measurement = 0.0

### Add initial state HERE ###


# initialize motion sigma (the standard deviation of the motions normal distribution)
motion_sig = 4.0
measurement_sig = 0.05
# distance since last filter cycle
motion = 0.0

### Add correct_step function HERE ###


### Add predict_step function HERE ###


# Callback function to handle new messages received
def odom_callback(data):
    global mu, sig

    global motion_sig
    global last_odom_pos

    global measurement_sig
    global last_measurement

    global filter_pub

    # distance since last filter cycle
    motion = data.data - last_odom_pos

    ### ADD KALMAN FILTER CYCLE HERE ###


    # keep for next filter cycle
    last_odom_pos = data.data

    # Publish filtered position estimate
    filter_pub.publish(mu)

def laser_ray_callback(msg):
    global last_measurement
    # keep position as measured by laser ray
    last_measurement = msg.data

if __name__ == '__main__':
    try:
        # Initialize a ROS node
        rospy.init_node('kalman_filter')

        # Subscribe to the ROS topic called "/noisy_odom_x"
        rospy.Subscriber("/noisy_odom_x", Float64, odom_callback)

        # Subscribe to the ROS topic called "/position_from_laser_ray"
        rospy.Subscriber("/position_from_laser_ray", Float64, laser_ray_callback)

        filter_pub = rospy.Publisher('/filtered_pos', Float64, queue_size=1)

        # Create a new ROS rate limiting timer
        # rate = rospy.Rate(5)

        # Print node start information
        print("-----------------------------------------------------\n")
        print("Started Kalman Filter node")
        print("-----------------------------------------------------\n")
          
        # Execute indefinitely until ROS tells the node to shut down.
        while not rospy.is_shutdown():

            # Sleep appropriately so as to limit the execution rate
            # rate.sleep()
            rospy.spin()

    except rospy.ROSInterruptException:
        pass