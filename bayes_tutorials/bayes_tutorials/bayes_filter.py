
# Original Code from  2020 @Roberto Zegers
# license License BSD-3-Clause (Attached in pkg folder)
#
# ROS 2 Renewal by Copyright 2021 @RoadBalance KimSooYoung
# Contact Email : tge1375@hanyang.ac.kr 

import rospy
from std_msgs.msg import Int32
from std_srvs.srv import Empty, EmptyResponse

# For Rviz visualization
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

### Add initial belief HERE ### 
belief = [0.1] * 10


### Add corridor map HERE ### 
corridor_map = [0, 1, 0, 1, 0, 0, 0, 1, 0, 0]

# change these values modifying ROS parameters
kernel = [.1, .8, .1]
light_sensor_accuracy_rate = 0.9

bayes_filter_iteration = 0

# Global, for Rviz Markers
marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)

### likelihood function ###
def likelihood(world_model, z, z_prob):
    """ Calculates likelihood that a measurement matches a positions in the map """
    # create output vector
    likelihood = [1] * (len(world_model))
    for index, grid_value in enumerate(world_model):
        if grid_value == z:
            likelihood[index] *= z_prob
        else:
            likelihood[index] *= (1 - z_prob)
    return likelihood

### Add normalize function HERE ###


### Add correct_step function HERE ###


### Add predict_step function HERE ###


def print_belief():
    """ Prints current belief to console """
    print("\nProbabilities of the robot presence at every grid cell:")
    for idx, prob_value in enumerate(belief):
      print("Grid cell %d probability: %f" % (idx, prob_value))
    print("\n")

    m_values = max(belief)
    max_probability_grid_cells = [i for i, j in enumerate(belief) if j == m_values]
    print("Current position best estimate %06.4f %%, corresponding to grid cell(s):" % (m_values*100) )
    print(max_probability_grid_cells)
    print("\n")


# Callback function to handle new movement data received
def movement_callback(movement_data):
    global belief
    global bayes_filter_iteration
    bayes_filter_iteration += 1
    print("-----------------------------------------------------\n")
    print("Bayes filter iteration: %d" % bayes_filter_iteration)
    print("Movement data received: '%d'" % (movement_data.data))

    ### ADD BAYES FILTER PREDICT STEP HERE ##
    

    ### Visualize in Rviz ###
    #create_rviz_markers(marker_publisher, belief)

# Callback function to handle new sensor data received
def sensor_callback(sensor_data):
    global belief
    print("Light sensor data received: '%d'" % (sensor_data.data))

    ### ADD BAYES FILTER CORRECT STEP HERE ###
    

    # Print current belief to console
    print_belief()

    ### Visualize in Rviz ###
    create_rviz_markers(marker_publisher, belief)

##### RVIZ MARKERS ########
def create_rviz_markers(marker_pub, dist):
  for idx, prob_value in enumerate(dist):
    # print("Grid cell %d probability: %f" % (idx, prob_value))
    text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=idx,
                lifetime=rospy.Duration(0),
                pose=Pose(Point(-4.5+idx, 0.0, 0.15+prob_value*10), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.01, 0.01, 0.4),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=str(prob_value)[:5])
    marker_pub.publish(text_marker)
    
    rospy.sleep(0.02)
    column_marker = Marker(
                type=Marker.CUBE,
                id=10+idx,
                lifetime=rospy.Duration(0),
                pose=Pose(Point(-4.5+idx, 0.0, (prob_value*10)/2), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.2, 0.01, prob_value*10),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
    marker_pub.publish(column_marker)
    rospy.sleep(0.02)

def motion_srv_callback(_):
  global belief
  # Call to Bayes Filter predict step (offset (distance) z = 1)
  distance = 1
  belief = predict_step(belief, distance, kernel)
  # Visualize in Rviz
  create_rviz_markers(marker_publisher, belief)
  response = EmptyResponse()
  return response

def measurement_srv_callback(_):
  global belief
  # Call to Bayes Filter correct step (sensor reading z = 1)
  likelihood_estimation = likelihood(corridor_map, z=1, z_prob=light_sensor_accuracy_rate)
  belief = correct_step(likelihood_estimation, belief)
  # Visualize in Rviz
  create_rviz_markers(marker_publisher, belief)
  response = EmptyResponse()
  return response

def main():
    # Initialize a ROS node
    rospy.init_node('bayes_filter')
    
    global kernel, light_sensor_accuracy_rate
    if not rospy.has_param("/odometry_noise_profile"):
        rospy.logwarn('Parameter [%s] not found, using default: %s' % (("/odometry_noise_profile"), "[.1, .8, .1]"))
        kernel = rospy.get_param("/odometry_noise_profile", [.1, .8, .1])

    # Light sensor accuracy rate, meaning the frequency of correct readings (value between 0.0 and 1.0)
    if not rospy.has_param("/light_sensor_accuracy_rate"):
        rospy.logwarn('Parameter [%s] not found, using default: %s' % (("/light_sensor_accuracy_rate"), "0.9"))
        kernel = rospy.get_param("/light_sensor_accuracy_rate", 0.9)

    # Subscribe to the ROS topic called "/movement_data"
    rospy.Subscriber("movement_data", Int32, movement_callback)
    # Subscribe to the ROS topic called "/sensor_data"
    rospy.Subscriber("sensor_data", Int32, sensor_callback)
    # motion_update service server
    motion_update = rospy.Service('/request_motion_update', Empty, motion_srv_callback)
    # measurement_update service server
    measurement_update = rospy.Service('/request_measurement_update', Empty, measurement_srv_callback)
    # Create a new ROS rate limiting timer
    rate = rospy.Rate(5)

    # Wait for at least 1 subscriber before publishing anything
    while marker_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
      #rospy.logwarn_once("Waiting for a subscriber to the topic...")
      print(marker_publisher.get_num_connections())
      rospy.logwarn("Waiting for a subscriber for: /visualization_marker")
      rospy.sleep(5)

    # Print node start information
    print("-----------------------------------------------------\n")
    print("Started Bayes Filter node")
    print("Waiting for sensor data...")
    print("-----------------------------------------------------\n")
    
    ### Visualize initial belief in Rviz ###
    create_rviz_markers(marker_publisher, belief)
    
    # Execute indefinitely until ROS tells the node to shut down.
    while not rospy.is_shutdown():

        # Sleep appropriately so as to limit the execution rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass