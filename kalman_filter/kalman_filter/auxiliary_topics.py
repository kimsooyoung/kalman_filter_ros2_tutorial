#! /usr/bin/env python
'''
Node that creates auxiliary topics to be used along with the 1D Kalman Filter Example
- Gets the ground truth robot position from Gazebo Model State and publishes the x component to a specified topic
- Generates noise for the model's pose x component and publishes it to a specified topic
- Calls service to reset Gazebo model position  if the current x-axis pose exceed a certain threshold value
Author: Roberto Zegers R.
Usage: rosrun [package_name] auxiliary_topics.py
'''

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from gazebo_msgs.srv import GetWorldProperties, GetWorldPropertiesRequest
from std_srvs.srv import Trigger, TriggerRequest
import numpy as np

rospy.init_node('odom_from_gazebo_pub', log_level=rospy.INFO, anonymous=False)

# this is the topic name for perfect odometry data
truth_pub=rospy.Publisher('/ground_truth_x', Float64, queue_size=1)
noisy_odom_pub = rospy.Publisher('/noisy_odom_x', Float64, queue_size=1)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

# wait for teleport service to be running
rospy.wait_for_service('/reset_model_pose')
# create connection to service
teleport_service = rospy.ServiceProxy('/reset_model_pose', Trigger)
# Create object of the type TriggerRequest for a Trigger service
teleport_model = TriggerRequest()

# get name of Gazebo model and specify a default name in the case that the parameter could not be retrieved
# it is important that the right model name is provided as this script will use it to listen to that model's pose
if rospy.has_param("/model_name"):
    model_name = rospy.get_param("/model_name")
else:
    model_name = "robot_base"
    rospy.logwarn("No 'model_name' parameter found in parameter server, using default value '%s'", model_name)

# get odometry noise parameter, specifies the expected noise in odometrys
# it is important that the right model name is provided as this script will use it to listen to that model's pose
if rospy.has_param("/alpha"):
    alpha = rospy.get_param("/alpha")
else:
    alpha = 8.0
    rospy.logwarn("No '/alpha' parameter found in parameter server, using default value '%s'", alpha)

# make sure gazebo world contains a model called 'model_name'
rospy.loginfo("Wait for service '/gazebo/get_world_properties'")
rospy.wait_for_service('/gazebo/get_world_properties', timeout=10)
world_loaded = False
while not world_loaded:
    srv_world_infos = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    try:
        req = GetWorldPropertiesRequest()
        res = srv_world_infos(req)
        for item in res.model_names:
            if item == model_name:
                world_loaded = True
                break
    except rospy.ServiceException, e:
        rospy.logerr("Service call to '/gazebo/get_world_properties' failed: {}".format(e))
    rospy.logwarn("Model '%s' not found in Gazebo world, waiting...", model_name)  
    rospy.sleep(5)
rospy.loginfo("Found model '%s' , start to track its pose...", model_name)  

model = GetModelStateRequest()
# name of the robot model in Gazebo, as defined above or in launch file
model.model_name = model_name

r = rospy.Rate(5) # Hz
ground_truth_delta = 0
last_ground_truth_x = 0
noisy_odom = 0

while not rospy.is_shutdown():
    # get ground truth pose
    result = get_model_srv(model)
    ground_truth_x = result.pose.position.x
    rospy.logdebug("Got ground truth pose.position.x: %s \n", ground_truth_x)
    if ground_truth_x >10:
        try:
            # send request through the connection
            result = teleport_service(teleport_model)
            rospy.loginfo("Requested teleport service: %s", result.message)
            # reset to zero
            ground_truth_delta = 0
            last_ground_truth_x = 0
            noisy_odom = 0
        except rospy.ServiceException, e:
            rospy.logerr("Service call to '/reset_model_pose' failed: {}".format(e))
        # jump back to the top of the loop
        continue
    # advertise ground thruth position in x-axis
    truth_pub.publish(ground_truth_x)
    # calculate ground truth distance delta
    ground_truth_delta = ground_truth_x - last_ground_truth_x
    # generate noisy odometry for x-axis
    # standard deviation for noise
    sd_trans = alpha*(ground_truth_delta)
    noisy_odom +=  np.random.normal(ground_truth_delta,sd_trans*sd_trans)
    noisy_odom_pub.publish(noisy_odom)
    # keep track to calculate delta in next loop iteration
    last_ground_truth_x = ground_truth_x

    r.sleep()

##### add service call to reset model position
