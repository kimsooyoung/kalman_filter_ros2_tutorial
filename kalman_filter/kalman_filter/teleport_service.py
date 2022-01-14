#! /usr/bin/env python
'''
ROS service server that teleports a Gazebo model to a new pose
Waits for request and sets a new pose and responds to the client
Author: Roberto Zegers R.
Usage: rosrun [package_name] gz_teleport_model.py
'''

import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetWorldProperties, GetWorldPropertiesRequest
from std_srvs.srv import Trigger, TriggerResponse

rospy.init_node('teleport_gazebo_model_srv', log_level=rospy.INFO, anonymous=False)

rospy.wait_for_service('gazebo/set_model_state', timeout=5)
set_model_state = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

# get name of Gazebo model and specify a default name in the case that the parameter could not be retrieved
# it is important that the right model name is provided as this script will use it to listen to that model's pose
if rospy.has_param("/model_name"):
    model_name = rospy.get_param("/model_name")
else:
    model_name = "robot_base"
    rospy.logwarn("No 'model_name' parameter found in parameter server, using default value '%s'", model_name)

# get teleport target pose from parameter server
if rospy.has_param("/teleport_target_pose"):
    target_pose = rospy.get_param("/teleport_target_pose")
else:
    # list containing x,y,z pose and X,Y,Z,W quaternion: (x,y,z,X,Y,Z,W)
    # 0,0,0,1 = Identity quaternion, no rotation
    target_pose = [0, 0, 0, 0, 0, 0, 1]
    rospy.logwarn("No '/teleport_target_pose' parameter found in parameter server, using default value '%s'", target_pose)

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
rospy.loginfo("Found model '%s' , starting teleport service server...", model_name)  

# Define new pose (teleportation target pose)
state_msg = ModelState()
state_msg.model_name = model_name
state_msg.reference_frame = 'ground_plane'
state_msg.pose.position.x = target_pose[0]
state_msg.pose.position.y = target_pose[1]
state_msg.pose.position.z = target_pose[2]
state_msg.pose.orientation.x = target_pose[3]
state_msg.pose.orientation.y = target_pose[4]
state_msg.pose.orientation.z = target_pose[5]
state_msg.pose.orientation.w = target_pose[6]

def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    global state_msg
    try:
      set_model_state(state_msg)
      return TriggerResponse(success=True, message="Teleportation success!")
    except rospy.ServiceException, e:
      rospy.logerr("Service call to 'gazebo/set_model_state' failed: {}".format(e))
      return TriggerResponse(success=False, message="Service call to 'gazebo/set_model_state' failed")   

# create a service, specifying its name, type, and callback
my_service = rospy.Service('/reset_model_pose', Trigger, trigger_response)

rospy.spin()