<?xml version="1.0" encoding="UTF-8"?>
<!-- Top-level launch file for the 1D Kalman Filter unit exercise -->

<launch>

  <!-- Spawn brick wall model in Gazebo -->
	<arg name= "pos_x" default = "11.5"/>
	<arg name= "pos_y" default = "-0.5"/>
	<arg name= "pos_z" default = "0.0"/>
	<arg name= "yaw" default = "1.5708"/>
	<arg name= "model_name" default= "brick_wall"/>

	<node name="spawn_wall" pkg="gazebo_ros" type="spawn_model"
	args="-file $(find kalman_filter)/models/brick_box_3x1x3/model.sdf
	-sdf
	-x $(arg pos_x)
	-y $(arg pos_y)
	-z $(arg pos_z)
	-Y $(arg yaw)
	-model $(arg model_name)"
	respawn="false" output="screen"/>
  
  <!-- The teleport service and associated parameters -->
  <include file="$(find kalman_filter)/launch/teleport_service.launch">
    <arg name="target_pose" default="[0, 0, 0, 0, 0, 0, 1]"/>
  </include>

  <!-- Starts Gazebo Odometry Utility Tool -->
  <include file="$(find kalman_filter)/launch/odom_utility_tools.launch" />
  
  <!-- Starts laser ray localization node -->
  <include file="$(find kalman_filter)/launch/laser_ray_localization.launch">
    <arg name="laser_scan_topic" default="/kobuki/laser/scan"/>
    <arg name="obstacle_in_front_distance" default="11"/>
  </include>
  
  <!-- Send command to move robot in a straight line at constant velocity -->
  <node pkg="rostopic" type="rostopic" name="twist_pub" 
        args="pub cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -r 10" />

</launch>