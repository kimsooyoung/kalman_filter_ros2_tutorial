
ros2 run bayes_tutorials rviz_pub_node
ros2 run bayes_tutorials bayes_filter_node
ros2 service call /request_motion_update std_srvs/srv/Empty
ros2 service call /request_measurement_update std_srvs/srv/Empty

service call with rqt

ros2 run bayes_tutorials simulated_robot
ros2 topic pub -1 /movement_cmd std_msgs/Int32 "data: 1"

# Moving the robot several times in sequence

rviz
bayes_filter
simulated_robot

ros2 launch bayes_tutorials bayes_filter_rviz.launch.py
ros2 run bayes_tutorials bayes_filter_node
ros2 run bayes_tutorials simulated_robot

# bayes_tutorials

print가 하도 많아서 따로따로 실행한다.

```
cbp bayes_tutorials && rosfoxy

ros2 launch bayes_tutorials bayes_filter_rviz.launch.py
ros2 launch bayes_tutorials bayes_filter.launch.py
ros2 launch bayes_tutorials simulated_robot.launch.py

ros2 topic pub -1 /movement_cmd std_msgs/Int32 "data: 1"
```
