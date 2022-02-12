ros2 launch kalman_filter launch_auxiliary_nodes.launch.py

get_world_properties => get_model_list
set_model_state => set_entity_state
https://github.com/ros-simulation/gazebo_ros_pkgs/issues/512

이거 위해서 world 파일 제일 위에 다음 문구 추가

```
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/</namespace>
        <argument>model_states:=model_states_demo</argument>
        <argument>link_states:=link_states_demo</argument>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>
```
https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Entity-states


teleport_service.py

/reset_model_pose 라는 서비스 생성

gazebo/get_world_properties => 이름 받아옴
gazebo/set_model_state => 위치 재설정

[0, 0, 0, 0, 0, 0, 1]

kobuki_model
gazebo_msgs/srv/SetEntityState

# Holds an entity's pose and twist
string name                 # Entity's scoped name.
                            # An entity can be a model, link, collision, light, etc.
                            # Be sure to use gazebo scoped naming notation (e.g. [model_name::link_name])
geometry_msgs/Pose pose     # Pose in reference frame.
geometry_msgs/Twist twist   # Twist in reference frame.
string reference_frame      # Pose/twist are expressed relative to the  frame of this entity.
                            # Leaving empty or "world" defaults to inertial world frame.


## odom_utility

get_model_state => get_entity_state (gazebo_msgs/GetEntityState)

꼬부기는 일직선으로 가질 못함 => 뉴런봇으로 변경

## laser_ray_localization

param
* obstacle_front_x_axis :11
* laser_scan_topic : /scan

communication 
* Subscriber : LaserScan
* Publisher : /position_from_laser_ray, Float64

 source /usr/share/gazebo/setup.sh

# Final execution

```
ros2 launch kalman_filter launch_auxiliary_nodes.launch.py
ros2 run kalman_filter kalman_1d
```