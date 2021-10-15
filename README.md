# navigation-stackP3AT
Pioneer 3-AT
install rosaria in a catkin:
http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA
installing Aria: sudo apt install aria2 libaria-dev
https://github.com/amor-ros-pkg/rosaria

check USB connection (star should be a number (initially 0))
ls -l /dev/ttyUSB*

add permissions
sudo chmod 777 /dev/ttyUSB*

catkin_make && source devel/setup.bash

connecting to robot:
rosrun rosaria RosAria (cable might be unstable) (port is 0 default, if USB was not 0, then: ‘rosrun rosaria RosAria _port:=/dev/ttyUSB1’)
(there is a slight chance, that you can only connect to the robot if you push it, so that the wheels are turning)

keyboard control:
rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/RosAria/cmd_vel

________________________
Lidar LMS200
requires direct serial connection between lidar and laptop 

do as it is written here:
https://github.com/jmscslgroup/sicktoolbox_wrapper
(maybe needed http://wiki.ros.org/sicktoolbox_wrapper/Tutorials/UsingTheSicklms )
rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400 _connect_delay:=30

(if it does not connect then add: ‘_connect_delay:=30’ to the end of the rosrun command)

___
short ros documentation, contains how to connect to … (better if downloaded)
https://github.com/MaoRodriguesJ/ROS-P3DX/blob/master/ros.pdf

(https://github.com/ros-drivers/sicktoolbox_wrapper)

________
GMAPPING

https://github.com/ros-perception/slam_gmapping
returns errors at catkin_make?

instal with apt:
sudo apt install ros-melodic-gmapping
sudo apt install ros-melodic-slam-gmapping
sudo apt install ros-melodic-openslam-gmapping
sudo apt install ros-melodic-map-server

How to create map
Session 1 (simplifications in session 4)
Terminal1: roscore
T2: rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400 _connect_delay:=30
T3: rosrun rosaria RosAria _port:=/dev/ttyUSB1
T4: rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/RosAria/cmd_vel
(maybe needed before T5: rosparam set use_sim_time true)
T5: rosrun tf2_ros static_transform_publisher 0.16 0 0.16 0 0 0 1 base_link laser
T6: rosrun gmapping slam_gmapping scan:=/scan  _odom_frame:=odom
T7: rosbag record /map

navigate around with the robot (teleop), then stop them
now the bag contains the map, to get it as image and yaml:

Session2
T1: roscore
T2: rosrun map_server map_saver (rerun this at the end of the bag)
T3: rosbag play abc.bag
at the end of the bag the command in T2 again

Session 3
amcl localization
run session 1, with T1-T5, then:
T6: rosrun map_server map_server map.yaml
T7: rosrun amcl amcl scan:=/scan map:=/map use_map_topic:=true
then in rviz you can see an approximation of the robot pose on the map (as you move the robot, the pose should change -- althought it is very insensitive, pose changes only after larger movements)

Session4 - Final Navigation stack
http://wiki.ros.org/navigation
http://wiki.ros.org/navigation/Tutorials/RobotSetup

Simplify things by creating roslaunch files:

1 pioneer_nav_configuration.launch:
<launch>
 
   <node pkg="sicktoolbox_wrapper" type="sicklms" name="sicklms" output="screen">
       <param name="port" value="/dev/ttyUSB0" />
       <param name="baud" value="38400" />
       <param name="connect_delay" value="30" />
   </node>
   <node pkg="rosaria" type="RosAria" name="RosAria" output="screen">
       <param name="port" value="/dev/ttyUSB1" />
   </node>
   <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" args="0.16 0 0.16 0 0 0 1 base_link laser" />
 
</launch>

2 move_base.launch:
<launch>
 
   <master auto="start"/>
<!-- Run the map server  -->
   <node name="map_server" pkg="map_server" type="map_server" args="$(find pioneer_nav)/map.yaml"/>
 
<!--- Run AMCL -->
   <include file="$(find amcl)/examples/amcl_omni.launch" />
 
   <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
       <rosparam file="$(find pioneer_nav)/config/costmap_common_params.yaml" command="load" ns="global_costmap" />
       <rosparam file="$(find pioneer_nav)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
       <rosparam file="$(find pioneer_nav)/config/local_costmap_params.yaml" command="load" />
       <rosparam file="$(find pioneer_nav)/config/global_costmap_params.yaml" command="load" />
       <rosparam file="$(find pioneer_nav)/config/base_local_planner_params.yaml" command="load" />
       <remap from="/cmd_vel" to="/RosAria/cmd_vel"/>
   </node>
 
</launch>

Create some config files (can be inside config folder, otherwise rewrite mvoe_base.launch):
1 base_local_planner_params.yaml:
TrajectoryPlannerROS:
 max_vel_x: 0.25
 min_vel_x: 0.1
 max_vel_theta: 1.0
 min_in_place_vel_theta: 0.4
 
 acc_lim_theta: 3.2
 acc_lim_x: 2.5
 acc_lim_y: 2.5
 
 holonomic_robot: true

2 global_costmap_params.yaml:
global_costmap:
 global_frame: map
 robot_base_frame: base_link
 update_frequency: 5.0
 static_map: true

3 costmap_common_params.yaml:
obstacle_range: 4.0
raytrace_range: 5.0
footprint: [[0.25, 0.25], [0.25, -0.25], [-0.25, -0.25], [-0.25, 0.25]]
#robot_radius: ir_of_robot
inflation_radius: 0.5
 
observation_sources: laser_scan_sensor point_cloud_sensor
 
laser_scan_sensor: {sensor_frame: laser, data_type: LaserScan, topic: /scan, marking: true, clearing: true}
 
# point_cloud_sensor: {sensor_frame: frame_name, data_type: PointCloud, topic: topic_name, marking: true, clearing: true}

4 local_costmap_params.yaml:
local_costmap:
 global_frame: odom
 robot_base_frame: base_link
 update_frequency: 5.0
 publish_frequency: 2.0
 static_map: false
 rolling_window: true
 width: 9.0
 height: 9.0
 resolution: 0.05


run!:
T1: roslaunch pioneer_nav pioneer_nav_configuration.launch
T2: roslaunch pioneer_nav move_base.launch
T3: rviz
in Rviz add map, footprint, plan

Set goal:
RVIZ: to pbar - click on 2D Nav Goal, and click on map where you want to go (while clicking you also decide the orientation, hold the click, and set the vector direction)
publish on topic: !Be careful when setting coordinates, they should be on the map!
#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
 
def talker():
   pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
   rospy.init_node('talker', anonymous=True)
   rate = rospy.Rate(5) # 10hz
   while not rospy.is_shutdown():
       goal = PoseStamped()
       goal.header.seq = 1
       goal.header.stamp = rospy.Time.now()
       goal.header.frame_id = "map"
 
       goal.pose.position.x = 0.5
       goal.pose.position.y = -0.5
       goal.pose.position.z = 0.0
 
       goal.pose.orientation.x = 0.0
       goal.pose.orientation.y = 0.0
       goal.pose.orientation.z = 0.0
       goal.pose.orientation.w = 1.0
       pub.publish(goal)
       rate.sleep()
 
if __name__ == '__main__':
   try:
       talker()
   except rospy.ROSInterruptException:
       pass


ROS REMOTE

Host device (both IP is the ip of host device, this is where roscore runs. These two has to be set in each terminal):
export ROS_MASTER_URI=http://192.168.1.1:11311
export ROS_IP=192.168.1.1   
rosccommand
Slave device (first IP is the ip of host device, second IP is the IP of the slave device, this is where roscore runs. These two has to be set in each terminal):
export ROS_MASTER_URI=http://192.168.1.1:11311
export ROS_IP=192.168.1.2   
rosccommand
