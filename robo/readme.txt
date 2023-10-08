######################################## TF PACKAGE ###################################################

# Run a static transform publisher to connect the ground-truth and robot frames (you can also add as a node to turtlebot3_playbag.launch)
- $ rosrun robo publish_initial_tf.sh odom # dont use this!
- $ rosrun robo publish_initial_tf.sh map # use this!
- $ rosrun robo publish_initial_tf.sh /odometry/filtered # use this!

######################################## ROSBAG ###################################################

# Play a rosbag file:
- $ rosbag play <bag> --rate <rate>

######################################## GAMAPPING PACKAGE ###################################################

# Run Gmapping:
- $ roslaunch robo turtlebot3_slam.launch slam_methods:=gmapping
- $ roslaunch robo turtlebot3_slam.launch slam_methods:=gmapping bag_name:=dataset/fixed_slam_easy
- $ roslaunch robo turtlebot3_slam.launch slam_methods:=gmapping bag_name:=lab/2023-09-18-10-18-29
- $ roslaunch robo turtlebot3_slam.launch slam_methods:=gmapping bag_name:=lab/2023-09-25-12-42-03
    Problem: This last rosbag has a problem. When we run gmapping, the base_footprint has a big tranformation from odom, meaning he thinks he already started rolling. The odom and base_footprint frames should be the same (transform == 0), and th wheel enconders, after moovement would result in a growing tranformation between these two frames. The problem is that this transform is already non 0 at the beggining of the gmapping process.
    Explanation: A possible explanation is that the robot was initiated, then moved to the front, and then "kidnaped" to the original position. Then, after initiating the gmapping, from the original position, the base_footprint will different from the odom frame, because the wheel encoder already sensored movement.

# Generate a map from a bag file (creates .pgm and .yaml files):
- $ rosrun map_server map_saver -f my_map

######################################## LOCALIZATION PACKAGE ###################################################

# Run Localization package:
- $ roslaunch robo turtlebot3_localization.launch
- $ roslaunch robo turtlebot3_localization.launch bag_name:=dataset/fixed_slam_easy map_file:=/home/miguel/catkin_ws/src/robo/maps/dataset/final.yaml
- $ roslaunch robo turtlebot3_localization.launch bag_name:=dataset/fixed_slam_easy map_file:=/home/miguel/catkin_ws/src/robo/maps/dataset/final.yaml odom0_differential:=true

######################################## AMCL PACKAGE ###################################################

# Run AMCL package:
    # see: https://www.mathworks.com/help/nav/ug/monte-carlo-localization-algorithm.html (meter na bibliografia do report)
    

- $ roslaunch robo turtlebot3_navigation.launch
- $ roslaunch robo turtlebot3_navigation.launch bag_name:=dataset/fixed_slam_easy map_file:=/home/miguel/catkin_ws/src/robo/maps/dataset/final.yaml rate:=1
- $ roslaunch robo turtlebot3_navigation.launch bag_name:=lab/2023-09-18-10-18-29 map_file:=/home/miguel/catkin_ws/src/robo/maps/lab/2023-09-18-10-18-29.yaml rate:=1
- $ roslaunch robo turtlebot3_navigation.launch bag_name:=lab/2023-09-25-12-42-03 map_file:=/home/miguel/catkin_ws/src/robo/maps/lab/2023-09-25-12-42-03.yaml rate:=1

# Use global localization to spread the particles uniformly:
    - $ rosservice call /global_localization

######################################## MOVE_BASE ###################################################
See:
- http://wiki.ros.org/amcl
- http://wiki.ros.org/costmap_2d

$ roslaunch turtlebot3_gazebo turtlebot3_world.launch x_pos:=-2.0 y_pos:=0.0
$ roslaunch robo turtlebot3_navigation.launch map_file:=/opt/ros/noetic/share/turtlebot3_navigation/maps/map.yaml play_rosbag:=false amcl_initial_pose_x:=-2.0 amcl_initial_pose_y:=0.0

$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch