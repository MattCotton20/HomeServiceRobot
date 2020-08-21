#!/bin/sh

# Define workspace location
path_catkin_ws="/home/workspace/catkin_ws"

# Run turtlebot_world.launch
xterm  -e  "cd ${path_catkin_ws}; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=${path_catkin_ws}/src/worlds/matt.world" &
sleep 5

# Run mapping_demo.launch
xterm  -e  "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5

# Run view_navigation.launch
xterm  -e  "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# Run keyboard_teleop.launch
xterm  -e  "cd ${path_catkin_ws} && source devel/setup.bash && roslaunch turtlebot_teleop keyboard_teleop.launch" 
