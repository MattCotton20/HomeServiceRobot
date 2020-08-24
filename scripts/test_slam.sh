#!/bin/sh

# Define workspace location
path_catkin_ws="/home/workspace/catkin_ws"

export TURTLEBOT_GAZEBO_WORLD_FILE=${path_catkin_ws}/src/worlds/matt.world
export ROBOT_INITIAL_POSE="-x 0 -y 0 -Y 1.5707"

# Run turtlebot_world.launch
xterm  -e  "cd ${path_catkin_ws}; source devel/setup.bash; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

# Run mapping_demo.launch
xterm  -e  "cd ${path_catkin_ws}; source devel/setup.bash; roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5

# Run view_navigation.launch
xterm  -e  "cd ${path_catkin_ws}; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

# Run keyboard_teleop.launch
xterm  -e  "cd ${path_catkin_ws}; source devel/setup.bash; roslaunch turtlebot_teleop keyboard_teleop.launch" 
