# Home Service Robot

Submission for project #5 within the Udacity Robotics Software Engineer nanodegree.

## Description

A demonstration of a robot that can map, localize, and navigate around its environment to pick up and deliver 'virtual objects'. The robot is built using the ROS framework and simulated using the Gazebo simulator.

This project uses the following ROS packages:
* `turtlebot_gazebo`: Used to deploy a Turtlebot robot into a Gazebo simulation environment, using the world file specified.
* `turtlebot_teleop`: Allows the Turtlebot to be manually controlled using keyboard commands.
* `slam_gmapping`: Uses the Turtlebot's laser range finder sensors, RGB-D camera, and odometry sensors to perform SLAM and build a map of the environment.
* `amcl_demo.launch` (within turtlebot_gazebo): Uses the Turtlebot's laser range finder sensors, RGB-D camera, and odometry sensors to perform localization against the map built using SLAM.
* `turtlebot_rviz_launchers`: Loads a preconfigured RViz workspace to visualise the Turtlebot mapping and/or localizing within its environment.
* `ROS Navigation Stack`: Performs path planning and navigation to navigate the Turtlebot around obstacles to reach a goal position.

I have also created two ROS packages for this project:
* `pick_objects`: Sends a navigation goal to the Turtlebot to navigate to a pre-defined 'pick up zone', wait 5 seconds to imitate an object being picked up, then drive to a 'drop off zone' and wait a further 5 seconds to simulate dropping the object off. This node also publishes a message to an `/object_status` topic when each event occurs.
* `add_markers`: Listens for `/object_status` messages and in response, publishes a message on the `/visualization_marker` topic to tell RViz to display/hide a 'virtual' green sphere imitating the object.

## Getting Started
#### Clone repo into catkin workspace:
```
$ git clone git@github.com:MattCotton20/HomeServiceRobot ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

#### Manually test SLAM:
```
$ cd ~/catkin_ws/src/scripts
$ ./test_amcl.sh
```
Use the `keyboard_teleop` terminal to manually control the Turtlebot. The mapping process can be seen in the RViz window.

#### Navigate automatically using AMCL:
```
$ cd ~/catkin_ws/src/scripts
$ ./test_navigation.sh
```
Within RViz, use the 2D Nav Goal button to send position commands to the Turtlebot. The AMCL particle swarm can be seen as the robot localizes itself.

#### Pick up and drop off a virtual object:
```
$ cd ~/catkin_ws/src/scripts
$ ./home_service.sh
```
The Turtlebot will automatically navigate to a virtual object (seen as a green sphere in RViz), pick it up, drive to another location and drop the object off.

## Acknowledgements
* ROS Wiki: Providing a template code snippet for generating and publishing Markers to RViz.
http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

## Author

* Matt Cotton
