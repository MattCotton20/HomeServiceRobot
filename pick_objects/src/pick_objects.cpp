#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Target positions
  float pickUpPose[3] = {0.1, 3.0, 3.1415};
  float dropOffPose[3] = {-0.5, 0.5, 0.0};

  // ---------- PICK UP ----------

  move_base_msgs::MoveBaseGoal pickUpGoal;

  // set up the frame parameters
  pickUpGoal.target_pose.header.frame_id = "map";
  pickUpGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  pickUpGoal.target_pose.pose.position.x = pickUpPose[0];
  pickUpGoal.target_pose.pose.position.y = pickUpPose[1];
  pickUpGoal.target_pose.pose.orientation.w = pickUpPose[2];

   // Send the goal position and orientation for the robot to reach
  ac.sendGoal(pickUpGoal);
  ROS_INFO("Robot is travelling to the pickup zone");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot picked up the virtual object");
    ros::Duration(5.0).sleep(); 
  }
  else
    ROS_INFO("Robot failed to travel to the pickup zone");


  // ---------- DROP OFF ----------

  move_base_msgs::MoveBaseGoal dropOffGoal;


  // set up the frame parameters
  dropOffGoal.target_pose.header.frame_id = "map";
  dropOffGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  dropOffGoal.target_pose.pose.position.x = dropOffPose[0];
  dropOffGoal.target_pose.pose.position.y = dropOffPose[1];
  dropOffGoal.target_pose.pose.orientation.w = dropOffPose[2];

   // Send the goal position and orientation for the robot to reach
  ac.sendGoal(dropOffGoal);
  ROS_INFO("Robot is travelling to the drop off zone");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot dropped off the virtual object");
    ros::Duration(5.0).sleep(); 
  }
  else
    ROS_INFO("Robot failed to travel to the drop off zone");

  return 0;
}
