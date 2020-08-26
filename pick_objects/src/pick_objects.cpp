#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Char.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Create a handle to the pick_objects node
  ros::NodeHandle n;

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Create a publisher that can tell the add_markers node the status of the virtual object
  ros::Publisher object_status_pub = n.advertise<std_msgs::Char>("/object_status", 1);

  // Wait for the move_base action server to start
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // object_status message variable
  std_msgs::Char status;

  // Publish initial status (at pick up zone)
  status.data = 'p';
  object_status_pub.publish(status);

  // Target positions
  float pickUpPose[3] = {7.0, 0.5, 1.5707};
  float dropOffPose[3] = {5.0, -3.5, 1.5707};

  // Create goal object
  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Set target location as pick up zone
  goal.target_pose.pose.position.x = pickUpPose[0];
  goal.target_pose.pose.position.y = pickUpPose[1];
  goal.target_pose.pose.orientation.w = pickUpPose[2];

  // Send pick up goal
  ac.sendGoal(goal);
  ROS_INFO("Travelling to the pickup zone");

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached the pick up goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Picked up the virtual object");

    // Publish updated status (virtual object is being held)
    status.data = 'h';
    object_status_pub.publish(status);

    // Pause 5 seconds to imitate picking up object
    ros::Duration(5.0).sleep();  

    // Update target location to be drop off zone
    goal.target_pose.pose.position.x = dropOffPose[0];
    goal.target_pose.pose.position.y = dropOffPose[1];
    goal.target_pose.pose.orientation.w = dropOffPose[2];

    // Send drop off goal
    ac.sendGoal(goal);
    ROS_INFO("Travelling to the drop off zone");

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Dropped off the virtual object");

      // Publish updated status (at drop off zone)
      status.data = 'd';
      object_status_pub.publish(status);

      // Pause 5 seconds to imitate dropping object
      ros::Duration(5.0).sleep(); 
    }
    else
      ROS_INFO("Failed to travel to the drop off zone");
  }
  else
    ROS_INFO("Failed to travel to the pick up zone");
}


