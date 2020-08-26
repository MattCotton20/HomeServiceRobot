#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Char.h>

// Define global variables
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void object_status_callback(const std_msgs::Char status)
{
  switch (status.data) {

  // Pick up
  case 'p':
    // Display marker at pick up position
    marker.pose.position.x = 7.0;
    marker.pose.position.y = 0.5;
    marker.pose.position.z = 0.15;

    // Make marker visible
    marker.color.a = 1.0;

    ROS_INFO("Marker displayed at pickup zone");
    break;

  // Held
  case 'h':
    // Make marker invisible
    marker.color.a = 0.0;

    ROS_INFO("Marker hidden");
    break;

  // Drop off
  case 'd':
    // Display marker at drop off position
    marker.pose.position.x = 5.0;
    marker.pose.position.y = -3.5;
    marker.pose.position.z = 0.15;

    // Make marker visible
    marker.color.a = 1.0;

    ROS_INFO("Marker displayed at drop off zone");
    break;

  default:
    ROS_WARN("Invalid object_status message");
  }
  marker_pub.publish(marker);
}


int main( int argc, char** argv )
{
  // Initialise the add_markers node and create a handle to it
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  // Define marker publisher to send marker data to RViz 
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Define object_status publisher to listen for when the virtual object is moved
  ros::Subscriber object_status_sub = n.subscribe("/object_status", 1, object_status_callback);

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "virtual_object";
  marker.id = 0;

  // Set the market shape type to be a sphere
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set default pose of the marker
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color to green, and invisible so it is not initially shown
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.0;

  marker.lifetime = ros::Duration();

  ros::spin();

  return 0;
}

