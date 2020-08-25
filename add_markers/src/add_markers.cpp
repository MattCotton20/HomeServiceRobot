#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

class AddMarkers {
public:

  visualization_msgs::Marker marker;

  AddMarkers()
  {
    //Publisher
    pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    //Subscribe to /object_status from the pick_objects node to know where to display marker
    sub_status = n_.subscribe("/object_status", 1, &AddMarkers::marker_status_callback, this);
	   
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type to sphere
    marker.type = visualization_msgs::Marker::SPHERE;

    // Add the marker with default orientation
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color to green
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
  }

  //Function to publish marker
  void PublishMarker()
  {
    pub_.publish(marker);
  }

  //Callback function for the /object_status topic
  void marker_status_callback(const std_msgs::String msg)
  {
    if (msg.data == "pickup")
    {
      // Display marker at pick up position
      this->SetMarkerPosition(7.0, 0.5, 0.15);
      this->SetVisibility(1.0);
      ROS_INFO("Marker displayed at pickup zone");
    }
    else if (msg.data == "held")
    {
      // Hide marker to indicate being held
      this->SetVisibility(0.0);
      ROS_INFO("Marker hidden");
    }
    else
    {
      // Display marker at drop off position
      this->SetMarkerPosition(5.0, -3.5, 0.15);
      this->SetVisibility(1.0);
      ROS_INFO("Marker displayed at pickup zone");
    }
    pub_.publish(marker);
  }


private:
  //Private variables
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_status;

  //Function to change visibility of marker
  void SetVisibility(float a){
     marker.color.a = a;
  }

  //Function to change position of marker
  void SetMarkerPosition (double x,double y, double z){

    // Set the pose of the marker
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
  }
 
};

int main( int argc, char** argv )
{
  // Initialise node
  ros::init(argc, argv, "add_markers");

  //Call AddMarkers constructor
  AddMarkers newAddMarkers;

  ros::spin();

  return 0;

}
