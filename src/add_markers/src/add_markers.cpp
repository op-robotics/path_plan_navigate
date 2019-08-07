#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//--- declare marker_pub publisher as a global variable
ros::Publisher marker_pub;
// Publish or erase marker at specified map position
bool display_marker(float x, float y, int marker_action){
    
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
    
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;
	    
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  // 1: marker is published  2: it is deleted 
  if (marker_action == 1)
    marker.action = visualization_msgs::Marker::ADD;
  else
    marker.action = visualization_msgs::Marker::DELETE;
    
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = .32;
  marker.scale.y = .32;
  marker.scale.z = .32;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.15f;
  marker.color.g = 0.2f;
  marker.color.b = 0.9f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Get number of subscribers
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  // Publish the marker      
  marker_pub.publish(marker);
     
  return true;
}//---- end display_marker   

int main( int argc, char** argv )
{
  // marker position and action to pass in to display_marker function
  float pos_x;
  float pos_y;
  int switch_marker;
  // Initialize add_markers node  
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle add_mark;
  // Initialize marker_pub publisher
  marker_pub = add_mark.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  ROS_INFO("--- add_markers node: ready to publish --- \n");
  // Get pickup zone location from the add_markers.launch
  add_mark.getParam( "goal_pickup_x", pos_x );
  add_mark.getParam( "goal_pickup_y", pos_y ); 
  // ADD in marker action 
  switch_marker = 1; 
  // Publish marker  
  display_marker(pos_x, pos_y, switch_marker);
  
  ros::Duration(5.0).sleep();   
  // Delete in marker action
  switch_marker = 0;
  // Erase marker at pickup zone  
  display_marker(pos_x, pos_y, switch_marker);
  
  ros::Duration(5.0).sleep();   
  // Get dropoff zone position
  add_mark.getParam( "goal_dropoff_x", pos_x );
  add_mark.getParam( "goal_dropoff_y", pos_y );  
  // ADD in marker action
  switch_marker = 1;
  // Publish marker
  display_marker(pos_x, pos_y, switch_marker);
  
  ros::spin();
  
  return 0;
}//--- end main 
