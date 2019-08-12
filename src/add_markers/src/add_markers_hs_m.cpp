#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

//--- declare global variables 
// Amcl pose estimate for x and y
float amcl_pos_x = 0.0;
float amcl_pos_y = 0.0;
// Robot's velocity broadcast from "/odom"
float odom_vel_lin_x = 0.0;

ros::Publisher pub_marker;

void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
  amcl_pos_x = amcl_msg ->pose.pose.position.x;
  amcl_pos_y = amcl_msg ->pose.pose.position.y;
  //ROS_INFO(" From AMCL - position x: %.2f, position y: %.2f \n ", float(amcl_pos_x),float(amcl_pos_y));
}// === end amcl_pose_callback

void odometer_callback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    
  odom_vel_lin_x = odom_msg->twist.twist.linear.x;
  //ROS_INFO(" From odom_callback - pos x: %.2f, lin_vel x: %.2f", (float)odom_pos_x, (float)odom_vel_lin_x); 
}// === end odometer_callback

bool display_blue_cylinder(float x, float y, int marker_action, int marker_id){
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
   
  visualization_msgs::Marker marker;
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cylinder";
  marker.id = marker_id;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
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
  marker.scale.x = .4;
  marker.scale.y = .4;
  marker.scale.z = .4;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.15f;
  marker.color.g = 0.2f;
  marker.color.b = 0.8f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Count number of subscribers to marker
  while (pub_marker.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  // Publish marker      
  pub_marker.publish(marker);
    
  return true;
}//--- end display_blue_cylinder

bool display_red_cube(float x, float y, int marker_action, int marker_id){
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
   
  visualization_msgs::Marker marker;
  
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = marker_id;
  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;
  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
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
  marker.scale.x = .4;
  marker.scale.y = .4;
  marker.scale.z = .4;
  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.8f;
  marker.color.g = 0.2f;
  marker.color.b = 0.15f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Count number of subscribers to marker
  while (pub_marker.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  // Publish marker      
  pub_marker.publish(marker);
    
  return true;
}//--- end display_red_cube

int main( int argc, char** argv )
{
  int add_marker; // Marker action - 1:publish  0:delete 
  int marker_no; // Marker ID
  
  // Variables to store navigation goals:
  float goal_pu1_x; 
  float goal_pu1_y; 
  float goal_pu2_x; 
  float goal_pu2_y; 
  float goal_do1_x; 
  float goal_do1_y; 
  //
  int drop = 0;
  // Initialize add_markers node
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle add_mark;
  // Initialize pub_marker publisher
  pub_marker = add_mark.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Initialize sub_amcl subcriber that will receive "/amcl_pose" messages and process them in amcl_pose_callback function  
  ros::Subscriber sub_amcl = add_mark.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, amcl_pose_callback);
  ros::Rate r(5);
  
  ROS_INFO("--- add_markers node: ready to publish --- \n");
  // Retrieve navigation goals from the pick_objects.launch
  // for pickup and dropoff zones
  add_mark.getParam( "goal_pickup1_x", goal_pu1_x ); // Pickup zone 1  
  add_mark.getParam( "goal_pickup1_y", goal_pu1_y ); // Pickup zone 1
  add_mark.getParam( "goal_pickup2_x", goal_pu2_x ); // Pickup zone 2
  add_mark.getParam( "goal_pickup2_y", goal_pu2_y ); // Pickup zone 2
  add_mark.getParam( "goal_dropoff1_x", goal_do1_x ); // Dropoff zone
  add_mark.getParam( "goal_dropoff1_y", goal_do1_y );     
  // Publish marker at pickup zone 1
  add_marker = 1; // ADD marker
  marker_no = 1;  // Marker ID = 1 
  display_blue_cylinder(goal_pu1_x, goal_pu1_y, add_marker, marker_no);//@pickup zone 1
  
  // Publish marker at pickup zone 2
  //marker_no = 2;
  display_red_cube(goal_pu2_x, goal_pu2_y, add_marker, marker_no);//@pickup zone 2
  // Keep the loop going while node is active 
  while (ros::ok()){
    // @pickup zone 1:
    if (abs(goal_pu1_x-amcl_pos_x) < 0.13 && abs(goal_pu1_y-amcl_pos_y) < 0.13){
      drop = 1;
      ROS_INFO("-- @pickup zone 1 --");
      ros::Duration(5.0).sleep(); 
      marker_no = 1; 
      add_marker = 0; // DELETE marker 
      display_blue_cylinder(goal_pu1_x, goal_pu1_y, add_marker, marker_no); // Delete marker @pickup zone 1
    }
    // @pickup zone 2:
    else if (abs(goal_pu2_x-amcl_pos_x) < 0.13 && abs(goal_pu2_y-amcl_pos_y) < 0.13){
      drop = 2;
      ROS_INFO("-- @pickup zone 2 --");
      ros::Duration(5.0).sleep(); 
      marker_no = 1;
      add_marker = 0; // Delete marker #2
      display_red_cube(goal_pu2_x, goal_pu2_y, add_marker, marker_no); // Delete marker @pickup zone 2
    }
    // @dropoff zone:
    else if (abs(goal_do1_x-amcl_pos_x) < 0.13 && abs(goal_do1_y-amcl_pos_y) < 0.13){
      if (drop == 1){

        ROS_INFO("-- @dropoff zone from pickup zone 1--");
        ros::Duration(5.0).sleep(); 
        add_marker = 1; // ADD marker
        marker_no = 2;
        display_blue_cylinder(goal_do1_x, goal_do1_y, add_marker, marker_no); // Publish marker

      }
      else{

        ROS_INFO("-- @dropoff zone from pickup zone 2--");
        ros::Duration(5.0).sleep(); 
        add_marker = 1; // ADD marker
        marker_no = 2;
        display_red_cube(goal_do1_x+.3, goal_do1_y+.3, add_marker, marker_no); // Publish marker

      }
    }//--- end else if
    
    ros::spinOnce();
    r.sleep();
  }//--- end while loop
  return 0;
}//--- end main 
