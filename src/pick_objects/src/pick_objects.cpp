#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// Global variables for amcl_pose estimates
float amcl_x, amcl_y;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// "amcl_pose" topic feed
// Bots pose estimates to be displayed at navigation goal
void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_msg){
   amcl_x = amcl_msg ->pose.pose.position.x;
   amcl_y = amcl_msg ->pose.pose.position.y;
   //ROS_INFO(" From AMCL - position x: %.2f, position y: %.2f \n ", float(amcl_x),float(amcl_y));
}//--- end amcl_pose_callback function

// Set a navigation goal for Turtlebot
void send_goal(float pos_x, float pos_y, float pos_w){
   
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
   
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.orientation.w = pos_w;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("From pick_objects node: \n");
  ROS_INFO("Sending goal - position x: %.2f, position y: %.2f \n", (float)pos_x, (float)pos_y );
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    
    ros::spinOnce();
    ROS_INFO(" Turtlebot has arrived at goal nav point - position x: %.2f, position y: %.2f \n", (float)amcl_x, (float)amcl_y);
    ros::Duration(5.0).sleep();  
  }
  else
    ROS_INFO("The base failed to arrive at goal for some reason");
  
  
}//--- end send_goal function

int main(int argc, char** argv){
  // 
  float goal_posX;
  float goal_posY;
  float goal_posW = 1.0;
  // Initialize the "pick_objects" node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle nh_po; //pick_objects node handle
  // Initialize subscriber to "/amcl_pose" topic
  ros::Subscriber sub_amcl = nh_po.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, amcl_pose_callback);
  // Get pickup zone 1 navigation goal
  nh_po.getParam( "goal_pickup1_x", goal_posX );
  nh_po.getParam( "goal_pickup1_y", goal_posY ); 
  //----- send nav goal 
  send_goal( goal_posX, goal_posY, goal_posW );  
    
  // Get dropoff zone navigation goal
  nh_po.getParam( "goal_dropoff1_x", goal_posX );
  nh_po.getParam( "goal_dropoff1_y", goal_posY ); 
  //----- send nav goal  
  send_goal( goal_posX, goal_posY, goal_posW );  
 
  // Get pickup zone 2 navigation goal
  nh_po.getParam( "goal_pickup2_x", goal_posX );
  nh_po.getParam( "goal_pickup2_y", goal_posY ); 
  //----- send nav goal 
  send_goal( goal_posX, goal_posY, goal_posW );  
  
  // Get dropoff zone navigation goal
  nh_po.getParam( "goal_dropoff1_x", goal_posX );
  nh_po.getParam( "goal_dropoff1_y", goal_posY ); 
  //----- send nav goal  
  send_goal( goal_posX, goal_posY, goal_posW ); 
  
  // Get back-to-base navigation goal
  nh_po.getParam( "goal_origin_x", goal_posX );
  nh_po.getParam( "goal_origin_y", goal_posY ); 
  //----- send nav goal  
  send_goal( goal_posX, goal_posY, goal_posW ); 
    
  ros::spin();
  
  return 0;
}
