# path_plan_navigate
Turtlebot accepts several navigation goals in Gazebo world, generates trajectories, and navigates itself towards the goals.  

For this project, several ROS packages were installed and cloned into /home/workspace/catkin_ws/src directory:

- turtlebot navigation stack: turtlebot_navigation
- turtlebot_gazebo
- turtlebot_teleop
- turtlebot_rviz_launchers
- gmapping

The packages created from scratch:

- pick_objects
- add_markers

These two packages share the same folder location as the other packages:
  /home/workspace/catkin_ws/src.

Navigation goals or map positions are retrieved from pick_objects.launch file and then pick_objects node sends several 
navigation requests for Turtlebot to reach. turtlebot_navigation is communicated through move_base_msgs. 

Virtual objects to be transported by the robot are modeled in rviz by markers which are basic 3D geometric shapes such 
as cubes or cylinders.  These markers will be published by add_markers_hs node to simulate loads being picked up,
moved, and dropped off by Turtlebot.  
  
All required nodes are launched in separate terminals from home_service.sh shell script.
