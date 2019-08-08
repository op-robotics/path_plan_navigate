#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/workspace/catkin_ws/src/maps/nav-test.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/maps/map.yaml

xterm  -e  " source /opt/ros/kinetic/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm  -e  " roslaunch add_markers add_markers_hs.launch " &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm  -e  " roslaunch pick_objects pick_objects.launch " 
