#!/bin/sh
# Fernando Passold, em 28.05.2021
cd .. # going down to folder /src
cd .. # going down to folder /catkin_ws
echo "Launch script file for Pick Objects Test"
# source devel/setup.bash # does not work separately from xterm!
echo "Launching gazebo with my_robot on myworld"
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 8 # gazebo takes a while to initialize
echo "Launching AMCL node..."
xterm -e " source devel/setup.bash; roslaunch my_robot amcl.launch " &
sleep 3
echo "Launching ROS Navigation stack"
xterm -e " source devel/setup.bash; roslaunch my_robot navigation.launch " &
sleep 3
echo "Launching RViz..."
xterm -e " rosrun rviz rviz -d src/my_robot/rvizConfig/rviz_amcl.rviz " &
sleep 5
echo "Activating Pick Objects node"
xterm -e " source devel/setup.bash; roslaunch my_robot pick_objects.launch " &
