#!/bin/sh
# Fernando Passold, em 03.05.2021, 26.05.2021
cd .. # going back to folder /src
cd .. # going back to folder /catkin_ws
# source devel/setup.bash # does not work separately from xterm!
echo "Launching gazebo with myworld.world and spawned my_robot"
xterm -e " source devel/setup.bash; roslaunch my_robot world.launch " &
sleep 8 # gazebo takes a while to initialize
echo "Launching ROS Navigation stack"
xterm -e " source devel/setup.bash; roslaunch my_robot navigation.launch " &
sleep 3
echo "Launching RViz..."
xterm -e " source devel/setup.bash; rosrun rviz rviz -d src/rvizConfig/rviz_amcl.rviz" &
sleep 3
echo "Activating Teleop (optional)..."
xterm -e " source devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py " &
