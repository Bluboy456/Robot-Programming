# Set-up commands

source /opt/ros/melodic/setup.bash
mkdir–p ~/catkin_ws/src•cd~/catkin_ws/src
catkin_init_workspace•catkin_makeorcatkin build

To create a new package:
cd ~/catkin_ws/src•catkin_create_pkg my_package_namerospy...
Point to note, always source bash when in new terminal by:
source devel/setup.bash

