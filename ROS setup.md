# Set-up commands

# to create workspace
source /opt/ros/melodic/setup.bash
mkdir–p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make or catkin build

# To create a new package:
cd ~/catkin_ws/src
catkin_create_pkg my_package_namerospy

# Point to note, 
always source bash when in new terminal by:
source devel/setup.bash  (now in setup.bash)

# to run thorvald:
roslaunch uol_cmp9767m_base thorvald-sim.launch
