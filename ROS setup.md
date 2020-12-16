# Set-up ROS and CMP9767m

sudo apt-get update && sudo apt-get upgrade
sudo apt-get install ros-melodic-uol-cmp9767m-base 
(if this latter resulted in error, there's a conflicting package installed, try sudo apt-get purge "*gazebo*" first, and then install again). 
source /opt/ros/melodic/setup.bash


# to create workspace
source /opt/ros/melodic/setup.bash  
mkdir–p ~/catkin_ws/src  
cd ~/catkin_ws/src  
catkin_init_workspace  
cd ~/catkin_ws  
catkin_make or catkin build  

# To create a new package:
MUST BE IN THE ~/catkin_ws/src DIRECTORY BEFORE RUNNING CATKIN_CREATE_PKG
cd ~/catkin_ws/src  
catkin_create_pkg my_package_namerospy  

# Point to note, 
always source bash when in new terminal by: 
source devel/setup.bash  (now in setup.bash)

# to run thorvald in gazebo:
roslaunch uol_cmp9767m_base thorvald-sim.launch fake_localisation:=true map_server:=true
to kill Gazebo after crash:  
killall -9 gzserver  

# to run move_base server
uol_cmp9767m_tutorial move_base.launch

# to run rviz
with a .rviz config file.  Can also load and saveconfig from within rviz  
rviz -d `rospack find uol_cmp9767m_base`/rviz/two_robots.rviz  

To add robot model  
add by display type, robot model  
change description to /thorvald_001/robot_description  

Dont forget that add bt topic has a lot of options:  
E.g.  particecloud, amcl_pose and scan to see localaization (select fixed frame to map)  
E.g. odometry/base_raw and odometry/filtered to effect of Kalman filer (select fixed frame to odom)




# git commands
Code should be in ~/catkin_ws/src/my_examples/src/scripts/Robot-Programming  
cd to this directory  
git add .  
git commit  
Type commit message, ctrl o ctrl x to save and exit  
git push Bluboy456  smxxxxxxxx8  

# robot steering 
rosrun rqt_robot_steering rqt_robot_steering  
set topic to /thorvald_001/teleop_joy/cmd_vel  
(can ignore warning message)  

  
# Linux boot issues
Reinstalling REfInd from MacOS fixes boot to grub problem  
NB two spaces for newline in this markdown!  
