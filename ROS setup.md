# DEMONSTRATION PROCEDURE

set up 4 windows in Terminator for gazebo, mapping, rviz and programm



1) No obstacles
map time = 30 sec
roslaunch uol_cmp9767m_base thorvald-sim.launch obstacles:=false map_server:=true fake_localisation:=true
roslaunch uol_cmp9767m_tutorial mapping_fake_localisation.launch

After spraying, put box in front in gazebo and demo move_base avoidance

Then discuss look ahead feature in global costmap, but doesn't work well - incorrect map
DEmo problems with obstacles:= true









2) With obstacles
map time = 30 sec
roslaunch uol_cmp9767m_base thorvald-sim.launch obstacles:=true map_server:=true fake_localisation:=true
roslaunch uol_cmp9767m_tutorial mapping_fake_localisation.launch


|  Demonstrate 'mapping', traversing and spraying on empty field
1) No obstacles, map_server:=true  	|  amcl_localisation	|  Show again with amcl localisation
2) No obstacles, map_server:=false 	|  fake_localisation	|  Move_base fails - why??
3) With obstacles, map_server:=true  	|  fake_localisation	|  robot gets stuck because only using local costmap
4) With obstacles, no map_server:=false |  fake_localisation	|   Fails!





# TO RUN PROGRAM

# 1) Launch Gazebo:
without gmapping, uses blank map:
roslaunch uol_cmp9767m_base thorvald-sim.launch obstacles:=true map_server:=true

with gmapping:
roslaunch uol_cmp9767m_base thorvald-sim.launch obstacles:=true 


# 2) launch move_base server:
roslaunch uol_cmp9767m_tutorial mapping_amcl.launch
or 
roslaunch uol_cmp9767m_tutorial mapping_fake_localisation.launch



# 3) run program:
rosrun my_examples move_avoid_zap.py



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

# to kill Gazebo after crash:  
killall -9 gzserver  

# to run rviz
rviz rviz -d ~/catkin_ws/src/my_examples/rviz_config/mapping.rviz


# To add robot model  
add by display type, robot model  
change description to /thorvald_001/robot_description  

# Dont forget that add bt topic has a lot of options:  
E.g.  particecloud, amcl_pose and scan to see localaization (select fixed frame to map)  
E.g. odometry/base_raw and odometry/filtered to effect of Kalman filer (select fixed frame to odom)

# Tune amcl
Adjust max and min point numbers to stop jumping


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


# Move_base complete messages

uint8 PENDING         = 0   # The goal has yet to be processed by the action server
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                            #   and has since completed its execution (Terminal State)
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                            #    to some failure (Terminal State)
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                            #    because the goal was unattainable or invalid (Terminal State)
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                            #    and has not yet completed execution
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                            #    but the action server has not yet confirmed that the goal is canceled
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                            #    and was successfully cancelled (Terminal State)
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                            #    sent over the wire by an action server

# example code for move_base callbacks
    # This answer from the ros community used in part for this function
    # https://answers.ros.org/question/292061/how-do-i-create-a-callback-based-actionlib-client-in-python/
        def counter_client():
        client = actionlib.SimpleActionClient('counter_as', CounterAction)
        rospy.loginfo("Waiting for action server to come up...")
        client.wait_for_server()

        client.send_goal(CounterGoal(10),
                        active_cb=callback_active,
                        feedback_cb=callback_feedback,
                        done_cb=callback_done)

        rospy.loginfo("Goal has been sent to the action server.")

    def callback_active():
        rospy.loginfo("Action server is processing the goal")

    def callback_done(state, result):
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))

    def callback_feedback(feedback):
        rospy.loginfo("Feedback:%s" % str(feedback))
    '''

