#! /usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class move_and_turn:

    '''
    move foward but turn  if within 1m of an obstacle
    '''

    def __init__(self):
        rate = rospy.Rate(3)
        self.sub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.callback)   #subscribe to laser to get distance messages to get distances
        self.pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)  #publisher for speed and turn to robot
        self.obstacle_pub = rospy.Publisher('obstacle_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.motion = Twist()
        self.direction = 'left'
        self.obstacle_x = 10.0
        self.obstacle_y = 10.0
        self.angle_to_obstacle = 0.0
        self.distance_to_obstacle = 10.0
        # get transform from laser scanner (hokuyo to base_link), only need to do it once as static
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('thorvald_001/hokuyo', 'thorvald_001/base_link', rospy.Time(0), rospy.Duration(5))
        self.laser_to_base = self.listener.lookupTransform('thorvald_001/hokuyo', 'thorvald_001/base_link', rospy.Time())
        rospy.spin()

    def callback (self, msg):      # called when a laser message arrives, extracts front, left and right distances
        rospy.spin
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment


       
        # calculate x, y co-ordinate of closest obstacle, realtive to scanner
        #first work out whether closest obstacle is in left or right sectors
        self.clearance_right, self.angle_right = self.right_sector_clearance()
        self.clearance_left, self.angle_left = self.left_sector_clearance()  
        if self.clearance_right < self.clearance_left: 
            self.distance_to_obstacle = self.clearance_right
            self.angle_to_obstacle = self.angle_right
        else:
            self.distance_to_obstacle = self.clearance_left
            self.angle_to_obstacle = self.angle_left
        #print ('distance_to_obstacle = ' + str(self.distance_to_obstacle))  #DEBUG
        #print ('angle_to_obstacle = ' + str(self.angle_to_obstacle +'\n'))    #DEBUG

        # Convert polar to cartesian coordinates
        self.obstacle_x = self.distance_to_obstacle * math.cos(self.angle_to_obstacle)
        self.obstacle_y = self.distance_to_obstacle * math.sin(self.angle_to_obstacle )   # negative to the right
        #print ('obstacle_x: ' + str(self.obstacle_x) )   #DEBUG
        #print ('obstacle_x: ' + str(self.obstacle_y) +'\n') #DEBUG

        #calculate obstacle postion relative to base_link and publish as a pose
        self.laser_to_base = self.listener.lookupTransform('thorvald_001/hokuyo', 'thorvald_001/base_link', rospy.Time())
        print('self.laser_to_base: ')
        print(self.laser_to_base)
        
        self.p1 = geometry_msgs.msg.PoseStamped()
        self.p1.header.frame_id = "thorvald_001/base_link"
        self.p1.pose.orientation.w = 1.0  # No rotation
        self.p1.pose.position.x = self.obstacle_x - self.laser_to_base[0][0]
        self.p1.pose.position.y = self.obstacle_y - self.laser_to_base[0][1]
        self.obstacle_pub.publish(self.p1)   
        print('self.p1.pose.position: ')
        print(self.p1.pose.position)
        
         #does it need to turn left?
        if self.clearance_right < 1.0:  
            self.rotate('left')
        
        #does it need to turn right?
        elif self.clearance_left <1.0:
            self.rotate('right')

        else:  #free to move forward
            self.motion.linear.x = 0.5   
            self.motion.angular.z = 0
            self.pub.publish(self.motion)


    def rotate(self, direction):
        self.motion.linear.x = 0.0
        if self.direction == 'right':
            self.motion.angular.z = -1 * 0.5   
        else:
            self.direction = 'left'
            self.motion.angular.z = 0.5   
        self.pub.publish(self.motion)    


    # returns minium distance in right front 90deg sector and angle to that closest obstacle
    def right_sector_clearance(self):
        self.min_distance = 10.0
        self.angle_to_closest = 0.0
        #for index in range(len(self.ranges)/2-1,0):
        for index in range(0,len(self.ranges)/2-1):
            if self.min_distance > self.ranges[index]:  
                self.min_distance = self.ranges[index]    #found a new minium distance
                self.angle_to_closest = -1*((len(self.ranges)/2-1) - index ) * self.angle_increment  # convert to radians  
                                                # angle of zero must be straight ahead 
                                                # clockwise angle (to right) so negative in polar co-ords
        return self.min_distance, self.angle_to_closest

   # returns minium distance in left front 90deg sector and angle to that closest obstacle
    def left_sector_clearance(self):
        self.min_distance = 10.0  
        self.angle_to_closest = 0.0
        for index in range(len(self.ranges)/2,len(self.ranges)-1):
            if self.min_distance > self.ranges[index]:  
                self.min_distance = self.ranges[index]  #found a new minium distance
                self.angle_to_closest = index * self.angle_increment  # convert to radians
                                                # anticlockwise angle (to left) so positive in polar co-ords
        return self.min_distance, self.angle_to_closest

        


# Main Code
# this is the Python way of checking that this file is run as an
# executable (like the main() function in C++ etc)
if __name__ == '__main__':
    # ALWAYS first initialise your ros node with a name,
    # optional it can have added a unique identifier (anonymous) to all
    # starting the same code several times.
    rospy.init_node('move_avoid', anonymous=True)
    # "try"/"except" is the Python way of exception handling
    try:
        # instantiate the object
       move_and_turn()
       rospy.spin
        # now enter the run method (will run until node terminates)
    except rospy.ROSInterruptException:
        # loggin a la ROS... here log the interruption as a warning
        rospy.logwarn("interrupted")
        pass

  

   


 