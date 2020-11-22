#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class move_and_turn:

    '''
    move foward but turn  if within 1m of an obstable 
    '''

    def __init__(self):
        rate = rospy.Rate(3)
        self.sub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.callback)   #subscribe to laser to get distance messages to get distances
        self.pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)  #publisher for speed and turn to robot
        self.motion = Twist()
        self.direction = 'left'
        rospy.spin()

    def callback (self, msg):      # called when a laser message arrives, extracts front, left and right distances
        rospy.spin
        self.ranges = msg.ranges
   
        '''
        print ('left clearance = ' + str(self.left_sector_clearance))    #DEBUG
        print ('right clearance = ' + str(self.right_sector_clearance))    #DEBUG
        '''


        # calculate x, y co-ordinate of closest obstacle, realtive to scanner
        #first work out whether closest obstable is in left or right sectors
        if self.right_sector_clearance[0] < self.left_sector_clearance[0]: 
            self.distance_to_closest_obstable = self.right_sector_clearance[0]
            self.angle_to_obstable = self.right_sector_clearance[1]
        else:
             self.distance_to_closest_obstable = self.left_sector_clearance[0]   
             self.angle_to_obstable = self.left_sector_clearance[1]

        # Converting polar to cartesian coordinates
        self.theta = self.angle_to_obstable * math.pi/180.0
        self.ahead_clearance = self.distance_to_closest_obstable * math.cos(self.theta)
        self.lateral_clearance = self.distance_to_closest_obstable * math.sin(self.theta)   # negative to the right



         #does it need to turn left?
        if self.right_sector_clearance[0] < 1.0:  
            self.rotate('left')
        
        #does it need to turn right?
        elif self.left_sector_clearance[0] <1.0:
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
        self.angle_to_closest
        for angle in range(0,359):
            if self.min_distance > self.ranges[angle]:  
               self.min_distance = self.ranges[angle]    #found a new minium distance
               self.angle_to_closest = angle/-4  # angle_increment is 0.25 degree - convert array index to degrees
                                                # clockwise angle (to right) so negative in polar co-ords
        return self.min_distance, self.angle_to_closest

   # returns minium distance in left front 90deg sector and angle to that closest obstacle
    def left_sector_clearance(self):
        self.min_distance = 10.0  
        for angle in range(360,719):
            if self.min_distance > self.ranges[angle]:  
               self.min_distance = self.ranges[angle]  #found a new minium distance
               self.angle_to_closest = angle/4  # angle_increment is 0.25 degree - convert array index to degrees
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

  

   


 