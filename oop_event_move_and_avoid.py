#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class move_and_turn:

    '''
    move foward but turn 90deg if within 1m of an obstable 
    '''

    def __init__(self):
        rate = rospy.Rate(2)
        self.sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.callback)   #subscribe to laser to get distance messages to get distances
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  #publisher for speed and turn to robot
        self.angular_speed = 0.5
        self.turn_in_progress = False #state variable
        self.turn_direction = 'left'
        self.motion = Twist()
        self.start_time = 0.0
        rospy.spin()

    def callback (self, msg):      # called when a laser message arrives, extracts front, left and right distances
        rospy.spin
        self.right_clearance = msg.ranges[0]
        self.front_clearance = msg.ranges[360]
        self.left_clearance = msg.ranges[719]
        '''
        print ('front clearance = ' + str(self.front_clearance))    #DEBUG
        print ('left clearance = ' + str(self.left_clearance))    #DEBUG
        print ('right clearance = ' + str(self.right_clearance))    #DEBUG
        '''
        
        # robot is already in a turn
        if self.turn_in_progress == True:
            #calulate current turn angle
            self.current_time = rospy.get_time()
            self.current_angle = abs(self.angular_speed*(self.current_time - self.start_time))
            if (self.current_angle > 1.57):  #1.57 is pi/2, ie 90deg        
                self.turn_in_progress = False        #no more turning   
            
        #robot is not turning, but does it need to?
        elif self.front_clearance < 1:  
            #needs to turn, set the direction of rotation
            if self.left_clearance < 1:      
                self.direction = 'right'
            elif self.right_clearance < 1:
                self.direction ='left'
            #turn left by default
            else:
                self.direction ='left'
            self.start_time = rospy.get_time()
            self.turn_in_progress = True
            self.rotate()

        else:  #free to move forward
            self.motion.linear.x = 0.5   
            self.motion.angular.z = 0
            self.pub.publish(self.motion)


    def rotate(self):
        self.motion.linear.x = 0.0
        if self.direction == 'right':
            self.motion.angular.z = -1 * 0.5   
        else:
            self.direction = 'left'
            self.motion.angular.z = 0.5   
        self.pub.publish(self.motion)    






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

  

   


 