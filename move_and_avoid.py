#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#globals


#functions

def laser_callback(msg):      # called when a laser message arrives, extracts front, left and right distances
    global left_clearance
    global front_clearance
    global right_clearance
    right_clearance = msg.ranges[0]
    front_clearance = msg.ranges[360]
    left_clearance = msg.ranges[719]

def turn_right():
    motion.linear.x = 0   
    motion.angular.z = -0.4   #positive values anticlockwise
    return motion

def turn_left():
    motion.linear.x = 0   
    motion.angular.z = 0.4
    return motion

def go_forward():
    motion.linear.x = 0.5   
    motion.angular.z = 0
    return motion


# Main Code
rospy.init_node('move_avoid')    
motion = Twist()
rate = rospy.Rate(2)
front_clearance = 2    #intialisation, these values will be overwritten with first value from laser
left_clearance = 2
right_clearance = 2
delay = 0.0
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, laser_callback)    #subscribe to laser to get distance messages to get distances
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  #publish speed and turn to robot
while not rospy.is_shutdown():     #Main loop, with avoidance logic
    #print ('front clearance = ' + str(front_clearance))    #DEBUG
    #print ('left clearance = ' + str(left_clearance))    #DEBUG
    #print ('right clearance = ' + str(right_clearance))    #DEBUG

    # turn if not enough room to move forward
    if front_clearance < 1:   
        delay = 4.2  # set time for robot to turn
        #avoid obstables to the side
        if left_clearance < 1:      
            motion = turn_right()
        elif right_clearance < 1:
            motion = turn_left()
       #avoid obstables to the side so turn left by default
        else:
            motion = turn_left()
    # otherwise move forward
    else:
        go_forward()
        delay = 0.0   #mustn't go to sleep if moving forward

    #print ('forward = ' + str(motion.linear.x) + '    turn = ' + str(motion.angular.z))    #DEBUG
    pub.publish(motion)
    rospy.sleep(delay)
rospy.spin()   

   


 