#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist



def publish_cmd_vel():
    rospy.init_node('cmd_vel_publish')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(2)
    motion = Twist()
    motion.linear.x = 0.5
    motion.angular.z = 0.5
    while not rospy.is_shutdown(): 
        pub.publish(motion)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass