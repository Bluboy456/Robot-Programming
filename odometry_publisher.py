#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('Odometry_publisher')
pub = rospy.Publisher('/Odometry', Odometry, queue_size=1)
rate = rospy.Rate(2)
position = rospy.get_param("/Odometry")

while not rospy.is_shutdown(): 
  pub.publish(position)
  count.data += 1
  rate.sleep()