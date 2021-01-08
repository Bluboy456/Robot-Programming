#!/usr/bin/env python
import rospy
import tf

def get_map_pose():
    tf_listener = tf.TransformListener()
    now = rospy.Time.now()
    tf_listener.waitForTransform('/map', '/thorvald_001/base_link', now, rospy.Duration(10))
    (t, r) = tf_listener.lookupTransform('/map', '/thorvald_001/base_link', now)
    print t


if __name__ == '__main__':
    rospy.init_node('debug', anonymous=True)
    get_map_pose()
