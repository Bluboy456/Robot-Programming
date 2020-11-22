#! /usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from uol_cmp9767m_tutorial.srv import *
from std_srvs.srv import Empty


class move_find_green:

    '''
    move foward but turn  if within 1m of an obstacle
    '''

    def __init__(self):
        rate = rospy.Rate(3)
        self.sub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.laser_callback)   #subscribe to laser to get distance messages to get distances
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.camera_callback)
        self.pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)  #publisher for speed and turn to robot
        self.base_pub = rospy.Publisher('base_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.scan_pub = rospy.Publisher('scan_pose', geometry_msgs.msg.PoseStamped, queue_size=1)
        self.motion = Twist()
        
        #starting values
        self.direction = 'left'
        self.obstacle_x = 10.0      
        self.obstacle_y = 10.0
        self.angle_to_obstacle = 0.0
        self.distance_to_obstacle = 10.0
        self.stopped = False

        self.bridge_object = CvBridge()
        
        # get transform from laser scanner (hokuyo to base_link), only need to do it once as static
        self.listener = tf.TransformListener()

        rospy.spin()

    def laser_callback (self, msg):      # called when a laser message arrives, extracts front, left and right distances
        if self.stopped == False:        #check stopped flag first before doing anything
            rospy.spin
            self.ranges = msg.ranges
            self.angle_increment = msg.angle_increment
            self.clearance_right, self.angle_right = self.right_sector_clearance()
            self.clearance_left, self.angle_left = self.left_sector_clearance()         
        
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

    def calculate_transforms(self):
    # calculate x, y co-ordinate of closest obstacle, realtive to scanner
        #first work out whether closest obstacle is in left or right sectors

        if self.clearance_right < self.clearance_left: 
            self.distance_to_obstacle = self.clearance_right
            self.angle_to_obstacle = self.angle_right
        else:
            self.distance_to_obstacle = self.clearance_left
            self.angle_to_obstacle = self.angle_left

        # Convert polar to cartesian coordinates and publish as a pose
        self.obstacle_x = self.distance_to_obstacle * math.cos(self.angle_to_obstacle)
        self.obstacle_y = self.distance_to_obstacle * math.sin(self.angle_to_obstacle )   # negative to the right

        self.p_scan = geometry_msgs.msg.PoseStamped()
        self.p_scan.header.frame_id = "thorvald_001/hokuyo"
        self.p_scan.pose.orientation.w = 1.0  # No rotation
        self.p_scan.pose.position.x = self.obstacle_x
        self.p_scan.pose.position.y = self.obstacle_y
        self.scan_pub.publish(self.p_scan)   
        
        #calculate obstacle postion relative to base_link and publish as a pose 
        self.laser_to_base = self.listener.lookupTransform('thorvald_001/hokuyo', 'thorvald_001/base_link', rospy.Time())
        self.p_base = geometry_msgs.msg.PoseStamped()
        self.p_base.header.frame_id = "thorvald_001/base_link"
        self.p_base.pose.orientation.w = 1.0  # No rotation
        self.p_base.pose.position.x = self.obstacle_x - self.laser_to_base[0][0]
        self.p_base.pose.position.y = self.obstacle_y - self.laser_to_base[0][1]
        self.base_pub.publish(self.p_base) 

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            #example_path = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/Filtering.png'
            #image = cv2.imread(example_path)
           
        except CvBridgeError as e:
            print(e)


        #Resizeimage so it is easier to work with
        image = cv2.resize(image,(300,300))

        #Change the color space to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #Hsv limits
        min_green = np.array([30,50,50])
        max_green = np.array([70,255,255])

        #Create a black & white mask for green
        mask_g = cv2.inRange(hsv, min_green, max_green)
        cv2.imshow('Green Mask',mask_g)
        #colured post-processed image
        res_g = cv2.bitwise_and(image,image, mask= mask_g)
        cv2.imshow('Green',res_g)
        #cv2.imshow('original', image)

        #Detect any blobs in green iamge
        # Set up the detector with default parameters.

        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(mask_g)
        print keypoints
	    # Draw detected blobs as red circles.
	    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        res_g_with_blobs = cv2.drawKeypoints(res_g, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # spray if blob (weed) found TODO move robot to spray in correct place
        if keypoints:
            #self.stopped = True                     #stop robot if keypoint (blob) found DEBUG
            rospy.wait_for_service('/thorvald_001/spray')
            try:
                spray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
                spray()
            except rospy.ServiceException as e:
                print("Spray service call failed: %s"%e)


        # Show blobs
        cv2.imshow("Keypoints", res_g_with_blobs)
        cv2.waitKey(1)




# Main Code
if __name__ == '__main__':
    rospy.init_node('move_avoid', anonymous=True)
    try:
       move_find_green()
       rospy.spin
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass

  

   


 
