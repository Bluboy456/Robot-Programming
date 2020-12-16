#! /usr/bin/env python
from __future__ import division   # force / operator to return float instead of int
import rospy
import math
import tf
import time
import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from uol_cmp9767m_tutorial.srv import *
from std_srvs.srv import Empty
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class mapping():

    # Move robot around, avoiding obstacles while map server running
    # Creates occupancy map in an array
    # move foward but turn  if within 1m of san obstacle


    def __init__(self):
        self.motion = Twist()
        self.direction = 'left'
        self.obstacle_x = 10.0          #all distances in metres
        self.obstacle_y = 10.0
        self.angle_to_obstacle = 0.0
        self.distance_to_obstacle = 10.0
        self.linear_speed = 0.7         #metres per second
        self.angular_speed = 0.7        #radians per second
        self.mapping_start_time = time.time()
        self.mapping_complete = False   #flag that can be interpreted to find out whether mapping is complete
        self.mapping_duration = 2      #duration of mapping stage in seconds
        self.mapping_complete = False
        self.rate = rospy.Rate(3) 



        
        
    def map (self):              #method to start mapping
        self.laser_sub = rospy.Subscriber('/thorvald_001/scan', LaserScan, self.laser_callback_mapping)   #subscribe to laser to get distance messages to get distances
        self.cmd_vel_pub = rospy.Publisher('/thorvald_001/twist_mux/cmd_vel', Twist, queue_size=1)  #publisher for speed and turn to robot
        self.motion = Twist()

    def laser_callback_mapping (self, msg):     # called when a laser message arrives,moves randomly and avoids
                                                #obstacles, used for initial mapping
    
        # check whether mapping time is over
        if (time.time() - self.mapping_start_time) > self.mapping_duration:
            self.mapping_shutdown()
            return

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
            self.motion.linear.x = self.linear_speed 
            # introduce a turn sometimes to give a random element to motion
            # dont use random function as that will average to zero over callbacks
            time_in_secs_mod_ten = 0     #just to avoid undeclared variable syntax warning
            time_in_secs_mod_ten = int(round(time.time())) % 10
            angular_vel = 0.0
            if (time_in_secs_mod_ten < 5):
                angular_vel = (time_in_secs_mod_ten-2)/10.0
            print 'random angular velocity: ' + str (angular_vel)  #DEBUG


            self.motion.angular.z = angular_vel
            self.cmd_vel_pub.publish(self.motion)

    def mapping_shutdown(self):
        self.mapping_complete = True
        self.laser_sub.unregister()  

    def rotate(self, direction):
        self.motion.linear.x = 0.0
        if self.direction == 'right':
            self.motion.angular.z = -1 * self.angular_speed   
        else:
            self.direction = 'left'
            self.motion.angular.z = self.angular_speed  
        self.cmd_vel_pub.publish(self.motion)    


    # returns minium distance in right front 90deg sector and angle to that closest obstacle
    def right_sector_clearance(self):
        self.min_distance = 10.0
        self.angle_to_closest = 0.0
        for index in range(0,len(self.ranges)//2-1):
            if self.min_distance > self.ranges[index]:  
                self.min_distance = self.ranges[index]    #found a new minium distance
                self.angle_to_closest = -1*((len(self.ranges)//2-1) - index ) * self.angle_increment  # convert to radians  
                                                # angle of zero must be straight ahead 
                                                # clockwise angle (to right) so negative in polar co-ords
        return self.min_distance, self.angle_to_closest

# returns minium distance in left front 90deg sector and angle to that closest obstacle
    def left_sector_clearance(self):
        self.min_distance = 10.0  
        self.angle_to_closest = 0.0
        for index in range(len(self.ranges)//2,len(self.ranges)-1):
            if self.min_distance > self.ranges[index]:  
                self.min_distance = self.ranges[index]  #found a new minium distance
                self.angle_to_closest = index * self.angle_increment  # convert to radians
                                                # anticlockwise angle (to left) so positive in polar co-ords
        return self.min_distance, self.angle_to_closest



    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            self.tf_listener.waitForTransform('thorvald_001/odom', 'thorvald_001/base_link', rospy.Time.now(), rospy.Duration(4.0))
            (trans, rot)  = self.tf_listener.lookupTransform('thorvald_001/odom', 'thorvald_001/base_link', rospy.Time())
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans))   

    
class weeding:   

    def __init__(self):
        self.occupancy_grid = rospy.wait_for_message('/map',OccupancyGrid )
        self.meta_data = rospy.wait_for_message('/map_metadata',MapMetaData ) 
        self.move_base_client = actionlib.SimpleActionClient('/move_base/',MoveBaseAction)
        resolution = self.meta_data.resolution  # metres per cell of occupancy grid
        self.grid_width = self.meta_data.width * resolution  #convert width from cell count to metres
        self.grid_height = self.meta_data.height * resolution
        self.grid_origin_x = self.meta_data.origin.position.x
        self.grid_origin_y = self.meta_data.origin.position.y
        '''
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.tf_listener = tf.TransformListener()
        self.calculate_camera_sprayer_transform()
        '''
    
    # moves robot acroos entire map
    # assumes crop planted in y direction so traverses are in y direction
    def scan_map(self):
        self.traverse_spacing = 1     #spacing in metres between traverses
        self.margin = 1               #distance from boundary of the turning points at end of a traverse (metres)

    # move to cell (0,0), ie the'bottom left' corner, allowing for margins
        self.next_x = self.grid_origin_x + self.margin
        self.next_y = self.grid_origin_y + self.margin
        self.move_to_goal (self.next_x, self.next_y)
        self.current_x = self.next_x
        self.current_y = self.next_y


        while True:                   
        # make a traverse   
            self.next_x = self.current_x 
            self.next_y = self.current_y + (self.grid_height - self.margin)
            self.move_to_goal (self.next_x, self.next_y)
            self.current_x = self.next_x
            self.current_y = self.next_y

        #move across    
            self.next_x = self.current_x + self.traverse_spacing
            if self.next_x > (self.grid_width - self.margin):
                break                                       #gone all the way across
            self.next_y
            
            self.current_y
            self.move_to_goal (self.next_x, self.next_y
            )
            self.current_x = self.next_x
            self.current_y = self.next_y

        # traverse  back 
            self.next_x = self.current_x 
            self.next_y
            
            self.current_y  - (self.height - self.margin)
            self.move_to_goal (self.next_x, self.next_y
            )
            self.current_x = self.next_x
            self.current_y = self.next_y

        #move across to complete one cycle up and down
            self.next_x = self.current_x + self.traverse_spacing
            if self.next_x > (self.grid_width - self.margin):
                break                                       #gone all the way across
            self.next_y
            
            self.current_y
            self.move_to_goal (self.next_x, self.next_y
            )
            self.current_x = self.next_x
            self.current_y = self.next_y



    # The following function is adapted from move_base action_client example at 
    # hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
    def move_to_goal (self, x_goal, y_goal):
        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.w = 1.0
        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()

'''

    def calculate_camera_sprayer_transform(self):
    # calculate offset from camera to sprayer so that sprayer can be moved over target

        self.tf_listener.waitForTransform('thorvald_001/kinect2_rgb_optical_frame', 'thorvald_001/sprayer', rospy.Time.now(), rospy.Duration(4.0))
        self.camera_to_sprayer = self.tf_listener.lookupTransform('thorvald_001/kinect2_rgb_optical_frame', 'thorvald_001/sprayer', rospy.Time())
        self.x_move = self.camera_to_sprayer[0][0]
        self.y_move = self.camera_to_sprayer[0][1]

    def camera_callback(self,data):
        if self.mapping:
            return                  #dont need to process camera image or spray when in mapping state
        try:
            image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
           
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
        #cv2.imshow('Green',res_g)
        #cv2.imshow('original', image)

        #Detect any blobs in green iamge
        # Set up the detector with default parameters.

        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(mask_g)
        # print keypoints #DEBUG
	    # Draw detected blobs as red circles.
	    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        res_g_with_blobs = cv2.drawKeypoints(res_g, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # spray if blob (weed) found TODO move robot to spray in correct place
        if keypoints:
            # TODO Cancel current movebase and push goal to stack
            self.move_over_and_spray()

        # Show blobs
        #cv2.imshow("Keypoints", res_g_with_blobs)
        cv2.waitKey(1)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            self.tf_listener.waitForTransform('thorvald_001/odom', 'thorvald_001/base_link', rospy.Time.now(), rospy.Duration(4.0))
            (trans, rot)  = self.tf_listener.lookupTransform('thorvald_001/odom', 'thorvald_001/base_link', rospy.Time())
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans))   

    def move_to_spray(self, x, y):
        self.pause_movement(1)  #DEBUG
        position = Point()
        move_cmd = Twist()
        goal_distance_x = x 
        goal_distance_y = y 
        print ('goal_distance_x:')#DEBUG
        print  goal_distance_x  #DEBUG
        print ('goal_distance_y:')#DEBUG
        print  goal_distance_y  #DEBUG
        # Set the movement command to forward motion
        move_cmd.linear.x = self.linear_speed
        move_cmd.linear.y = self.linear_speed    

        # Get the starting position values     
        position = self.get_odom()
                        
        x_start = position.x
        y_start = position.y

        distance_x = 0
        distance_y = 0
        move_cmd.linear.x = self.linear_speed
        move_cmd.linear.y = self.linear_speed  
        #move in x_direction
        position = self.get_odom()                
        x_start = position.x
        while distance_x < goal_distance_x :
            if self.obstacle_present:   #Abort motion if obstacle present and return False to prevent spray
                print 'spray move aborted'
                return False
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
            # Get the new position
            position = self.get_odom()
            distance_x = math.fabs(position.x - x_start)

            print ('x distance:' + str(distance_x))       #DEBUG
        
        #move in y_direction
        position = self.get_odom()                
        y_start = position.y
        while distance_y < goal_distance_y :
            if self.obstacle_present:   #Abort motion if obstacle present and return False to prevent spray
                print 'spray move aborted'
                return False
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
            # Get the new position
            position = self.get_odom()
            distance_y = math.fabs(position.y - y_start)


            print ('y distance:' + str(distance_y))  #DEBUG

        return True
    def move_over_and_spray(self):
        #If move to correct spray position was successful, then spray
        if self.move_to_spray(self.x_move, self.y_move):   
            self.pause_movement(1)  #DEBUG
            rospy.wait_for_service('/thorvald_001/spray')
            try:
                spray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
                spray()
            except rospy.ServiceException as e:
                print("Spray service call failed: %s"%e)

    def pause_movement(self, pause_time):  #TODO ?pause routine not working
        self.movement_paused = True     #set flag to stop laser callback moving robot
        move_cmd = Twist()
        move_cmd.linear.x = 0
        move_cmd.linear.y = 0
        self.cmd_vel_pub.publish(move_cmd)
        rospy.sleep(pause_time)
        self.movement_paused = False     #pause is over        
'''
# Main Code
if __name__ == '__main__':
    # CREATE MAP
    rospy.init_node('mapping', anonymous=True)
    try:
        m = mapping()
        '''
        m.map()
        #wait while mapping phase takes place
        while not(m.mapping_complete):
            time.sleep(1)
        '''
        # TRAVERSE FIELD, ID WEEDS AND SPRAY
        #rospy.init_node('weeding', anonymous=True)
        w = weeding()        
        move_result = w.move_to_goal(100,100)
        if move_result:
            rospy.loginfo("Goal reached")



    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass





 
