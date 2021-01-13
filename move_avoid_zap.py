#! /usr/bin/env python
from __future__ import division   # force / operator to return float instead of int
import rospy
import math
import tf
import time
import geometry_msgs.msg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
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
        
        #all distances in metres, just default starting values
        self.obstacle_x = 10.0          
        self.obstacle_y = 10.0
        self.angle_to_obstacle = 0.0
        self.distance_to_obstacle = 10.0

        self.clearance_from_obstacle = 3  #robot turns at this distance when mapping (metres)

        self.linear_speed = 0.7         #robot speed when mapping, metres per second
        self.angular_speed = 0.7        #robot rotation rate when mapping,radians per second
        self.mapping_start_time = time.time()
        self.mapping_complete = False   #flag that can be interpreted to find out whether mapping is complete
        self.mapping_duration = 45      #duration of mapping stage in seconds
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
        if self.clearance_right < self.clearance_from_obstacle :  
            self.rotate('left')
        
        #does it need to turn right?
        elif self.clearance_left <self.clearance_from_obstacle :
            self.rotate('right')

        else:  #free to move forward
            self.motion.linear.x = self.linear_speed 
            # introduce a turn sometimes to give a random element to motion
            # dont use random function as that will average to zero over callbacks
            time_in_secs_mod_ten = 0     #just to avoid undeclared variable syntax warning
            time_in_secs_mod_ten = int(round(time.time())) % 10
            angular_vel = 0.0
            if (time_in_secs_mod_ten < 5):
                angular_vel = (time_in_secs_mod_ten-2)/5.0


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
        self.min_distance = 10.0  #default starting values
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
        self.min_distance = 10.0  #default starting values
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

        #mapping settings

     
        map_meta_data = rospy.wait_for_message('/map_metadata',MapMetaData ) 
    
        #convert occupancy grid to an array and extract metadata
        occupancy_grid_message = rospy.wait_for_message('/map',OccupancyGrid )
        self.occupancy_array = np.asarray(occupancy_grid_message.data, dtype=np.int8).reshape(map_meta_data.height, map_meta_data.width)
        self.map_width = self.occupancy_array.shape[0]
        self.map_height = self.occupancy_array.shape[1]
        self.map_resolution = map_meta_data.resolution  # metres per cell of occupancy grid
        self.world_origin_x = map_meta_data.origin.position.x
        self.world_origin_y = map_meta_data.origin.position.y

        print  'origin: ' + str(self.world_origin_x)+ ',' + str(self.world_origin_y)  #DEBUG
        print  'resolution: ' + str(self.map_resolution) #DEBUG
        print  'map_width: ' + str(self.map_width) #DEBUG
        print  'map_height: ' + str(self.map_height) #DEBUG

             
        self.world_width = self.map_width * self.map_resolution  #width is x direction, convert width from cell count to metres
        self.world_height = self.map_height * self.map_resolution #height is y direction
        print  'world_width: ' + str(self.world_width) #DEBUG
        print  'world_height: ' + str(self.world_height) #DEBUG
        
        #convert occupancy grid to an array
        occupancy_grid_message = rospy.wait_for_message('/map',OccupancyGrid )
        self.occupancy_array = np.asarray(occupancy_grid_message.data, dtype=np.int8).reshape(self.map_height, self.map_width)
        print 'occupancy array:'
        print self.occupancy_array


        self.move_base_client = actionlib.SimpleActionClient('/move_base/',MoveBaseAction)
       
       
        # Camera settings
        self.image_sub = rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.camera_interval = 0.2  #time between processing images
        self.camera_timer_start_time = time.time()
        self.tf_listener = tf.TransformListener()

        # traversing settings
        self.traverse_spacing = 1.0     #spacing in metres between traverses (float)
        self.margin = 3.0              #distance from boundary of the turning points at end of a traverse (metres, float)
        self.obs_breadth = 20              #max distance to detect obstacles either side of path (map cells, integer)


        # Get distance of sprayer behind camera
        self.tf_listener.waitForTransform('thorvald_001/kinect2_rgb_optical_frame', 'thorvald_001/sprayer', rospy.Time.now(), rospy.Duration(4.0))
        self.camera_to_sprayer = self.tf_listener.lookupTransform('thorvald_001/kinect2_rgb_optical_frame', 'thorvald_001/sprayer', rospy.Time())
        self.camera_sprayer_distance = self.camera_to_sprayer[0][0]

        #spraying settings
        self.weed_location_queue = []
        self.pose_sub = rospy.Subscriber("/Pose",Pose,self.update_pose)
        self.spraying_enabled = False
        self.weed_location_tolerance = 0.1  #allowable error in sprayer position relative to weed

        self.rate = rospy.Rate(3)



    
    # moves robot acroos entire map
    # assumes crop planted in y direction so traverses are in y direction
    def traverse_field(self):
        self.traversing_done = False    
        self.traverse_up = False      #going downwards
        self.spraying_enabled = False #don't spray until traversing started

        # origin is (0,0), this is the bottom right corner in the initial Gazebo view
        # +ve x is upwards in this Gazebo view
        # +ve y is leftwards in this Gazebo view
        # move to cell (maximum x, maximum y), but inside by margin
        #  ie the'top left' corner in the initial Gazebo view
        #self.next_x = self.world_origin_x + self.world_width - self.margin
        #self.next_y = self.world_origin_y + self.world_height - self.margin  

        
        #DEBUG use this alternative code to move straight to centre so seeing weeds quickly for debug
        self.next_x = self.world_origin_x + self.world_width - self.margin
        self.next_y = int(self.world_origin_y + self.world_height * 0.6)
       

        self.move_to_goal (self.next_x, self.next_y)



        # do the traversing
        while self.traversing_done == False:  
            self.traverse_down_and_across()     
            self.traverse_up_and_across()

    def traverse_down_and_across(self):
    # traverse  back down (-ve x direction)
        print 'traverse down started:'  #DEBUG
        self.traverse_up = False
        self.spraying_enabled = True
            # update position, (on assumption that previous goal was reached)
        self.current_x = self.next_x  
        self.current_y = self.next_y
        self.next_x = self.calc_clear_down_distance()  # set goal at margin before first obstacle
        self.next_y = self.current_y 
        self.current_x = self.next_x
        self.current_y = self.next_y
        self.move_to_goal (self.next_x, self.next_y)

        
        #move across   
        print 'traverse across after down started:'  #DEBUG 
        self.spraying_enabled = False     #don't spray while moving to a new row
        # update current position, (on assumption that previous goal was reached)
        self.current_x = self.next_x
        self.current_y = self.next_y
        self.next_y = self.current_y - self.traverse_spacing
        if self.next_y < self.world_origin_y + self.margin:
            self.traversing_done = True                                      #gone all the way across
        self.move_to_goal (self.next_x, self.next_y)



    def traverse_up_and_across(self):
        print 'traverse_up started:'  #DEBUG
        # make a traverse up (+ve x direction)
        self.traverse_up = True
        self.spraying_enabled = True
        # update position, (on assumption that previous goal was reached)
        self.current_x = self.next_x  
        self.current_y = self.next_y
        self.next_x = self.calc_clear_up_distance()  # set goal at margin before first obstacle
        self.next_y = self.current_y                                        # no sideways movement
        self.move_to_goal (self.next_x, self.next_y)

        
        #move across   
        print 'traverse across after up started:'  #DEBUG 
        self.spraying_enabled = False     #don't spray while moving to a new row
        # update current position, (on assumption that previous goal was reached)
        self.current_x = self.next_x
        self.current_y = self.next_y
        self.next_y = self.current_y - self.traverse_spacing
        if self.next_y < self.world_origin_y + self.margin:
            self.traversing_done = True                                      #gone all the way across
        self.move_to_goal (self.next_x, self.next_y)



    def calc_clear_down_distance(self):
        # convert world postion to map co-ordinates
        current_map_x = self.world_to_map_x(self.current_x)
        current_map_y = self.world_to_map_y(self.current_y)
        print 'clear_down entered'
        print 'world x:' + str(self.current_x) + ',' + 'world y:' + str(self.current_y)
        print 'map x:' + str(current_map_x) +','+ 'map y:' + str(current_map_y)
        
        for map_x in range(current_map_x,  0, -1):   #scan down from current postion to find obstacle
            #scan either side of path
            for map_y in range(current_map_y - self.obs_breadth, current_map_y + self.obs_breadth):
                if self.occupancy_array[map_x, map_y] == 100:   #100 = occupied
                    obstacle_in_world_x = self.map_to_world_x(map_x)
                    print 'obstacle found at map_x = ' + str(map_x)  + 'world_x = ' + str(obstacle_in_world_x)
                    safe_x_in_world = float(obstacle_in_world_x + self.margin)
                    return safe_x_in_world  #convert from occupancy grid data points to meteres
        #deal with case where no obstacle found
        bottom_world_safe_x =  float(self.map_to_world_x(0) +self.margin)
        print 'no obstacle found: safe world_x = ' + str(bottom_world_safe_x) 
        return bottom_world_safe_x   #deal with case where no obstacle found


    def calc_clear_up_distance(self):
        # convert world postion to map co-ordinates
        current_map_x = self.world_to_map_x(self.current_x)
        current_map_y = self.world_to_map_y(self.current_y)
        print 'up entered'
        print 'world x:' + str(self.current_x) + ',' + 'world y:' + str(self.current_y)
        print 'map x:' + str(current_map_x) +','+ 'map y:' + str(current_map_y)
        for map_x in range(current_map_x, self.map_width-1): #scan up from current postion to find obstacle
            #scan either side of path
            for map_y in range(current_map_y - self.obs_breadth, current_map_y + self.obs_breadth):
                if self.occupancy_array[map_x, map_y] == 100:   #100 = occupied
                    obstacle_in_world_x = self.map_to_world_x(map_x)       
                    print 'obstacle found at map_x = ' + str(map_x)  + '  world_x = ' + str(obstacle_in_world_x)                     
                    return float(obstacle_in_world_x - self.margin)  #move base wants goals as floats, so make sure
        #deal with case where no obstacle found
        top_world_safe_x = float(self.map_to_world_x(self.map_width-1) - self.margin)
        return top_world_safe_x  #deal with case where no obstacle found

    # helper functions to convert between occupancy map and world
    def map_to_world_x(self, map_x):
            return self.map_resolution * map_x + self.world_origin_x
    def map_to_world_y(self, map_y):
            return self.map_resolution * map_y + self.world_origin_y

    def world_to_map_x(self, world_x):
            return  int((world_x - self.world_origin_x)/self.map_resolution)
    def world_to_map_y(self, world_y):
            return  int((world_y - self.world_origin_y)/self.map_resolution)
    
    
    def move_to_goal (self, x_goal, y_goal):
        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal

        #rotate 90 deg in appropriate sense, to be ready to head off in correct direction 
        goal.target_pose.pose.orientation.z = 0.7071
        #if self.traverse_up == True:  #TODO debug rotation directions
        goal.target_pose.pose.orientation.w = -0.7071       
        #else:     #TODO debug rotation directions
         #   goal.target_pose.pose.orientation.w = +0.7071   #TODO debug rotation directions

        print 'x goal: '  + str(x_goal) + '   y goal: ' + str(y_goal)  #DEBUG
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()



    
    
    
    def update_pose(self,msg):
            self.pose_x = msg.point.x
            self.pose_y = msg.point.y


            
    def camera_callback(self,data):

        # control how often camera image is processed
        if time.time() < self.camera_timer_start_time + self.camera_interval:   
            return                                   #do nothing if camera interval not up
        else:
            self.camera_timer_start_time = time.time()  #else reset timer and carry on to process image
        

        # convert image message to OpenCV format
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
        #cv2.imshow('Green Mask',mask_g) #DEBUG
        #colured post-processed image
        res_g = cv2.bitwise_and(image,image, mask= mask_g)
        #cv2.imshow('Green',res_g)
        #cv2.imshow('original', image)

        #Detect any blobs in green iamge
        # Set up the detector with default parameters.

        detector = cv2.SimpleBlobDetector_create()
        keypoints = detector.detect(mask_g)
	    # Draw detected blobs as red circles.
	    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        res_g_with_blobs = cv2.drawKeypoints(res_g, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
       
        # Show blobs DEBUG
        cv2.imshow("Keypoints", res_g_with_blobs) #DEBUG
        cv2.waitKey(1)
       
       # get current pose of base_link in map
        now = rospy.Time.now()
        self.tf_listener.waitForTransform('/map', '/thorvald_001/base_link', now, rospy.Duration(5))
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/thorvald_001/base_link', now)
        self.x_pose = trans[0]
        self.y_pose = trans[1]

        #clean up weed queue by removing locations that have already been passed, these won't be sprayed

        length = len(self.weed_location_queue)
        if length > 0:    #only clean up if there are weeds in the queue
            if self.traverse_up:          #going upwards 
                length = len(self.weed_location_queue)
                for i in range(length):
                    # handle exception caused by queue being modified by another callback thread
                    try:
                        if self.weed_location_queue[i] <= self.x_pose + self.weed_location_tolerance:
                            self.weed_location_queue.pop(i)
                    except IndexError:
                        break
            
            else:       #going downwards
                for i in range(len(self.weed_location_queue)):
                    try:
                        if self.weed_location_queue[i] >= self.x_pose + self.weed_location_tolerance:
                            self.weed_location_queue.pop(i)
                    except IndexError:
                        break


        # weed found ...
        if (keypoints and self.spraying_enabled):
            # add weed location to to-spray list, taking into account direction of motion
            if self.traverse_up:
                self.weed_location_queue.append(self.x_pose + self.camera_sprayer_distance)
            else:
                self.weed_location_queue.append(self.x_pose - self.camera_sprayer_distance)

        # check whether current base_link position is such that sprayer is over a weed
        # looking at the head of the queue of weed locations as that should be the next weed the robot passes over
        # TODO  Dont spray if y error too large (because avoiding obstacle)
        if self.weed_location_queue:                                #if queue has weed_locations in it
            if abs(self.y_pose - self.next_y) < 0.1:         #only spray if cross-track error small
                if abs(self.weed_location_queue[0]- self.x_pose) < self.weed_location_tolerance:    #and sprayer is within 10cm of weed TODO reduce this if possible
                    self.spray()
                    self.weed_location_queue.pop(0)          # remove location from the queue now that it has been sprayed


    def spray(self):
        #trigger spray service
            rospy.wait_for_service('/thorvald_001/spray')
            try:
                spray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
                spray()
            except rospy.ServiceException as e:
                print("Spray service call failed: %s"%e)
     

# Main Code
if __name__ == '__main__':
    # CREATE MAP
    rospy.init_node('mapping', anonymous=True)
    try:
        
        m = mapping()
        m.map()
        #wait while mapping phase takes place
        while not(m.mapping_complete):
            time.sleep(1)      #Just check periodically to reduce computational load

        # TRAVERSE FIELD, ID WEEDS AND SPRAY
        w = weeding()     
        w.traverse_field()   


    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass





 
