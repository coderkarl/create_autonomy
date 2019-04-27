#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16, Empty

import tf
from geometry_msgs.msg import Quaternion, Point, Pose, PoseStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatusArray

import time
import sys

class WaypointManager():
    def __init__(self):        
        self.tf_listener = tf.TransformListener()
        
        self.tf_offset = rospy.Duration(0.0)
        
        rospy.init_node('waypoint_manager')
        
        print('Initializing Waypoint Manager.')
        
        self.goal = PoseStamped()
        self.goal.header.frame_id = "odom"
        self.goal.pose.orientation.w = 1.0
        
        self.cur_wp = PoseStamped()
        self.cur_wp.header.frame_id = "odom"
        self.cur_wp.pose.orientation.w = 1.0
        
        self.map_wp = PoseStamped()
        self.map_wp.header.frame_id = "odom"
        self.map_wp.pose.orientation.w = 1.0
        
        waypoints  = np.array([
          [0, 0],
          [1.3, -0.6],
          [4.1, 0.0],
          ])
          #[6.6, 1.2]
          #])
           
        self.num_waypoints = len(waypoints)
        
        [x0,y0] = waypoints[0]
        self.wp_map = np.zeros(waypoints.shape)
        self.wp_map[:,0] = waypoints[:,0]-x0
        self.wp_map[:,1] = waypoints[:,1]-y0
        
        self.wp_k = 1
        
        self.time_since_cone = rospy.Time.now()
        self.found_cone = False
        
        self.goal_dx = 0.0
        self.goal_dy = 0.0
        self.status_received = False
        self.path_found = False
        self.laser_found_cone_count = 0
        self.cam_found_cone_count = 0
        
        self.cur_wp_in_base_pub = rospy.Publisher('wp_goal_in_base', PoseStamped, queue_size=5)
        self.wp_pub = rospy.Publisher('wp_goal', PoseStamped, queue_size=2)
        self.wp_cone_in_laser = rospy.Publisher('expected_cone_in_laser', PoseStamped, queue_size=5) # used to filter where to check for cone circles in laser scan
        self.laser_cone_in_odom_pub = rospy.Publisher('laser_cone_in_odom', PoseStamped, queue_size=2) # published if raw laser cone is acceptable, stop and turn toward this
        self.found_cone_pub = rospy.Publisher('found_cone', Int16, queue_size=2)
        self.found_laser_cone_pub = rospy.Publisher('found_laser_cone', Int16, queue_size=2)
        self.next_wp_pub = rospy.Publisher('next_wp', Empty, queue_size = 1)
        
        time.sleep(1.0)
        #self.tf_listener.waitForTransform("laser", "odom", rospy.Time(), rospy.Duration(4.0))
        self.update_waypoint()
        self.last_update_time = rospy.Time.now()
        
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.clicked_goal_callback, queue_size = 2)
        rospy.Subscriber('raw_cone_pose', PoseStamped, self.raw_cone_callback, queue_size=2)
        rospy.Subscriber('laser_cone_pose', PoseStamped, self.laser_cone_callback, queue_size=10)
        rospy.Subscriber('obs_cone_pose', PoseStamped, self.obs_cone_callback, queue_size=2)
        #rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        #rospy.Subscriber('/move_base/status', GoalStatusArray, self.plan_status_callback, queue_size=1)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)
        rospy.Subscriber('/touched_cone', Empty, self.touched_cone_callback, queue_size = 1)
    
    def plan_status_callback(self,msg):
        if(len(msg.status_list) > 0):
            goalStatus = msg.status_list[-1]
            status_id = goalStatus.status
            if(status_id >= 4 or status_id == 1):
                self.path_found = False
            else:
                self.path_found = True
            self.status_received = True
    def path_callback(self, data):
        if(len(data.poses) > 0):
            self.path_found = True
        else:
            self.path_found = False
        self.status_received = True
    
    def touched_cone_callback(self, msg):
        now = rospy.Time.now()
        if( (now-self.last_update_time).to_sec() > 1.0):
            self.update_waypoint()
        self.last_update_time = now
    
    def update_waypoint(self):
        print "Update Waypoint"
        if(self.wp_k < self.num_waypoints):
            self.map_wp.header.stamp = rospy.Time.now()
            self.map_wp.pose.position.x = self.wp_map[self.wp_k,0]
            self.map_wp.pose.position.y = self.wp_map[self.wp_k,1]
            print "map_wp"
            print self.map_wp
            p_in_odom = self.xy_in_odom(self.map_wp)
            print "p in odom"
            print p_in_odom
            if(p_in_odom):
                self.cur_wp = p_in_odom
                self.wp_k += 1
                print "wp_goal published"
                self.wp_pub.publish(self.cur_wp)
                msg = Empty()
                self.next_wp_pub.publish(msg)
            if(self.wp_k >= self.num_waypoints):
                self.wp_k = 1
        self.laser_found_cone_count = 0
        self.cam_found_cone_count = 0
            
        
    def clicked_goal_callback(self,data):
        print "Clicked Goal Callback"
        if(data.header.frame_id != "odom"):
            p_in_odom = self.xy_in_odom(data)
            if(p_in_odom):
                self.cur_wp = p_in_odom
        else:
            self.cur_wp = data
        
        self.goal = self.cur_wp
        self.map_wp = self.cur_wp # Allows the raw cone to check against clicked goal vs. hard-coded waypoints
        
        #self.goal.pose.position.x = self.cur_wp.pose.position.x + self.goal_dx
        #self.goal.pose.position.y = self.cur_wp.pose.position.y + self.goal_dy
        
        xdir = [0,  0, 1, 1,  1, -1, -1, -1]
        ydir = [1, -1, 0, 1, -1,  0,  1, -1]
        #xdir = [0.0,  0.0, 1.0, -1.0]
        #ydir = [1.0, -1.0, 0.0,  0.0]
        dir_ind = 0
        ndir = 8
        r = 0.1
        
        self.wp_pub.publish(self.goal)
        
        #TEMP LOOP TO TEST OUT CUSTOM GOAL NEAR OBSTACLE
        #~ self.status_received = False
        #~ while(not self.status_received):
            #~ print('waiting')
            #~ time.sleep(0.05)
        #~ while(not self.path_found ):
            #~ self.goal_dx = r*xdir[dir_ind]
            #~ self.goal_dy = r*ydir[dir_ind]
            #~ self.goal = self.cur_wp
            #~ self.goal.pose.position.x = self.cur_wp.pose.position.x + self.goal_dx
            #~ self.goal.pose.position.y = self.cur_wp.pose.position.y + self.goal_dy
            #~ self.wp_pub.publish(self.goal)
            #~ self.status_received = False
            #~ while(not self.status_received):
                #~ print('waiting')
                #~ time.sleep(0.05)
            #~ dir_ind = (dir_ind+1)%ndir
            #~ if(dir_ind == 0):
                #~ r += 0.1
            
    def raw_cone_callback(self, data):
        #print "Raw Cone Callback"
        p_in_odom = self.xy_in_odom(data)
        if(p_in_odom):
            dx = self.map_wp.pose.position.x - p_in_odom.pose.position.x
            dy = self.map_wp.pose.position.y - p_in_odom.pose.position.y
            dist_sqd = dx*dx + dy*dy
            print("dist sqd: ", dist_sqd)
            if(dist_sqd < 0.3): #0.3
                self.cam_found_cone_count += 1
                if(self.cam_found_cone_count > 1): # and self.laser_found_cone_count > 0):
                    self.cam_found_cone_count = 0
                    print("publish cone wp_goal")
                    self.cur_wp = p_in_odom
                    self.wp_pub.publish(self.cur_wp)
                    if(data.pose.position.x < 0.7 and abs(data.pose.position.y) < 0.3): #Do NOT go straight to cone unless we are close, this should keep us on the global path around obstacles, even if we see thru obs
                        msg = Int16() #NOTE, this is not used now by local planner
                        msg.data = 1
                        self.found_cone_pub.publish(msg)
                        self.time_since_cone = rospy.Time.now()
                    
    def laser_cone_callback(self, data):
        #print "Raw Cone Callback"
        p_in_odom = self.xy_in_odom(data)
        if(p_in_odom):
            dx = self.map_wp.pose.position.x - p_in_odom.pose.position.x
            dy = self.map_wp.pose.position.y - p_in_odom.pose.position.y
            dist_sqd = dx*dx + dy*dy
            print("dist sqd: ", dist_sqd)
            if(dist_sqd < 0.3): #use 0.5, 2.0 for testing
                self.laser_found_cone_count += 1
                if(self.laser_found_cone_count > 1):
                    #self.laser_found_cone_count = 0
                    if(data.pose.position.x < 0.7):
                        print("publish laser cone near wp_goal")
                        self.laser_cone_in_odom_pub.publish(p_in_odom) #consider publishing inside if above
                        msg = Int16()
                        msg.data = 1
                        self.found_laser_cone_pub.publish(msg)
                        self.time_since_cone = rospy.Time.now()
    
    def obs_cone_callback(self, data):
        self.cur_wp = data
        self.wp_pub.publish(self.cur_wp)
        msg = Int16()
        msg.data = 1
        self.found_cone_pub.publish(msg)
        #print("Waypoint Manager Received nearest obstacle to cone")
    
    def xy_in_odom(self, poseStamped):
        src_frame = poseStamped.header.frame_id
        p_in_odom = None
        count = 0
        try:
            self.tf_listener.waitForTransform("odom", src_frame, rospy.Time.now(), rospy.Duration(1.0))
            p_in_odom = self.tf_listener.transformPose("odom", poseStamped)
        except:
            rospy.logwarn("Error converting to odom frame")
            p_in_odom = None
        
        return p_in_odom
        
    def xy_in_laser(self, poseStamped):
        src_frame = poseStamped.header.frame_id
        p_in_laser = None
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform("laser", src_frame, now, rospy.Duration(1.0))
            p_in_laser = self.tf_listener.transformPose("laser", poseStamped)
            #self.tf_listener.waitForTransform("base_link", src_frame, now, rospy.Duration(1.0))
            #p_in_base = self.tf_listener.transformPose("base_link", poseStamped)
        except:
            rospy.logwarn("Error converting to laser frame")
            p_in_laser = None
            self.tf_offset = rospy.Duration(0.1)
        
        return p_in_laser
        
    def xy_in_base(self, poseStamped):
        src_frame = poseStamped.header.frame_id
        p_in_base = None
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform("base_link", src_frame, now, rospy.Duration(1.0))
            p_in_base = self.tf_listener.transformPose("base_link", poseStamped)
        except:
            rospy.logwarn("Error converting to base_link frame")
            p_in_base = None
        
        return p_in_base
        
    def execute(self):
        self.cur_wp.header.stamp = rospy.Time.now()
        self.map_wp.header.stamp = rospy.Time.now()
        if(self.found_cone):
            if( (self.time_since_cone - rospy.Time.now()).to_sec() > 0.5 ): #0.5 for camera?, 10.0 for laser cone finder
                self.found_cone = False
                msg = Int16()
                msg.data = 0
                self.found_cone_pub.publish(msg)
        expected_cone_in_laser = self.xy_in_laser(self.map_wp)
        if(expected_cone_in_laser):
            self.wp_cone_in_laser.publish(expected_cone_in_laser)
        cone_in_base = self.xy_in_base(self.cur_wp)
        if(cone_in_base):
            self.cur_wp_in_base_pub.publish(cone_in_base)

if __name__ == '__main__':
    try:
        wp_man = WaypointManager()
        print("Starting Waypoint Manager")

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            wp_man.execute()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
