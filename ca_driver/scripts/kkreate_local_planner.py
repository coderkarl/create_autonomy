#!/usr/bin/env python

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int16, Bool, Empty
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ca_msgs.msg import Bumper, PlaySong
import tf
from sensor_msgs.msg import LaserScan #force robot to at least slow down, stop, backup when close to obstacles

from DiffDriveController import DiffDriveController

PATH_STATE = 1
SEEKING_CONE_STATE = 2
SPINNING_STATE = 3
TOUCHED_CONE_STATE = 4
STOPPED_STATE = 0

class PathController():
    def __init__(self):

        self.vx = 0.0
        self.cum_err = 0
        
        MAX_SPEED = 0.3
        MAX_OMEGA = 1.0
        self.diff_drive_controller = DiffDriveController(MAX_SPEED, MAX_OMEGA)
        
        x_i = 0.
        y_i = 0.
        theta_i = -99.
        self.state = np.array([[x_i], [y_i], [theta_i] ])
        self.goal = np.array([[x_i], [y_i] ])
        self.goal_reached = True
        
        self.waypoints = []
        self.path_debounce_count = 0
        self.v = 0.0
        self.w = 0.0
        
        self.reverse_flag = False
        
        self.tf_listener = tf.TransformListener()
        
        self.found_cone = False
        self.local_cone_x = 1.0 # None
        self.local_cone_y = 0.0 # None
        self.bumped_cone = False
        self.ir_sensors = [0, 0, 0, 0, 0, 0]
        self.bump_count = 0
        self.stopped = False
        self.touched_cone = False
        
        rospy.init_node('path_controller')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.play_song_pub = rospy.Publisher('play_song', PlaySong, queue_size=1)
        self.touched_cone_pub = rospy.Publisher('touched_cone', Empty, queue_size=1)
        self.status_pub = rospy.Publisher('kstate', Int16, queue_size = 10)
        
        #rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size = 1)
        rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.path_callback, queue_size = 1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        rospy.Subscriber('found_cone', Int16, self.found_cone_callback, queue_size = 1)
        rospy.Subscriber('raw_cone_pose', PoseStamped, self.raw_cone_callback, queue_size=2)
        #rospy.Subscriber('wp_goal', PoseStamped, self.goal_callback, queue_size=2)
        rospy.Subscriber('bumper', Bumper, self.bumper_callback, queue_size=2)
        rospy.Subscriber('clean_button', Empty, self.clean_button_callback, queue_size=2)
        rospy.Subscriber('next_wp', Empty, self.next_wp_callback, queue_size = 2)
        
        reset_found_cone_time = rospy.Time.now()
        
        self.status = PATH_STATE
    
    def found_cone_callback(self, msg):
        if(msg.data > 0):
            self.found_cone = True
            self.reset_found_cone_time = rospy.Time.now()
        else:
            self.found_cone = False
            
    def next_wp_callback(self,msg):
        self.touched_cone = False
        
            
    def clean_button_callback(self,msg):
        self.found_cone = True
        self.stopped = True
        self.status = STOPPED_STATE
        
    def scan_callback(self, data):
        ranges = data.ranges
        amin = data.angle_min
        amax = data.angle_max
        step = data.angle_increment
        nAng = (amax-amin)/step
        FOV_int = int(45*3.14/180./step)
        ind1 = int(nAng/2 - FOV_int/2)
        ind2 = int(nAng/2 + FOV_int/2)
        ind1 = 0
        ind2 = 10
        min_range1 = min(ranges[ind1:ind2])
        ind1 = 350
        ind2 = 359
        min_range2 = min(ranges[ind1:ind2])
        min_range = min([min_range1, min_range2])
        if(not self.found_cone and min_range < 0.36):
            self.reverse_flag = True
        elif(self.reverse_flag and min_range > 0.45):
            self.reverse_flag = False
            
        # Loop thru ranges to find cone candidates
        #~ cone_xy_list = []
        #~ dist_list = []
        #~ dAngle_list = []
        #~ init_k = -15
        #~ k = init_k
        #~ n = 0
        #~ while(k < len(ranges)):
            #~ r = ranges[k]
            #~ a = amin + k*step
            #~ x = r*math.cos(a+3.14) #NOTE, hardcoded tf from laser to base_link
            #~ y = r*math.sin(a+3.14) #NOTE, hardcoded tf from laser to base_link
            #~ if(n > 0):
                #~ dx = x-x1
                #~ dy = y-y1
                #~ dist = math.sqrt(dx*dx + dy*dy)
                #~ angle = math.atan2(dy,dx)
                #~ if(n > 1):
                    #~ dAngle = angle - angle1
                    #~ if(dAngle > 3.14):
                        #~ dAngle -= 6.28
                    #~ elif(dAngle <= -3.14):
                        #~ dAngle += 6.28
                    #~ #if(abs(a) > 2.8 and 0.5 < r and r < 1.1):
                    #~ #    print("dist, dAngle, x, y: ", dist, dAngle, x, y)
                    #~ if(-0.6 < dAngle and dAngle < -0.3):
                        #~ dAngle_list.append(dAngle)
                        #~ dist_list.append(dist)
                    #~ else:
                        #~ if(len(dist_list) > 2):
                            #~ cone_radius = sum(dist_list)/sum(dAngle_list)
                            #~ if(-0.15 < cone_radius and cone_radius < -0.05):
                                #~ print("cone local xy, r: ", x1, y1, cone_radius)
                                #~ print("dAngles: ", dAngle_list)
                                #~ print("dists: ", dist_list)
                        #~ dAngle_list = []
                        #~ dist_list = []
                    #~ #end
                #~ #end
                #~ dist1 = dist
                #~ angle1 = angle
            #~ #end
                    
            #~ x1 = x
            #~ y1 = y
            #~ k += 2
            #~ n += 1
        #~ #end while
            
    
    def odom_callback(self,odom):
        # This odom pose is in the odom frame, we want the pose in the map frame
        #   Now done in execute_plan() at a fixed rate using a tf.TransformListener()
        #~ quat = odom.pose.pose.orientation
        #~ quat_list = [quat.x, quat.y, quat.z, quat.w]
        #~ (roll, pitch, yaw) = euler_from_quaternion (quat_list)
        
        #~ self.state[0] = odom.pose.pose.position.x + 5.0
        #~ self.state[1] = odom.pose.pose.position.y
        #~ self.state[2] = yaw
        
        self.vx = odom.twist.twist.linear.x
    
    def path_callback(self,data):
        poses = data.poses
        nPose = len(poses)
        self.waypoints = []
        if(nPose > 0):
            self.path_debounce_count = 0
            wp = np.zeros([2,1])
            if(nPose == 1):
                init = 0
                wp_step = 1
            else:
                # approximate actual path length by breaking into 4 sub-linesegments
                path_dist = 0
                nSegments = 4
                stride = max(int(nPose/nSegments), 1)
                ind1 = 0
                ind2 = 0
                done_flag = False
                k = 0
                while(k <= nSegments):
                    ind2 += stride
                    if(ind2 >= nPose):
                        ind2 = nPose -1
                        done_flag = True
                    dx = poses[ind2].pose.position.x - poses[ind1].pose.position.x
                    dy = poses[ind2].pose.position.y - poses[ind1].pose.position.y
                    path_dist += np.sqrt(dx**2 + dy**2)
                    if(done_flag):
                        print "reached final point for path_dist"
                        break
                    ind1 = ind2
                    k += 1
                
                step_size_meters = 0.4 
                wp_step = int(step_size_meters/path_dist * nPose)
                if(wp_step >= nPose or wp_step <= 0):
                    wp_step = 1
                #wp_step = 1		

                init = wp_step
                print "path dist: ", path_dist
                print "wp step: ", wp_step
            for k in xrange(init,nPose,wp_step):
                wp[0] = poses[k].pose.position.x
                wp[1] = poses[k].pose.position.y
                self.waypoints.append(wp.copy())
            if ( not k==nPose-1):
                wp[0] = poses[-1].pose.position.x
                wp[1] = poses[-1].pose.position.y
                self.waypoints.append(wp.copy())
            
            #print "waypoints: ", self.waypoints
            self.goal = self.waypoints.pop(0)
            print "initial goal: ", self.goal
            
            self.goal_reached = False
        else:
            self.path_debounce_count += 1
            if(self.path_debounce_count > 10):
                self.goal_reached = True
                self.waypoints = []
            
    def raw_cone_callback(self, data): #Need to be careful, cannot use self.local_cone_x, self.local_cone_y in touch_cone() if ill-defined
        self.local_cone_x = data.pose.position.x
        self.local_cone_y = data.pose.position.y
        
    def bumper_callback(self, data):
        self.bumped_cone = data.is_left_pressed or data.is_right_pressed
        self.ir_sensors = [data.light_signal_left, data.light_signal_front_left, data.light_signal_center_left,
                           data.light_signal_center_right, data.light_signal_front_right, data.light_signal_right]
        
    def touch_cone(self):
        
        if ( (rospy.Time.now()-self.reset_found_cone_time).to_sec() > 1.0):
            self.found_cone = False
        
        if( (self.bumped_cone and max(self.ir_sensors[1:4]) > 100) or max(self.ir_sensors[1:4]) > 600):
            self.bump_count += 1
            s = PlaySong()
            s.song = 0
            self.play_song_pub.publish(s)
            
        cone_close = False
        for k in range(6):
            if(self.ir_sensors[k] > 30 or self.local_cone_x < 0.4):
                cone_close = True
                break
        
        # NEED TO PREVENT FALSE POSITIVE CONE BUMPS, falling off carpet to floor causes bump
        # ACTUALLY, YOU FORGOT TO RESET bump_count! Was hard to debug b/c it didn't matter until close to next cone
        if(self.bump_count > 2 and not self.touched_cone):
            self.v = 0
            self.w = 0
            self.bump_count = 0
            #self.stopped = True #Temporarily set to true to stay stopped
            self.found_cone = False
            self.touched_cone = True
            self.reverse_flag = True
            msg = Empty()
            self.touched_cone_pub.publish(msg)
        else:
            self.v = 0.2
            angle_error = math.atan2(self.local_cone_y,self.local_cone_x)
            #self.w = 1.0*self.local_cone_y
            self.w = 1.0*angle_error
            if(abs(self.w) > 1.0):
                self.w = 1.0*np.sign(self.w)
                self.v = 0.1 #0.1 or 0.05 for cam vs laser cone find
            if(cone_close):
                self.v = 0.05
        
        self.status = SEEKING_CONE_STATE
        if(self.stopped or self.touched_cone):
            if(self.stopped):
                self.status = STOPPED_STATE
            else:
                self.status = TOUCHED_CONE_STATE
            self.v = 0
            self.w = 0
            
        twist = Twist()
        twist.linear.x = self.v #Ks*v+out
        twist.angular.z = self.w
        self.cmd_pub.publish(twist)
        
        status_msg = Int16()
        status_msg.data = self.status
        self.status_pub.publish(status_msg)
        
    def execute_plan(self):
        # Update robot pose state from tf listener
        try:
            (trans,quat) = self.tf_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            self.state[0] = trans[0]
            self.state[1] = trans[1]
            #quat_list = [quat.x, quat.y, quat.z, quat.w]
            (roll, pitch, yaw) = euler_from_quaternion (quat)
            self.state[2] = yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        # Local planner (no dynamic obstacle avoidance, hopefully to be done by global planner called frequently)
        if(not self.goal_reached or self.found_cone):
            self.v,self.w,self.goal_reached, alpha, pos_beta, rho = self.diff_drive_controller.compute_vel(self.state,self.goal)
            self.v = 0.2
            if(abs(alpha) > 0.15): #0.1
                print "large error, slow down! alpha: ", alpha
                self.v = 0.07 #0.05
            
            if(self.goal_reached):
                print "wp goal reached"
                #print "v: ", v
                #print "w: ", w
                print "state: ", self.state
                print "goal: ", self.goal
        elif( len(self.waypoints) > 0 and (self.goal_reached) ): # or abs(alpha) > 3.14/2) ):
            self.goal = self.waypoints.pop(0)
            print "wp goal: ", self.goal
            self.goal_reached = False
        else:
            self.v = 0.
            self.w = 0.5 # Eventually choose direction based on heading and dir from bot to cone
            
        if(self.reverse_flag):
            self.v = -0.1
            #self.w = 0.0
        
        #PI control for desired linear speed v
        # controller will output an offset command to add to v
        Ks = 2.5
        Ki = 1.5
        Kp = 1.5
        err = self.v - self.vx
        self.cum_err += err
        self.cum_err = min(1.0, self.cum_err)
        self.cum_err = max(-1.0, self.cum_err)
        out = Kp*err + Ki*self.cum_err
        
        self.status = PATH_STATE
        if(self.stopped or self.touched_cone):
            if(self.stopped):
                self.status = STOPPED_STATE
            else:
                self.status = TOUCHED_CONE_STATE
            self.v = 0
            self.w = 0
        
        twist = Twist()
        twist.linear.x = self.v #Ks*v+out
        twist.angular.z = self.w
        self.cmd_pub.publish(twist)
        
        status_msg = Int16()
        status_msg.data = self.status
        self.status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        path_control = PathController()
        rospy.loginfo("Starting Wheele Path Controller")
        #rospy.spin()
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if(path_control.found_cone):
                path_control.touch_cone()
            else:
                path_control.execute_plan()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
