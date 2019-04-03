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

from least_square_circle import *

class LaserConeDetect():
    def __init__(self): 
        #self.tf_listener = tf.TransformListener()
        
        rospy.init_node('laser_cone_detect')
        
        self.pub_cone_pose = rospy.Publisher("laser_cone_pose", PoseStamped, queue_size = 10)
        
        #rospy.Subscriber('cmd_vel', Twist, self.drive_callback, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size = 1)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size = 1)
        rospy.Subscriber('expected_cone_in_laser', PoseStamped, self.expected_cone_pose_callback, queue_size=5)
        
        reset_found_cone_time = rospy.Time.now()
        
        self.debug_count = 0
        self.scan_count = 0
        
        self.search_angle_index = 0
        self.nNeighbors = 12
    
    def expected_cone_pose_callback(self,data):
        x = data.pose.position.x
        y = data.pose.position.y
        angle_deg = math.atan2(y,x)*180/3.14
        search_angle_index = int(angle_deg+180)
        self.search_angle_index = search_angle_index % 360 # this is only valid for rplidar with angle_increment = 1 deg
    
    def scan_callback(self, data):
        self.scan_count += 1
        ranges = data.ranges
        amin = data.angle_min
        amax = data.angle_max
        step = data.angle_increment
        nAng = len(ranges)
        
        angles = np.linspace(amin, amax, nAng) + 3.14 #hard coded laser to base_link tf
        ranges_np = np.array(ranges)
        x_all = ranges_np*np.cos(angles)
        y_all = ranges_np*np.sin(angles)
        x_all = x_all.tolist()
        y_all = y_all.tolist()
        
        if(self.debug_count < 1):
            print('\nx,y')
            for (x,y) in zip(x_all,y_all):
                print(x,y)
        self.debug_count += 1
        
        min_resid = 10000.0
        laser_cone_found = False
        # require that the fit circle includes about 90 deg worth of poitnts (arc length about 90 deg worth)
        #    use a dynamic nNeighbors based on distance to xvec,yvec
        #    8 cm radius, 90 deg, is .08*3.14/2 = 0.12 m arc length desired
        #    1 m away, point spacing will be about pi/180*1m
        cone_radius = 0.08 # meters
        nNeighbors = self.nNeighbors
        des_circle_portion_deg = 90
        des_arc_length = 3.14/180*des_circle_portion_deg * cone_radius
        min_neighbors = 4
        
        if(self.scan_count >= 2):
            start = rospy.Time.now()
            self.scan_count = 0
            #for i in range( len(x_all)):
            #    xmid = x_all[i-int(nNeighbors/2)]
            #    ymid = y_all[i-int(nNeighbors/2)]
            
            nom_ind = self.search_angle_index
            offset_list = [0,-1,1,-2,2,-3,3,-4,4,-5,5,-6,6,-7,7,-8,8]*2 #search around self.search_angle_index
            for offset in offset_list:
                center_ind = nom_ind +offset 
                i = (center_ind + int(nNeighbors/2)) % nAng #i is the right index, should be 0 to 359
                
                xmid = x_all[i-int(nNeighbors/2)]
                ymid = y_all[i-int(nNeighbors/2)]
                if(abs(xmid)  < 10 and abs(ymid) < 10):
                    dist = np.sqrt(xmid**2 + ymid**2)
                    ddist = 3.14/180*dist #should use angle_increment, not 3.14/180
                    if(dist > 0):
                        nNeighbors = max(int(des_arc_length/ddist),min_neighbors)
                if(i-nNeighbors < 0):
                    xvec = x_all[i-nNeighbors:]+x_all[:i]
                    yvec = y_all[i-nNeighbors:]+y_all[:i]
                else:
                    xvec = x_all[i-nNeighbors:i]
                    yvec = y_all[i-nNeighbors:i]
                has_inf = False
                for x in xvec:
                    if(x == float('inf') or x == float('-inf') ):
                        has_inf = True
                        break
                if(not has_inf):
                    [xc,yc,R,resid] = leastsq_circle(xvec,yvec)
                    if(0.07 < R and R < 0.09 and resid < 3e-5): #0.06 to 0.1, 5e-5
                        #circle_x.append(xc)
                        #circle_y.append(yc)
                        #circle_R.append(R)
                        # require the circle to be convex w.r.t. lidar
                        pt_dist_sqd = np.mean(xvec)**2 + np.mean(yvec)**2
                        fit_dist_sqd = xc**2 + yc**2
                        if(fit_dist_sqd > pt_dist_sqd):
                            if(resid < min_resid):
                                #min_resid = resid
                                cone_pose = PoseStamped()
                                cone_pose.header.frame_id = "base_link"
                                cone_pose.header.stamp = rospy.Time.now()
                                cone_pose.pose.orientation.w = 1.0
                                cone_pose.pose.position.x = xc
                                cone_pose.pose.position.y = yc
                                laser_cone_found = True
                                self.pub_cone_pose.publish(cone_pose)
                                #print(xvec)
                                #print(yvec)
                                print(xc, yc, R, resid)
                            #end new min
                        #end if convex
                    #end good R
                #end
            #end for
            #if(laser_cone_found):
            #    self.pub_cone_pose.publish(cone_pose)
            print("duration: ", (rospy.Time.now()-start).to_sec())
            self.nNeighbors = nNeighbors
        #end if every other scan
        
    
    def scan_callback2(self, data):
        ranges = data.ranges
        amin = data.angle_min
        amax = data.angle_max
        step = data.angle_increment
        nAng = (amax-amin)/step
        
        start = rospy.Time.now()
            
        # Loop thru ranges to find cone candidates
        cone_xy_list = []
        dist_list = []
        dAngle_list = []
        init_k = -15
        k = init_k
        n = 0
        while(k < len(ranges)):
            r = ranges[k]
            a = amin + k*step
            x = r*math.cos(a+3.14) #NOTE, hardcoded tf from laser to base_link
            y = r*math.sin(a+3.14) #NOTE, hardcoded tf from laser to base_link
            if(n > 0):
                dx = x-x1
                dy = y-y1
                dist = math.sqrt(dx*dx + dy*dy)
                angle = math.atan2(dy,dx)
                if(n > 1):
                    dAngle = angle - angle1
                    if(dAngle > 3.14):
                        dAngle -= 6.28
                    elif(dAngle <= -3.14):
                        dAngle += 6.28
                    if(abs(a) > 2.8 and 0.5 < r and r < 1.1):
                        print("dist, dAngle, x, y: ", dist, dAngle, x, y)
                    if(-0.7 < dAngle and dAngle < -0.3):
                        dAngle_list.append(dAngle)
                        dist_list.append(dist)
                    else:
                        if(len(dist_list) > 2):
                            cone_radius = sum(dist_list)/sum(dAngle_list)
                            if(-0.15 < cone_radius and cone_radius < -0.05):
                                print("cone local xy, r: ", x1, y1, cone_radius)
                                print("dAngles: ", dAngle_list)
                                print("dists: ", dist_list)
                                cone_pose = PoseStamped()
                                cone_pose.header.frame_id = "base_link"
                                cone_pose.header.stamp = rospy.Time.now()
                                cone_pose.pose.orientation.w = 1.0
                                cone_pose.pose.position.x = x1
                                cone_pose.pose.position.y = y1
                                self.pub_cone_pose.publish(cone_pose)
                        dAngle_list = []
                        dist_list = []
                    #end
                #end
                dist1 = dist
                angle1 = angle
            #end
                    
            x1 = x
            y1 = y
            k += 2
            n += 1
        #end while
        print("duration: ", (rospy.Time.now()-start).to_sec())
            
    
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
            
    def raw_cone_callback(self, data):
        self.local_cone_x = data.pose.position.x
        self.local_cone_y = data.pose.position.y

if __name__ == '__main__':
    try:
        laser_cone_detect = LaserConeDetect()
        rospy.loginfo("Starting Laser Cone Detector")
        #rospy.spin()
        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
