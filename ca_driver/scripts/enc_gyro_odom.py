#!/usr/bin/env python

import rospy, math
import numpy as np

from std_msgs.msg import Int16

from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist, Quaternion, Point, Pose, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu, JointState
from ca_msgs.msg import Bumper

import time
import sys

class KreateOdom():
    def __init__(self):
        self.mag_heading_deg = 0
        self.odom_heading_deg = 0
        #self.map_broadcaster = tf.TransformBroadcaster()
        
        self.tf_listener = tf.TransformListener()
        
        rospy.init_node('kkreate_odom')
        
        self.prev_time = rospy.Time.now()
        
        self.gyroz_rad = 0.0
        
        self.enc_dmeters = 0.0
        self.prev_encx = 0.0
        self.prev_ency = 0.0
        
        self.left_enc = 0
        self.right_enc = 0
        self.prev_left_enc = -1
        self.prev_right_enc = -1
        self.prev_delta_left = 0 #I saw the raw create wheel dist data jump at times, when switching directions
        self.prev_delta_right = 0
        self.enc_init_flag = False
        print('Initializing kkreate odom.')
        
        self.dist_sum = 0
        self.time_sum = 0
        self.vx = 0

        self.bot_deg = 0
        self.botx = 0
        self.boty = 0
        
        self.gyro_sum = 0.0
        self.gyro_count = 0
        self.gyro_bias_rad = 0.0
        
        self.bump_switch_pub = rospy.Publisher('bump_switch', Int16, queue_size = 1)
        
        rospy.Subscriber('imu', Imu, self.imu_callback, queue_size=2)
        rospy.Subscriber('joint_states', JointState, self.enc_callback, queue_size=2)
        rospy.Subscriber('odom_enc', Odometry, self.odom_enc_callback, queue_size = 1)
        rospy.Subscriber('bumper', Bumper, self.bumper_callback, queue_size=2)
        
        self.odom_pub = rospy.Publisher('odom_gyro', Odometry, queue_size=5)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        while(not self.enc_init_flag):
            time.sleep(0.1)
            self.prev_left_enc = self.left_enc
            self.prev_right_enc = self.right_enc
    
    def bumper_callback(self, data):
        bumped_cone = data.is_left_pressed or data.is_right_pressed
        ir_sensors = [data.light_signal_left, data.light_signal_front_left, data.light_signal_center_left,
                           data.light_signal_center_right, data.light_signal_front_right, data.light_signal_right]
        raw_sig = Int16()
        if( (bumped_cone and max(ir_sensors[1:4]) > 100) or max(ir_sensors[1:4]) > 600):
            raw_sig.data = 1
        else:
            raw_sig.data = 0
        self.bump_switch_pub.publish(raw_sig)
    
    def odom_enc_callback(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        dx = x - self.prev_encx
        dy = y - self.prev_ency
        self.prev_encx = x
        self.prev_ency = y
        delta = np.sqrt(dx**2 + dy**2)*np.sign(v)
        self.enc_dmeters += delta
        
    def enc_callback(self,msg):
        self.left_enc = msg.position[0]
        self.right_enc = msg.position[1]
        left_vel = msg.velocity[0]
        right_vel = msg.velocity[1]
        self.enc_init_flag = True
        

    def imu_callback(self, data):
        accx = data.linear_acceleration.x
        accy = data.linear_acceleration.y
        self.gyroz_rad = data.angular_velocity.z
        if(self.gyro_count < 100):
            self.gyro_count += 1
            self.gyro_sum += self.gyroz_rad
            if(self.gyro_count == 100):
                self.gyro_bias_rad = self.gyro_sum / self.gyro_count
                print('gyro bias deg: ', self.gyro_bias_rad*180/3.14)
                
    def update_odom(self):
        t2 = rospy.Time.now()
        t1 = self.prev_time
        self.prev_time = t2
        dt = (t2-t1).to_sec()
        
        gyro_thresh_dps = 0.04*180/3.14159
        g_bias_dps = self.gyro_bias_rad*180/3.14159
        MAX_DTHETA_GYRO_deg = 100
        BOT_WIDTH = 0.235 #meters
        COUNTS_PER_METER = 1.0
        
        gyroz_raw_dps = float(self.gyroz_rad) * 180.0 / 3.14159
        #print 'gyroz raw dps: ', gyroz_raw_dps
        
        delta_left_enc = self.get_delta_enc(self.left_enc, self.prev_left_enc)
        if(delta_left_enc > 1.0): #meters for create
            delta_left_enc = self.prev_delta_left
            print 'left enc jump'
        else:
            self.prev_delta_left = delta_left_enc
            
        delta_right_enc = self.get_delta_enc(self.right_enc, self.prev_right_enc)
        if(delta_right_enc > 1.0): #meters for create
            delta_right_enc = self.prev_delta_right
            print 'right enc jump'
        else:
            self.prev_delta_right = delta_right_enc
        
        dtheta_enc_deg = float(delta_right_enc - delta_left_enc) / BOT_WIDTH * 180.0 / 3.14159
        
        dmeters = float(delta_left_enc + delta_right_enc)/2.0 / COUNTS_PER_METER #1.0 counts/meter ideal for the create encoder data
        dmeters = self.enc_dmeters
        self.enc_dmeters = 0.0
        #print 'dmeters: ', dmeters
        
        if(abs(gyroz_raw_dps-g_bias_dps) < gyro_thresh_dps):
            gz_dps = 0
            dtheta_gyro_deg = 0
        else:
            gz_dps = gyroz_raw_dps-g_bias_dps
            dtheta_gyro_deg = gz_dps*dt*1.002 #*375.0/360.0 #HACK, WHY!!??

        if(abs(dtheta_gyro_deg) > 100.0):
            print 'no gyro'
            dtheta_deg = dtheta_enc_deg
        else:
            #print 'use gyro'
            dtheta_deg = dtheta_gyro_deg
            
        #print 'dtheta gyro deg:', dtheta_gyro_deg
        #print 'dtheta enc deg:', dtheta_enc_deg

        #update bot position
        #self.bot.move(dmeters,dtheta_deg,use_gyro_flag)
        self.bot_deg = self.bot_deg + dtheta_deg
        dx = dmeters*np.cos(self.bot_deg*3.1416/180)
        dy = dmeters*np.sin(self.bot_deg*3.1416/180)
        self.botx = self.botx + dx
        self.boty = self.boty + dy
        
        # update bot linear x velocity every 150 msec
        # need to use an np array, then push and pop, moving average
        self.dist_sum = self.dist_sum + dmeters
        self.time_sum = self.time_sum + dt
        if(self.time_sum > 0.15):
            self.vx = self.dist_sum / self.time_sum
            self.dist_sum = 0
            self.time_sum = 0
        
        #bot.botx*100,bot.boty*100,bot.bot_deg
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.bot_deg*3.14159/180.0)
        self.odom_broadcaster.sendTransform(
        (self.botx, self.boty, 0.),
        odom_quat,
        t2,
        "base_link",
        "odom"
        )
        
        odom = Odometry()
        odom.header.stamp = t2
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.botx, self.boty, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, gz_dps*3.1416/180.0))

        # odom twist covariance
        odom.twist.covariance[0] = 0.001 #vx
        odom.twist.covariance[7] = 1.e-9 #vy
        # publish the message
        self.odom_pub.publish(odom)      
        
        self.prev_left_enc = self.left_enc
        self.prev_right_enc = self.right_enc

    def get_delta_enc(self, cur, prev):
        delta = cur - prev
        # I saw the create wheel distances go to +3000 m so far
        #~ if(delta > 60000):
            #~ delta = -(65535 - delta)
        #~ elif(delta < -60000):
            #~ delta = 65535 + delta
        return delta

if __name__ == '__main__':
    try:
        kodom = KreateOdom()
        print("Starting KKreate Odom")

        r = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            kodom.update_odom()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
