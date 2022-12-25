#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  9 01:29:05 2021

@author: shubham
"""

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

from unpickle_traj_analysis import traj_csv

# ROS Headers
#import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# SLAM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
#from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd
import time

class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0
        
        
    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(10)

        
        self.index = 0
        self.last_gear= 2

        self.pose_sub   = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_pose_callback)
        self.x        = 0.0
        self.y       = 0.0
        self.yaw    = 0.0
        self.dist= 0
        self.last_dist=1000

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 0.0
        


        # read waypoints into the system 
        #self.read_waypoints('Trajectory_perp_park_decent_found_0.csv') 
        
        
        self.desired_speed = 0.5
        self.max_accel     = 0.38 # % of acceleration
        self.pid_x     = PID(0.8, 0.03, 0.01, wg=10)
        self.x_filter  = OnlineFilter(1.2, 30, 4)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)
        # -------------------- PACMod setup --------------------

        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True


        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 5.0 # radians/second


    # def inspva_callback(self, inspva_msg):
    #     self.lat     = inspva_msg.latitude  # latitude
    #     self.lon     = inspva_msg.longitude # longitude
    #     self.heading = inspva_msg.azimuth   # heading in degrees
    
    def write_waypoints(self,data,name):

        # read recorded GPS lat, lon, heading
        #dirname  = os.path.dirname(__file__)
        filename = '/home/gem/workspaces/group_11_ws/gem_gnss/primitivies/motion_primitive'+ name + '.csv'

        with open(filename,'a') as f:
            writer= csv.writer(f)
            writer.writerow(data)
    
    
    def slam_pose_callback(self, pose_msg):
        self.x    =  pose_msg.pose.position.x  # x-coordinate
        self.y     = pose_msg.pose.position.y # y-coordinate
        orientation_q = pose_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_,_,self.yaw) = euler_from_quaternion (orientation_list)

    def speed_callback(self, msg):
        self.speed = round(msg.data, 3) # forward velocity in m/s

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data



    def get_gem_state(self):

        return self.x, self.y, self.yaw

    # find the angle bewtween two vectors    
    def calc(self, name):
        gear=2; steer=0
        print(name)
        if name == "front_filt":
            gear=3; steer= 0
        elif name == "front_left_filt":
            gear=3; steer= 10.996
        elif name == "front_right_filt":
            gear=3; steer= -10.996
        elif name == "reverse_filt":
            gear=1; steer= 0
        elif name == "reverse_left_filt":
            gear=1; steer= 10.996
        elif name== "reverse_right_filt":
            gear=1; steer= -10.996
        else:
            pass
        
        return steer, gear


    def start_pp(self):
        
        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if(self.pacmod_enable == False):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = self.last_gear

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True

            
            #self.path_points_x = np.array(self.path_points_x)

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            

            # steering_angle in degree
            name= "front_filt"
            steering_angle, gear = self.calc(name)
            print("gear: ", gear, "last gear:", self.last_gear)
            #print(self.last_gear)
            if gear != self.last_gear:
                self.last_gear= gear
                for i in range(5):
                    self.gear_cmd.ui16_cmd= self.last_gear
                    self.gear_pub.publish(self.gear_cmd)
                    time.sleep(0.1)
                print("gear changed")
                
                
            

            current_time = rospy.get_time()
            filt_vel    = self.speed
            self.write_waypoints([curr_x, curr_y, curr_yaw, filt_vel], name)
            
            
            #output_accel = self.pid_x.get_control(current_time, abs(self.path_points_x[self.index] - curr_x ))
            output_accel = self.pid_x.get_control(current_time, abs(filt_vel- self.desired_speed))
            
            if filt_vel>self.desired_speed:
                output_accel=0
                self.brake_cmd.f64_cmd = 0.3
                self.brake_pub.publish(self.brake_cmd)
                print("Exceeded velocity applying brake")
            #else:
            #    self.brake_cmd.f64_cmd = 0
            #    self.brake_pub.publish(self.brake_cmd)
                
          

            if output_accel > self.max_accel:
                output_accel = self.max_accel
                

            if output_accel < 0.2:
                output_accel = 0.2
 ################################################################################# Uncomment for x PID control               
            # if self.last_gear ==3 :
            #     if self.path_points_x[self.index] < curr_x :
            #         self.index +=1
            #         self.brake_cmd.f64_cmd = 0.5
            #         self.brake_pub.publish(self.brake_cmd)
            #         print("reached waypoint and applying brake")
            #         self.pid_x.reset()
            #         time.sleep(1)
            #     else:
            #         self.accel_cmd.f64_cmd = output_accel
            #         self.steer_cmd.angular_position = steering_angle
            #         self.accel_pub.publish(self.accel_cmd)
            #         self.steer_pub.publish(self.steer_cmd)
                    
            # elif self.last_gear==1 :
            #     if self.path_points_x[self.index] > curr_x :
            #         self.index +=1
            #         self.brake_cmd.f64_cmd = 0.5
            #         self.brake_pub.publish(self.brake_cmd)
            #         print("reached waypoint and applying brake")
            #         self.pid_x.reset()
            #         time.sleep(1)
            #     else:
            #         print(output_accel)
            #         self.accel_cmd.f64_cmd = output_accel
            #         self.steer_cmd.angular_position = steering_angle
            #         self.accel_pub.publish(self.accel_cmd)
            #         self.steer_pub.publish(self.steer_cmd)
                    
            # else:
            #     pass
            
    ################################################################################# Uncomment for Dist PID control   
            #print("last dist",self.last_dist)
  


          
            if filt_vel< self.desired_speed:
                print("Applied Acceleration is: ", output_accel)
                self.accel_cmd.f64_cmd = output_accel
                self.accel_pub.publish(self.accel_cmd)
            self.steer_cmd.angular_position = steering_angle
            self.steer_pub.publish(self.steer_cmd)
        
           
            time.sleep(0.1)


def pure_pursuit():

    rospy.init_node('slam_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


