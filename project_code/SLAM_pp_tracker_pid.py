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
from sos import Node
import sys



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
    
        #CSV_filename= traj_csv(file_name, True, True)

        self.pose_sub   = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_pose_callback)
        self.x        = 0.0
        self.y       = 0.0
        self.yaw    = 0.0
        
        self.dist= 0
        self.last_e_error=0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/as_tx/vehicle_speed", Float64, self.speed_callback)
        self.speed      = 0.0
        
        CSV_filename = traj_csv( './Live_plans/Trajectory_test_plan_virtual_0')
        #CSV_filename = traj_csv( './Live_plans/Trajectory_test_plan_live_map_0')
        #CSV_filename = traj_csv( './Good_plans/Trajectory_test_plan_virtual_0_parallel_DEC_18')#################################### Can feed good plans directly
        
        #CSV_filename = './Good_plans/Trajectory_DEC_15_perpendicular_decent.csv'
        #CSV_filename = './Good_plans/Trajectory_DEC_17_live_map_found_0.csv'
        #CSV_filename = './Good_plans/Trajectory_DEC_17_parallel_found_0.csv'
        #CSV_filename = ' '
        self.read_waypoints(CSV_filename)
        
        self.desired_speed = 0.5
        self.max_accel     = 0.38 # % of acceleration
        self.pid_x     = PID(0.8, 0.02, 0.01, wg=10)
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

        #self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=10)


    
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


    def write_waypoints(self, data):

        file_name= 'Current_Trajectory.csv'
        with open(file_name,'w') as f:
            writer= csv.writer(f)
            writer.writerow(data)


    def read_waypoints(self, filename =  'Trajectory_found_2.csv'):

      
        
        print(filename)
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # x towards East and y towards North
        self.path_points_x   = np.array([float(point[0]) for point in path_points]) # x
        self.path_points_y   = [float(point[1]) for point in path_points] # y
        self.path_points_yaw = [float(point[2]) for point in path_points] # yaw
        self.motion_seq      = [ str(point[3]) for point in path_points] # motion sequence
        print(len(self.motion_seq))

    def get_gem_state(self):

        return self.x, self.y, self.yaw

   
    def calc(self):
        gear=2; steer=0; dist_f_index=0 ##### 1- eucledian, 0- C_space_with only SO_2 theta
        print("Index:",self.index)
        try:
            if self.motion_seq[self.index] == "front_filt":
                gear=3; steer= 0; dist_f_index=1
            elif self.motion_seq[self.index] == "front_left_filt":
                gear=3; steer= 10.996; dist_f_index=0
            elif self.motion_seq[self.index] == "front_right_filt":
                gear=3; steer= -10.996; dist_f_index=0
            elif self.motion_seq[self.index] == "reverse_filt":
                gear=1; steer= 0; dist_f_index=1 
            elif self.motion_seq[self.index] == "reverse_left_filt":
                gear=1; steer= 10.996; dist_f_index=0
            elif self.motion_seq[self.index] == "reverse_left_half_filt":
                gear=1; steer= 4.526; dist_f_index=0
            elif self.motion_seq[self.index] == "reverse_right_filt":
                gear=1; steer= -10.996; dist_f_index=0
            elif self.motion_seq[self.index] == "reverse_right_half_filt":
                gear=1; steer= -4.526;  dist_f_index=0
            else:
                pass

        except:

            self.brake_cmd.f64_cmd = 0.5
            self.brake_pub.publish(self.brake_cmd)
            print("Reached Destination")
            self.steer_cmd.angular_position = 0
            self.steer_pub.publish(self.steer_cmd)
            time.sleep(1)
            node = Node()
            node.run()
            sys.exit(1)
            


        
        return steer, gear, dist_f_index

    # computes the Euclidean distance between two 2D points
    def e_dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)
    
    def c_space_dist(self,p1, p2):
        x=p1[0]; y= p1[1]; th = p1[2]
        x_n= p2[0]; y_n= p2[1]; th_n= p2[2]
        th_so2=min(abs(th_n-th),2*np.pi-abs(th_n-th))
        dis=0*math.sqrt((x-x_n)**2+(y-y_n)**2) + 1*th_so2
        return round(dis, 3)

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
            self.write_waypoints([curr_x, curr_y])

            steering_angle, gear,  dist_f_index = self.calc()      
            curr_goal=[self.path_points_x[self.index], self.path_points_y[self.index], self.path_points_yaw[self.index]]
            
            err_threshold=0
            if dist_f_index ==0:
                self.dist= self.c_space_dist( curr_goal, [curr_x, curr_y, curr_yaw])  ########## OR e_dist
                err_threshold=0.1
            elif dist_f_index ==1:
                self.dist= self.e_dist( curr_goal, [curr_x, curr_y, curr_yaw])
                err_threshold=0.25 + self.last_e_error
                #self.last_e_error+= 0.1
            else:
                pass


            
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
            print("Current state in slam: ", curr_x,curr_y, curr_yaw)
            #curr_x = self.x_filter.get_data(curr_x)
            #filt_vel    = self.speed_filter.get_data(self.speed)
            print('Raw velocity= ',self.speed , ',   Filtered velocity is= ', filt_vel)
            
            
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
            print("dist",self.dist)


            if self.dist > err_threshold:
                if filt_vel < self.desired_speed:

                    print("Applied Acceleration is: ", output_accel)
                    #self.last_dist = self.dist
                    self.accel_cmd.f64_cmd = output_accel
                    self.steer_cmd.angular_position = steering_angle
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                
            else:
                
                self.index +=1
                self.brake_cmd.f64_cmd = 0.5
                self.brake_pub.publish(self.brake_cmd)
                print("reached waypoint and applying brake")
                self.pid_x.reset()
                time.sleep(1)
                self.brake_cmd.f64_cmd = 0
                self.brake_pub.publish(self.brake_cmd)
                self.accel_cmd.f64_cmd = 0.25
                self.accel_pub.publish(self.accel_cmd)
                
                curr_x, curr_y, curr_yaw = self.get_gem_state()
                self.last_e_error= self.e_dist( curr_goal, [curr_x, curr_y, curr_yaw])
                

                
                #self.last_dist=1000
                
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


