#!/usr/bin/env python3


from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np

# ROS Headers
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64

from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd


class Motion_Primitives(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(50)

        self.pose_sub   = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_callback)
        self.x        = 0.0
        self.y       = 0.0
        self.yaw    = 0.0


    def goal_pose_callback(self, pose_msg):
        self.x    =  pose_msg.pose.position.x  # x-coordinate
        self.y     = pose_msg.pose.position.y # y-coordinate
        orientation_q = pose_msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_,_,self.yaw) = euler_from_quaternion (orientation_list)   # yaw


    def write_waypoints(self,data):

        # read recorded GPS lat, lon, heading
        #dirname  = os.path.dirname(__file__)
        filename = '/home/sambhu/gem_gnss/waypoints/goal_pose.csv'

        with open(filename,'a') as f:
            writer= csv.writer(f)
            writer.writerow(data)
   
    def start_pp(self):
        
        while not rospy.is_shutdown():


            #self.write_waypoints([self.x, self.y, self.yaw, self.speed])

            self.write_waypoints([self.x, self.y, self.yaw])
 
            self.rate.sleep()


def pure_pursuit():

    rospy.init_node('motion_primitives_node', anonymous=True)
    pp = Motion_Primitives()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


