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

from nav_msgs.msg import OccupancyGrid
from map import Map

import time

class Collision_Checker(object):
    
    def __init__(self):



        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.map=None


    def map_callback(self, grid_map):
        self.map = Map(grid_map)


    def start_cc(self,state):

        if self.map is not None:
            #start_time = time.time()
            return self.map.is_allowed(state)
            #print("--- %s seconds ---" % (time.time() - start_time))

        else:
            return None




def collision_check(state):

    rospy.init_node('collision_checker_node', anonymous=True)
    cc = Collision_Checker()

    try:
        cc.start_cc(state)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    x= 3.00068187714
    y= -0.481993913651
    yaw=np.pi/4


    state=(x,y,yaw)

    
    print(collision_check(state))
    


