#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 21 20:56:20 2021

@author: shubham
"""
import glob
import os
import csv
import numpy as np
import re



def create_motion_prim_dic():
    cwd = os.getcwd()
    filenames = glob.glob(cwd + "/primitives_pid_filtered/*.csv")
    #print (filenames)
    primitive_dict={}
    control_dict={}

    for file in filenames:
        temp= re.split("/", file)
        temp=temp[-1]
        temp=re.split(".csv", temp)
        temp=temp[0]
        
        with open(file) as f:
            readCSV = csv.reader(f, delimiter=',')
            j=0
            for row in readCSV:
                if j ==0:
                    primitive_dict [temp]=[[float(row[0]), float(row[1]), float(row[2]),float(row[3])]]
                    #control_dict [temp]= [[float(row[3]), float(row[4]), float(row[5])]]
                    j+=1
                else:
                    primitive_dict [temp]+=[[float(row[0]), float(row[1]), float(row[2]),float(row[3])]]
                   # control_dict [temp]+= [[float(row[3]), float(row[4]), float(row[5])]]
                  
                
        
    
    return primitive_dict, control_dict



if __name__ == "__main__":
    [primitive_dict, control_dict] = create_motion_prim_dic()
    