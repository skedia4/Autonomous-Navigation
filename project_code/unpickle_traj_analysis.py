#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec  6 01:29:05 2021

@author: shubham
"""
##Data_write+=[[N.x, N.y, N.th, N.edge.x, N.edge.y, N.edge.th, N.edge.primitive_index, N.edge.length]]


import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import random
import numpy as np
#import motion_dict_create
import pickle
import regex as re

def traj_csv( filename = './Live_plans/Trajectory_test_plan_virtual_0', Start_x=0, Start_y=0, plot_flag= False, csv_convert_flag= True):
    #filename= 'Trajectory_found_1'
    with open (filename, 'rb') as fp:
        itemlist = pickle.load(fp)
    
    itemlist.reverse()
    
    
    Nodes_all=[]
    Edges_all_x=[]
    Edges_all_y=[]
    Edges_all_th=[]
    primitive_dir=[]
    motion_seq=[]
    for i in itemlist:
        Nodes_all+=[[i[0],i[1],i[2]]]
        #print(type(i[3]),'  ',len(i[3]))
        Edges_all_x= np.concatenate((Edges_all_x, i[3]),axis=0)
        Edges_all_y= np.concatenate((Edges_all_y, i[4]),axis=0)
        Edges_all_th= np.concatenate((Edges_all_th, i[5]),axis=0)
        motion_seq+=[i[6]]
        a=re.split('_', i[6])
        a=a[0]
        
        if a == 'front':
            primitive_dir+= [3]
        elif a == 'reverse':
            primitive_dir+= [1]
        else:
            primitive_dir+= [2]
            
            
    Nodes_all=np.array(Nodes_all)
    Edges_all=np.stack((Edges_all_x, Edges_all_y, Edges_all_th),axis=1)
    
    
    
    motion_seq=np.array(motion_seq)
    print(motion_seq)
    if plot_flag:
        plt.figure()
        plt.plot(Start_x, Start_y,'.', color='b', markersize =20)     ################### Start
        plt.plot(Nodes_all[:-1,0], Nodes_all[:-1,1],'.',color='g' , markersize =5)
        plt.plot(Nodes_all[-1,0], Nodes_all[-1,1],'.', color='r', markersize =20)################### Goal
        plt.plot(Edges_all[:,0], Edges_all[:,1],'r', linewidth=0.7)
        plt.show()        
        
        ################################################# Plot theta or Yaw
        #Edges_all[:,2]=Edges_all[:,2]#*-180/np.pi
        plt.figure()
        plt.plot(Edges_all[:,2]*180/np.pi,'y', linewidth=0.7)
        plt.show()
        
    
    Nodes_all=np.array(Nodes_all, dtype='<U32')
    pid_seq=np.stack((Nodes_all[:,0], Nodes_all[:,1], Nodes_all[:,2], motion_seq),axis=1)
    pid_seq=pid_seq.tolist()
    if csv_convert_flag:
        np.savetxt(filename +'.csv', pid_seq, delimiter=",",fmt ='% s')
        #np.savetxt(filename +'EDGES_all.csv',Edges_all, delimiter=",")
     
    stri= filename +'.csv'
    return stri

if __name__ == '__main__':
    traj_csv()

