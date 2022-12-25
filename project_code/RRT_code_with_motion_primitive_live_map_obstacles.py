#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 21 18:30:14 2021

@author: shubham
"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import random
import numpy as np
#from sklearn.neighbors import KDTree
import motion_dict_create
import pickle

import rospy
import sys

from collision2 import Collision_Checker

#from collisions_rotated import Collision_virtual_check

import time

class node:
    def __init__(self,parent, x,y,th,v):
        self.parent=parent
        self.x = x
        self.y = y
        self.th=th
        self.v=v
        self.edge=None

class edge:
    def __init__(self,parent, child,x,y,th,v,primitive_index,ln):
        
        self.parent=parent
        self.child=child
        self.x = x
        self.y = y
        self.th=th
        self.v=v
        self.primitive_index=primitive_index
        self.cost=None
        self.length=ln

class tree:
    def __init__(self):
        self.nodes=[]
        self.edges=[]
    def add_n(self,node):
        (self.nodes).append(node)
    def add_e(self,edge):
        (self.edges).append(edge)
    def nearest_neigh(self,x,y,th=random.uniform(0,2*np.pi),v=random.uniform(-5,5)):
        N=self.nodes
        dis_min=100000

        N_node=None
        for i in N:
            x_n=i.x
            y_n=i.y
            th_n=i.th
            v_n=i.v
            
            th_so2=min(abs(th_n-th),2*np.pi-abs(th_n-th))

            dis=0.2*math.sqrt((x-x_n)**2+(y-y_n)**2) + 0.8*th_so2 #+(v-v_n)**2 ############ Dist function
            if dis<dis_min:
                dis_min=dis
                N_node=i
        return N_node,dis
    
    def distance_nodes(self, Node , Node_n):
       
        th_so2=min(abs(Node_n.th-Node.th),2*np.pi-abs(Node_n.th-Node.th))
        dis=0.3*math.sqrt((Node.x-Node_n.x)**2+(Node.y-Node_n.y)**2) + 0.7*th_so2 #+(v-v_n)**2 ############ Dist function
        return dis
    
    
    def r_nearest_neigh(self,x,y,th=random.uniform(0,2*np.pi),v=random.uniform(-5,5)):
        N=self.nodes
        dis_min=100000
        Nodes=[]; dist_all=[]
        N_node=None
        for i in N:
            x_n=i.x
            y_n=i.y
            th_n=i.th
            v_n=i.v
            
            th_so2=min(abs(th_n-th),2*np.pi-abs(th_n-th))

            dis=0.3*math.sqrt((x-x_n)**2+(y-y_n)**2) + 0.7*th_so2 #+(v-v_n)**2 ############ Dist function
            if dis <0.3:
                Nodes+=[i]
                dist_all+=[dis]
                
            if dis<dis_min:
                dis_min=dis
                N_node=i
                
        if not Nodes:  
            print('No multi')
            return [N_node],[dis_min]
        else:
            return Nodes, dist_all
 
    
    def motion(self, Node, primitive_dict,cc):
        index, traj = random.choice(list(primitive_dict.items()))       #print(index)
        #print(index)
        s=np.array(traj)
        #print(s.shape)
        
        l=s.shape[0]
        ln=l
        ln=int(np.random.uniform(l/3,l,1))########Controls the length of primitives or random cutting
        s=s[0:ln,:]
        #print(s.shape)
        
        
        x_i=s[:, 0]
        y_i=s[:, 1]
        th_i=s[:, 2]
        v_i=s[:, 3]
        temp=np.stack((x_i,y_i),axis=0)
        R=np.array([[np.cos(Node.th),-np.sin(Node.th)], [np.sin(Node.th), np.cos(Node.th)]])
        pos=np.matmul(R, temp)
        pos=pos.T
        s=np.stack((pos[:, 0], pos[:, 1],th_i,v_i),axis=1)
        
        s=s+ np.array([Node.x,Node.y,Node.th, 0]) ####Broadcasting to all the edge points
        np.where(s[:,2] > 2*np.pi, s[:,2]-2*np.pi, s[:,2])
        
        
        Nod_ext=node(Node,s[-1, 0], s[-1, 1] ,s[-1, 2],s[-1, 3])
        Edge_ext=edge(Node,Nod_ext,s[:, 0], s[:, 1] ,s[:, 2],s[:, 3],index,ln)
        Nod_ext.edge= Edge_ext
        
        ############################################################################ Collision Checker virtual 
        '''Ed_check=5
        for i in range(Ed_check):
            temp=int((i+1)*ln/Ed_check)
            if Collision_virtual_check(s[temp-1, 0], s[temp-1, 1] ,s[temp-1, 2]):
                return None, None'''
    
        ############################################################################ Collision Checker with MAP info
        Ed_check=5
        for i in range(Ed_check):
            temp=int((i+1)*ln/Ed_check)
            map_ready=False
            while not map_ready:
                #print(cc.start_cc((s[temp-1, 0], s[temp-1, 1] ,s[temp-1, 2])))
                if  cc.start_cc((s[temp-1, 0], s[temp-1, 1] ,s[temp-1, 2]))==None:
                    pass
                else:
                    map_ready= True
            if  not cc.start_cc((s[temp-1, 0], s[temp-1, 1] ,s[temp-1, 2])):
                print("hi")
                return None, None           
        
        return Nod_ext,Edge_ext
    
    def Tree_search (self,N_start, N_goal, plot_flag= False,index=0):
        
        prim_seq=[]
        lengt=[]
        data_write_flag = True
        N=N_goal
        Data_write=[]
        
        if plot_flag:
            fig_1=plt.figure()
            plt.plot(N_start.x, N_start.y,'.',color='b', markersize =20)
            plt.plot(N_goal.x, N_goal.y,'.',color='y', markersize =20)
        while N.edge is not None:
            prim_seq+=[N.edge.primitive_index]
            lengt+=[N.edge.length]
            
            if plot_flag:
                plt.plot(N.x, N.y,'.',color='g', markersize =7)
                plt.plot(N.edge.x, N.edge.y,'r', linewidth=0.7)
                
            if data_write_flag:
               Data_write+=[[N.x, N.y, N.th, N.edge.x, N.edge.y, N.edge.th, N.edge.primitive_index, N.edge.length]]
               
            N=N.parent
             
        if data_write_flag:
            with open('Trajectory_test_plan_live_map_'+ str(index), 'wb') as fp: ########## Note data is in reverse from goal
                pickle.dump(Data_write, fp)
                
        if plot_flag:
            plt.show()    
        prim_seq.reverse()
        lengt.reverse()
        return prim_seq, lengt#, fig_1
    
 






    
if __name__ == "__main__":
    Tr=tree()  
    N_start=node(None, -0.18 , 0.05  ,0.0, 0)
    ################## Goals
    ##### node(None, 4, 3, np.pi/2, 0) perpendicular parking
    ##### node(None, 4, 3, np.pi/2, 0) perpendicular parking
    
    
    N_goal=node(None, -11, 2 ,np.pi/4, 0)
    Tr.add_n(N_start)

    rospy.init_node('collision_checker_node', anonymous=True)
    cc = Collision_Checker()

    
    p_g=0.2########################## Goal biasing
    early_term_flag=True ########################## Flag for early termination
    
    [ primitive_dict,control_dict]= motion_dict_create.create_motion_prim_dic()
    
    for i in range(2000):
        if random.uniform(0,1)<p_g:
            N, _=Tr.nearest_neigh(N_goal.x,N_goal.y,N_goal.th,N_goal.v)
    
        else :
            x=random.uniform(-20,20)           ####################expansion in state space x
            y=random.uniform(-1,3)          ####################expansion in state space y
            th=random.uniform(-0.5*np.pi,0.5*np.pi)
            v=random.uniform(-0.5,0.5)
            N,_=Tr.nearest_neigh(x,y,th,v)

    
        [N_new,E_new]=Tr.motion(N, primitive_dict,cc)############################ For Map need to ensure
        if N_new is not None:
            Tr.add_n(N_new)
            Tr.add_e(E_new)
            if Tr.distance_nodes (N_new , N_goal) < 0.2 and early_term_flag:
                break

    N_final, dist =Tr.r_nearest_neigh(N_goal.x,N_goal.y,N_goal.th,N_goal.v)
    
    cost=[]
    Motion_seq_list=[]
    lengt_list=[]
    x=0
    if len(N_final)>5:
       N_final= N_final[0:4]
    for i in N_final:
        [Motion_seq, lengt]=Tr.Tree_search(N_start, i, True, x)#####True for Trajectort plot
        Motion_seq_list+=[Motion_seq]
        lengt_list+=[lengt]
        cost+=[sum(lengt)]
        x+=1
    
    index_min = np.argmin(np.array(cost))
    Motion_seq= Motion_seq_list[index_min]
    lengt = lengt_list[index_min]
    dis_least_cost= dist[index_min]
    print('Best trajectory is figure: ', index_min+2)
    
    x=[]
    y=[]
    
    for i in Tr.nodes:
        x.append(i.x)
        y.append(i.y)
   
    plt.figure()
    plt.plot(x, y,'.',markersize =5)
    
  
    for i in Tr.edges:
        plt.plot(i.x, i.y,'k', linewidth=0.4)
        

    
    plt.show()
    
    '''x=0
    rospy.init_node('pp_node', anonymous=True)
    publish = PurePursuit()
    for i in Motion_seq:
 
        publish.read_waypoints(np.array(control_dict[i]), i)
        len_temp= lengt[x]

        start_time = time.time()

        #publish.gear_pub.publish(publish.gear_cmd)
    

        try:

            publish.start_pp(len_temp)


        except rospy.ROSInterruptException:
            pass
        x+=1
        print("--- %s seconds ---" % (time.time() - start_time))'''

    
        
    
    
    
