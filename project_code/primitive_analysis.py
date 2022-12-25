#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  3 21:04:09 2021

@author: shubham
"""
import csv
import numpy as np
from alvinxy import *
#reload(alvinxy)
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import pandas as pd 


Lat=[];Lon=[];x=[];y=[];th=[]; c1=[]; c2=[]; c3=[]; v=[]
with open('./primitives_pid_backup/reverse_left_filt.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        #Lat+=[row[7]];Lon+=[row[8]];
        x+=[row[0]];y+=[row[1]];th+=[row[2]]; v+=[row[3]]
        #c1+=[row[3]]; c2+=[row[4]]; c3+=[row[5]]; v+=[row[6]]
    
#Lat=np.asarray(Lat,dtype=np.float32);Lon=np.asarray(Lon,dtype=np.float32)
x=np.asarray(x,dtype=np.float32);y=np.asarray(y,dtype=np.float32)
th=np.asarray(th,dtype=np.float32)
#c1=np.asarray(c1,dtype=np.float32); c2=np.asarray(c2,dtype=np.float32); c3=np.asarray(c3,dtype=np.float32)
v= np.asarray(v,dtype=np.float32)
end=-1
x=x[1: end]
y=y[1: end]
th=th[1: end]
v=v[1: end]

xhat = savgol_filter(x, 51, 3)
yhat=  savgol_filter(y, 51, 3)
thhat = savgol_filter(th, 51, 3)

#yhat=yhat*0 ; thhat=thhat*0


# origin = [Lat[0], Lon[0]]
# [xx,yy] = ll2xy(Lat,Lon,origin[0],origin[1])

plt.figure()
#plt.scatter(x, th*180/np.pi)
plt.plot(xhat, yhat,'k', linewidth=0.3)
#plt.axis('equal')

plt.figure()
plt.plot(xhat, thhat*180/np.pi,'b', linewidth=0.3)

#plt.plot(x, y,'.',markersize =3)
if 1:
    temp=np.stack((xhat, yhat, thhat, v), axis=1)
    pd.DataFrame(temp).to_csv("./primitives_pid_filtered/reverse_left_filt.csv", header=None, index=None)

# with open('./primitives_filtered/reverse_left_filt.csv', mode='w') as csvfile:
#     c_writer = csv.writer(csvfile, delimiter=',')
#     c_writer.writerow([xhat, yhat, thhat, c1, c2, c3, v])