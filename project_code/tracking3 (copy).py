from matplotlib import pyplot as plt
from matplotlib import animation
import math
import numpy as np
import regex as re
import random
import pickle
from shapely.geometry import Polygon
from shapely import affinity

import csv


x2 = 5
y2 = 15
len2 = 4.7
wid2 = 1.7
theta2 = -20

POLARIS_LEN = 2.62
POLARIS_WID = 1.41

# make true if you only want to plot the car positions that will cause a collision
plot_only_colliding_flag = False

class Car:
	def __init__(self, x, y, length, width, theta):		# theta in radians
		self.x = x
		self.y = y
		self.length = length
		self.width = width
		self.theta = 90 + (theta * 180 / math.pi)	# TODO: convert to radians

# theta = 0 means the cars are pointing to the right
my_car = Car(0, 0, 2.62, 1.41, 0)	# our car starts off at (0,0) and pointing to the right

car1 = Car(2, 4, 4, 2.5, 1.5708) ; car2 = Car(7, 4, 4, 2.5, 1.5708)	 # perpendicular parking
#car1 = Car(-2, 3, 4, 2.5, 0); car2 = Car(8, 3, 4, 2.5, 0)  #paralle parking



def collided(our_car, c1 = Car(-1, 4, 4.6, 1.5, 0), c2 = Car(4, 4, 4.7, 1.7, 0)):
	my_car_final = Car(float(final_dest[0]), float(final_dest[1]), 2.62, 1.41, float(final_dest[2]))
	print(float(final_dest[2]))

	x = my_car_final.x
	y = my_car_final.y
	length = my_car_final.length
	width = my_car_final.width
	theta = my_car_final.theta

	ourf_top_left = (x - (width / 2), y + (length / 2))
	ourf_top_right = (x + (width / 2), y + (length / 2))
	ourf_bottom_left = (x - (width / 2), y - (length / 2))
	ourf_bottom_right = (x + (width / 2), y - (length / 2))
	
	if theta != 0:
		ourf_top_left_rotated = my_rotate(ourf_top_left[0], ourf_top_left[1], x, y, theta)
		ourf_top_right_rotated = my_rotate(ourf_top_right[0], ourf_top_right[1], x, y, theta)
		ourf_bottom_left_rotated = my_rotate(ourf_bottom_left[0], ourf_bottom_left[1], x, y, theta)
		ourf_bottom_right_rotated = my_rotate(ourf_bottom_right[0], ourf_bottom_right[1], x, y, theta)
		our_car_final = Polygon([ourf_top_left_rotated, ourf_top_right_rotated, ourf_bottom_right_rotated, ourf_bottom_left_rotated])
		
	else:
		our_car_final = Polygon([ourf_top_left, ourf_top_right, ourf_bottom_right, ourf_bottom_left])

	# four non-rotated points of our car
	x = our_car.x
	y = our_car.y
	length = our_car.length
	width = our_car.width
	theta = our_car.theta

	our_top_left = (x - (width / 2), y + (length / 2))
	our_top_right = (x + (width / 2), y + (length / 2))
	our_bottom_left = (x - (width / 2), y - (length / 2))
	our_bottom_right = (x + (width / 2), y - (length / 2))
	
	if theta != 0:
		our_top_left_rotated = my_rotate(our_top_left[0], our_top_left[1], x, y, theta)
		our_top_right_rotated = my_rotate(our_top_right[0], our_top_right[1], x, y, theta)
		our_bottom_left_rotated = my_rotate(our_bottom_left[0], our_bottom_left[1], x, y, theta)
		our_bottom_right_rotated = my_rotate(our_bottom_right[0], our_bottom_right[1], x, y, theta)
		our_car = Polygon([our_top_left_rotated, our_top_right_rotated, our_bottom_right_rotated, our_bottom_left_rotated])
		
	else:
		our_car = Polygon([our_top_left, our_top_right, our_bottom_right, our_bottom_left])

	x = c1.x
	y = c1.y
	length = c1.length
	width = c1.width
	theta = c1.theta

	# four non-rotated points of car1
	car1_top_left = (x - (width / 2), y + (length / 2))
	car1_top_right = (x + (width / 2), y + (length / 2))
	car1_bottom_left = (x - (width / 2), y - (length / 2))
	car1_bottom_right = (x + (width / 2), y - (length / 2))

	if theta != 0:
		car1_top_left_rotated = my_rotate(car1_top_left[0], car1_top_left[1], x, y, theta)
		car1_top_right_rotated = my_rotate(car1_top_right[0], car1_top_right[1], x, y, theta)
		car1_bottom_left_rotated = my_rotate(car1_bottom_left[0], car1_bottom_left[1], x, y, theta)
		car1_bottom_right_rotated = my_rotate(car1_bottom_right[0], car1_bottom_right[1], x, y, theta)
		car1_p = Polygon([car1_top_left_rotated, car1_top_right_rotated, car1_bottom_right_rotated, car1_bottom_left_rotated])
		
	else:
		car1_p = Polygon([car1_top_left, car1_top_right, car1_bottom_right, car1_bottom_left])

	x = c2.x
	y = c2.y
	length = c2.length
	width = c2.width
	theta = c2.theta

	# four non-rotated points of car2
	car2_top_left = (x - (width / 2), y + (length / 2))
	car2_top_right = (x + (width / 2), y + (length / 2))
	car2_bottom_left = (x - (width / 2), y - (length / 2))
	car2_bottom_right = (x + (width / 2), y - (length / 2))

	if theta != 0:
		car2_top_left_rotated = my_rotate(car2_top_left[0], car2_top_left[1], x, y, theta)
		car2_top_right_rotated = my_rotate(car2_top_right[0], car2_top_right[1], x, y, theta)
		car2_bottom_left_rotated = my_rotate(car2_bottom_left[0], car2_bottom_left[1], x, y, theta)
		car2_bottom_right_rotated = my_rotate(car2_bottom_right[0], car2_bottom_right[1], x, y, theta)
		car2_p = Polygon([car2_top_left_rotated, car2_top_right_rotated, car2_bottom_right_rotated, car2_bottom_left_rotated])
		
	else:
		car2_p = Polygon([car2_top_left, car2_top_right, car2_bottom_right, car2_bottom_left])

	if plot_only_colliding_flag:
		if (our_car.intersects(car1_p) or our_car.intersects(car2_p)):
			plt.plot(*our_car.exterior.xy)
	else:
		plt.plot(*our_car.exterior.xy)
	plt.plot(*our_car_final.exterior.xy)
	plt.plot(*car1_p.exterior.xy)
	plt.plot(*car2_p.exterior.xy)
	# plt.xlim([-7, 7])
	# plt.ylim([-7, 7])
	# plt.show()

	return (our_car.intersects(car1_p) or our_car.intersects(car2_p))




def read_waypoints(filename =  'Current_Trajectory.csv'):


	with open(filename) as f:
	    path_points = [tuple(line) for line in csv.reader(f)]

	# x towards East and y towards North
	path_points_x   = np.array([float(point[0]) for point in path_points]) # x
	path_points_y   = [float(point[1]) for point in path_points] # y

	return path_points_x[0], path_points_y[0]

def my_rotate(x, y, center_x, center_y, theta):
	# center_x = center_y = 0
	# center_x, center_y - center of square coordinates
	# x, y - coordinates of a corner point of the square
	# theta is the angle of rotation
	theta_rad = math.pi * theta / 180

	# translate point to origin
	tempX = x - center_x;
	tempY = y - center_y;

	# now apply rotation
	rotatedX = tempX * math.cos(theta_rad) - tempY * math.sin(theta_rad);
	rotatedY = tempX * math.sin(theta_rad) + tempY * math.cos(theta_rad);

	# translate back
	new_x = rotatedX + center_x;
	new_y = rotatedY + center_y;

	return (new_x, new_y)

final_dest = []

def set_final_dest(value):
    global final_dest
    final_dest = value


#filename = 'Trajectory_DEC_17_parallel_found_0'
#filename ='Trajectory_DEC_17_live_map_found_0'
#filename = 'Trajectory_DEC_15_perpendicular_decent'
#filename ='./Live_plans/Trajectory_test_plan_virtual_0'

def traj_csv2( filename = './Live_plans/Trajectory_test_plan_virtual_0', Start_x=0, Start_y=0, plot_flag= True, csv_convert_flag= False):
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
    # print(motion_seq)
    if plot_flag:
        fig = plt.figure()
        plt.plot(Start_x, Start_y,'.', color='b', markersize =20)     ################### Start
        plt.plot(Nodes_all[:-1,0], Nodes_all[:-1,1],'.',color='g' , markersize =5)
        plt.plot(Nodes_all[-1,0], Nodes_all[-1,1],'.', color='r', markersize =20)################### Goal
        plt.plot(Edges_all[:,0], Edges_all[:,1],'r', linewidth=0.7)

    # TODO: plot our car on final destination and along all waypoints, add a flag

    Nodes_all=np.array(Nodes_all, dtype='<U32')
    pid_seq=np.stack((Nodes_all[:,0], Nodes_all[:,1], Nodes_all[:,2], motion_seq),axis=1)
    pid_seq = pid_seq.tolist()

    set_final_dest(pid_seq[len(pid_seq) - 1])

    #print(pid_seq)

    return fig, plt



def update_points(fig, x, y, points):
    
    
    new_x, new_y = read_waypoints() 

    #new_x = random.randrange(-1, 4)
    #new_y = random.randrange(-1, 4)

    # update properties
    points.set_data(new_x,new_y)
    # print("updating")

fig, plt = traj_csv2()

#plt.xlim([-15,5]) 		# can adjust, values are in meters
#plt.ylim([-5,5]) 		# can adjust, values are in meters

x = random.randrange(-1, 4)
y = random.randrange(-1, 4)
points, = plt.plot(x, y, '*',markersize=15, color= 'g')
ani = animation.FuncAnimation(fig, update_points, frames = 10, fargs = (x, y, points))

collision = collided(my_car, car1, car2)
print("Collision?", collision)

# X = np.linspace(0,5,100)
# Y1 = X + 2*np.random.random(X.shape)
# plt.scatter(X,Y1,color='k')
plt.figure(1, figsize=(1, 1))
plt.show()