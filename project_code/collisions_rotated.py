# all theta values are respect to the positive x axis and increment counterclockwise
# all numerical measurements are expressed in meters
from shapely.geometry import Polygon
from shapely import affinity
import matplotlib.pyplot as plt
import math
import numpy as np

class Car:
	def __init__(self, x, y, length, width, theta):
		self.x = x
		self.y = y
		self.length = length
		self.width = width
		self.theta = np.degrees(theta) - 90

def collided(our_car, c1, c2):
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

	if not (our_car.intersects(car1_p) or our_car.intersects(car2_p)):
		plt.plot(*our_car.exterior.xy)
	plt.plot(*car1_p.exterior.xy)
	plt.plot(*car2_p.exterior.xy)
	plt.xlim([-10, 10])
	plt.ylim([-10, 10])
	plt.figure(1, figsize=(3, 3))
	plt.show()

	return (our_car.intersects(car1_p) or our_car.intersects(car2_p))

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


def Collision_virtual_check(x,y,th, car1 = Car(-2, 3, 4, 2.5, 0)	, car2 = Car(8, 3, 4, 2.5, 0) ):
    
    my_car = Car(x, y, 2.62, 1.41, th) ###### x, y reverse theta negative goal(3, 4.5)
    
    	# arbitrary numbers, can be changed
    
    
    
    
    # Perpendicular park , car1 = Car(4, 2, 4.6, 1.5, 90), car2 = Car(4, 7, 4.7, 1.7, 90)
    
    POLARIS_LEN = 2.62
    POLARIS_WID = 1.41
    
    # cars should be objects, make car class
    # 3d tracking (animated plot)
    # see if the RRT expands around the car obstacles
    
    collision = collided(my_car, car1, car2)
    return collision
    
    
if __name__ == '__main__':
    
    x=4; y=3; th=0 ## Parallel parking
    Collision_virtual_check(x,y,th)