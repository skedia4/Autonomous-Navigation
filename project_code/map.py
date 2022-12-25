from nav_msgs.msg import OccupancyGrid
#from exceptions import IndexError
from geometry_msgs.msg import Point
import rospy
import math

def rotate(x,y,xo,yo,theta): #rotate x,y around xo,yo by theta (rad)
    xr=math.cos(theta)*(x-xo)-math.sin(theta)*(y-yo)   + xo
    yr=math.sin(theta)*(x-xo)+math.cos(theta)*(y-yo)  + yo
    return [xr,yr]
def car_vertices(h,k,l,w,yaw):

    points_c=[[h+l/2.0,k+w/2.0],[h+l/4.0,k+w/2.0],[h,k+w/2.0],[h-l/4.0,k+w/2.0],[h-l/2.0,k+w/2.0],
        [h+l/2.0,k+w/4.0],[h-l/2.0,k+w/4.0],[h+l/2.0,k],[h-l/2.0,k],[h+l/2.0,k-w/4.0],[h-l/2.0,k-w/4.0],
              [h+l/2.0,k-w/2.0],[h+l/4.0,k-w/2.0],[h,k-w/2.0],[h-l/4.0,k-w/2.0],[h-l/2.0,k-w/2.0]]

    points_v=[]

    for i in range(len(points_c)):
        points_v.append(rotate(points_c[i][0],points_c[i][1],h,k,yaw))

    return points_v


class Map:
    def __init__(self, grid_map):
        self.map = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution

        self.car_width= 1.41 # meters
        self.car_length=2.62 # meters


        self.origin = Point()
        self.origin.x = grid_map.info.origin.position.x
        self.origin.y = grid_map.info.origin.position.y

    def get_by_index(self, i, j):
        return self.map.data[i*self.width + j]

    # i is for row (y), j is for col (x)
    def get_by_coord(self, x, y):
        return self.get_by_index(*self.coord_to_indices(x, y))

    def coord_to_indices(self, x, y):
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return (i, j)

    def are_indices_in_range(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def is_allowed(self, state):

        vertices=car_vertices(state[0],state[1],self.car_length,self.car_width,state[2])
        for n in range(len(vertices)):
            i, j = self.coord_to_indices(vertices[n][0], vertices[n][1])
            cell = self.get_by_index(i, j)
            if cell == 100 or cell == -1:
                return 0

        return 1



