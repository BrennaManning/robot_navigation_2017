""" An implementation of an occupancy field that you can use to implement
    your particle filter's laser_update function """

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors

class OccupancyGrid(object):
    """ 
    """

    def __init__(self, map):
        self.map = map      # save this for later

        # build up a numpy array of the coordinates of each grid cell in the map
        self.occupancy_grid = np.zeros((self.map.info.width,self.map.info.height))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                self.occupancy_grid[i, j] =self.map.data[ind]    