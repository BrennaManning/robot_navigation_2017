""" An implementation of an occupancy field, adapted from CompRobo Robot Localization project."""

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
    Occupancy Grid for map.
    """

    def __init__(self, map):
        """ Initialize class. """
        self.map = map   
        self.occupancy_grid = np.zeros((self.map.info.width,self.map.info.height))
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                self.occupancy_grid[i, j] =self.map.data[ind]    