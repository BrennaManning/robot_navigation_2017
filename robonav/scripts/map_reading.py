#!/usr/bin/env python

""" Read from a map.  """


import rospy
from occupancy_grid import OccupancyGrid
from nav_msgs.srv import GetMap 

class MapReadingNode(object):
    """ A node to read from the map"""

    def __init__(self):
        """ Intitialize map reading node. """
        rospy.wait_for_service('static_map')
        try:
            static_map = rospy.ServiceProxy('static_map', GetMap)
            resp = static_map()
            map = resp.map
            if map is not None:
                print "========================="
                print ("MAP TYPE:,", type(map))
                print "========================="
            else:
                print "no map :("
            self.grid_map = OccupancyGrid(map)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        self.initialized = True


mymap = MapReadingNode()