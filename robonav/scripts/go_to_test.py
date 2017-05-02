#!/usr/bin/env python
# the neato drives to given point from given coordinate.

from map_reading import MapReadingNode
import rospy
from geometry_msgs.msg import Vector3, Twist, Point, Pose
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy
import random
import math

map_reading = MapReadingNode()    
node_values = map_reading.grid_map.occupancy_grid

print("completed reading map")

rospy.init_node('go_to_test')
class drive_node(object):
    def __init__(self, destination, start, startangle):
    	""" Initialize node"""
        self.initialize = False
        map_reading = MapReadingNode()    
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time = rospy.Time.now()
        self.current_pos = start
        self.current_angle = 0
        self.destination = destination
        self.destination_reached = False    

        self.turnspeed = math.pi/16 #rad/s
        self.drive_speed = .2 #m/s    

        self.angular_twist = Vector3(0,0,0)
        self.linear_twist = Vector3(0,0,0)

        self.initialize = True
        print("initialized? ", self.initialize)

    def stop(self):
        """ This function is called on shutdown and publishes
        a twist with linear and angular velocities of 0 to stop the Neato."""
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))

    def turn(self):
    	""" The function to turn towards the destination """
    	print "turning"
        if self.destination[0] == self.current_pos[0]:
            turn = 0
            print "destination straight ahead"
        else:
            xdiff = self.destination[0] - self.current_pos[0] 
            ydiff = self.destination[1] - self.current_pos[1]
            angle = np.arctan(xdiff/ydiff) # In radians
            if xdiff > 0:
            	print "destination is to the right"
            	self.turnspeed = -1 * abs(self.turnspeed)
            else:
            	print "destination is to the left"
            	self.turnspeed = abs(self.turnspeed)

            print "destination " + str(angle) + " radians away"
            turn_time = abs(angle/self.turnspeed) #radians/(radians/second) = seconds
            #turn_time = 4
            #turn_speed = 2*math.pi/8
            self.linear_twist  = Vector3(0, 0, 0)
            self.time = rospy.Time.now()
            #self.turnspeed = 1
            print "turn time = " + str(turn_time)
            while(rospy.Time.now() - self.time < rospy.Duration(turn_time)):
                self.angular_twist = Vector3(0, 0, self.turnspeed)
                self.publish_twist()
                
        self.angular_twist = Vector3(0,0,0)
        print "done turning"
        self.publish_twist()


    def drive(self):
    	""" The function to drive forward towards the destination. """
    	print "driving forward"
    	xdiff = self.destination[0] - self.current_pos[0] 
        ydiff = self.destination[1] - self.current_pos[1]

    	distance = math.sqrt(xdiff**2 + ydiff**2) # meters
    	print ("distance = ", distance)

    	drive_time = abs(distance/self.drive_speed)
    	self.time = rospy.Time.now()
    	while(rospy.Time.now() - self.time < rospy.Duration(drive_time)):
                self.linear_twist = Vector3(self.drive_speed, 0, 0)
                self.publish_twist()
                print "driving for " + str(drive_time) + " seconds"
        self.destination_reached = True
        print "done driving forward"
    def publish_twist(self):
        self.publisher.publish(Twist(linear=self.linear_twist, angular=self.angular_twist))
      
    def run(self):
        """This function is the main run loop"""
        print "running"
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            self.r.sleep()
            if not self.destination_reached:
                self.turn()
                self.drive()
            else:
                self.stop()
        self.stop()

rospy.init_node('go_to_test')
time = rospy.Time.now()
node = drive_node((1,1),(0,0), 0)
node.run()