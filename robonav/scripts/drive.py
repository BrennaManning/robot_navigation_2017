#!/usr/bin/env python
# the neato drives to given point from given coordinate.

# IMPORTS
from map_reading import MapReadingNode
import rospy
from geometry_msgs.msg import Vector3, Twist, Point, Pose
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy
import random
import math


class drive_node(object):
    def __init__(self, start, destination, startangle):
        """ Initialize node"""
        self.initialize = False
        
        self.r = rospy.Rate(5)

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time = rospy.Time.now()

        # Initial values
        self.startangle = startangle
        self.current_pos = start
        self.destination = destination
        self.destination_reached = False    

        # Speeds
        self.turnspeed = math.pi/16 
        self.drive_speed = .1   

        # Start not moving
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
            if ydiff == 0:
                target_angle = math.pi/2
            else:
                target_angle = np.arctan(xdiff/ydiff) # In radians
            angle = target_angle - self.startangle
            if angle > 0 and angle < 3.14:
                print "destination is to the right"
                self.turnspeed = 1* abs(self.turnspeed)
            else:
                print "destination is to the left"
                self.turnspeed = -1* abs(self.turnspeed)

            print "destination " + str(angle) + " radians away"
            turn_time = abs(angle/self.turnspeed) #radians/(radians/second) = seconds
           
            self.linear_twist  = Vector3(0, 0, 0)
            self.time = rospy.Time.now()

            print "turn time = " + str(turn_time)
            while(rospy.Time.now() - self.time < rospy.Duration(turn_time)):
                self.angular_twist = Vector3(0, 0, self.turnspeed)
                self.publish_twist()
                
        self.angular_twist = Vector3(0,0,0)
        print "done turning"
        self.publish_twist()


    def drive(self):
        """ The function to drive forward towards the destination. """
        xdiff = self.destination[0] - self.current_pos[0] 
        ydiff = self.destination[1] - self.current_pos[1]

        distance = (math.sqrt(xdiff**2 + ydiff**2)) # meters
        print ("distance = ", distance)

        drive_time = abs(distance/self.drive_speed)
        self.time = rospy.Time.now()
        while(rospy.Time.now() - self.time < rospy.Duration(drive_time)):
                self.linear_twist = Vector3(self.drive_speed, 0, 0)
                self.publish_twist()
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
                return
        self.stop()


if __name__ == '__main__':
    rospy.init_node('go_to_test')
    time = rospy.Time.now()
    node = drive_node((0,0),(1,1), 0)
    node.run()