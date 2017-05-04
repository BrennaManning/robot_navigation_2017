#!/usr/bin/env python
# Use a_star algorithm to go to point on map from point on map
import rospy
from map_reading import MapReadingNode
from a_star import search_algorithm
print "HELLO"
from a_star import graph_node
# from drive import drive_node
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
import time
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy
import random




class neato_navigation(object):
	def __init__(self, start=(9, 50), dest=(40,40)): # , start, dest, startangleamcl_pose 
		self.map_reading = MapReadingNode()	
		self.nav_map = self.map_reading.grid_map
		self.nav_map_info = self.map_reading.info
		print self.nav_map_info
		self.start = start
		self.dest = dest
		self.node_values = self.nav_map.occupancy_grid
		self.a_star = search_algorithm(start,dest)
		self.a_star.find_path()
		self.waypoints = self.a_star.waypoint_list
		self.waypoints_meters = []
		self.x = 0
		self.y = 0

		self.vizx = 0
		self.vizy = 0

		self.quaternion = []
		self.updates = 0
		self.viz_count = 0


		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pos)

		self.xm = 0 # x position in  meters
		self.ym = 0 # y position in meters

		self.r = rospy.Rate(5)
		print('this is doing something')

		self.tf = TransformListener



	def visualize(self, close=False):
		"""
		Visualises neato locatoin
		"""
		viz_grid = self.a_star.viz_grid
		plt.clf()
		plt.imshow(viz_grid, interpolation='nearest')
		plt.title(str(rospy.Time.now()))
		plt.colorbar()
		plt.draw()
		plt.pause(.01)
		plt.show(close)

	def update_pos(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.get_position_meters()
		self.vizx = (self.xm/self.nav_map_info.resolution)
		self.vizy = (self.ym/self.nav_map_info.resolution)
		self.a_star.viz_grid[int(self.vizx), int(self.vizy)] = 9
		# print("POSITION :", self.xm, "meters ",self.ym, "meters" )
		# print("UPDATING VIZ POS TO :", int(self.vizy), int(self.vizx))
		self.angle = euler_from_quaternion(self.quaternion)[2]
		print('angle', self.angle)
		#self.visualize()



		# print('msg frame', msg.header.frame_id)
		# t = self.tf.getLatestCommonTime(msg.header.frame_id, "map")
		# position, quaternion = self.tf.lookupTransform(msg.header.frame_id, "map", t)
		# print('transform stuff')
		# print position, quaternion


		return



	def get_position_meters(self):
		
		self.xm = self.x - self.nav_map_info.origin.position.x 
		self.ym = self.y - self.nav_map_info.origin.position.y 
		print('positions', self.xm, self.ym)
		
	def get_waypoints_meters(self):
		for waypoint in self.waypoints:
			# print('non metered waypoint', waypoint)
			waypoint_meter_x = waypoint[1] * self.nav_map_info.resolution - self.nav_map_info.origin.position.x * self.nav_map_info.resolution
			waypoint_meter_y = waypoint[0] * self.nav_map_info.resolution - self.nav_map_info.origin.position.y * self.nav_map_info.resolution
			waypoint_meter = (waypoint_meter_x, waypoint_meter_y)
			self.waypoints_meters.append(waypoint_meter)
			print('metered waypoint', waypoint_meter)

	def check_location(self, expected):
		pass
		# if location != expected:
		# self.drive_node(location, expected, )
		# self.drive_node.run()

	def runner(self):
		self.get_waypoints_meters()
		r = rospy.Rate(0.5)
		while not rospy.is_shutdown():
			#print("POSITION ", self.xm, self.ym)
			self.visualize()
			r.sleep()

			pass
	# def run(self):
	# 	for i in len(self.waypoints-1):
	# 		if i = 0:
	#     		self.drive_node(start, waypoints[i])
	#     		self.drive_node.run()
	#     	else:
	#     		self.drive_node(waypoints[i-1], waypoints[i])
	#     		self.drive_node.run()




rospy.init_node('nav')

N = neato_navigation() # (9,50),(40,40)
print "LINE 81"
N.runner()
