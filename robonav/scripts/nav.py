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
from drive import drive_node
import random




class neato_navigation(object):
	def __init__(self, dest=(40,40)): # , start, dest, startangleamcl_pose 
		self.initialized = False
		self.map_reading = MapReadingNode()	
		self.nav_map = self.map_reading.grid_map
		self.nav_map_info = self.map_reading.info
		print self.nav_map_info
		# self.start = start
		self.dest = dest
		self.node_values = self.nav_map.occupancy_grid
		self.a_star = None
		self.waypoints = []
		self.waypoints_meters = []

		self.x = 0
		self.y = 0

		self.vizx = 0
		self.vizy = 0

		self.quaternion = []
		self.angle = 0

		self.updates = 0
		self.viz_count = 0

		self.xp = 0
		self.yp = 0

		self.path_found = False

		self.xm = 0 # x position in  meters
		self.ym = 0 # y position in meters

		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pos)

		rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial)

		self.r = rospy.Rate(15)
		print('this is doing something')

		self.tf = TransformListener

	def update_initial(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

		self.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.angle = euler_from_quaternion(self.quaternion)
		print('updating the initial')
		self.initialized = True

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
		self.angle = euler_from_quaternion(self.quaternion)[2]
		print('angle', self.angle)
		# if self.a_star:
		self.update_viz()


	def update_viz(self):
		if not (self.initialized):

			return
		if not (self.a_star):
			return
		# print('visualizing')
		self.vizx = (self.xm/self.nav_map_info.resolution)
		self.vizy = (self.ym/self.nav_map_info.resolution)
		self.a_star.viz_grid[int(self.vizx), int(self.vizy)] = 9

		# print("POSITION :", self.xm, "meters ",self.ym, "meters" )
		# print("UPDATING VIZ POS TO :", int(self.vizy), int(self.vizx))
			#self.visualize()
		# print('initializing', self.xm, self.ym)



		# print('msg frame', msg.header.frame_id)
		# t = self.tf.getLatestCommonTime(msg.header.frame_id, "map")
		# position, quaternion = self.tf.lookupTransform(msg.header.frame_id, "map", t)
		# print('transform stuff')
		# print position, quaternion


		return



	def get_position_meters(self):
		if not (self.initialized):

			return
		self.xm = self.x - self.nav_map_info.origin.position.x # mult by res? 
		self.ym = self.y - self.nav_map_info.origin.position.y 
		print('positions', self.xm, self.ym)

	def get_position_pixels(self):
		self.xp = int((-self.nav_map_info.origin.position.x + self.x) / self.nav_map_info.resolution)
		self.yp = int((-self.nav_map_info.origin.position.y + self.y) / self.nav_map_info.resolution)

	def get_pixels_meters(self, x, y):
		xp = x / self.nav_map_info.resolution
		yp = y / self.nav_map_info.resolution
		return xp, yp
		
	def get_waypoints_meters(self):
		if not (self.a_star):
			return
		for waypoint in self.a_star.waypoint_list:
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

	def driver(self, destination):
		"""drives to a destination"""
		if not (self.initialized):
			# print('not initialized yet')
			return
		# print('driving')
		if not (self.a_star):
			return
		d = drive_node((self.ym, self.xm), destination, self.angle)
		d.run()

	def a_star_creation(self, destination):
		if not (self.initialized):
			return

		if (self.a_star):
			return
		print('doing a_star stuff')
		# x's and y's are flipped for this part
		self.a_star = search_algorithm((int(self.yp), int(self.xp)), destination)
		# self.a_star = search_algorithm((19,60), (13,60))
		self.a_star.find_path()
		self.path_found = True



	def runner(self):
		r = rospy.Rate(0.5)
		# while not rospy.is_shutdown():
		# 	if self.initialized:
		# 		print('in running loop')
		# 		print('self.xm, self.ym', self.xm, self.ym)
		# 		if (self.xm, self.ym) != (None,None):
		# 			print('in if statement')
		# 			self.xm = int(self.xm)
		# 			self.ym = int(self.ym)
		# 			print('coordinates', self.xm, self.ym)
		# 			print('types', type(self.xm), type(self.ym), type(self.dest[0]), type(self.dest[1]))
		# 			self.a_star = search_algorithm((self.xm, self.ym), (self.dest))
		# 			self.a_star.find_path()
		# 			print('find path completed')
		# 			self.get_waypoints_meters()

		# 		print('waypoints in meters', self.waypoints_meters)
		# 	else:
		# 		print('stuck', self.node_values[self.vizy, self.vizx])
		# 		r.sleep()
		# 	while len(self.waypoints_meters) != 0:
		# 		print('in second while loop')
		# 		target = self.waypoints_meters.pop(0)
		# 		self.driver(target)

		# 		self.visualize()
		# 		r.sleep()
		# r.sleep()

		# def run(self):
		# 	for i in len(self.waypoints-1):
		# 		if i = 0:
		#     		self.drive_node(start, waypoints[i])
		#     		self.drive_node.run()
		#     	else:
		#     		self.drive_node(waypoints[i-1], waypoints[i])
		#     	self.drive_node.run()
		driven_to = False
		while not (rospy.is_shutdown()):
			self.get_position_meters()
			self.get_position_pixels()
			dx, dy = self.get_pixels_meters(self.xm + 1, self.ym + 1)
			#test_dest = (13, 60)
			test_dest = (40, 52)
			self.a_star_creation(test_dest)
			self.get_waypoints_meters()
			# print('driving')
			# print('destination', (self.xm + 1, self.ym + 1))
			# print('done driving')
			if len(self.waypoints_meters)>0 and not driven_to:
				point = self.waypoints_meters[0]
				print ("POINT", point)
				# print(point)
				self.driver((point[1],point[0]))
				driven_to = True

			#self.driver((self.xm - 3, self.ym - 3))
			# if (self.path_found):
			# 	print('path found')
			# 	# point = self.waypoints_meters[0]
			# 	print('waypoint', point)
			# 	# self.driver(point)
			# # print('driving to', test_dest, (self.xm, self.ym))
			# 	# rospy.sleep(1)
			# 	print('if statemenet in runner')
			r.sleep


if __name__ == '__main__':
	rospy.init_node('nav')
	N = neato_navigation(dest=(13,60)) # (9,50),(40,40)
	N.runner()
