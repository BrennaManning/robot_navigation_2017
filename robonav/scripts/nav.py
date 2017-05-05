#!/usr/bin/env python
# Use a_star algorithm to go to point on map from point on map

# IMPORTS

# ROS
import rospy
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

# Transforms
import tf
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler

# General
import matplotlib.pyplot as plt
import numpy as np

# Other classes
from map_reading import MapReadingNode
from drive import drive_node
from a_star import search_algorithm
from a_star import graph_node


class neato_navigation(object):
	""" The main neato navigation class: bringing it all together"""
	def __init__(self, dest=(40,40)):
		self.initialized = False

		# Goal Destination
		self.dest = dest

		# Map
		self.map_reading = MapReadingNode()	
		self.nav_map = self.map_reading.grid_map
		self.nav_map_info = self.map_reading.info
		
		# Later used for a_star class
		self.a_star = None

		# Lists for generated waypoints
		self.waypoints = []
		self.waypoints_meters = []

		# Pos from amcl builtin localizer
		self.x = 0
		self.y = 0

		# Pos converted to visualization graph coordinate frame
		self.vizx = 0
		self.vizy = 0

		# Pos in pixels
		self.xp = 0
		self.yp = 0

		# Pos converted to meters
		self.xm = 0 
		self.ym = 0 

		# Angle from localizer
		self.quaternion = []
		self.angle = 0
		
		# Subscribers
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pos)
		rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial)

		self.r = rospy.Rate(15)

		print("Please choose an initial pose")


	def update_viz(self):
		"""
		Color points that the neato has been visited on the matrix for visualization plot.
		"""
		if not (self.initialized):

			return
		if not (self.a_star):
			return
		self.vizx = (self.xm/self.nav_map_info.resolution)
		self.vizy = (self.ym/self.nav_map_info.resolution)
		self.a_star.viz_grid[int(self.vizx), int(self.vizy)] = 9
		#self.visualize() #Uncomment for RC path following w/ teleop

		return

	def visualize(self, close=False):
		"""
		Visualises neato location over path generated from A* 
		"""
		viz_grid = self.a_star.viz_grid
		plt.clf()
		plt.imshow(viz_grid, interpolation='nearest')
		plt.title(str(rospy.Time.now()))
		plt.colorbar()
		plt.draw()
		plt.pause(.01)
		plt.show(close)

	def update_initial(self, msg):
		"""
		Updates the initial pose: when 2D pose estimate is made in RVIZ
		"""
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

		self.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.angle = euler_from_quaternion(self.quaternion)
		print('updating the initial')
		self.initialized = True

	

	def update_pos(self, msg):
		"""
		Callback function to update the position when position in localizer.
		"""
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		self.get_position_meters()
		self.angle = euler_from_quaternion(self.quaternion)[2]
		
		self.update_viz() 


	def get_position_meters(self):
		"""
		Convert position from raw particle filter input to position in meters.
		"""
		if not (self.initialized):

			return
		self.xm = self.x - self.nav_map_info.origin.position.x  
		self.ym = self.y - self.nav_map_info.origin.position.y 

	def get_position_pixels(self):
		""" Get current location pixel coordinates. """
		self.xp = int((-self.nav_map_info.origin.position.x + self.x) / self.nav_map_info.resolution)
		self.yp = int((-self.nav_map_info.origin.position.y + self.y) / self.nav_map_info.resolution)

	def get_pixels_meters(self, x, y):
		""" Convert any x and y coordinates (in meters) to pixels. """
		xp = x / self.nav_map_info.resolution
		yp = y / self.nav_map_info.resolution
		return xp, yp
		
	def get_waypoints_meters(self):
		""" Converts all waypoints in waypoints list to units of meters """
		if not (self.a_star):
			return
		for waypoint in self.a_star.waypoint_list:
			waypoint_meter_x = waypoint[1] * self.nav_map_info.resolution + self.nav_map_info.origin.position.x * self.nav_map_info.resolution
			waypoint_meter_y = waypoint[0] * self.nav_map_info.resolution + self.nav_map_info.origin.position.y * self.nav_map_info.resolution
			waypoint_meter = (waypoint_meter_x, waypoint_meter_y)
			self.waypoints_meters.append(waypoint_meter)

	def driver(self, destination):
		"""drives to a destination using drive_node"""
		if not (self.initialized):
			return
		if not (self.a_star):
			return

		d = drive_node((self.ym, self.xm), destination, self.angle)
		d.run()

	def a_star_creation(self, destination):
		""" Finds a path using a_star. """ 
		if not (self.initialized):
			return

		if (self.a_star):
			return
		# initialize search algorithm
		self.a_star = search_algorithm((int(self.yp), int(self.xp)), destination)
		# find path
		self.a_star.find_path()
	

	def runner(self):
		""" Main run loop """
		r = rospy.Rate(0.5)
		destination_reached = False

		while not (rospy.is_shutdown()) and destination_reached == False:
			self.get_position_meters()
			self.get_position_pixels()
			dx, dy = self.get_pixels_meters(self.xm + 1, self.ym + 1)
			test_dest = (80, 80)
			self.a_star_creation(test_dest)
			self.get_waypoints_meters()
			

			# Comment this block for RC path following w/ teleop 
			if not (destination_reached):
				
				if len(self.waypoints_meters)>0:
					for point in self.waypoints_meters:
						print('waypoint', point)
						print('current pos', self.ym, self.xm)
						print ("POINT", point)
						self.driver((point[0],point[1]))
						print ("reached point ", point)
						self.visualize()
						rospy.sleep(2)
					destination_reached = True
					print("destination_reached: ", destination_reached)
			# End Comment for RC path following w/ teleop
			
			r.sleep


if __name__ == '__main__':
	rospy.init_node('nav')
	N = neato_navigation(dest=(13,60)) # (9,50),(40,40)
	N.runner()
