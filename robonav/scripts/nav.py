#!/usr/bin/env python
# Use a_star algorithm to go to point on map from point on map

from map_reading import MapReadingNode
from a_star import searchalgorithm
from a_star import graph_node
from drive import drive_node
from sensor.msgs.msg import LaserScan, PointCloud
from geometry.msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion



class neato_navigation(object):
	def __init__(self, start, dest, startangle):
		self.start = start
		self.dest = dest
		#self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
	
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
		#self.a_star = searchalgorithm(start,dest)
		#self.a_star.find_path()
		#self.waypoints = self.a_star.waypoins_list


	self.r = rospy.Rate(5)

	def pose_callback(self, s):
		print s

	def check_location(self, expected):
		pass
		# if location != expected:
		# self.drive_node(location, expected, )
		# self.drive_node.run()

	def run(self):
		for i in len(self.waypoints-1):
			if i == 0:
				self.drive_node(start, waypoints[i])
				self.drive_node.run()
			else:
				self.drive_node(waypoints[i-1], waypoints[i])
				self.drive_node.run()


rospy.init_node('nav')
N = neato_navigation((9,50),(40,40))