#!/usr/bin/env python
# Use a_star algorithm to go to point on map from point on map
import rospy
# from map_reading import MapReadingNode
# from a_star import searchalgorithm
# from a_star import graph_node
# from drive import drive_node
from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
import time

class neato_navigation(object):
	def __init__(self): # , start, dest, startangleamcl_pose 
		# self.start = start
		# self.dest = dest
		# self.a_star = searchalgorithm(start,dest)
		# self.a_star.find_path()
		# self.waypoints = self.a_star.waypoins_list
		self.pos = 0
		self.updates = 0

		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pos)

		self.r = rospy.Rate(5)
		print('this is doing something')


	def update_pos(self, msg):
		self.pos = msg.pose.pose.position

	def check_location(self, expected):
		pass
		# if location != expected:
		# self.drive_node(location, expected, )
		# self.drive_node.run()


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
N.runner()