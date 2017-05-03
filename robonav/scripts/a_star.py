#!/usr/bin/env python
# the a* search algorithm used to traverse the occupancy grid

# IMPORTS
from map_reading import MapReadingNode
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy
import random

# Read Map
map_reading = MapReadingNode()	
# matrix of values from map
node_values = map_reading.grid_map.occupancy_grid


print("completed reading map")

class search_algorithm(object):
	def __init__(self, destination, start):
		""" 
		Initialize Search Algorithm
		tuple destination: position to navigate to
		tuple start: estimated starting position
		"""

		# Before __init__ is complete
		self.initialized = False
		

		# Initialize the ros node
		rospy.init_node('a_star')

		# viz_grid used for map visualizations will be modified in this class
		# Matrix copy for visualizations
		self.viz_grid = copy.deepcopy(node_values)
		# Set new values for colorbar
		self.viz_grid[self.viz_grid > 10] = 10
		self.viz_grid[self.viz_grid == 0] = 1
		self.viz_grid[self.viz_grid < 0] = 0
		



		# Set coordinates of viz_grid corresponding to start and end positions 
		# so that they will stand out in the visualization
		self.viz_grid[start[1], start[0]] = 3
		self.viz_grid[destination[1], destination[0]] = 3

		#Create nodes for the start and destination
		start_node = graph_node(start[0], start[1])
		start_node.g = 0
		self.start_node = start_node
		self.destination = graph_node(destination[0], destination[1])
		self.current_node = start_node # Set current node to start_node
		self.current_node.prev_node = start_node # Initialize the start node's previous node to also be at the start with a 0 g cost.
		self.start_node = start_node
		print(self.start_node)
		self.waypoint_list = [] # the list of waypoints to go to

		# Calculate total cost of the current (starting) node.
		self.total_cost(self.current_node)

		# Initialize list for nodes to search next, dictionary of nodes we've already visited
		self.todo = []	# nodes to search
		self.dead = {}	# nodes we've searched
		# Initialize list of final path of zzzzznodes to visit in order
		self.final = []

	
		# Initialize for visualization purposes
		self.viz_update_count = 0 # How many times has the viz_grid been updated
		self.viz_update_period = 30 # How many viz_grid updates before the visualization is updated?
		
		
		# Initially, the destination has not been reached yet.
		self.destination_reached = False

		

		# Initialization complete
		self.initialized = True
		
		if self.initialized:
			print("Initialized.")


	def get_manhattan(self, node):
		""" 
		Finds manhattan distance of a node to the destination.
		The Manhattan distance is our heuristic.
		"""

		distx = abs(self.destination.x - node.x)
		disty = abs(self.destination.y - node.y)
		node.h = distx +disty
		return node

	def append_g_cost(self, node):
		""" finds g cost of a node, or in our case how many steps from start to reach node"""
		node.g = node.prev_node.g + 1

	def total_cost(self, node):
		""" Sum of manhattan distance and g cost. To be used for todo ranking. """ 
		self.get_manhattan(node)
		self.append_g_cost(node)
		node.cost = node.h + node.g



	def get_neighbors(self):
		"""Create list of neighbor nodes to visit."""
		curr_neighbor_nodes = []

		# If the left neighbor has not been visited yet
		# create a node, set the current node as the previous node, and add it to list of neighbors
		if (self.current_node.x-1, self.current_node.y) not in self.dead:
			left_node = graph_node(self.current_node.x-1, self.current_node.y)
			left_node.prev_node = self.current_node
			curr_neighbor_nodes.append(left_node)
		# If the left neighbor has already been visited
		# Check whether the g cost of reaching it through the current node is smaller. If so, replace the prev_node.
		else:
			if self.dead[(self.current_node.x-1, self.current_node.y)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x-1, self.current_node.y)].prev_node = self.current_node

		# If the right neighbor has not been visited yet
		# create a node, set the current node as the previous node, and add it to list of neighbors
		if (self.current_node.x+1, self.current_node.y) not in self.dead:
			right_node = graph_node(self.current_node.x+1, self.current_node.y)
			right_node.prev_node = self.current_node
			curr_neighbor_nodes.append(right_node)
		# If the right neighbor has already been visited
		# Check whether the g cost of reaching it through the current node is smaller. If so, replace the prev_node.
		else:
			if self.dead[(self.current_node.x+1, self.current_node.y)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x+1, self.current_node.y)].prev_node = self.current_node

		# If the above neighbor has not been visited yet
		# create a node, set the current node as the previous node, and add it to list of neighbors
		if (self.current_node.x, self.current_node.y-1) not in self.dead:
			up_node = graph_node(self.current_node.x, self.current_node.y-1)
			up_node.prev_node = self.current_node
			curr_neighbor_nodes.append(up_node)
		# If the above neighbor has already been visited
		# Check whether the g cost of reaching it through the current node is smaller. If so, replace the prev_node.
		else:
			if self.dead[(self.current_node.x, self.current_node.y-1)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x, self.current_node.y-1)].prev_node = self.current_node

		# If the below neighbor has not been visited yet
		# create a node, set the current node as the previous node, and add it to list of neighbors
		if (self.current_node.x, self.current_node.y+1) not in self.dead:
			down_node = graph_node(self.current_node.x, self.current_node.y+1)
			down_node.prev_node = self.current_node
			curr_neighbor_nodes.append(down_node)
		# If the below neighbor has already been visited
		# Check whether the g cost of reaching it through the current node is smaller. If so, replace the prev_node.
		else:
			if self.dead[(self.current_node.x, self.current_node.y+1)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x, self.current_node.y+1)].prev_node = self.current_node

		# For all unvisited nodes that have been added to the neighbors list 
		for node in curr_neighbor_nodes: 
			self.total_cost(node) # Calculate the total cost for each neighbor node
		
		self.current_node.neighbors = curr_neighbor_nodes # Set the neighbors of the current node equal to this list.


	def visualize(self, close=False):
		"""
		Visualises algorithm progress.
		and set to new values according to how they are being visualized.
		close input tells us whether  the plot should continuously update or wait for interaction.
		"""
		plt.clf()
		plt.imshow(self.viz_grid, interpolation='nearest')
		plt.title(str(rospy.Time.now()))
		plt.colorbar()
		plt.draw()
		plt.pause(.01)
		plt.show(close)

	def backtrack(self):
		"""
		Once destination is reached, this function creates a list of the optimal path to reach that point.
		This list is self.final.
		"""
		while (self.current_node.x, self.current_node.y) != (self.start_node.x, self.start_node.y):
			print('in backtrack loop')
			self.final.append(self.current_node.prev_node)
			self.current_node = self.current_node.prev_node

	def waypoints(self):
		""" turns the list from backtrack into an actual waypoint """
		prev_diff_x = 0
		prev_diff_y = 0
		counter = 0
		
		for node in reversed(self.final):
			print('node', node)
			diff_x = node.prev_node.x - node.x
			diff_y = node.prev_node.y - node.y
			print('diff x', diff_x, 'diff y', diff_y)
			print('pdif x', prev_diff_x, 'pdif y', prev_diff_y)
			if (diff_x == prev_diff_x) and (diff_y == prev_diff_y):
				prev_diff_x = diff_x
				prev_diff_y = diff_y
				counter = 0
				continue
			elif counter == 3:
				prev_diff_x = diff_x
				prev_diff_y = diff_y
				self.waypoint_list.append((node.x, node.y))
				counter = 0
			elif counter == 0:
				self.waypoint_list.append((node.prev_node.x, node.prev_node.y))
				prev_diff_x = diff_x
				prev_diff_y = diff_y
			else:
				prev_diff_x = diff_x
				prev_diff_y = diff_y
				counter += 1

		self.waypoint_list.append((self.destination.x, self.destination.y))
		for waypoint in self.waypoint_list:
			self.viz_grid[waypoint[1], waypoint[0]] = 8


		

	def find_path(self):
		""" Function to traverse graph to destination using A* algorithm."""
		
		self.viz_update_count = 0
		
		# Until destination is reached
		while self.current_node.x != self.destination.x or self.current_node.y != self.destination.y:	
			# Get neighbors of curr node
			self.get_neighbors()
			# Append each neighbor
			for neighbor in self.current_node.neighbors:
				# If neighbor is available space add to todo list of nodes to visit
				if node_values[neighbor.y, neighbor.x] == 0:
					self.todo.append(neighbor)	
			
			# Skip doubles on todo list (for speed)
			if (self.current_node.x, self.current_node.y) == (self.todo[0].x, self.todo[0].y):
				self.current_node = self.todo.pop(0)

			# Sort todo: smallest total cost fist! 
			self.todo.sort(key=lambda x: x.cost, reverse=False)
			self.dead[self.current_node.x, self.current_node.y]= self.current_node
			

			# Update self.current_node
			self.current_node = self.todo.pop(0)

			# Color visited nodes in visualization
			self.viz_grid[self.current_node.y, self.current_node.x] = 2
			self.viz_update_count += 1
			# Update visualization graph
			if self.viz_update_count % self.viz_update_period == 0:
				self.visualize()

		# Destination has been reached!
		self.destination_reached = True

		print("destination reached")
		print('backtracking')
		self.backtrack()
		self.final
		# Color nodes of final path.
		for node in self.final:
			self.viz_grid[node.y, node.x] = 5
		print('final path', self.final)
		self.waypoints()
		print('waypoints', self.waypoint_list)
		self.visualize(True)


		
	
class graph_node(object):
	""" the nodes used to actually do stuff"""
	def __init__(self, x, y):
		self.x = x
		self.y = y
		self.g = sys.maxint
		self.h = 0 
		self.cost = 0
		self.value = node_values[x,y]
		self.prev_node = None
		self.neighbors = []

	def __repr__(self):
		return "node at %d,%d" % (self.x, self.y)


		
SA = search_algorithm((9,50),(40,40))
SA.find_path()