#!/usr/bin/env python
# the a* search algorithm used to traverse the occupancy grid

from map_reading import MapReadingNode
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np
import copy

map_reading = MapReadingNode()	
node_values = map_reading.grid_map.occupancy_grid
viz_grid = copy.deepcopy(node_values)
viz_grid[viz_grid > 10] = 4
viz_grid[viz_grid == 0] = 1
viz_grid[viz_grid < 0] = 0

print("completed reading map")

class search_algorithm(object):
	def __init__(self, destination, start):
		print("hello a_star")
		self.initialize = False
		rospy.init_node('a_star')
		
		viz_grid[start[0], start[1]] = 3
		viz_grid[destination[0], destination[1]] = 3

		self.destination = graph_node(destination[0], destination[1])
		start_node = graph_node(start[0], start[1])
		start_node.g = 0
		self.current_node = start_node
		self.current_node.prev_node = start_node
		self.start_node = start_node
		print(self.start_node)

		# self.get_manhattan(self.current_node)
		self.total_cost(self.current_node)

		self.todo = []	# nodes to search
		self.dead = {}	# nodes we've searched

		self.all_nodes = []

		self.came_from = {}
		self.final = []

		# making the nodes
		map_reading = MapReadingNode()	
		#self.node_values = map_reading.grid_map.occupancy_grid
		
		#for i in range(1, node_values.shape[0]-1):
		#	for j in range(1, node_values.shape[1]-1):
	#			self.all_nodes.append(graph_node(i,j))

		
		
		self.initialize = True
		print("initialized? ", self.initialize)


	def get_manhattan(self, node):
		""" finds manhattan distance"""
		distx = abs(self.destination.x - node.x)
		disty = abs(self.destination.y - node.y)
		node.h = distx +disty
		# print("manhattan = ", node.h)
		return node

	def append_g_cost(self, node):
		""" Appends g cost, or in our case how many steps to reach node"""
		node.g = node.prev_node.g + 1

	def total_cost(self, node):
		""" Sum of manhattan distance and g cost. To be used for todo ranking. """ 
		self.get_manhattan(node)
		self.append_g_cost(node)
		node.cost = node.h + node.g



	def get_neighbors(self):
		curr_neighbor_nodes = []
		if (self.current_node.x-1, self.current_node.y) not in self.dead:
			left_node = graph_node(self.current_node.x-1, self.current_node.y)
			left_node.prev_node = self.current_node
			# print('addding left node', left_node.prev_node)
			curr_neighbor_nodes.append(left_node)
		else:
			if self.dead[(self.current_node.x-1, self.current_node.y)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x-1, self.current_node.y)].prev_node = self.current_node

		if (self.current_node.x+1, self.current_node.y) not in self.dead:
			right_node = graph_node(self.current_node.x+1, self.current_node.y)
			right_node.prev_node = self.current_node
			# print('addding right node', right_node.prev_node)

			curr_neighbor_nodes.append(right_node)
		else:
			if self.dead[(self.current_node.x+1, self.current_node.y)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x+1, self.current_node.y)].prev_node = self.current_node

		if (self.current_node.x, self.current_node.y-1) not in self.dead:
			up_node = graph_node(self.current_node.x, self.current_node.y-1)
			up_node.prev_node = self.current_node
			# print('addding up node', up_node.prev_node)			
			curr_neighbor_nodes.append(up_node)
		else:
			if self.dead[(self.current_node.x, self.current_node.y-1)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x, self.current_node.y-1)].prev_node = self.current_node


		if (self.current_node.x, self.current_node.y+1) not in self.dead:
			down_node = graph_node(self.current_node.x, self.current_node.y+1)
			down_node.prev_node = self.current_node
			curr_neighbor_nodes.append(down_node)
		else:
			if self.dead[(self.current_node.x, self.current_node.y+1)].prev_node.g > self.current_node.g:
				self.dead[(self.current_node.x, self.current_node.y+1)].prev_node = self.current_node

		
		for node in curr_neighbor_nodes: # for each neighbor
			if node.prev_node.g + 1 < node.g:
				self.came_from[(node.x, node.y)] = node.prev_node
			node.prev_node = self.current_node

		for node in curr_neighbor_nodes:
			self.total_cost(node)
		
		self.current_node.neighbors = curr_neighbor_nodes


	def visualize(self):
		print("updating viz")
		img = plt.imshow(viz_grid)
		plt.colorbar(img)
		plt.imshow(img)
		plt.close()
		print("past plt.show")

	def backtrack(self):
		while (self.current_node.x, self.current_node.y) != (self.start_node.x, self.start_node.y):
			print('in backtrack loop')
			self.final.append(self.current_node.prev_node)
			self.current_node = self.current_node.prev_node
			# print('backtracking',self.current_node)
		# return final

	def find_path(self):
		print("finding path")

		while self.current_node.x != self.destination.x or self.current_node.y != self.destination.y:	
			print(self.current_node)
			# print("loop")
			# Get neighbors of curr node
			self.get_neighbors()
			# Append each neighbor
			for neighbor in self.current_node.neighbors:
				self.todo.append(neighbor)	

			# Sort todo: smallest total cost fist! 
			self.todo.sort(key=lambda x: x.cost, reverse=False)
			#print("TODO ", self.todo)
			self.dead[(self.current_node.x, self.current_node.y)]= self.current_node
			#print ("self.DEAD " ,self.dead)

			# Update self.current_node
			self.current_node = self.todo.pop(0)
			viz_grid[self.current_node.x, self.current_node.y] = 2
			#self.visualize()
			# print("done visualizing")
		print('backtracking')
		self.backtrack()
		print('final path', self.final)

		print("destination reached")
		print("YAYYYY")

	
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
		#print("INITIALIZED NODE ", self.x, self.y, " /WITH VALUE ", self.value)

	def __repr__(self):
		return "node at %d,%d" % (self.x, self.y)


		
SA = search_algorithm((984,1392),(984,1272))
SA.find_path()
# print("TODO ", SA.todo)

# print("CAME FROM DICT ", SA.came_from)

# to do
# iterate through everything
# pathfind