#!/usr/bin/env python
# the a* search algorithm used to traverse the occupancy grid

from map_reading import MapReadingNode
import rospy

class search_algorithm(object):
	def __init__(self, destination, start):
		self.initialize = False
		rospy.init_node('a_star')

		self.destination = destination
		self.current_node = start
		self.todo = []	# nodes to search
		self.dead = []	# nodes we've searched

		self.all_nodes = []

		# making the nodes
		map_reading = MapReadingNode()	
		self.node_values = map_reading.grid_map.occupancy_grid
		for i in range(1, self.node_values.shape[0]-1):
			for j in range(1, self.node_values.shape[1]-1):
				self.all_nodes.append(graph_node(i,j, self.node_values))
		self.initialize = True


	def get_manhattan(self, node):
		""" finds manhattan distance"""
		distx = abs(self.destination[0] - node[0])
		disty = abs(self.destination[1] - node[1])
		return distx + disty



class graph_node(object):
	""" the nodes used to actually do stuff"""
	def __init__(self, x, y, node_values):
		self.node_values = node_values
		self.x = x
		self.y = y
		self.cost = 1
		self.no_neighbors = False
		if x > 0 and x < node_values.shape[0]:
			if y > 0 and y < node_values.shape[1]:
				self.surrounding_nodes = [ self.node_values[x-1,y], self.node_values[x, y+1], self.node_values[x+1,y], self.node_values[x, y-1] ]
				# print('surrounding_nodes', self.surrounding_nodes)
				self.neighbors = [node for node in self.surrounding_nodes if node == 0]
				# print('neighbors',self.neighbors)
				if len(self.neighbors) == 0:
					self.no_neighbors = True
				# else:
					# print('node %d , %d has neighbor', self.x, self.y)
					# print(self.neighbors)

	def append_cost(self, old_node):
		self.cost = old_node.cost + 1

		
search_algorithm((300,300),(1,1))