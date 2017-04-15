#!/usr/bin/env python
# the a* search algorithm used to traverse the occupancy grid

from map_reading import MapReadingNode
import rospy
import sys


map_reading = MapReadingNode()	
node_values = map_reading.grid_map.occupancy_grid
print("completed reading map")

class search_algorithm(object):
	def __init__(self, destination, start):
		print("hello a_star")
		self.initialize = False
		rospy.init_node('a_star')
		
		self.destination = graph_node(destination[0], destination[1])
		start_node = graph_node(start[0], start[1])
		start_node.g = 0
		self.current_node = start_node
		self.current_node.prev_node = start_node


		self.get_manhattan(self.current_node)


		self.todo = []	# nodes to search
		self.dead = {}	# nodes we've searched

		self.all_nodes = []

		self.came_from = {}

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
		print("getting manhattan")
		distx = abs(self.destination.x - node.x)
		disty = abs(self.destination.y - node.y)
		node.h = distx +disty
		print("manhattan = ", node.h)
		return node

	def append_g_cost(self, node):
		""" Appends g cost, or in our case how many steps to reach node"""
		print("appending g cost")
		node.g = node.prev_node.g + 1

	def total_cost(self, node):
		""" Sum of manhattan distance and g cost. To be used for todo ranking. """ 
		print("total cost")
		self.get_manhattan(node)
		self.append_g_cost(node)
		node.cost = node.h + node.g



	def get_neighbors(self):
		print("Getting neighbors")
		if (self.current_node.x-1, self.current_node.y) not in dead:
			left_node = graph_node(self.current_node.x-1, self.current_node.y)
			left_node.prev_node = current_node
			curr_neighbor_nodes.append(left_node)
		else:
			if dead[(self.current_node.x-1, self.current_node.y)].prev_node.g > current_node.g:
				dead[(self.current_node.x-1, self.current_node.y)].prev_node = current_node

		if (self.current_node.x+1, self.current_node.y) not in dead:
			right_node = graph_node(self.current_node.x+1, self.current_node.y)
			right_node.prev_node = current_node
			curr_neighbor_nodes.append(right_node)
		else:
			if dead[(self.current_node.x+1, self.current_node.y)].prev_node.g > current_node.g:
				dead[(self.current_node.x+1, self.current_node.y)].prev_node = current_node

		if (self.current_node.x, self.current_node.y-1) not in dead:
			up_node = graph_node(self.current_node.x, self.current_node.y-1)
			curr_neighbor_nodes.append(up_node)
		else:
			if dead[(self.current_node.x, self.current_node.y-1)].prev_node.g > current_node.g:
				dead[(self.current_node.x, self.current_node.y-1)].prev_node = current_node


		if (self.current_node.x, self.current_node.y+1) not in dead:
			down_node = graph_node(self.current_node.x, self.current_node.y+1)
			down_node.prev_node = current_node
			curr_neighbor_nodes.append(down_node)
		else:
			if dead[(self.current_node.x, self.current_node.y+1)].prev_node.g > current_node.g:
				dead[(self.current_node.x, self.current_node.y+1)].prev_node = current_node

		
		for neighbor in curr_neighbor_nodes:
			
			if node.prev_node.g + 1 < node.g:
				self.came_from[(node.x, node.y)] = node.prev_node
			neighbor.prev_node = self.current_node

		curr_neighbor_nodes = [self.total_cost(node) for node in curr_neighbor_nodes]
		

		self.current_node.neighbors = curr_neighbor_nodes




	def find_path(self):
		print("finding path")
		# Get neighbors of curr node
		self.get_neighbors()

		# Append each neighbor
		for neighbor in self.current_node.neighbors:
			self.todo.append(neighbor)	

		# Sort todo: smallest total cost fist!
		self.todo.sort(key=lambda x: x.cost, reverse=True)
		print("TODO ", self.todo)
		self.dead[(current_node.x, current_node.y)]= current_node
		print ("DEAD " ,self.dead)
		self.current_node = self.todo.pop(0)


	
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
		print("INITIALIZED NODE ", self.x, self.y, " /WITH VALUE ", self.value)


		
SA = search_algorithm((300,300),(1,1))
SA.find_path()
print("TODO ", SA.todo)

print("CAME FROM DICT ", SA.came_from)
