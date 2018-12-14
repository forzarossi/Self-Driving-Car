from collections import defaultdict
import numpy as np
import sys

# graph to represent board
class Graph():
	def __init__(self):
		"""
		self.edges is a dict of all possible next states
		e.g. {'X': ['A', 'B', 'C', 'E'], ...}
		self.weights has all the weights between two states,
		with the two states as a tuple as the key
		e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
		self.connections has all the connection types between two states,
		with the two states as a tuple as the key
		e.g. {('X', 'A'): 'S', ('X', 'B'): 'L', ...}
		"""
		self.edges = defaultdict(list)
		self.weights = {}
		self.connections = {}
	
	def add_edge(self, from_state, to_state, weight, connection):
		self.edges[from_state].append(to_state)
		self.weights[(from_state, to_state)] = weight
		self.connections[(from_state, to_state)] = connection

# edges entry: (from state #, to state #, weight, direction in intersection)
#	Weight: # of tiles in between state X and Y
# 	Intersection direction types:
#		L: left
#		R: right
#		S: straight
edges = [
	(1, 4, 2, 'L'),
	(1, 12, 2, 'S'),
	(2, 4, 2, 'R'),
	(2, 8, 2, 'S'),
	(3, 8, 2, 'R'),
	(3, 12, 2, 'L'),
	(4, 7, 4, 'L'),
	(4, 11, 4, 'R'),
	(5, 3, 2, 'L'),
	(5, 7, 4, 'S'),
	(6, 3, 2, 'R'),
	(6, 11, 4, 'S'),
	(7, 1, 2, 'L'),
	(7, 10, 8, 'S'),
	(8, 6, 4, 'R'),
	(8, 10, 8, 'L'),
	(9, 1, 2, 'R'),
	(9, 5, 4, 'S'),
	(10, 2, 2, 'L'),
	(10, 5, 4, 'S'),
	(11, 2, 2, 'R'),
	(11, 9, 10, 'S'),
	(12, 5, 4, 'L'),
	(12, 9, 8, 'R')
]

# returns shortest path between start and end state
def dijsktra(graph, start, end):
	# shortest paths is a dict of states
	# whose value is a tuple of (previous state, weight)
	shortest_paths = {start: (None, 0)}
	current_state = start
	visited = set()
	
	while current_state != end:
		visited.add(current_state)
		destinations = graph.edges[current_state]
		weight_to_current_state = shortest_paths[current_state][1]

		for next_state in destinations:
			weight = graph.weights[(current_state, next_state)] + weight_to_current_state
			if next_state not in shortest_paths:
				shortest_paths[next_state] = (current_state, weight)
			else:
				current_shortest_weight = shortest_paths[next_state][1]
				if current_shortest_weight > weight:
					shortest_paths[next_state] = (current_state, weight)
		
		next_destinations = {state: shortest_paths[state] for state in shortest_paths if state not in visited}
		if not next_destinations:
			return "Route Not Possible"
		# next state is the destination with the lowest weight
		current_state = min(next_destinations, key=lambda k: next_destinations[k][1])
	
	# Work back through destinations in shortest path
	path = []
	while current_state is not None:
		path.append(current_state)
		next_state = shortest_paths[current_state][0]
		current_state = next_state
	# Reverse path
	path = path[::-1]
	return path

# returns connection type between states within path
def get_connections_for_path(path):
	connections = []
	for index in range(len(path)):
		if index < len(path) - 1:
			for edge in edges:
				if edge[0] == path[index] and edge[1] == path[index + 1]:
					connections.append(edge[3])

	return connections

 
if __name__== "__main__":
	graph = Graph()
	for edge in edges:
		graph.add_edge(*edge)
	start = int(sys.argv[1])
	end = int(sys.argv[2])
	path = dijsktra(graph, start, end)
	connections = get_connections_for_path(path)
	print(path)
	print(connections)
