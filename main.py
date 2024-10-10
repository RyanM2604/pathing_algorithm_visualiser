import csv
from heapq import heapify, heappop, heappush

with open('/Users/ryanmaudgalya/Desktop/Code/Dijkstra/nodes.csv', newline='') as csvfile:
    nodes = list(csv.reader(csvfile))

with open('/Users/ryanmaudgalya/Desktop/Code/Dijkstra/edges.csv', newline='') as csvfile:
    edges = list(csv.reader(csvfile))

num_nodes = len(nodes)
num_edges = len(edges)

class Node:
    def __init__(self, id, lon, lat) -> None:
        self.id = int(id)
        self.lon = float(lon)
        self.lat = float(lat)

class Edge:
    def __init__(self, id, source, target, length) -> None:
        self.id = int(id)
        self.source = int(source)
        self.target = int(target)
        self.length = float(length)

node_list = []
edges_list = []

for i in range(1, num_nodes):
    node_list.append(Node(nodes[i][0], nodes[i][1], nodes[i][2]))

node_dict = {node.id: (node.lon, node.lat) for node in node_list}

for i in range(1, num_edges):
    edges_list.append(Edge(edges[i][1], edges[i][2], edges[i][3], edges[i][4]))

class Graph:
    def __init__(self, graph: dict = {}) -> None:
        self.graph = graph
    
    def create_edge(self, source, target, length):
        if source not in self.graph:
            self.graph[source] = {} # adding source node to graph if not available
        if target not in self.graph:
            self.graph[target] = {}
        self.graph[source][target] = length # accessing the source node dict in graph dict and adding weight of edge to target node key value
    
    def dijkstra(self, source: str):
        distances = {node: float('inf') for node in self.graph} # initializing distances of all from source
        distances[source] = 0

        pq = [(0, source)] # creates a priority queue which orders tuple elements as per their priority number
        heapify(pq)

        visited = set() # set of visited nodes

        while pq: # while the queue exists
            curr_dist, curr_node = heappop(pq) # get node with min distance from current node

            if curr_node in visited:
                continue
            visited.add(curr_node)

            for neighbor, length in self.graph[curr_node].items():
                temp_dist = curr_dist + length
                if temp_dist < distances[neighbor]:
                    distances[neighbor] = temp_dist
                    heappush(pq, (temp_dist, neighbor))

        return distances

    
graph = Graph()

for i in range(num_edges-1):
    graph.create_edge(edges_list[i].source, edges_list[i].target, edges_list[i].length)

distances = graph.dijkstra(int(input("first node: ")))

to_second_node = distances[int(input("second node: "))]
print(f"The shortest distance from the first node to the second node is {to_second_node}")



#2631870334
#12069393983