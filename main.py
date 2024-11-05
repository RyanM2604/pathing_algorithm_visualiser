import csv
from heapq import heapify, heappop, heappush
import folium

with open('/Users/ryanmaudgalya/Desktop/Code/Dijkstra_program/nodes.csv', newline='') as csvfile:
    nodes = list(csv.reader(csvfile))

with open('/Users/ryanmaudgalya/Desktop/Code/Dijkstra_program/edges.csv', newline='') as csvfile:
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

        predecessors = {node: None for node in self.graph}

        for node, distance in distances.items():
            for neighbor, weight in self.graph[node].items():
                if distances[neighbor] == distance + weight:
                    predecessors[neighbor] = node

        return distances, predecessors

                
    def shortest_path(self, source: str, target: str):
        # Generate the predecessors dict
        _, predecessors = self.dijkstra(source)

        path = []
        current_node = target

        # Backtrack from the target node using predecessors
        while current_node:
            path.append(current_node)
            current_node = predecessors[current_node]

        # Reverse the path and return it
        path.reverse()

        return path
    
graph = Graph()

for i in range(num_edges-1):
    graph.create_edge(edges_list[i].source, edges_list[i].target, edges_list[i].length)

distances, predecessors = graph.dijkstra(250197412)
print(distances, "\n")

# Access the distance to F specifically
to_node = distances[299276434]
print(f"The shortest distance from 250197412 to 299276434 is {to_node}")

# Find the path from B to F
path = graph.shortest_path(250197412, 299276434)
print(f"The path from source to target is {path}")

def build_map():
    center = (43.4764947, -80.5291953)  # Center map around the first node
    folium_map = folium.Map(location=center, zoom_start=15)

    # Add all nodes as markers
    for node in node_list:
        folium.CircleMarker(location=(node.lat, node.lon), radius=3, color="green", fill=True).add_to(folium_map)
    
    return folium_map

#build_map().save("map.html")

#3163088314
#9996176257