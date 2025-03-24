import json
from collections import deque

# Number of controlled subsystems (e.g., 4)
NUM_SUBSYSTEMS = 4

# Helper function to create a placeholder setpoints array.
def placeholder_setpoints():
    return [0.0] * NUM_SUBSYSTEMS

# Define nodes with placeholder setpoints using your earlier data.
nodes = [
    {
        "name": "Stow",
        "setpoints": [0,0,0,0]
    },
    {
        "name" : "Intake Intermediate",
        "setpoints": [0.275, 0, 0, -3.14]
    },
    {
        "name": "Coral Ground Pickup",
        "setpoints": [0.35, 0.05, -1.89, -3.14]
    },
    {
        "name": "Lollipop Coral Pickup",
        "setpoints": [0,0,-1.46,-1.56]
    },
    {
        "name": "Reef Align",
        "setpoints": [1.57, 0, -1.38, -1.73]
    },
    {
        "name": "L4 Intermediate",
        "setpoints": [1.65, 0.77, -1.38, -1.53]
    },
    {
        "name": "L4 Init",
        "setpoints": [1.65, 0.77, -1.02, -1.53]
    },
    {
        "name": "L4 Dunk",
        "setpoints": [1.65 ,0.77,-0.37,-1.53]
    },
    {
        "name":"L3 Intermediate",
        "setpoints":[1.5, 0.05,-1.38,-1.53]
    },
    {
        "name": "L3 Init",
        "setpoints": [1.5, .05, -1.38, -1.53]
    },
    {
        "name": "L3 Dunk",
        "setpoints": [1.75, 0, -0.76, -1.53]
    },
    {
        "name" : "Net Intermediate",
        "setpoints" : [1.76, 0, -1.12, 3.14]
    },
    {
        "name": "Net Score", 
        "setpoints": [1.76, 1, -1.12, 3.14]
    },
    {
        "name": "L2 Algae Pickup",
        "setpoints": [1.21, 0, -0.01, 3.14]
    },
    {
        "name": "L3 Algae Pickup",
        "setpoints": [1.43, 0.23, -0.27, 3.14]
    },
    {
        "name": "Algae Ground Pickup",
        "setpoints": [0,0,-1.43,-0.13]
    },
    {
        "name": "Processor Score",
        "setpoints": [0.01, 0, -1.14, 0]
    },
    {
        "name": "Coral HP Pickup",
        "setpoints": [0.99, 0, -1.44, -3.14]
    },
    {
        "name": "L2 Align",
        "setpoints": [1.37, 0, -0.2, -1.56]
    },
    {
        "name": "L1 Score",
        "setpoints": [1.4, 0, 0, 0.04]
    },
]

# Define edges between nodes without descriptions.
edges = [
    {
        "start": "Stow",
        "end": "Intake Intermediate"
    },
    {
        "start": "Stow",
        "end": "Lollipop Coral Pickup"
    },
    {
        "start": "Stow",
        "end": "Reef Align"
    },
    {
        "start": "Stow",
        "end": "L1 Score"
    },
    {
        "start": "Stow",
        "end": "L2 Align"
    },
    {
        "start": "Stow",
        "end": "Coral HP Pickup "
    },
    {
        "start": "Stow",
        "end": "Algae Ground Pickup"
    },
    {
        "start": "Stow",
        "end": "Processor Score"
    },
    {
        "start":"Stow",
        "end":"Net Intermediate"
    },
    {
        "start": "Intake Intermediate",
        "end": "Stow"
    },
    {
        "start": "Intake Intermediate",
        "end": "Coral Ground Pickup"
    },
    {
        "start": "Coral Ground Pickup",
        "end": "Intake Intermediate"
    },
    {
        "start": "Coral Ground Pickup",
        "end": "Reef Align"
    },
    {
        "start": "Lollipop Coral Pickup",
        "end": "Stow"
    },
    {
        "start": "Lollipop Coral Pickup",
        "end": "Reef Align"
    },
    {
        "start": "Reef Align",
        "end": "Stow"
    },
    {
        "start": "Reef Align",
        "end": "Lollipop Coral Pickup"
    },
    {
        "start": "Reef Align",
        "end": "Coral Ground Pickup"
    },
    {
        "start": "Reef Align",
        "end": "L3 Intermediate"
    },
    {
        "start": "Reef Align",
        "end": "L4 Intermediate"
    },
    {
        "start": "L3 Intermediate",
        "end": "L3 Init"
    },
    {
        "start": "Reef Align",
        "end": "L3 Algae Pickup"
    },
    {
        "start": "Reef Align",
        "end": "L2 Algae Pickup"
    },
    {
        "start": "L3 Intermediate",
        "end": "Reef Align"
    },
    {
        "start": "L4 Intermediate",
        "end": "L4 Init"
    },
    {
        "start": "L4 Intermediate",
        "end": "Reef Align"
    },
    {
        "start": "L4 Init",
        "end": "L4 Dunk"
    },
    {
        "start": "L4 Init",
        "end": "L3 Init"
    },
    {
        "start": "L3 Init",
        "end": "L3 Dunk"
    },
    {
        "start": "L3 Init",
        "end": "L4 Init"
    },
    {
        "start": "L3 Init",
        "end": "Reef Align"
    },
    {
        "start": "L4 Dunk",
        "end": "Reef Align"
    },
    {
        "start": "L4 Dunk",
        "end": "L3 Algae pickup"
    },
    {
        "start": "L4 Dunk",
        "end": "L2 Algae Pickup"
    },
    {
        "start": "L3 Dunk",
        "end": "Reef Align"
    },
    {
        "start": "L3 Algae Pickup",
        "end": "Reef Align"
    },
    {
        "start": "L2 Algae Pickup",
        "end": "Reef Align"
    },
    {
        "start": "Processor Score",
        "end": "Stow"
    },
    {
        "start": "Algae Ground Pickup",
        "end": "Stow"
    },
    {
        "start": "Algae Ground Pickup",
        "end": "Processor Score"
    },
    {
        "start": "Coral HP Pickup",
        "end": "Stow"
    },
    {
        "start": "L2 Align",
        "end": "Stow"
    },
    {
        "start": "L1 Score",
        "end": "Stow"
    },
    {
        "start":"Net Intermediate",
        "end" : "Stow"
    },
    {
        "start":"Net Intermediate",
        "end" : "Net Score"
    },
    {
        "start":"Net Score",
        "end": "Net Intermediate"
    }
]

# Build an adjacency list representation.
graph_adj = {}
for node in nodes:
    graph_adj[node["name"]] = []
for edge in edges:
    if edge["start"] in graph_adj:
        graph_adj[edge["start"]].append(edge["end"])
    else:
        graph_adj[edge["start"]] = [edge["end"]]

def bfs_shortest_path(start, end):
    """
    Uses BFS to compute the shortest path from start to end.
    Returns a list of node names representing the path.
    If no path exists, returns an empty list.
    """
    if start == end:
        return [start]
    
    visited = {start}
    queue = deque([[start]])
    
    while queue:
        path = queue.popleft()
        current = path[-1]
        for neighbor in graph_adj.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = path + [neighbor]
                if neighbor == end:
                    return new_path
                queue.append(new_path)
    return []  # No path found

def compute_all_pairs_shortest_paths():
    """
    Computes the shortest path from each node to every other node.
    Returns a nested dictionary in the format:
    {
      startNode: { endNode: [list of nodes in path], ... },
      ...
    }
    """
    all_paths = {}
    for start in graph_adj.keys():
        all_paths[start] = {}
        for end in graph_adj.keys():
            path = bfs_shortest_path(start, end)
            all_paths[start][end] = path
    return all_paths

# Precompute all pairs shortest paths.
shortest_paths = compute_all_pairs_shortest_paths()

# Combine nodes, edges, and precomputed shortest paths into one structure.
graph_data = {
    "nodes": nodes,
    "edges": edges,
    "shortestPaths": shortest_paths
}

# Write the JSON file.
with open("./src/main/deploy/graph_data.json", "w") as f:
    json.dump(graph_data, f, indent=2)

print("graph_data.json generated successfully with setpoints and precomputed shortest paths!")