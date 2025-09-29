import json
from collections import deque

# Ordered list of subsystem keys that correspond to entries in the generated graph JSON.
SUBSYSTEM_KEYS = ["pivot", "elevator", "wristPitch", "wristRoll"]

# Helper function to create a placeholder setpoints dictionary.
def placeholder_setpoints():
    return {key: 0.0 for key in SUBSYSTEM_KEYS}

# Define nodes with placeholder setpoints using your earlier data.
nodes = [
    {
        "name": "Stow",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Coral Ground Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Lollipop Coral Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Reef Align",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L4 Init",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L4 Dunk",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L3 Init",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L3 Dunk",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L3 Algae Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L3 Algae Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Algae Ground Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Processor Score",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "Coral HP Pickup",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L2 Align",
        "setpoints": placeholder_setpoints()
    },
    {
        "name": "L1 Score",
        "setpoints": placeholder_setpoints()
    },
]

# Define edges between nodes without descriptions.
edges = [
    {
        "start": "Stow",
        "end": "Coral Ground Pickup"
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
        "end": "Coral HP Pickup"
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
        "start": "Coral Ground Pickup",
        "end": "Stow"
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
        "end": "L4 Init"
    },
    {
        "start": "Reef Align",
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
        "start": "L4 Init",
        "end": "L4 Dunk"
    },
    {
        "start": "L4 Init",
        "end": "L3 Init"
    },
    {
        "start": "L4 Init",
        "end": "Reef Align"
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
    "subsystems": SUBSYSTEM_KEYS,
    "nodes": nodes,
    "edges": edges,
    "shortestPaths": shortest_paths
}

# Write the JSON file.
with open("./src/main/resources/frc/lib/util/graph/graph_data.json", "w") as f:
    json.dump(graph_data, f, indent=2)

print("graph_data.json generated successfully with setpoints and precomputed shortest paths!")