import json
from collections import deque, defaultdict

# Example input data (could also parse from a file):
# The graph is stored as an adjacency list. 
# For an unweighted graph, each edge is just a neighbor reference.
graph = {
    "Stow": ["Coral Ground Pickup", "Reef Align", "Lollipop Coral Pickup", "Processor Score", "Algae Ground Pickup", "Coral HP Pickup", "L2 Align", "L1 Score"],
    "Lollipop Coral Pickup": ["Stow", "Reef Align"],
    "Coral Ground Pickup": ["Stow", "Reef Align"],
    "Reef Align": ["Stow", "L4 Init", "L3 Init", "Lollipop Coral Pickup", "L3 Algae Pickup", "L2 Algae Pickup", "Coral Ground Pickup"],
    "L4 Init": ["L3 Init", "Reef Align", "L4 Dunk"],
    "L3 Init": ["L4 Init", "Reef Align", "L3 Dunk"],
    "L3 Dunk": ["Reef Align"],
    "L4 Dunk": ["Reef Align", "L3 Algae Pickup", "L2 Algae Pickup"],
    "L3 Algae Pickup": ["Reef Align"],
    "L2 Algae Pickup": ["Reef Align"],
    "Algae Ground Pickup": ["Stow", "Processor Score"],
    "Processor Score": ["Stow"],
    "L1 Score": ["Stow"],
    "L2 Align": ["Stow"],
    "Coral HP Pickup": ["Stow"],
    # etc...
}

def bfs_shortest_path(graph, start, end):
    """
    Return the shortest path from start to end using BFS.
    If no path, return an empty list.
    """
    visited = set([start])
    queue = deque([[start]])
    
    while queue:
        path = queue.popleft()
        node = path[-1]
        
        if node == end:
            return path
        
        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    
    return []  # No path found

def compute_all_pairs_paths(graph):
    """
    For each node, compute the shortest path to every other node.
    Returns a dict: all_paths[start][end] = [list of nodes in path].
    """
    all_paths = {}
    for start_node in graph.keys():
        all_paths[start_node] = {}
        for end_node in graph.keys():
            if start_node == end_node:
                all_paths[start_node][end_node] = [start_node]
            else:
                path = bfs_shortest_path(graph, start_node, end_node)
                all_paths[start_node][end_node] = path
    return all_paths

if __name__ == "__main__":
    all_paths = compute_all_pairs_paths(graph)
    
    # For convenience, store in JSON
    with open("./src/main/java/frc/lib/util/graph/all_paths.json", "w") as f:
        json.dump(all_paths, f, indent=2)
    
    print("All pairs shortest paths computed and saved to all_paths.json!")