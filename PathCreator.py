import json
from collections import deque

# Number of controlled subsystems (e.g., pivot, elevator, wrist pitch/roll, intake)
NUM_SUBSYSTEMS = 5


def placeholder_setpoints():
    return [0.0] * NUM_SUBSYSTEMS


def normalize_setpoints(values):
    """Ensure each node setpoint list matches NUM_SUBSYSTEMS by padding or truncating."""
    if values is None:
        values = []
    length = len(values)
    if length < NUM_SUBSYSTEMS:
        values = list(values) + [0.0] * (NUM_SUBSYSTEMS - length)
    elif length > NUM_SUBSYSTEMS:
        values = list(values[:NUM_SUBSYSTEMS])
    return values


# Utility helpers for setpoints ------------------------------------------------
def deg(angle_degrees: float) -> float:
    """Convenience for entering angles in degrees."""
    return angle_degrees * 3.141592653589793 / 180.0


# Editable setpoints for each node. Update these as real poses are characterised.
# Values default to 0.0; feel free to use deg(...) for readability.
SETPOINTS = {
    "Start": [0.0, 0.0, 0.0, 0.0, 3.56],
    "Idle": [1.14, 0.0, 0.45, 0.0, 3.56],
    "Ground Intake": [1.14, 0.0, -2.15, 1.57, -0.27],
    "Handoff": [1.14, 0.0, -2.15, 1.57, 2.23],
    "Score L1": [1.14, 0.0, -2.15, 1.57, 1.33],
    "Lollipop Intake": [0.0, 0.0, -0.24, 0.0, 3.59],
    "HP Intake": [0.9, 0.0, -0.37, 1.57, 3.59],
    "Waiting to Score": [1.14, 0.0, 0, 1.57, 3.59],
    "Score L2 Front": [0.68, 0.0, 0.11, 0.0, 3.59],
    "Score L3 Front": [0.96, 0.26, 0.0, 0.0, 3.59],
    "Score L4 Front": [1.29, 0.92, -0.48, 0.0, 3.59],
    "Score L2 Back": [1.7, 0.0, 0.73, 0.0, 3.59],
    "Score L3 Back": [1.59, 0.05, 0.5, 0.0, 3.59],
    "L4 Back Intermediate": [1.5, 0.93, 0, 0.0, 3.59],
    "Score L4 Back": [1.5, 0.93, 1.03, 0.0, 3.59],
    "Ground Intake Algae": [0.0, 0.0, 0.0, 0.0, 3.59],
    "Holding Algae": [1.1, 0.0, 0.2, 0.0, 3.59],
    "Score Processor": [0.0, 0.0, 0.14, 0.0, 3.59],
    "Score Barge": [1.65, 0.95, 0.2, 0.0, 3.59],
    "Algae L2 Front": [1.0, 0.15, -1.06, 1.57, 3.59],
    "Algae L3 Front": [1.08, 0.31, -0.61, 1.57, 3.59],
    "Algae L2 Back": [1.0, 0.0, 0.0, 0.0, 0.0],
    "Algae L3 Back": [1.38, 0.28, 1.57, 1.57, 3.59],
    "Climb Init": [1.57, 0.0, -1.29, 0, 3.59],
    "Climb Final": [-0.2, 0, 1.57, 0, 3.59],
}


# Nodes from the updated manipulator state diagram -----------------------------
node_names = [
    "Start",
    "Idle",
    "Ground Intake",
    "Handoff",
    "Score L1",
    "Lollipop Intake",
    "HP Intake",
    "Waiting to Score",
    "Score L2 Front",
    "Score L3 Front",
    "Score L4 Front",
    "Score L2 Back",
    "Score L3 Back",
    "L4 Back Intermediate",
    "Score L4 Back",
    "Ground Intake Algae",
    "Holding Algae",
    "Score Processor",
    "Score Barge",
    "Algae L2 Front",
    "Algae L3 Front",
    "Algae L2 Back",
    "Algae L3 Back",
    "Climb Init",
    "Climb Final",
]

nodes = []
for name in node_names:
    setpoints = normalize_setpoints(SETPOINTS.get(name, placeholder_setpoints()))
    nodes.append({"name": name, "setpoints": setpoints})

# Directed edges derived from the diagram.
edge_pairs = [
    ("Start", "Idle"),
    ("Start", "Waiting to Score"),
    ("Idle", "Ground Intake"),
    ("Ground Intake", "Idle"),
    ("Ground Intake", "Handoff"),
    ("Handoff", "Score L1"),
    ("Score L1", "Idle"),
    ("Score L1", "Handoff"),
    ("Idle", "Lollipop Intake"),
    ("Lollipop Intake", "Waiting to Score"),
    ("Idle", "HP Intake"),
    ("HP Intake", "Waiting to Score"),
    ("HP Intake", "Idle"),
    ("Handoff", "Waiting to Score"),
    ("Waiting to Score", "Score L2 Front"),
    ("Score L2 Front", "Idle"),
    ("Waiting to Score", "Score L3 Front"),
    ("Score L3 Front", "Idle"),
    ("Waiting to Score", "Score L4 Front"),
    ("Score L4 Front", "Idle"),
    ("Waiting to Score", "Score L2 Back"),
    ("Score L2 Back", "Idle"),
    ("Waiting to Score", "Score L3 Back"),
    ("Score L3 Back", "Idle"),
    ("Waiting to Score", "L4 Back Intermediate"),
    ("L4 Back Intermediate", "Score L4 Back"),
    ("Score L4 Back", "Idle"),
    ("Waiting to Score", "Algae L2 Front"),
    ("Algae L2 Front", "Holding Algae"),
    ("Waiting to Score", "Algae L3 Front"),
    ("Algae L3 Front", "Holding Algae"),
    ("Waiting to Score", "Algae L2 Back"),
    ("Algae L2 Back", "Holding Algae"),
    ("Waiting to Score", "Algae L3 Back"),
    ("Algae L3 Back", "Holding Algae"),
    ("Idle", "Ground Intake Algae"),
    ("Ground Intake Algae", "Holding Algae"),
    ("Ground Intake Algae", "Idle"),
    ("Holding Algae", "Idle"),
    ("Holding Algae", "Score Processor"),
    ("Holding Algae", "Score Barge"),
    ("Score Processor", "Idle"),
    ("Score Barge", "Idle"),
    ("Idle", "Climb Init"),
    ("Climb Init", "Climb Final"),
    ("Climb Init", "Idle"),
    ("Climb Final", "Climb Final"),
]

edges = [{"start": start, "end": end} for start, end in edge_pairs]

# Remove any duplicate edges while preserving order.
deduped_edges = []
seen_edges = set()
for edge in edges:
    key = (edge["start"], edge["end"])
    if key not in seen_edges:
        seen_edges.add(key)
        deduped_edges.append(edge)
edges = deduped_edges

# Build an adjacency list representation.
graph_adj = {}
for node in nodes:
    node["setpoints"] = normalize_setpoints(node["setpoints"])
    graph_adj[node["name"]] = []
for edge in edges:
    graph_adj.setdefault(edge["start"], []).append(edge["end"])


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
    "shortestPaths": shortest_paths,
}

# Write the JSON file.
with open("./src/main/deploy/graph_data.json", "w") as f:
    json.dump(graph_data, f, indent=2)

print("graph_data.json generated successfully with setpoints and precomputed shortest paths!")
