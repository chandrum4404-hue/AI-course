from collections import deque
import heapq

#! 1. Environment definition

graph = {
    "A": ["B", "C"],
    "B": ["D", "E"],
    "C": ["F", "G"],
    "D": ["H"],
    "E": ["H"],
    "F": ["I"],
    "G": ["J"],
    "H": ["K"],
    "I": ["K"],
    "J": ["K"],
    "K": ["L"],
    "L": [],
}

weighted_graph = {
    "A": [("B", 2), ("C", 4)],
    "B": [("D", 3), ("E", 1)],
    "C": [("F", 2), ("G", 5)],
    "D": [("H", 2)],
    "E": [("H", 4)],
    "F": [("I", 3)],
    "G": [("J", 1)],
    "H": [("K", 2)],
    "I": [("K", 1)],
    "J": [("K", 6)],
    "K": [("L", 2)],
    "L": [],
}

START = "A"
GOAL = "L"

#! 2. Breadth-First Search (BFS)--------------------------------------------------------------------------------------------------------------

def bfs(graph, start, goal):
    """Finds the path with the fewest number of edges (hops)."""
    frontier = deque([[start]])
    visited = {start}
    nodes_expanded = 0

    while frontier:
        path = frontier.popleft()
        node = path[-1]
        nodes_expanded += 1

        if node == goal:
            return path, nodes_expanded

        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                frontier.append(path + [neighbor])

    return None, nodes_expanded



#! 3. Uniform Cost Search (UCS)------------------------------------------------------------------------------------------------------------

def ucs(weighted_graph, start, goal):
    """Finds the minimum total-cost path using a priority queue."""
    
    frontier = [(0, [start])]
    best_cost = {start: 0}
    nodes_expanded = 0

    while frontier:
        cost, path = heapq.heappop(frontier)
        node = path[-1]
        nodes_expanded += 1

        if node == goal:
            return path, cost, nodes_expanded

        
        if cost > best_cost.get(node, float("inf")):
            continue

        for neighbor, edge_cost in weighted_graph.get(node, []):
            new_cost = cost + edge_cost
            if new_cost < best_cost.get(neighbor, float("inf")):
                best_cost[neighbor] = new_cost
                heapq.heappush(frontier, (new_cost, path + [neighbor]))

    return None, float("inf"), nodes_expanded

#! 4. Iterative Deepening Search (IDS)-----------------------------------------------------------------------------------------------------

def depth_limited_dfs(graph, node, goal, limit, path, visited, stats):
    stats["nodes_expanded"] += 1
    path = path + [node]

    if node == goal:
        return path
    if limit <= 0:
        return None

    visited.add(node)
    for neighbor in graph.get(node, []):
        if neighbor not in visited:
            result = depth_limited_dfs(
                graph, neighbor, goal, limit - 1, path, visited, stats
            )
            if result is not None:
                return result
    visited.remove(node)
    return None


def ids(graph, start, goal, max_depth=20):
    """Repeats depth-limited DFS with an increasing depth cutoff."""
    total_nodes_expanded = 0
    for depth in range(max_depth + 1):
        stats = {"nodes_expanded": 0}
        result = depth_limited_dfs(graph, start, goal, depth, [], set(), stats)
        total_nodes_expanded += stats["nodes_expanded"]
        if result is not None:
            return result, depth, total_nodes_expanded
    return None, -1, total_nodes_expanded

def path_cost(weighted_graph, path):
    """Utility: compute the total cost of a given path on the weighted graph."""
    total = 0
    for a, b in zip(path, path[1:]):
        for neighbor, cost in weighted_graph[a]:
            if neighbor == b:
                total += cost
                break
    return total


if __name__ == "__main__":
    print("=" * 60)
    print("EMERGENCY RESCUE ROUTE PLANNER")
    print(f"Start (Entrance): {START}   Goal (Patient): {GOAL}")
    print("=" * 60)

    
    bfs_path, bfs_nodes = bfs(graph, START, GOAL)
    print("\n[1] Breadth-First Search (BFS)")
    print(f"    Path found : {' -> '.join(bfs_path)}")
    print(f"    Hops       : {len(bfs_path) - 1}")
    print(f"    Nodes expanded: {bfs_nodes}")

    
    ucs_path, ucs_cost, ucs_nodes = ucs(weighted_graph, START, GOAL)
    print("\n[2] Uniform Cost Search (UCS)")
    print(f"    Path found : {' -> '.join(ucs_path)}")
    print(f"    Total cost : {ucs_cost}")
    print(f"    Nodes expanded: {ucs_nodes}")

    
    ids_path, ids_depth, ids_nodes = ids(graph, START, GOAL)
    print("\n[3] Iterative Deepening Search (IDS)")
    print(f"    Path found : {' -> '.join(ids_path)}")
    print(f"    Depth reached: {ids_depth}")
    print(f"    Total nodes expanded (across all iterations): {ids_nodes}")

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"BFS path (fewest hops)   : {' -> '.join(bfs_path)}  ({len(bfs_path)-1} hops)")
    print(f"UCS path (lowest cost)   : {' -> '.join(ucs_path)}  (cost {ucs_cost})")
    print(f"IDS path (depth-limited) : {' -> '.join(ids_path)}  (depth {ids_depth})")