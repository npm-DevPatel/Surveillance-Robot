"""
APT3010-VA - Introduction to Artificial Intelligence
Assignment 3 - Surveillance Robot Navigation

The building map has 12 scan points (nodes 0–11).
The node values come directly from the map image:
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11

The COST to travel between two connected nodes =
  abs(node_value_A - node_value_B)

This single program supports 4 tasks chosen by the agent:
  i)   BFS  - Fewest stopovers (quickest to investigate)
  ii)  DFS  - Show ALL available paths
  iii) UCS  - Cheapest path (lowest energy cost)
  iv)  A*   - Cheapest path using heuristics (distance + cost)
"""

import heapq    # Used for priority queues in UCS and A*
from collections import deque  # Used for the BFS queue

# ─────────────────────────────────────────────────────────
# GRAPH DEFINITION
# Each node has a value read from the map (the number shown
# next to each red circle on the building map).
# ─────────────────────────────────────────────────────────

# Node values as labelled on the map
NODE_VALUES = {
    0:  0,
    1:  1,
    2:  2,
    3:  3,
    4:  4,
    5:  5,
    6:  6,
    7:  7,
    8:  8,
    9:  9,
    10: 10,
    11: 11
}

# Connections between nodes (edges) read from the map.
# Each tuple is (neighbour, cost).
# Cost = abs(value of node A - value of node B)
def build_graph():
    """
    Build and return the adjacency list for the building map.
    Connections are based on the paths visible in the map image.
    Cost between nodes = absolute difference of their values.
    """
    # Raw connections (undirected) — both directions added below
    connections = [
        (7, 8),   # top row: A zone
        (8, 9),   # top row: B-C zone
        (0, 1),   # middle row
        (1, 2),   # middle row
        (2, 3),   # middle row
        (6, 0),   # left column
        (7, 0),   # vertical: 7 down to 0
        (8, 1),   # vertical: 8 down to 1
        (9, 2),   # vertical: 9 down to 2
        (3, 4),   # right side
        (0, 5),   # middle area
        (5, 4),   # middle to right
        (5, 10),  # middle down
        (6, 11),  # left down
        (11, 10), # bottom row
        (10, 4),  # bottom right
        (1, 5),   # centre connections
        (2, 9),   # upper right area
    ]

    graph = {i: [] for i in range(12)}

    for a, b in connections:
        cost = abs(NODE_VALUES[a] - NODE_VALUES[b])
        graph[a].append((b, cost))
        graph[b].append((a, cost))

    return graph


# ─────────────────────────────────────────────────────────
# TASK i) — BFS (Breadth-First Search)
# Goal: Find the path with the FEWEST STOPOVERS
# BFS explores level by level, so the first time it reaches
# the goal it has taken the fewest possible steps.
# ─────────────────────────────────────────────────────────
def bfs(graph, start, goal):
    """
    Breadth-First Search — finds the path with the least number of nodes visited.
    Returns the path as a list of node numbers, or None if no path exists.
    """
    # Queue stores: (current_node, path_taken_so_far)
    queue = deque([(start, [start])])

    # Keep track of visited nodes so we don't loop
    visited = set()
    visited.add(start)

    while queue:
        current, path = queue.popleft()  # Take the next node to explore

        # If we've reached the goal, return the path
        if current == goal:
            return path

        # Explore all neighbours of the current node
        for neighbour, cost in graph[current]:
            if neighbour not in visited:
                visited.add(neighbour)
                queue.append((neighbour, path + [neighbour]))  # Extend path

    return None  # No path found


# ─────────────────────────────────────────────────────────
# TASK ii) — DFS (Depth-First Search) — ALL PATHS
# Goal: Find and display ALL possible paths from start to goal
# DFS dives deep into one path before backtracking.
# We keep exploring even after finding the goal to get all paths.
# ─────────────────────────────────────────────────────────
def dfs_all_paths(graph, start, goal):
    """
    Depth-First Search — finds ALL paths from start to goal.
    Returns a list of paths (each path is a list of nodes).
    """
    all_paths = []  # Will collect every valid path found

    # Stack stores: (current_node, path_taken_so_far, visited_in_this_path)
    stack = [(start, [start], {start})]

    while stack:
        current, path, visited_in_path = stack.pop()

        # If goal reached, save this path
        if current == goal:
            all_paths.append(path)
            continue  # Keep going to find more paths

        # Explore neighbours, but only those not already in this path
        # (avoids infinite loops while still allowing different paths)
        for neighbour, cost in graph[current]:
            if neighbour not in visited_in_path:
                new_visited = visited_in_path | {neighbour}  # Add to visited set
                stack.append((neighbour, path + [neighbour], new_visited))

    return all_paths


# ─────────────────────────────────────────────────────────
# TASK iii) — UCS (Uniform Cost Search)
# Goal: Find the CHEAPEST path (lowest total energy cost)
# UCS always expands the lowest-cost node first using a
# priority queue (min-heap). Guarantees the optimal path.
# ─────────────────────────────────────────────────────────
def ucs(graph, start, goal):
    """
    Uniform Cost Search — finds the path with the lowest total energy cost.
    Returns (total_cost, path) or (None, None) if no path exists.
    """
    # Priority queue: (total_cost, current_node, path_so_far)
    # heapq always pops the item with the SMALLEST total_cost first
    priority_queue = [(0, start, [start])]

    # Track the best cost to reach each node (to avoid revisiting expensively)
    best_cost = {start: 0}

    while priority_queue:
        cost, current, path = heapq.heappop(priority_queue)

        # If we reached the goal, return the result
        if current == goal:
            return cost, path

        # Explore each neighbour
        for neighbour, edge_cost in graph[current]:
            new_cost = cost + edge_cost  # Total cost to reach this neighbour

            # Only proceed if this is a cheaper way to reach the neighbour
            if neighbour not in best_cost or new_cost < best_cost[neighbour]:
                best_cost[neighbour] = new_cost
                heapq.heappush(priority_queue, (new_cost, neighbour, path + [neighbour]))

    return None, None  # No path found


# ─────────────────────────────────────────────────────────
# TASK iv) — A* Search with Heuristics
# Goal: Find cheapest path using BOTH actual cost AND estimated
#       remaining distance (heuristic) to guide the search smarter.
#
# The heuristic here uses the node VALUE difference as a proxy
# for distance — nodes with values closer to the goal value are
# assumed to be "closer" geographically.
#
# f(n) = g(n) + h(n)
#   g(n) = actual cost so far
#   h(n) = estimated cost remaining (heuristic)
# ─────────────────────────────────────────────────────────
def heuristic(node, goal):
    """
    Heuristic function for A* search.
    Estimates the remaining cost from 'node' to 'goal'
    using the absolute difference in their node values.
    This acts as a distance proxy — nodes with values close
    to the goal's value are considered to be nearby.
    """
    return abs(NODE_VALUES[node] - NODE_VALUES[goal])


def astar(graph, start, goal):
    """
    A* Search — finds the cheapest path using actual cost + heuristic estimate.
    Returns (total_cost, path) or (None, None) if no path exists.
    """
    # Priority queue: (f_score, g_score, current_node, path_so_far)
    # f = g + h  (total estimated cost)
    # g = actual cost so far
    h_start = heuristic(start, goal)
    priority_queue = [(h_start, 0, start, [start])]

    # Best actual cost (g) to reach each node
    best_g = {start: 0}

    while priority_queue:
        f, g, current, path = heapq.heappop(priority_queue)

        # Goal reached — return actual cost (g) and path
        if current == goal:
            return g, path

        # Prune: skip if we've already found a cheaper way to this node
        if g > best_g.get(current, float('inf')):
            continue

        # Explore neighbours
        for neighbour, edge_cost in graph[current]:
            new_g = g + edge_cost               # Actual cost to neighbour
            new_h = heuristic(neighbour, goal)  # Estimated remaining cost
            new_f = new_g + new_h               # Combined score

            # Only proceed if cheaper than any previous route to this neighbour
            if new_g < best_g.get(neighbour, float('inf')):
                best_g[neighbour] = new_g
                heapq.heappush(priority_queue, (new_f, new_g, neighbour, path + [neighbour]))

    return None, None  # No path found


# ─────────────────────────────────────────────────────────
# DISPLAY HELPERS
# ─────────────────────────────────────────────────────────
def path_to_string(path):
    """Convert a list of node numbers into a readable arrow string."""
    return " → ".join(str(n) for n in path)

def calculate_path_cost(graph, path):
    """Calculate the total energy cost of a given path."""
    total = 0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        # Find the edge cost between a and b
        for neighbour, cost in graph[a]:
            if neighbour == b:
                total += cost
                break
    return total


# ─────────────────────────────────────────────────────────
# MAIN MENU
# ─────────────────────────────────────────────────────────
def main():
    graph = build_graph()

    print("=" * 55)
    print("   SURVEILLANCE ROBOT NAVIGATION SYSTEM")
    print("=" * 55)
    print("Available scan points (nodes): 0 to 11")
    print()

    # Get start and goal from user
    try:
        start = int(input("Enter START node (0-11): "))
        goal  = int(input("Enter GOAL  node (0-11): "))
    except ValueError:
        print("Invalid input. Please enter numbers only.")
        return

    if start not in range(12) or goal not in range(12):
        print("Nodes must be between 0 and 11.")
        return

    print()
    print("Select task:")
    print("  1 - Fewest stopovers (BFS)")
    print("  2 - Show all paths   (DFS)")
    print("  3 - Cheapest path    (UCS)")
    print("  4 - Smart cheapest   (A* with heuristics)")
    print()

    try:
        choice = int(input("Enter choice (1-4): "))
    except ValueError:
        print("Invalid input.")
        return

    print()
    print("-" * 55)

    # ── Task i) BFS ──
    if choice == 1:
        print(f"TASK i) BFS — Fewest stopovers from {start} to {goal}")
        print()
        path = bfs(graph, start, goal)
        if path:
            cost = calculate_path_cost(graph, path)
            print(f"  Path    : {path_to_string(path)}")
            print(f"  Stops   : {len(path) - 2} intermediate stops")
            print(f"  Total nodes visited: {len(path)}")
            print(f"  Energy cost: {cost}")
        else:
            print("  No path found between these nodes.")

    # ── Task ii) DFS ALL PATHS ──
    elif choice == 2:
        print(f"TASK ii) DFS — All paths from {start} to {goal}")
        print()
        paths = dfs_all_paths(graph, start, goal)
        if paths:
            # Sort by number of stops for easier reading
            paths.sort(key=lambda p: len(p))
            print(f"  Found {len(paths)} path(s):\n")
            for i, path in enumerate(paths, 1):
                cost = calculate_path_cost(graph, path)
                stops = len(path) - 2
                print(f"  Path {i}: {path_to_string(path)}")
                print(f"          Stops: {stops}  |  Energy cost: {cost}")
                print()
        else:
            print("  No paths found between these nodes.")

    # ── Task iii) UCS ──
    elif choice == 3:
        print(f"TASK iii) UCS — Cheapest path from {start} to {goal}")
        print()
        cost, path = ucs(graph, start, goal)
        if path:
            print(f"  Path        : {path_to_string(path)}")
            print(f"  Total stops : {len(path) - 2} intermediate stops")
            print(f"  Energy cost : {cost}")
        else:
            print("  No path found between these nodes.")

    # ── Task iv) A* ──
    elif choice == 4:
        print(f"TASK iv) A* — Smart cheapest path from {start} to {goal}")
        print()
        print("  Heuristic: |node_value(current) - node_value(goal)|")
        print()
        cost, path = astar(graph, start, goal)
        if path:
            print(f"  Path        : {path_to_string(path)}")
            print(f"  Total stops : {len(path) - 2} intermediate stops")
            print(f"  Energy cost : {cost}")
            print()
            # Show heuristic values along the path for transparency
            print("  Heuristic values along path:")
            for node in path:
                h = heuristic(node, goal)
                print(f"    Node {node:2d}  →  h = {h}")
        else:
            print("  No path found between these nodes.")

    else:
        print("Invalid choice. Please enter 1, 2, 3, or 4.")

    print("-" * 55)
    input("\nPress Enter to exit...")


# Run the program
if __name__ == "__main__":
    main()
