# 🤖 Surveillance Robot Navigation System

## 📘 Overview
This project was developed as part of **APT3010A: Introduction to Artificial Intelligence**.

It simulates a **surveillance robot navigating through a building** using different AI search algorithms to optimize movement based on:

- Speed (fewest stops)
- Path exploration
- Energy efficiency
- Smart decision-making using heuristics

---

## 🗺️ Problem Description
The building consists of **12 scan points (nodes 0–11)**.

- Each node represents a location where the robot can scan.
- Connections between nodes represent valid movement paths.
- The **energy cost** of moving between nodes is:


Cost = |NodeA - NodeB|


---

## ⚙️ Features

This system allows the agent (user) to choose between **4 intelligent navigation strategies**:

### 1️⃣ Breadth-First Search (BFS)
- Finds the path with the **fewest stopovers**
- Best for **urgent investigations**
- Guarantees shortest path in terms of steps

---

### 2️⃣ Depth-First Search (DFS)
- Explores **all possible paths**
- Useful for **analysis and exploration**
- Returns every valid path between two nodes

---

### 3️⃣ Uniform Cost Search (UCS)
- Finds the **lowest energy cost path**
- Considers actual movement cost
- Guarantees **optimal solution**

---

### 4️⃣ A* Search Algorithm
- Combines **cost + heuristic**
- Smarter and faster than UCS
- Uses estimated distance to guide search

#### Heuristic Used:

h(n) = |value(node) - value(goal)|


---

## 🧠 Algorithms Explained

### 🔹 BFS (Queue-Based)
- Uses FIFO structure
- Explores level by level

### 🔹 DFS (Stack-Based)
- Uses LIFO structure
- Explores depth first

### 🔹 UCS (Priority Queue)
- Expands lowest-cost node first

### 🔹 A* (Optimized Search)
- Uses:

f(n) = g(n) + h(n)

Where:
- `g(n)` = actual cost
- `h(n)` = estimated cost

---

## 🏗️ Project Structure


surveillance-robot/
│
├── main.py # Main program file
├── README.md # Project documentation


---

## ▶️ How to Run

### 🔹 Requirements
- Python 3.x

### 🔹 Run the program
```bash
python main.py
💻 User Interaction
Enter start node (0–11)
Enter goal node (0–11)
Choose algorithm:
1 → BFS
2 → DFS
3 → UCS
4 → A*
📊 Example Output
TASK iii) UCS — Cheapest path from 0 to 4

Path        : 0 → 5 → 4
Total stops : 1 intermediate stop
Energy cost : 5
🔍 Key Functions
build_graph()

Creates adjacency list with weighted edges.

bfs()

Returns shortest path (fewest nodes).

dfs_all_paths()

Returns all possible paths.

ucs()

Returns lowest cost path.

astar()

Returns optimal path using heuristics.