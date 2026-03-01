# Dynamic Pathfinding Agent

A grid-based pathfinding visualizer built with Python and Tkinter that demonstrates informed search algorithms with dynamic obstacle support.

## Project Overview

This project implements a Dynamic Pathfinding Agent capable of navigating a grid-based environment using informed search algorithms. The agent can handle real-time obstacle spawning and automatically re-plans its path when obstacles block its current route.

## Algorithms Implemented

- **A\* Search** — Uses `f(n) = g(n) + h(n)` combining path cost and heuristic
- **Greedy Best-First Search (GBFS)** — Uses only `f(n) = h(n)` (heuristic only)

## Heuristics Supported

- **Manhattan Distance** — Sum of absolute differences in row and column
- **Euclidean Distance** — Straight-line distance between two nodes

## Features

- Dynamic grid sizing (user-defined rows and columns)
- Random maze generation with configurable obstacle density
- Interactive map editor (click to place/remove walls)
- Real-time agent movement with dynamic obstacle spawning
- Automatic re-planning when path is blocked
- Step-by-step search visualization
- Live metrics dashboard (nodes visited, path cost, execution time, re-plans)

## Requirements

- Python 3.8 or higher
- Tkinter (included with standard Python installation)

No additional pip packages are required.

## Installation & Setup

### Step 1 — Install Python
Download and install Python from [python.org](https://www.python.org/downloads/)

During installation make sure to check:
- ✅ Add Python to PATH
- ✅ tcl/tk and IDLE

### Step 2 — Verify Tkinter
```bash
python -m tkinter
```
A small window should appear confirming tkinter is working.

### Step 3 — Clone or Download the Repository
```bash
git clone https://github.com/YOURUSERNAME/dynamic-pathfinding-agent.git
cd dynamic-pathfinding-agent
```
Or simply download the ZIP from GitHub and extract it.

### Step 4 — Run the Application
```bash
python dynamic_pathfinding.py
```

## How to Use

| Action | Control |
|---|---|
| Toggle wall | Left-click on any cell |
| Draw walls | Left-click and drag |
| Set Start / Goal | Right-click on any cell |
| Generate random maze | Set rows, cols, density → click Generate Map |
| Run search | Choose algorithm & heuristic → click Run Search |
| Enable dynamic mode | Check "Enable Dynamic Obstacles" → click Run Search |
| Stop agent | Click Stop Agent |
| Reset visualization | Click Reset Visited |
| Clear everything | Click Clear All |

## Visual Guide

| Color | Meaning |
|---|---|
| 🟢 Teal | Start node |
| 🟡 Orange | Goal node |
| ⬛ Dark | Wall / Obstacle |
| 🟡 Yellow | Frontier nodes (in priority queue) |
| 🔵 Blue | Visited / Expanded nodes |
| 🟢 Green | Final path |
| 🔴 Red dot | Agent current position |

## 📊 Metrics Dashboard

The GUI displays real-time metrics during search:
- **Nodes Visited** — Total number of expanded nodes
- **Path Cost** — Length of the final path
- **Execution Time** — Time taken to compute the path in milliseconds
- **Re-plans** — Number of times the agent had to re-calculate its path

## Project Structure
```
dynamic-pathfinding-agent/
│
├── dynamic_pathfinding.py   # Main application source code
├── requirements.txt         # Dependencies
├── README.md               # Project documentation
└── screenshots/            # GUI screenshots for report
    ├── astar_manhattan_best.png
    ├── astar_manhattan_worst.png
    ├── astar_euclidean_best.png
    ├── astar_euclidean_worst.png
    ├── gbfs_manhattan_best.png
    ├── gbfs_manhattan_worst.png
    ├── gbfs_euclidean_best.png
    └── gbfs_euclidean_worst.png
```

## Algorithm Comparison

| Feature | A* Search | Greedy BFS |
|---|---|---|
| Evaluation | f(n) = g(n) + h(n) | f(n) = h(n) |
| Optimality | ✅ Always optimal | ❌ Not guaranteed |
| Speed | Moderate | Faster |
| Memory | Higher | Lower |
| Best for | Guaranteed shortest path | Quick approximate path |

## Author

- **Name:** Chaudhry Muhammad Abdullah Tahir 
- **ID:** 24F-0020
- **Course:** Artificial Intelligence  
- **Institution:** Fast National University
