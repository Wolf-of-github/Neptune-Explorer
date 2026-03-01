## Exploring Neptune

### Overview
This project implements path planning algorithms to navigate a rover across a 3D terrain on Neptune. The objective is to compute the shortest valid path from the `start` location to the `goal` location while respecting uphill energy and momentum constraints.

The program supports the following algorithms:
- **Breadth-First Search (BFS)** – Unit step cost (1 per move)
- **Uniform-Cost Search (UCS)** – 2D Euclidean distance (x, y)
- **A\*** – 3D Euclidean distance (x, y, z) with an admissible heuristic

---

### Program Behavior

- Reads input from `input.txt`
- Parses:
  - Algorithm type (BFS, UCS, or A*)
  - Energy limit
  - Momentum limit
  - Safe locations (3D coordinates)
  - Safe path segments
- Validates each move according to:
  - Uphill energy constraint
  - Momentum accumulation and consumption rules
- Writes the solution path to `output.txt`

If a path exists:
start … goal
If no valid path exists:
FAIL

---

### Input Format (`input.txt`)
Algorithm (BFS / UCS / A*)
Energy limit
Momentum limit
Number of locations (N)
N lines: name x y z
Number of path segments (M)
M lines: name1 name2

One location is guaranteed to be `start` and one to be `goal`.

---

### How to Run

Ensure `input.txt` is in the same directory as: explorer.py

Run:

```bash
python3 explorer.py
