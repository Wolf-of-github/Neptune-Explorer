## Exploring Neptune

### Overview
This project implements search algorithms to help a rover navigate a 3D terrain on Neptune. The objective is to compute the shortest valid path from the `start` location to the `goal` location while respecting uphill energy and momentum constraints.

The program supports the following algorithms:
- **Breadth-First Search (BFS)** – Unit step cost (1 per move)
- **Uniform-Cost Search (UCS)** – 2D Euclidean distance (x, y)
- **A\*** – 3D Euclidean distance (x, y, z) with an admissible heuristic

---

### Program Behavior
The program reads input from a file named `input.txt`, parses the algorithm type (BFS, UCS, or A*), the uphill energy limit, the momentum limit, the list of safe locations with 3D coordinates, and the list of safe path segments. It then applies the selected search algorithm while validating every move according to the uphill energy constraint and momentum rules (momentum accumulation capped at the momentum limit and consumed before energy when moving uphill). Finally, it writes the resulting path to `output.txt`.

---

### Input Format (`input.txt`)
Algorithm (BFS / UCS / A*)  
Energy limit  
Momentum limit  
Number of locations (N)  
N lines: name x y z  
Number of path segments (M)  
M lines: name1 name2  

Exactly one location is named `start` and exactly one location is named `goal`. Path segments are undirected.

Example input files and path data are available here:  
https://drive.google.com/drive/folders/1_1lqySwl6K94EBxJmgi-hlSNZRO3QAKE?usp=sharing

---

### Output Format (`output.txt`)
If a valid path exists, the output contains a single line with a space-separated list of location names starting with `start` and ending with `goal`. If no valid path exists, the output contains a single word: `FAIL`.

---

### How to Run
Ensure `input.txt` is in the same directory as `explorer.py`, then run:

```bash
python3 explorer.py
```

The program will generate `output.txt` in the same directory.

---

### Notes
- No command-line arguments are required.
- The program strictly follows the required input/output format.
- Floating-point calculations use double precision.
- Any optimal solution within the allowed tolerance is accepted.
