from collections import deque
import heapq

class NeptuneExplorer:
    
    def __init__(self):
        self.input_file = "input.txt"
        self.output_file = "output.txt"
        self.algorithm = None
        self.max_uphill_energy_units = None
        self.max_momentum_units = None
        self.locations = {}
        self.graph = {}
        self.start = None
        self.goal = None
        self.goal_coords = None

    def read_input_build_graph(self):
        with open(self.input_file, "r") as f:
            self.algorithm = f.readline().strip()
            self.max_uphill_energy_units = int(f.readline())
            self.max_momentum_units = int(f.readline())
            location_count = int(f.readline())
            
            locations = {}
            for _ in range(location_count):
                parts = f.readline().split()
                name = parts[0]
                coords = (int(parts[1]), int(parts[2]), int(parts[3]))
                locations[name] = coords
                if name == "start":
                    self.start = name
                elif name == "goal":
                    self.goal = name
                    self.goal_coords = coords
            
            self.locations = locations
            self.graph = {name: [] for name in locations}
            
            path_count = int(f.readline())
            graph = self.graph
            for _ in range(path_count):
                src, dst = f.readline().split()
                graph[src].append(dst)
                graph[dst].append(src)

    def construct_path(self, parent, end_state):
        path = []
        while end_state is not None:
            loc, _ = end_state
            path.append(loc)
            end_state = parent[end_state]
        path.reverse()
        return " ".join(path)

    def bfs(self):
        start_state = (self.start, 0) 
        queue = deque([start_state])
        visited = {start_state} 
        goal, max_energy, max_momentum, locations, graph = self.goal, self.max_uphill_energy_units, self.max_momentum_units, self.locations, self.graph
        parent = {start_state: None} 

        while queue:
            current_loc, current_momentum = queue.popleft()
            if current_loc == goal:
                return self.construct_path(parent, (current_loc, current_momentum)) 
            
            z_loc = locations[current_loc][2] 
            for nei in graph[current_loc]: 
                z_difference = locations[nei][2] - z_loc 
                if z_difference <= current_momentum + max_energy:
                    
                    if z_difference <= 0:
                        new_momentum = min(current_momentum - z_difference, max_momentum)
                    else:
                        new_momentum = max(0, current_momentum - z_difference)
                    
                    next_state = (nei, new_momentum) 
                    if next_state not in visited:
                        visited.add(next_state) 
                        parent[next_state] = (current_loc, current_momentum) 
                        queue.append(next_state) 
        return "FAIL"

    def ucs(self):
        start_state = (self.start, 0)
        parent = {start_state: None}
        best_g = {start_state: 0.0}
        counter = 0
        pq = [(0.0, counter, start_state)]
        goal, max_energy, max_momentum, locations, graph = self.goal, self.max_uphill_energy_units, self.max_momentum_units, self.locations, self.graph
        
        while pq:
            total_cost, _ , current_state = heapq.heappop(pq)
            current_loc, current_momentum = current_state
            
            if total_cost > best_g.get(current_state, float('inf')):
                continue
            
            if current_loc == goal:
                return self.construct_path(parent, current_state)
            
            x1, y1, z1 = locations[current_loc]
            for nei in graph[current_loc]:
                z_diff = locations[nei][2] - z1
                if z_diff <= current_momentum + max_energy:
                    x2, y2, _ = locations[nei]
                    cost = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) ** 0.5
                    new_cost = total_cost + cost
                    
                    if z_diff <= 0:
                        new_momentum = min(current_momentum - z_diff, max_momentum)
                    else:
                        new_momentum = max(0, current_momentum - z_diff)
                    
                    next_state = (nei, new_momentum)
                    
                    if new_cost < best_g.get(next_state, float('inf')):
                        best_g[next_state] = new_cost
                        parent[next_state] = current_state
                        counter += 1
                        heapq.heappush(pq, (new_cost, counter, next_state))
        
        return "FAIL"

    def a_star(self):
        goal_coords, locations, goal, max_energy, max_momentum, graph = self.goal_coords, self.locations, self.goal, self.max_uphill_energy_units, self.max_momentum_units, self.graph
        counter = 0
        x1, y1, z1 = locations[self.start]
        x2, y2, z2 = goal_coords
        h_start = ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1)) ** 0.5
        
        start_state = (self.start, 0)
        pq = [(h_start, 0.0,  counter, start_state)]
        parent = {start_state: None}
        best_g = {start_state: 0.0}
        
        while pq:
            f_cost, g_cost, _, current_state = heapq.heappop(pq)
            current_loc, current_momentum = current_state
            
            if g_cost > best_g.get(current_state, float('inf')):
                continue
            
            if current_loc == goal:
                return self.construct_path(parent, current_state)
            
            x1, y1, z1 = locations[current_loc]
            for neighbor in graph[current_loc]:
                x2, y2, z2 = locations[neighbor]
                z_diff = z2 - z1
                
                if z_diff <= current_momentum + max_energy:
                    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
                    edge_cost = (dx * dx + dy * dy + dz * dz) ** 0.5
                    new_g_cost = g_cost + edge_cost
                    
                    if z_diff <= 0:
                        new_momentum = min(current_momentum - z_diff, max_momentum)
                    else:
                        new_momentum = max(0, current_momentum - z_diff)
                    
                    next_state = (neighbor, new_momentum)
                    
                    if new_g_cost < best_g.get(next_state, float('inf')):
                        best_g[next_state] = new_g_cost
                        parent[next_state] = current_state
                        
                        dx, dy, dz = goal_coords[0] - x2, goal_coords[1] - y2, goal_coords[2] - z2
                        h_cost = (dx * dx + dy * dy + dz * dz) ** 0.5
                        new_f_cost = new_g_cost + h_cost
                        
                        counter += 1
                        heapq.heappush(pq, (new_f_cost, new_g_cost, counter, next_state))
        
        return "FAIL"

    def write_output(self, result):
        with open(self.output_file, "w") as f:
            f.write(result)

if __name__ == "__main__":
    explorer = NeptuneExplorer()
    explorer.read_input_build_graph()
    algorithms = {
        "BFS": explorer.bfs,
        "UCS": explorer.ucs,
        "A*": explorer.a_star
    }
    solver = algorithms.get(explorer.algorithm)
    result = solver() if solver else "FAIL"
    explorer.write_output(result)
