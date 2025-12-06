import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
from PIL import Image

TERRAIN_GROUND = 0
TERRAIN_WATER = 1

def plot_line_segments(segments, **kwargs):
    if not segments: return
    label = kwargs.pop('label', None)
    first_p1, first_p2 = segments[0]
    plt.plot([first_p1[0], first_p2[0]], [first_p1[1], first_p2[1]], label=label, **kwargs)
    for p1, p2 in segments[1:]:
        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], **kwargs)

class AStarGraphSolver:
    """
    A terrain-aware A* solver for a pre-computed graph.
    """
    def __init__(self, graph, nodes, start_idx, goal_idx, terrain_lookup_func):
        self.graph = graph
        self.nodes = nodes
        self.start_idx = start_idx
        self.goal_idx = goal_idx
        self.terrain_lookup_func = terrain_lookup_func # Function to query terrain type

        self.open_set = [(0, self.start_idx)]
        self.came_from = {}
        
        self.cost_to_arrive = {i: float('inf') for i in self.graph.keys()}
        self.cost_to_arrive[self.start_idx] = 0
        
        self.est_cost_through = {i: float('inf') for i in self.graph.keys()}
        self.est_cost_through[self.start_idx] = self.heuristic(self.start_idx)
        
        self.path = None

    def heuristic(self, node_idx):
        """Standard Euclidean distance heuristic. It's unaware of terrain."""
        return np.linalg.norm(self.nodes[node_idx] - self.nodes[self.goal_idx])

    def get_neighbors(self, node_idx):
        return self.graph.get(node_idx, {}).items()

    # --- Method to calculate terrain-based cost ---
    def _calculate_edge_cost(self, idx1, idx2, geometric_cost):
        """Calculates the traversal cost of an edge based on terrain."""
        type1 = self.terrain_lookup_func(self.nodes[idx1])
        type2 = self.terrain_lookup_func(self.nodes[idx2])

        ####################### Code starts here #######################
        # geometric_cost is the distance value to be multiplied by the relevant value
        # type1 and type2 will return values equal to TERRAIN_GROUND or TERRAIN_WATER, seen above



        
        ####################### Code ends here #########################

    def reconstruct_path(self):
        path_indices = [self.goal_idx]
        current_idx = self.goal_idx
        while current_idx in self.came_from:
            current_idx = self.came_from[current_idx]
            path_indices.append(current_idx)
        return list(reversed(path_indices))

    def solve(self):
        """A* search loop, now using terrain-modified costs."""
        while self.open_set:
            _, current_idx = heapq.heappop(self.open_set)

            if current_idx == self.goal_idx:
                self.path = self.reconstruct_path()
                return True

            for neighbor_idx, geometric_cost in self.get_neighbors(current_idx):
                
                traversal_cost = self._calculate_edge_cost(current_idx, neighbor_idx, geometric_cost)
                
                tentative_cost_to_arrive = self.cost_to_arrive[current_idx] + traversal_cost
                
                if tentative_cost_to_arrive < self.cost_to_arrive.get(neighbor_idx, float('inf')):
                    self.came_from[neighbor_idx] = current_idx
                    self.cost_to_arrive[neighbor_idx] = tentative_cost_to_arrive
                    self.est_cost_through[neighbor_idx] = tentative_cost_to_arrive + self.heuristic(neighbor_idx)
                    heapq.heappush(self.open_set, (self.est_cost_through[neighbor_idx], neighbor_idx))
        
        return False


class PRM:
    """PRM class, with background image support."""
    def __init__(self, statespace_lo, statespace_hi, obstacles, ground_regions, background_image_path=None):
        self.statespace_lo = np.array(statespace_lo)
        self.statespace_hi = np.array(statespace_hi)
        self.obstacles = obstacles
        self.ground_regions = ground_regions # Store island location definitions
        self.background_image_path = background_image_path # Store image path
        self.nodes, self.graph, self.path = [], {}, None

    def get_terrain_type(self, point):
        """Returns the terrain type for a given point."""
        ####################### Code starts here #######################
        # The function must return TERRAIN_GROUND if the query point "point" is contained within an island and return "TERRAIN_WATER" otherwise
        # point[0]: horizontal position of point, point[1]: vertical position of point
        # Each (rectangular) island is defined with four values:
        # x, y: x and y positions of lower left vertex
        # w, h: width and height of island
        for region in self.ground_regions:
            x, y, w, h = region
            pass
        

        ####################### Code ends here #########################

    def is_collision_free_point(self, point):
        ####################### Code starts here #######################
        # The function must return False if the query point "point" is contained in at least one of the obstacles (otherwise, return True)
        # The (rectangular) obstacle is defined with four values:
        # obs[0], obs[1]: x and y positions of lower left vertex
        # obs[2], obs[3]: width and height of rectangle
        for obs in self.obstacles:
            pass

        
        ####################### Code ends here #########################
        
    def is_collision_free_edge(self, p1, p2, num_steps=20):
        for i in range(num_steps + 1):
            if not self.is_collision_free_point(p1 * (1 - i/num_steps) + p2 * (i/num_steps)): return False
        return True

        
    def build_roadmap(self, num_samples, k_neighbors):
        print(f"Building roadmap with {num_samples} samples and k={k_neighbors}...")
        while len(self.nodes) < num_samples: # samples 
            sample = np.random.uniform(self.statespace_lo, self.statespace_hi)
            if self.is_collision_free_point(sample): self.nodes.append(sample) #add candidate point to nodes if not in an obstacle 
        self.nodes = np.array(self.nodes)
        self.graph = {i: {} for i in range(len(self.nodes))}
        for i in range(len(self.nodes)):
            distances = np.linalg.norm(self.nodes - self.nodes[i], axis=1)
            ####################### Code starts here #######################
            # Identify the "k_neighbors" nearest nodes in the graph and for each, save the connection between it and the current node if path between them is free
            # Use is_collision_free_edge(self.nodes[i],self.nodes[j]) to determine if the path between nodes "i" and "j" is clear
            # To create a connection between node "i" and "j", set self.graph[i][j] and self.graph[j][i] to be the distance between the nodes
            
            
            ####################### Code ends here #########################
        print("Roadmap built.")

        
    def solve(self, x_init, x_goal, k_neighbors_query=10):
        self.x_init, self.x_goal = np.array(x_init), np.array(x_goal)
        if not self.is_collision_free_point(self.x_init) or not self.is_collision_free_point(self.x_goal):
            print("Error: Start or Goal position is in collision."); return False
        query_nodes = np.vstack([self.nodes, self.x_init, self.x_goal])
        start_idx, goal_idx = len(self.nodes), len(self.nodes) + 1
        query_graph = {i: v.copy() for i, v in self.graph.items()}
        query_graph[start_idx], query_graph[goal_idx] = {}, {}
        distances_to_start = np.linalg.norm(self.nodes - self.x_init, axis=1)
        for neighbor_idx in np.argsort(distances_to_start)[:k_neighbors_query]:
            if self.is_collision_free_edge(self.x_init, query_nodes[neighbor_idx]):
                query_graph[start_idx][neighbor_idx] = query_graph[neighbor_idx][start_idx] = distances_to_start[neighbor_idx]
        distances_to_goal = np.linalg.norm(self.nodes - self.x_goal, axis=1)
        for neighbor_idx in np.argsort(distances_to_goal)[:k_neighbors_query]:
            if self.is_collision_free_edge(self.x_goal, query_nodes[neighbor_idx]):
                query_graph[goal_idx][neighbor_idx] = query_graph[neighbor_idx][goal_idx] = distances_to_goal[neighbor_idx]
        astar_solver = AStarGraphSolver(query_graph, query_nodes, start_idx, goal_idx, self.get_terrain_type)
        if astar_solver.solve():
            self.path = [query_nodes[i] for i in astar_solver.path]; print("Solution found!"); return True
        else:
            self.path = None; print("Solution not found!"); return False

    def plot(self):
        plt.figure(figsize=(10, 10))
        ax = plt.gca()

        # 1. Plot background image if provided
        if self.background_image_path:
            try:
                img = plt.imread(self.background_image_path)
                ax.imshow(img, zorder=-1,
                          extent=[self.statespace_lo[0], self.statespace_hi[0],
                                  self.statespace_lo[1], self.statespace_hi[1]])
            except FileNotFoundError:
                print(f"Warning: Background image not found at '{self.background_image_path}'")

        # Plot ground regions
        for i, region in enumerate(self.ground_regions):
            ax.add_patch(patches.Rectangle((region[0], region[1]), region[2], region[3],
                                            facecolor='lightgreen', edgecolor='darkgreen', zorder=0,
                                            alpha = 0.3,label="Ground Terrain" if i == 0 else ""))

        # Plot the rest of the elements
        for i, obs in enumerate(self.obstacles):
            ax.add_patch(patches.Rectangle((obs[0], obs[1]), obs[2], obs[3],
                                            facecolor='saddlebrown', edgecolor='black', zorder=1,
                                            alpha = 0.5, label="Obstacles" if i == 0 else ""))
        segments = []
        for node_idx, neighbors in self.graph.items():
            for neighbor_idx in neighbors:
                if node_idx < neighbor_idx:
                    segments.append((self.nodes[node_idx], self.nodes[neighbor_idx]))
        plot_line_segments(segments, color="skyblue", linewidth=0.7, alpha=0.8, label="PRM Graph", zorder=2)
        if hasattr(self, 'x_init'):
            plt.scatter(self.x_init[0], self.x_init[1], color="lime", s=100, zorder=10, label="Start")
            plt.scatter(self.x_goal[0], self.x_goal[1], color="red", s=100, zorder=10, marker='*', label="Goal")
        if self.path:
            plt.plot(np.array(self.path)[:, 0], np.array(self.path)[:, 1], color='gold', linewidth=3, zorder=8, label="A* Solution Path")

        # Finalize plot
        plt.xlim(self.statespace_lo[0], self.statespace_hi[0])
        plt.ylim(self.statespace_lo[1], self.statespace_hi[1])
        plt.title("PRM with Terrain-Aware A* (Islands)")
        ax.set_aspect('equal', adjustable='box')
        plt.legend()
        plt.grid(True, which='both', linestyle='--', linewidth=0.5)
        plt.show()