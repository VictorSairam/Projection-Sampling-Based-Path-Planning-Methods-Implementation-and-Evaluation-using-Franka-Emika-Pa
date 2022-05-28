from time import time
import numpy as np
from kdtree import KDTree
import collections
import heapq
import pickle
import itertools
import math
import random


class GraphPRM:

    def __init__(self, dim, capacity=100000):
        self._edges = collections.defaultdict(list)
        self._kd = KDTree(dim, capacity)
        self.start_id = None
        self.target_id = None

    def __len__(self):
        return len(self._kd)

    def add_node(self, point):
        node_id = self._kd.insert(point)
        return node_id

    def add_edge(self, node_id, neighbor_id):
        self._edges[node_id].append(neighbor_id)
        self._edges[neighbor_id].append(node_id)

    def find_out_parent(self, child_id):
        return self._edges[child_id]

    def return_corresponding_point(self, node_id):
        return self._kd.get_node(node_id).point

    def find_the_nearest_neighbor(self, point):
        return self._kd.find_nearest_point(point)

    def list_neighbors_within_distances(self, point, radius):
        """
        Return a list of node_id within the radius
        """
        return self._kd.find_points_within_radius(point, radius)


class cell:
    def __init__(self):
        self.g = float('inf')
        self.parent = -1


class OBPRM:
    def __init__(self, pr, is_colliding):
        self._pr = pr
        self._is_colliding = is_colliding

        self._q_step_size = 0.08
        self.size_of_joint = math.sqrt(0.04 ** 2 / 7) / 2
        self._radius = 0.7
        self._k = 8
        self.maximum_number_of_specified_nodes = int(100000)

        self.projection_step_size = 1e-1
        self._constraint_th = 1e-3

        self._smoothed_nodes = 0

    def sample_valid_configurations(self):
        """
        The sampled configuration must be within the joint limits, but it does not check for collisions.
        """
        q = np.random.random(self._pr.num_dof) * (self._pr.upper_joint_limits - self._pr.lower_joint_limits) + self._pr.lower_joint_limits
        return q

    def sample_configurations_at_joints(self, q):
        q_near = q
        for i in range(len(q)):
            lower = max(self._pr.lower_joint_limits[i], q[i] - self.size_of_joint)
            upper = min(self._pr.upper_joint_limits[i], q[i] + self.size_of_joint)
            q_near[i] = np.random.random() * (upper - lower) + lower
        return q_near

    def _is_seg_valid(self, q0, q1):
        """
        Check if the edge between q0 and q1 is collision free by interpolating the segment
        """
        qs = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._q_step_size))
        for q in qs:
            if self._is_colliding(q):
                return False
        return True

    def projection_constraint(self, q0, constr):
        projected_point = q0.copy()
        error_q, gradient_g = constr(q0)
        while error_q > self._constraint_th:
            J = self._pr.jacobian(projected_point)
            projected_point -= self.projection_step_size * J.T.dot(gradient_g)
            error_q, gradient_g = constr(projected_point)
        return projected_point

    def preprocess(self, graph, constr=None):
        num_edges = 0
        had_collision = False
        sample_count = 0
        while len(graph) < self.maximum_number_of_specified_nodes:
            # Sample valid joints
            if had_collision and sample_count < 80:
                sampled_joint_angle = self.sample_configurations_at_joints(q_new)
            else:
                sampled_joint_angle = self.sample_valid_configurations()
            # Project to constr
            q_new = self.projection_constraint(sampled_joint_angle, constr)

            # Add the new node to the graph if it is collision free
            if not self._is_colliding(q_new):
                sample_count = 0
                node_id = graph.add_node(q_new)
                neighbor_node_ids = graph.list_neighbors_within_distances(q_new, self._radius)
                print('OB_PRM: Number of neighbors: {}'.format(len(neighbor_node_ids)))
                num_valid_neighbor = 0
                for neighbor_id in neighbor_node_ids:
                    q_neighbor = graph.return_corresponding_point(neighbor_id)
                    if self._is_seg_valid(q_new, q_neighbor):
                        graph.add_edge(node_id, neighbor_id)
                        num_edges += 1
                        num_valid_neighbor += 1
                        if num_valid_neighbor >= self._k:
                            break
                print('OB_PRM: number of nodes: {}'.format(len(graph)))
                print('OB_PRM: number of edges {}'.format(num_edges))
                had_collision = False
            else:
                sample_count += 1
                had_collision = True
        print("OB_PRM: Graph is built successfully!")

    def smooth_path(self, path):
        print("OB_PRM: Start path smooth!")

        def getDistance(p):
            distance_in_path = 0
            prev = p[0]
            for q in p[1:]:
                distance_in_path += np.linalg.norm(q - prev)
                prev = q
            return distance_in_path

        for num_smoothed in range(self._smoothed_nodes):
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if self._is_seg_valid(path[i], path[j]):
                if getDistance(path[i] + path[j]) < getDistance(path[i:j + 1]):
                    path = path[:i + 1] + path[j:]

        # Interpolating between two nodes
        path = [np.linspace(path[i], path[i + 1], int(np.linalg.norm(path[i + 1] - path[i]) / self._q_step_size)) for i
                in range(len(path) - 1)]
        path = list(itertools.chain.from_iterable(path))

        print("OB_PRM: Final path length after smooth is {}.".format(len(path)))

        return path

    def search(self, graph):

        def get_heuristic(graph, cur_id, target_id, use_heur=False):
            if use_heur:
                return np.linalg.norm(graph.return_corresponding_point(target_id) - graph.return_corresponding_point(cur_id))
            else:
                return 1

        road_map = collections.defaultdict(cell)  # stores the g_value, parent for the node
        road_map[graph.start_id].g = 0

        open_list = []
        heapq.heappush(open_list, (0, graph.start_id))  # Min-Heap, [f_value, node_id]
        close_list = set()

        found_path = False

        while open_list and not found_path:
            _, cur_id = heapq.heappop(open_list)
            if cur_id in close_list:
                continue

            close_list.add(cur_id)

            # find the neighbor
            neighbor_node_ids = graph.find_out_parent(cur_id)
            for next_id in neighbor_node_ids:
                if next_id == graph.target_id:
                    print("Path is Found!")
                    found_path = True
                    road_map[graph.target_id].parent = cur_id
                    break

                if road_map[next_id].g > road_map[cur_id].g + \
                        np.linalg.norm(graph.return_corresponding_point(next_id) - graph.return_corresponding_point(cur_id)):
                    road_map[next_id].g = road_map[cur_id].g + \
                                          np.linalg.norm(graph.return_corresponding_point(next_id) - graph.return_corresponding_point(cur_id))

                    f_value = road_map[next_id].g + get_heuristic(graph, next_id, graph.target_id, use_heur=False)
                    heapq.heappush(open_list, (f_value, next_id))
                    road_map[next_id].parent = cur_id

        path = []
        if found_path:
            backtraced_path = [graph.return_corresponding_point(graph.target_id)]
            node_id = road_map[graph.target_id].parent
            while node_id != -1:
                backtraced_path.append(graph.return_corresponding_point(node_id))
                node_id = (road_map[node_id]).parent

            path = backtraced_path[::-1]

            print("OB_PRM: Found a path! Path length is {}. ".format(len(path)))

        else:
            print('OB_PRM: Was not able to find a path!')

        return path

    def plan(self, q_start, q_target, constr=None, args=None):
        if args.map3:
            graph_name = 'graph_obprm_map3.pickle'
        elif args.map2:
            graph_name = 'graph_obprm_map2.pickle'
        else:
            graph_name = 'graph_obprm_map1.pickle'

        if args.reuse_graph:
            graph = pickle.load(open(graph_name, 'rb'))
            print("OB_PRM: Reuse the graph.")
        else:
            graph = GraphPRM(len(q_start), capacity=180000)
            s = time()
            self.preprocess(graph, constr)
            print('OB_PRM: Build the graph in {:.2f}s'.format(time() - s))

            with open(graph_name, 'wb') as f:
                pickle.dump(graph, f, -1)
                print('OB_PRM: Graph is saved!')

        s = time()
        graph.start_id = graph.add_node(q_start)
        neighbor_node_ids = graph.list_neighbors_within_distances(q_start, 1.0)
        print('OB_PRM: Found neighbor {} with q_start'.format(len(neighbor_node_ids)))
        for neighbor_id in neighbor_node_ids:
            q_neighbor = graph.return_corresponding_point(neighbor_id)
            if self._is_seg_valid(q_start, q_neighbor):
                graph.add_edge(graph.start_id, neighbor_id)

        graph.target_id = graph.add_node(q_target)
        neighbor_node_ids = graph.list_neighbors_within_distances(q_target, 1.0)
        print('OB_PRM: Found neighbor {} with q_target'.format(len(neighbor_node_ids)))
        for neighbor_id in neighbor_node_ids:
            q_neighbor = graph.return_corresponding_point(neighbor_id)
            if self._is_seg_valid(q_target, q_neighbor):
                graph.add_edge(graph.target_id, neighbor_id)

        print('OB_PRM: Number of nodes connected with start: {}'.format(len(graph.find_out_parent(graph.start_id))))
        print('OB_PRM: Number of nodes connected with target: {}'.format(len(graph.find_out_parent(graph.target_id))))

        print('OB_PRM: Total number of nodes: {}'.format(len(graph)))
        path = self.search(graph)
        path = self.smooth_path(path)
        print('OB_PRM: Found the path in {:.2f}s'.format(time() - s))

        return path
