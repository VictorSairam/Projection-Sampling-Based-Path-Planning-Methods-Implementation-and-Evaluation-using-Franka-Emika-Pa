from time import time
import numpy as np
from kdtree import KDTREE
import collections
import heapq
import pickle
import itertools
import random


class Graph_PRM:

    def __init__(self, dim, capacity=100000):
        self._edges = collections.defaultdict(list)
        self._kd = KDTREE(dim, capacity)
        self.start_id = None
        self.target_id = None

    def __len__(self):
        return len(self._kd)

    def insert_next_node(self, point):
        node_id = self._kd.insert_node(point)
        return node_id

    def insert_edge(self, node_id, neighbor_id):
        self._edges[node_id].append(neighbor_id)
        self._edges[neighbor_id].append(node_id)

    def find_nearest_node(self, point):
        return self._kd.find_closest_point(point)

    def find_point(self, node_id):
        return self._kd.find_node(node_id).point

    def find_parent(self, child_id):
        return self._edges[child_id]

    def find_neighbor_within_distance(self, point, radius):
        """
        Return a list of node_id within the radius
        """
        return self._kd.get_points_within_distance(point, radius)


class celles:
    def __init__(self):
        self.g = float('inf')
        self.parent = -1


class ProbabilisticRoadMap:
    def __init__(self, panda_arm, is_in_collision):
        self._panda_robot = panda_arm
        self._is_in_collision = is_in_collision

        self._angle_step_size = 0.015
        self._radius = 0.5
        self._k = 15
        self._maximum_nodes_allowed = int(150000)

        self._project_step_size = 1e-1
        self._constraint_th = 1e-3

        self._smoothened_nodes = 0

    def sample_valid_joint_values(self):
        """
        The sampled configuration must be within the joint limits, but it does not check for collisions.
        """
        q = np.random.random(self._panda_robot.num_dof) * (self._panda_robot.upper_joint_limits - self._panda_robot.lower_joint_limits) + self._panda_robot.lower_joint_limits
        return q

    def _is_valid_segment(self, q0, q1):
        """
        Check if the edge between q0 and q1 is collision free by interpolating the segment
        """
        qs = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._angle_step_size))
        for q in qs:
            if self._is_in_collision(q):
                return False
        return True

    def project_to_constraint(self, q0, constraint):
        joint_projected = q0.copy()
        error_, grad = constraint(q0)
        while error_ > self._constraint_th:
            J = self._panda_robot.jacobian(joint_projected)
            joint_projected -= self._project_step_size * J.T.dot(grad)
            error_, grad = constraint(joint_projected)
        return joint_projected

    def preprocess_nodes(self, graph, constraint=None):
        number_of_edges = 0
        while len(graph) < self._maximum_nodes_allowed:
            # Sample valid joints
            q_sample = self.sample_valid_joint_values()
            # Project to constraint
            q_new = self.project_to_constraint(q_sample, constraint)

            # Add the new node to the graph if it is collision free
            if not self._is_in_collision(q_new):
                node_id = graph.insert_new_node(q_new)
                neighbor_node_ids = graph.get_neighbor_within_radius(q_new, self._radius)
                number_of_valid_neighbors = 0
                for neigh_id in neighbor_node_ids:
                    joint_neighbor = graph.get_point(neigh_id)
                    if self._is_valid_segment(q_new, joint_neighbor):
                        graph.add_edge(node_id, neigh_id)
                        number_of_edges += 1
                        number_of_valid_neighbors += 1
                        if number_of_valid_neighbors >= self._k:
                            break
                print('PRM: number of nodes: {}'.format(len(graph)))
                print('PRM: number of edges {}'.format(number_of_edges))
        print("PRM: Graph is built successfully!")

    def path_smoothing(self, path):
        print("PRM: Start path smooth!")
        def getDistance(p):
            dist = 0
            previous = p[0]
            for jointt in p[1:]:
                dist += np.linalg.norm(jointt - previous)
                previous = jointt
            return dist

        for num_smoothed in range(self._smoothened_nodes):
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            if self._is_valid_segment(path[i], path[j]):
                if getDistance(path[i] + path[j]) < getDistance(path[i:j + 1]):
                    path = path[:i + 1] + path[j:]

        # Interpolating between two nodes
        path = [np.linspace(path[i], path[i + 1], int(np.linalg.norm(path[i + 1] - path[i]) / self._angle_step_size)) for i in range(len(path) - 1)]
        path = list(itertools.chain.from_iterable(path))

        print("PRM: Final path length after smooth is {}.".format(len(path)))

        return path

    def search(self, graph):

        def get_heuristic(graph, cur_id, target_id, use_heur=False):
            if use_heur:
                return np.linalg.norm(graph.get_point(target_id) - graph.get_point(cur_id))
            else:
                return 1

        road_map = collections.defaultdict(celles)  # stores the g_value, parent for the node
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
            neighbor_ids = graph.get_parent(cur_id)
            for next_id in neighbor_ids:
                if next_id == graph.target_id:
                    found_path = True
                    road_map[graph.target_id].parent = cur_id
                    break

                if road_map[next_id].g > road_map[cur_id].g + \
                        np.linalg.norm(graph.get_point(next_id) - graph.get_point(cur_id)):
                    road_map[next_id].g = road_map[cur_id].g + \
                                          np.linalg.norm(graph.get_point(next_id) - graph.get_point(cur_id))

                    f_value = road_map[next_id].g + get_heuristic(graph, next_id, graph.target_id, use_heur=False)
                    heapq.heappush(open_list, (f_value, next_id))
                    road_map[next_id].parent = cur_id

        path_to_traverse = []
        if found_path:
            backtrace_path = [graph.get_point(graph.target_id)]
            node_id = road_map[graph.target_id].parent
            while node_id != -1:
                backtrace_path.append(graph.get_point(node_id))
                node_id = (road_map[node_id]).parent

            path_to_traverse = backtrace_path[::-1]

            print("PRM: Found a path! Path length is {}. ".format(len(path_to_traverse)))

        else:
            print('PRM: Was not able to find a path!')

        return path_to_traverse

    def plan(self, q_start, q_target, constraint=None, args=None):
        if args.map3:
            graph_name = 'graph_map3.pickle'
        elif args.map2:
            graph_name = 'graph_map2.pickle'
        else:
            graph_name = 'graph_map1.pickle'

        if args.reuse_graph:
            graph = pickle.load(open(graph_name, 'rb'))
            print("PRM: Reuse the graph.")
        else:
            graph = Graph_PRM(len(q_start), capacity=180000)
            s = time()
            self.preprocess_nodes(graph, constraint)
            print('PRM: Build the graph in {:.2f}s'.format(time() - s))

            with open(graph_name, 'wb') as f:
                pickle.dump(graph, f, -1)
                print('PRM: Graph is saved!')

        s = time()
        graph.start_id = graph.insert_next_node(q_start)
        neighbor_ids = graph.find_neighbor_within_distance(q_start, 1.5)  # 1.5 for map3
        print('PRM: Found neighbor {} with q_start'.format(len(neighbor_ids)))
        for neighbor_id in neighbor_ids:
            q_neighbor = graph.find_point(neighbor_id)
            if self._is_valid_segment(q_start, q_neighbor):
                graph.insert_edge(graph.start_id, neighbor_id)

        graph.target_id = graph.insert_next_node(q_target)
        neighbor_ids = graph.find_neighbor_within_distance(q_target, 1.5)  # 1.5 for map3
        print('PRM: Found neighbor {} with q_target'.format(len(neighbor_ids)))
        for neighbor_id in neighbor_ids:
            q_neighbor = graph.find_point(neighbor_id)
            if self._is_valid_segment(q_target, q_neighbor):
                graph.insert_edge(graph.target_id, neighbor_id)

        print('PRM: Number of nodes connected with start: {}'.format(len(graph.find_parent(graph.start_id))))
        print('PRM: Number of nodes connected with target: {}'.format(len(graph.find_parent(graph.target_id))))

        print('PRM: Total number of nodes: {}'.format(len(graph)))
        path = self.search(graph)
        path = self.path_smoothing(path)
        print('PRM: Found the path in {:.2f}s'.format(time() - s))

        return path
