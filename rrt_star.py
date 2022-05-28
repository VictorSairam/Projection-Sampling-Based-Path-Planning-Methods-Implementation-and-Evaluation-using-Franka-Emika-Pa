from time import time
import numpy as np

from kdtree import KDTREE
from panda_arm_fk_ik import PandaArm


class SimpleTree:

    def __init__(self, dim):
        self._parents_map = {}
        self._kd = KDTREE(dim)

    def insert_new_node(self, point, parent=None):
        node_id = self._kd.insert_node(point)
        self._parents_map[node_id] = parent

        return node_id

    def get_parent(self, child_id):
        return self._parents_map[child_id]

    def get_point(self, node_id):
        return self._kd.find_node(node_id).point

    def get_nearest_node(self, point):
        return self._kd.find_closest_point(point)

    def get_neighbor_within_radius(self, point, radius):
        """
        Return a list of node_id within the radius
        """
        return self._kd.get_points_within_distance(point, radius)


class RRT:

    def __init__(self, fr, is_in_collision):
        self._fr = fr
        self._is_in_collision = is_in_collision

        '''
        TODO: You can tune these parameters to improve RRT performance.
        However, make sure the values satisfy the following conditions:
            self._constraint_th < 2e-3
            self._q_step_size < 0.1
        '''
        self._project_step_size = 1e-1  # Default:1e-1
        self._constraint_th = 1e-3  # Default: 1e-3

        self._q_step_size = 0.02  # Default: 0.02, 0.045 for map2
        self._target_p = 0.2  # Default: 0.2
        self._max_n_nodes = int(1e5)
        self.vertices = []

    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                             np.linspace(node1.col, node2.col, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def sample_valid_joints(self):
        '''
        TODO: Implement sampling a random valid configuration.
        The sampled configuration must be within the joint limits, but it does not check for collisions.
        Please use the following in your code:
            self._fr.joint_limis_low - lower joint limits
            self._fr.joint_limis_high - higher joint limits
            self._fr.num_dof - the degree of freedom of franka
        '''
        # Random Sample [1, num_dof] in the configuration space between lower joint limits and higher joint limits
        q = (self._fr.joint_limits_high - self._fr.joint_limits_low) * \
             np.random.random(self._fr.num_dof) + self._fr.joint_limits_low
        return q

    def project_to_constraint(self, q, constraint):
        '''
        TODO: Implement projecting a configuration to satisfy a constraint function using gradient descent.
        Please use the following parameters in your code:
            self._project_step_size - learning rate for gradient descent
            self._constraint_th - a threshold lower than which the constraint is considered to be satisfied
        Input:
            q - the point to be projected
            constraint - a function of q that returns (constraint_value, constraint_gradient)
                         constraint_value is a scalar - it is 0 when the constraint is satisfied
                         constraint_gradient is a vector of length 6 - it is the gradient of the
                                constraint value w.r.t. the end-effector pose (x, y, z, r, p, y)
        Output:
            q_proj - the projected point
        You can obtain the Jacobian by calling self._fr.jacobian(q)
        '''
        q_proj = q.copy()
        err, grad = constraint(q)
        while err > self._constraint_th:
            # print('The error is: ', err)
            J = self._fr.jacobian(q_proj)
            q_proj -= self._project_step_size * J.T.dot(grad)
            # q_proj -= self._project_step_size * J.T.dot(np.linalg.inv(J.dot(J.T))).dot(grad)
            err, grad = constraint(q_proj)
        return q_proj

    def extend(self, tree, q_target, constraint=None):
        '''
        TODO: Implement the constraint extend function.
        Input:
            tree - a SimpleTree object containing existing nodes
            q_target - configuration of the target state, in shape of [1, num_dof]
            constraint - a constraint function used by project_to_constraint
                         do not perform projection if constraint is None
        Output:
            target_reached - bool, whether or not the target has been reached
            new_node_id - node_id of the new node inserted into the tree by this extend operation
                         Note: tree.insert_new_node returns a node_id
        '''
        target_reached = False
        new_node_id = None
        is_collision = True

        while is_collision:
            if np.random.random(1) < self._target_p:
                # Make sure it will approach to the target
                q_sample = q_target
            else:
                q_sample = self.sample_valid_joints()

            # Find the nearest node (q_near) of the sampling point in current nodes tree
            # Make a step from the nearest node (q_near) to become a new node (q_new) and expand the nodes tree
            nearest_node_id = tree.get_nearest_node(q_sample)[0]
            q_near = tree.get_point(nearest_node_id)
            q_new = q_near + min(self._q_step_size, np.linalg.norm(q_sample - q_near)) * (
                q_sample - q_near) / np.linalg.norm(q_sample - q_near)

            # Check if the new node has collision with the constraint
            q_new = self.project_to_constraint(q_new, constraint)

            if self._is_in_collision(q_new):
                is_collision = True
                continue
            else:
                is_collision = False

            # Add the q_new as vertex, and the edge between q_new and q_near as edge to the tree
            new_node_id = tree.insert_new_node(q_new, nearest_node_id)

            # if the new state (q_new) is close to the target state, then we reached the target state
            if np.linalg.norm(q_new - q_target) < self._q_step_size:
                target_reached = True

        return target_reached, new_node_id

    # def get_neighbors(self, new_node, neighbor_size):
    #     '''Get the neighbors that is within the neighbor distance from the node
    #     arguments:
    #         new_node - a new node
    #         neighbor_size - the neighbor distance

    #     return:
    #         neighbors - a list of neighbors that are within the neighbor distance
    #     '''
    #     # Use kdtree to find the neighbors within neighbor size
    #     samples = [[v.row, v.col] for v in self.vertices]
    #     kdtree = spatial.cKDTree(samples)
    #     ind = kdtree.query_ball_point(
    #         [new_node.row, new_node.col], neighbor_size)
    #     neighbors = [self.vertices[i] for i in ind]
    #     # Remove the new_node itself
    #     neighbors.remove(new_node)
    #     return neighbors

    def path_cost(self, start_node, end_node, tree_1):
        '''Compute path cost starting from start node to end node
        arguments:
            start_node - path start node
            end_node - path end node

        return:
            cost - path cost
        '''
        cost = 0
        curr_node = end_node
        while start_node != curr_node:
            # Keep tracing back until finding the start_node 
            # or no path exists
            parent = tree_1.get_parent(curr_node)
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
        
        return cost

    def rewire(self, new_node, neighbors, tree):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        # If no neighbors, skip
        if neighbors == []:
            return

        tree_1 = tree

        # Compute the distance from the new node to the neighbor nodes
        distances = [np.linalg.norm(self.get_point(node)) - (self.get_point(new_node)) for node in neighbors]

        # Rewire the new node
        # compute the least potential cost
        costs = [d + self.path_cost(self.start, neighbors[i], tree_1)
                                    for i, d in enumerate(distances)]
        indices = np.argsort(np.array(costs))
        # check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break

        # Rewire new_node's neighbors
        for i, node in enumerate(neighbors):
            # new cost
            new_cost = self.path_cost(self.start, new_node, tree_1) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(self.start, node, tree_1) > new_cost and \
                    not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]

    def plan(self, q_start, q_target, constraint=None):
        tree = SimpleTree(len(q_start))
        tree.insert_new_node(q_start)

        s = time()
        for n_nodes_sampled in range(self._max_n_nodes):
            if n_nodes_sampled > 0 and n_nodes_sampled % 50 == 0:
                print('RRT: Sampled {} nodes'.format(n_nodes_sampled))

            reached_target, node_id_new = self.extend(
                tree, q_target, constraint)
            # Rewire
            if node_id_new is not None:
                neighbors = self.get_neighbors(node_id_new, neighbor_size, tree)
                self.rewire(node_id_new, neighbors, tree)

            if reached_target:
                break

        print('RRT: Sampled {} nodes in {:.2f}s'.format(
            n_nodes_sampled, time() - s))

        path = []
        if reached_target:
            backward_path = [q_target]
            node_id = node_id_new
            while node_id is not None:
                backward_path.append(tree.get_point(node_id))
                node_id = tree.get_parent(node_id)
            path = backward_path[::-1]

            print('RRT: Found a path! Path length is {}.'.format(len(path)))
        else:
            print('RRT: Was not able to find a path!')

        return path
