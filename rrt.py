from time import time
import numpy as np

from kdtree import KDTree
from franka_robot import PandaRobot


class TreeBranch:

    def __init__(self, dim):
        self._parents_map = {}
        self._kd = KDTree(dim)

    def add_node(self, point, parent=None):
        node_id = self._kd.insert(point)
        self._parents_map[node_id] = parent

        return node_id
        
    def find_out_parent(self, child_id):
        return self._parents_map[child_id]

    def return_corresonding_point(self, node_id):
        return self._kd.get_node(node_id).point

    def find_the_nearest_neighbor(self, point):
        return self._kd.find_nearest_point(point)


class RRT:

    def __init__(self, pr, is_colliding):
        self._pr = pr
        self._is_colliding = is_colliding

        '''
        TODO: We can tune these parameters to improve RRT performance.
        '''
        self._project_step_size = 1e-1  
        self._constraint_th = 1e-3  

        self._q_step_size = 0.02  
        self._target_p = 0.2 
        self.maximum_number_of_nodes = int(1e5)

    def sample_valid_configurations(self):
        '''
        TODO: Here, we implement sampling to sample a random valid configuration.

        '''
        # Random Sample [1, num_dof] in the configuration space between lower joint limits and higher joint limits
        q = (self._pr.upper_joint_limits - self._pr.lower_joint_limits) * np.random.random(self._pr.num_dof) + self._pr.lower_joint_limits
        return q

    def projection_constraint(self, q, constr):
        '''
        TODO: Implement projecting a configuration to satisfy a constr function using gradient descent.

        Input:
            q - the point to be projected
            constr - a function of q that returns (constraint_value, constraint_gradient)

        Output:
            projected_point - the projected point
        '''
        projected_point = q.copy()
        error_q, gradient_g = constr(q)
        while error_q > self._constraint_th:
            # print('The error is: ', error_q)
            J = self._pr.jacobian(projected_point)
            projected_point -= self._project_step_size * J.T.dot(gradient_g)
            # projected_point -= self._project_step_size * J.T.dot(np.linalg.inv(J.dot(J.T))).dot(gradient_g)
            error_q, gradient_g = constr(projected_point)
        return projected_point

    def extend(self, tree, q_target, constr=None):
        '''
        Here, we implement the constraint_extend function.

        Input: 
            tree - a TreeBranch object containing existing nodes
            q_target - configuration of the target state, in shape of [1, num_dof]
            constr - a constr function used by projection_constraint
                         do not perform projection if constr is None

        Output:
            target_reached - bool, whether or not the target has been reached
            new_node_id - node_id of the new node inserted into the tree by this extend operation
        '''
        target_reached = False
        new_node_id = None
        is_collision = True

        while is_collision:
            if np.random.random(1) < self._target_p:
                # Ensure it approaches the target
                sampled_joint_angle = q_target
            else:
                sampled_joint_angle = self.sample_valid_configurations()

            # Find the nearest neighboring node of the sampling point in current tree
            # Move a step from the nearest node to get a new node and expand the tree
            nearest_node_id = tree.find_the_nearest_neighbor(sampled_joint_angle)[0]
            q_near = tree.return_corresonding_point(nearest_node_id)
            q_new = q_near + min(self._q_step_size, np.linalg.norm(sampled_joint_angle - q_near)) * (sampled_joint_angle - q_near) / np.linalg.norm(sampled_joint_angle - q_near)

            # Check if the newly sampled node has collision with the constraint
            q_new = self.projection_constraint(q_new, constr)

            if self._is_colliding(q_new):
                is_collision = True
                continue
            else:
                is_collision = False

            # We insert the q_new as vertex, and the edge between q_new and q_near as edge to the tree
            new_node_id = tree.add_node(q_new, nearest_node_id)

            # if the new vertex is near the target node, then we attain the target node
            if np.linalg.norm(q_new - q_target) < self._q_step_size:
                target_reached = True

        return target_reached, new_node_id

    def plan(self, q_start, q_target, constr=None):
        tree = TreeBranch(len(q_start))
        tree.add_node(q_start)

        s = time()
        for num_of_sampled_nodes in range(self.maximum_number_of_nodes):
            if num_of_sampled_nodes > 0 and num_of_sampled_nodes % 50 == 0:
                print('RRT: Sampled {} nodes'.format(num_of_sampled_nodes))

            arrived_at_target, node_id_new = self.extend(tree, q_target, constr)

            if arrived_at_target:
                break

        print('RRT: Sampled {} nodes in {:.2f}s'.format(num_of_sampled_nodes, time() - s))

        path = []
        if arrived_at_target:
            backtraced_path = [q_target]
            node_id = node_id_new
            while node_id is not None:
                backtraced_path.append(tree.return_corresonding_point(node_id))
                node_id = tree.find_out_parent(node_id)
            path = backtraced_path[::-1]

            print('RRT: Found a path! Path length is {}.'.format(len(path)))
        else:
            print('RRT: Was not able to find a path!')
        
        return path
