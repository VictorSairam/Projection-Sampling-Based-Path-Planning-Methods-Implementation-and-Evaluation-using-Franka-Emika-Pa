from time import time
import numpy as np
import random

from kdtree import KDTree

class TreeBranch:

    def __init__(self, dim):
        self._parents_map = {}
        self._kd = KDTree(dim)

    def __len__(self):
        return len(self._kd)

    def add_node(self, point, parent=None):
        node_id = self._kd.insert(point)
        self._parents_map[node_id] = parent

        return node_id

    def find_out_parent(self, child_id):
        return self._parents_map[child_id]

    def return_corresponding_point(self, node_id):
        return self._kd.get_node(node_id).point

    def find_the_nearest_neighbor(self, point):
        return self._kd.find_nearest_point(point)

    def construct_path_to_root(self, leaf_node_id):
        path = []
        node_id = leaf_node_id
        while node_id is not None:
            path.append(self.return_corresponding_point(node_id))
            node_id = self.find_out_parent(node_id)

        return path

    def get_num_nodes(self):
        return len(self._parents_map)


class RRTConnect:

    def __init__(self, pr, is_colliding):
        self._pr = pr
        self._is_colliding = is_colliding

        self._q_step_size = 0.04   # Default：0.015
        self.maximum_number_of_specified_nodes = int(1e5)
        self._smoothed_nodes = 60

        self.projection_step_size = 1e-1
        self._constraint_th = 1e-3

    def sample_valid_configurations(self):
        q = np.random.random(self._pr.num_dof) * (
                    self._pr.upper_joint_limits - self._pr.lower_joint_limits) + self._pr.lower_joint_limits
        return q

    def projection_constraint(self, q0, constr):
        projected_point = q0.copy()
        error_q, gradient_g = constr(q0)
        while error_q > self._constraint_th:
            J = self._pr.jacobian(projected_point)
            projected_point -= self.projection_step_size * J.T.dot(gradient_g)
            error_q, gradient_g = constr(projected_point)
        return projected_point

    def _is_seg_valid(self, q0, q1):
        qs = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._q_step_size))
        for q in qs:
            if self._is_colliding(q):
                return False
        return True

    def constrained_extend(self, tree, q_near, q_target, q_near_id, constr=None):
        '''
        TODO: Implement extend for RRT Connect
        - Only perform self.projection_constraint if constr is not None
        - Use self._is_seg_valid, self._q_step_size
        '''
        qs = qs_old = q_near
        qs_id = qs_old_id = q_near_id
        while True:
            if (q_target == qs).all():
                return qs, qs_id
            if np.linalg.norm(q_target - qs) > np.linalg.norm(qs_old - q_target):
                return qs_old, qs_old_id
            qs_old = qs
            qs_old_id = qs_id

            qs = qs + min(self._q_step_size, np.linalg.norm(q_target - qs)) * (q_target - qs) / np.linalg.norm(
                q_target - qs)
            if constr:
                qs = self.projection_constraint(qs, constr)

            if not self._is_colliding(qs) and self._is_seg_valid(qs_old, qs):
                qs_id = tree.add_node(qs, qs_old_id)
            else:
                return qs_old, qs_old_id

    def getDistance(self, p):
        dist = 0
        prev = p[0]
        for q in p[1:]:
            dist += np.linalg.norm(q - prev)
            prev = q
        return dist

    def smoothPath(self, path, constr):
        for num_smoothed in range(self._smoothed_nodes):
            tree = TreeBranch(len(path[0]))
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            q_reach, q_reach_id = self.constrained_extend(tree, path[i], path[j], None, constr)
            if not (q_reach == path[j]).all():
                continue
            # print(i, j, q_reach_id)
            temp_path = tree.construct_path_to_root(q_reach_id)
            # print(temp_path[::-1])
            # print(path[i:j+1])
            if self.getDistance(temp_path) < self.getDistance(path[i:j + 1]):
                path = path[:i + 1] + temp_path[::-1] + path[j + 1:]
        return path

    def plan(self, q_start, q_target, constr=None):
        tree_0 = TreeBranch(len(q_start))
        tree_0.add_node(q_start)

        tree_1 = TreeBranch(len(q_target))
        tree_1.add_node(q_target)

        q_start_is_tree_0 = True

        s = time()
        for num_of_sampled_nodes in range(self.maximum_number_of_specified_nodes):
            if num_of_sampled_nodes > 0 and num_of_sampled_nodes % 20 == 0:
                print('RRTC: Sampled {} nodes'.format(num_of_sampled_nodes))
            q_rand = self.sample_valid_configurations()
            node_id_near_0 = tree_0.find_the_nearest_neighbor(q_rand)[0]
            q_near_0 = tree_0.return_corresponding_point(node_id_near_0)
            qa_reach, qa_reach_id = self.constrained_extend(tree_0, q_near_0, q_rand, node_id_near_0, constr)

            node_id_near_1 = tree_1.find_the_nearest_neighbor(qa_reach)[0]
            q_near_1 = tree_1.return_corresponding_point(node_id_near_1)
            qb_reach, qb_reach_id = self.constrained_extend(tree_1, q_near_1, qa_reach, node_id_near_1, constr)

            if (qa_reach == qb_reach).all():
                print("YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                arrived_at_target = True
                break

            q_start_is_tree_0 = not q_start_is_tree_0
            tree_0, tree_1 = tree_1, tree_0

        print('RRTC: {} nodes extended in {:.2f}s'.format(len(tree_0) + len(tree_1), time() - s))

        # if not q_start_is_tree_0:
        #     tree_0, tree_1 = tree_1, tree_0

        if arrived_at_target:
            tree_0_backward_path = tree_0.construct_path_to_root(qa_reach_id)
            tree_1_forward_path = tree_1.construct_path_to_root(qb_reach_id)

            # q0 = tree_0_backward_path[0]
            # q1 = tree_1_forward_path[0]
            # tree_01_connect_path = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._q_step_size))[1:].tolist()
            if not q_start_is_tree_0:
                path = tree_1_forward_path[::-1] + tree_0_backward_path
            else:
                path = tree_0_backward_path[::-1] + tree_1_forward_path
            print('RRTC: Found a path! Path length is {}.'.format(len(path)))
        else:
            path = []
            print('RRTC: Was not able to find a path!')
        print('RRTC: Start path smooth')
        path = self.smoothPath(path, constr)
        print('RRTC: Path length after path smooth is {}.'.format(len(path)))

        return path
