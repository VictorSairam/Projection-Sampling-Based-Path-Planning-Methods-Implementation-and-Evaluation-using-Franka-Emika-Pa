from time import time
import numpy as np
import random

from kdtree import KDTREE


class Sim_Tree:

    def __init__(self, dim):
        self._parents_map = {}
        self._kd = KDTREE(dim)

    def __len__(self):
        return len(self._kd)

    def insert_new_node(self, point, parent=None):
        node_id = self._kd.insert_node(point)
        self._parents_map[node_id] = parent

        return node_id

    def find_parent(self, child_id):
        return self._parents_map[child_id]

    def find_point(self, node_id):
        return self._kd.find_node(node_id).point

    def find_nearest_node(self, point):
        return self._kd.find_closest_point(point)

    def make_path_to_root_node(self, leaf_node_id):
        path = []
        node_id = leaf_node_id
        while node_id is not None:
            path.append(self.find_point(node_id))
            node_id = self.find_parent(node_id)

        return path

    def find_number_of_nodes(self):
        return len(self._parents_map)


class RRTConnect:

    def __init__(self, panda_arm_, is_in_collision):
        self._panda_arm_ = panda_arm_
        self._is_in_collision = is_in_collision

        self._q_step_size = 0.04   # Default：0.015
        self._maximum_number_of_nodes = int(1e5)
        self._smoothed_nodes = 60

        self._projection_step_size = 1e-1
        self._constraint_th = 1e-3

    def sample_valid_joint_angles(self):
        q = np.random.random(self._panda_arm_.num_dof) * (
                    self._panda_arm_.upper_joint_limits - self._panda_arm_.lower_joint_limits) + self._panda_arm_.lower_joint_limits
        return q

    def projection_to_constraints(self, q0, constraint):
        q_proj = q0.copy()
        err, grad = constraint(q0)
        while err > self._constraint_th:
            J = self._panda_arm_.analytical_jacobian(q_proj)
            q_proj -= self._projection_step_size * J.T.dot(grad)
            err, grad = constraint(q_proj)
        return q_proj

    def _is_valid_segment(self, q0, q1):
        qs = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._q_step_size))
        for joint__ in qs:
            if self._is_in_collision(joint__):
                return False
        return True

    def const_extend(self, tree, q_close, q_target, q_close_id, constraint=None):
        '''
        TODO: Implement extend for RRT Connect
        - Only perform self.project_to_constraint if constraint is not None
        - Use self._is_seg_valid, self._q_step_size
        '''
        qs = qs_prev = q_close
        qs_id = qs_prev_id = q_close_id
        while True:
            if (q_target == qs).all():
                return qs, qs_id
            if np.linalg.norm(q_target - qs) > np.linalg.norm(qs_prev - q_target):
                return qs_prev, qs_prev_id
            qs_prev = qs
            qs_prev_id = qs_id

            qs = qs + min(self._q_step_size, np.linalg.norm(q_target - qs)) * (q_target - qs) / np.linalg.norm(
                q_target - qs)
            if constraint:
                qs = self.projection_to_constraints(qs, constraint)

            if not self._is_in_collision(qs) and self._is_valid_segment(qs_prev, qs):
                qs_id = tree.insert_new_node(qs, qs_prev_id)
            else:
                return qs_prev, qs_prev_id

    def findDistance(self, p):
        dist = 0
        prev = p[0]
        for q in p[1:]:
            dist += np.linalg.norm(q - prev)
            prev = q
        return dist

    def smoothen_Path(self, path, constraint):
        for num_smoothed in range(self._smoothed_nodes):
            tree = Sim_Tree(len(path[0]))
            i = random.randint(0, len(path) - 2)
            j = random.randint(i + 1, len(path) - 1)
            q_reach, q_reach_id = self.const_extend(tree, path[i], path[j], None, constraint)
            if not (q_reach == path[j]).all():
                continue
            # print(i, j, q_reach_id)
            temp_path = tree.make_path_to_root_node(q_reach_id)
            # print(temp_path[::-1])
            # print(path[i:j+1])
            if self.findDistance(temp_path) < self.findDistance(path[i:j + 1]):
                path = path[:i + 1] + temp_path[::-1] + path[j + 1:]
        return path

    def plan_path(self, q_start, q_target, constraint=None):
        tree_0 = Sim_Tree(len(q_start))
        tree_0.insert_new_node(q_start)

        tree_1 = Sim_Tree(len(q_target))
        tree_1.insert_new_node(q_target)

        q_start_is_tree_0 = True

        s = time()
        for num_nodes_sampled in range(self._maximum_number_of_nodes):
            if num_nodes_sampled > 0 and num_nodes_sampled % 20 == 0:
                print('RRTC: Sampled {} nodes'.format(num_nodes_sampled))
            q_random = self.sample_valid_joint_angles()
            node_id_near_0 = tree_0.find_nearest_node(q_random)[0]
            q_near_0 = tree_0.find_point(node_id_near_0)
            qa_reach, qa_reach_id = self.const_extend(tree_0, q_near_0, q_random, node_id_near_0, constraint)

            node_id_near_1 = tree_1.find_nearest_node(qa_reach)[0]
            q_near_1 = tree_1.find_point(node_id_near_1)
            qb_reach, qb_reach_id = self.const_extend(tree_1, q_near_1, qa_reach, node_id_near_1, constraint)

            if (qa_reach == qb_reach).all():
                print("YESSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS")
                reached_target = True
                break

            q_start_is_tree_0 = not q_start_is_tree_0
            tree_0, tree_1 = tree_1, tree_0

        print('RRTC: {} nodes extended in {:.2f}s'.format(len(tree_0) + len(tree_1), time() - s))

        # if not q_start_is_tree_0:
        #     tree_0, tree_1 = tree_1, tree_0

        if reached_target:
            tree_0_backtrace_path = tree_0.make_path_to_root_node(qa_reach_id)
            tree_1_forward_traverse_path = tree_1.make_path_to_root_node(qb_reach_id)

            # q0 = tree_0_backward_path[0]
            # q1 = tree_1_forward_path[0]
            # tree_01_connect_path = np.linspace(q0, q1, int(np.linalg.norm(q1 - q0) / self._q_step_size))[1:].tolist()
            if not q_start_is_tree_0:
                path = tree_1_forward_traverse_path[::-1] + tree_0_backtrace_path
            else:
                path = tree_0_backtrace_path[::-1] + tree_1_forward_traverse_path
            print('RRTC: Found a path! Path length is {}.'.format(len(path)))
        else:
            path = []
            print('RRTC: Was not able to find a path!')
        print('RRTC: Start path smooth')
        path = self.smoothen_Path(path, constraint)
        print('RRTC: Path length after path smooth is {}.'.format(len(path)))

        return path
