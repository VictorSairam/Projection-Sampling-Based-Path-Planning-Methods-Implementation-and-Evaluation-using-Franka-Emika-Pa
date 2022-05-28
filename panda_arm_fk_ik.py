import numpy as np
import quaternion
from itertools import product

import rospy
from sensor_msgs.msg import JointState
from pub_collision_bounding_boxes import BoundingBoxPublish


class PandaArm:

    lower_joint_limits = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper_joint_limits = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    poses_home = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])

    denavit_params = np.array([[0, 0.333, 0, 0],
                                [0, 0, -np.pi/2, 0],
                                [0, 0.316, np.pi/2, 0],
                                [0.0825, 0, np.pi/2, 0],
                                [-0.0825, 0.384, -np.pi/2, 0],
                                [0, 0, np.pi/2, 0],
                                [0.088, 0, np.pi/2, 0],
                                [0, 0.107, 0, 0],
                                [0, 0.1034, 0, 0]])

    mat_d_translation = np.array([
                        [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, -1],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    mat_theta_rotation = np.array([
                        [-1, -1, 0, 0],
                        [-1, -1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    mat_alpha_rotation = np.array([
                        [1, 0, 0, 0],
                        [0, -1, -1, 0],
                        [0, -1, -1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    mat_a_translation = np.array([
                        [1, 0, 0, -1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)


    num_dof = 7

    shape_of_bounding_boxes = np.array([
        [0.23, 0.2, 0.1],
        [0.13, 0.12, 0.1], 
        [0.12, 0.1, 0.2],
        [0.15, 0.27, 0.11],
        [0.12, 0.1, 0.2],
        [0.13, 0.12, 0.25],
        [0.13, 0.23, 0.15],
        [0.12, 0.12, 0.4],
        [0.12, 0.12, 0.25],
        [0.13, 0.23, 0.12],
        [0.12, 0.12, 0.2],
        [0.08, 0.22, 0.17]
    ])
    links_of_collision_bounding_boxes = [1, 1, 1, 1, 1, 3, 4, 5, 5, 5, 7, 7]
    poses_bounding_boxes_raw = np.array([
        [-.04, 0, -0.283, 1, 0, 0, 0],
        [-0.009, 0, -0.183, 1, 0, 0, 0],
        [0, -0.032, -0.082, 0.95141601, 0.30790838, 0, 0],
        [-0.008, 0, 0, 1, 0, 0, 0],
        [0, .042, .067, 0.95141601, 0.30790838, 0, 0],
        [0.00687, 0, -0.139, 1, 0, 0, 0],
        [-0.008, 0.004, 0, 0.70710678, -0.70710678, 0, 0],
        [0.00422, 0.05367, -0.121, 0.9961947, -0.08715574, 0, 0],
        [0.00422,  0.00367, -0.263, 1, 0, 0, 0],
        [0.00328, 0.0176, -0.0055, 1, 0, 0, 0],
        [-0.0136, 0.0092, 0.0083, 0, 1, 0, 0],
        [0.0136,  -0.0092,  0.1457, 0.92387953, 0, 0, -0.38268343]
    ])

    def __init__(self):
        self.pub_joint_state = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.pub_bounding_boxes = BoundingBoxPublish('franka_collision_boxes')

        self.bounding_boxes_data = np.zeros((len(self.shape_of_bounding_boxes), 10))
        self.bounding_boxes_data[:, -3:] = self.shape_of_bounding_boxes

        # Precompute things and preallocate np memory for collision checking
        self.poses_bounding_boxes = []
        for pose in self.poses_bounding_boxes_raw:
            T = np.eye(4)
            T[:3, 3] = pose[:3]
            T[:3, :3] = quaternion.as_rotation_matrix(quaternion.quaternion(*pose[3:]))
            self.poses_bounding_boxes.append(T)

        self.bounding_box_off_vertices = []
        self.bounding_box_hdiags = []
        self.signs_vertex_off = np.array(list(product([1, -1],[1,-1], [1,-1])))
        for dimss_ in self.shape_of_bounding_boxes:
            horiz_sizes = dimss_/2
            self.bounding_box_off_vertices.append(self.signs_vertex_off * horiz_sizes)
            self.bounding_box_hdiags.append(np.linalg.norm(dimss_/2))
        self.bounding_box_off_vertices = np.array(self.bounding_box_off_vertices)
        self.bounding_box_hdiags = np.array(self.bounding_box_hdiags)

        self.axes_of_collision_projection = np.zeros((3, 15))
        self._box_vertices_offset = np.ones([8, 3])
        self._box_transform = np.eye(4)

    def fk(self, r_joints):
        '''
        Calculate the position of each joint using the denavit_params
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 4x4 transformation matrices from the base to the position of each joint.
        '''
        fk = np.zeros((len(self.denavit_params), 4, 4))
        transform_prev = np.eye(4)

        for i in range(len(self.denavit_params)):
            a, d, alpha_rot, angle_theta = self.denavit_params[i]

            if i < self.num_dof:
                angle_theta = angle_theta + r_joints[i]

            ca, sa = np.cos(alpha_rot), np.sin(alpha_rot)
            ct, st = np.cos(angle_theta), np.sin(angle_theta)
            self.mat_alpha_rotation[1, 1] = ca
            self.mat_alpha_rotation[1, 2] = -sa
            self.mat_alpha_rotation[2, 1] = sa
            self.mat_alpha_rotation[2, 2] = ca

            self.mat_a_translation[0, 3] = a
            self.mat_d_translation[2, 3] = d

            self.mat_theta_rotation[0, 0] = ct
            self.mat_theta_rotation[0, 1] = -st
            self.mat_theta_rotation[1, 0] = st

            transform = self.mat_alpha_rotation.dot(self.mat_a_translation).dot(self.mat_d_translation).dot(self.mat_theta_rotation)

            fk[i] = transform_prev.dot(transform)
            transform_prev = fk[i]

        return fk

    def end_effector_pose(self, joints):
        '''
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the [x, y, z, roll, pitch, yaw] location of the end-effector.
        '''
        fk = self.fk(joints)
        end_effector_frame = fk[-1,:,:]

        x, y, z = end_effector_frame[:-1,3]
        roll = np.arctan2(end_effector_frame[2,1], end_effector_frame[2,2])
        pitch = np.arcsin(-end_effector_frame[2,0])
        yaw = np.arctan2(end_effector_frame[1, 0], end_effector_frame[0, 0])

        return np.array([x, y, z, roll, pitch, yaw])

    def analytical_jacobian(self, joints):
        '''
        Calculate the jacobians analytically using your forward kinematics
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 6 x num_dof end-effector jacobian.
        '''
        jacobian_matrix = np.zeros((6,self.num_dof))
        fk = self.fk(joints)
        end_ef_pos = fk[-1,:3,3]

        for i in range(self.num_dof):
            joint_pos = fk[i,:3,3]
            joint_ax = fk[i,:3,2]
            jacobian_matrix[:3,i] = np.cross(joint_ax, (end_ef_pos - joint_pos)).T
            jacobian_matrix[3:,i] = joint_ax.T

        return jacobian_matrix

    def i_kine(self, desired_pose_of_end_effector, present_joints):
        '''
        Arguments: desired_end_effector_position_and_orientation which is a np array of [x, y, z, r, p, y] which represent the desired end-effector position of the robot
                   current_joints which represents the current location of the robot
        Returns: A numpy array that contains the joints required in order to achieve the desired end-effector position.
        '''
        joint_angles = present_joints.copy()
        current_ee_pos = self.end_effector_pose(joint_angles)
        error_in_end_effector_pose = desired_pose_of_end_effector - current_ee_pos
        alpha = 0.1

        while np.linalg.norm(error_in_end_effector_pose) > 1e-3:
            jacob = self.analytical_jacobian(joint_angles)
            joint_angles += alpha * jacob.T.dot(error_in_end_effector_pose.T)
            
            current_ee_pos = self.end_effector_pose(joint_angles)
            error_in_end_effector_pose = desired_pose_of_end_effector - current_ee_pos

        return joint_angles

    def is_self_collision(self, joints):
        '''
        Arguments: joints represents the current location of the robot
        Returns: A boolean where True means the arm has self-collision and false means that there are no collisions.
        '''

        panda_box_poses = self.get_bounding_collision_poses(joints)
        for i, principal_box_pose in enumerate(panda_box_poses):
            main_box_poses = principal_box_pose[:3, 3]
            main_box_axes = principal_box_pose[:3, :3]

            main_box_vertex_off = self.bounding_box_off_vertices[i]
            main_box_vertices = main_box_vertex_off.dot(main_box_axes.T) + main_box_poses

            for j, next_box_pose in enumerate(panda_box_poses):
                if i - 4 <= j <= i + 4:
                    continue

                other_box_pose = next_box_pose[:3, 3]
                other_box_axes = next_box_pose[:3, :3]
                other_box_vertex_offsets = self.bounding_box_off_vertices[j]
                other_box_vertices = other_box_vertex_offsets.dot(other_box_axes.T) + other_box_pose

                # Axes constructed
                pairs_of_cros_products = np.array(list(product(main_box_axes.T, other_box_axes.T)))
                axes_of_cross_products = np.cross(pairs_of_cros_products[:, 0], pairs_of_cros_products[:, 1]).T
                self.axes_of_collision_projection[:, :3] = main_box_axes
                self.axes_of_collision_projection[:, 3:6] = other_box_axes
                self.axes_of_collision_projection[:, 6:] = axes_of_cross_products

                # projection of the boxes
                main_box_projections = main_box_vertices.dot(self.axes_of_collision_projection)
                other_box_projections = other_box_vertices.dot(self.axes_of_collision_projection)
                main_box_projections_min, main_box_projections_max = main_box_projections.min(axis=0), main_box_projections.max(axis=0)
                other_box_projections_min, other_box_projections_max = other_box_projections.min(axis=0), other_box_projections.max(axis=0)

                # Check for presence of plane of separation
                if np.all([main_box_projections_min <= other_box_projections_max, main_box_projections_max >= other_box_projections_min]):
                    return True
        return False

    def check_box_collision(self, joints, box):
        '''
        Arguments: joints represents the current location of the robot
                   box contains the position of the center of the box [x, y, z, r, p, y] and the length, width, and height [l, w, h]
        Returns: A boolean where True means the box is in collision with the arm and false means that there are no collisions.
        '''
        position_of_box, roll_pitch_yaw_of_box, hori_sizes_boxes = box[:3], box[3:6], box[6:]/2
        q_box = quaternion.from_euler_angles(roll_pitch_yaw_of_box)
        axes_of_box = quaternion.as_rotation_matrix(q_box)

        self._box_vertices_offset[:,:] = self.signs_vertex_off * hori_sizes_boxes
        box_vertices = (axes_of_box.dot(self._box_vertices_offset.T) + np.expand_dims(position_of_box, 1)).T

        hdiag_box = np.linalg.norm(hori_sizes_boxes)
        dists_col_min = hdiag_box + self.bounding_box_hdiags

        franka_box_poses = self.get_bounding_collision_poses(joints)
        for i, franka_box_pose in enumerate(franka_box_poses):
            fbox_axes = franka_box_pose[:3, :3]
            fbox_pos = franka_box_pose[:3, 3]

            # collision check
            if np.linalg.norm(fbox_pos - position_of_box) > dists_col_min[i]:
                continue

            fbox_vertex_offsets = self.bounding_box_off_vertices[i]
            fbox_vertices = fbox_vertex_offsets.dot(fbox_axes.T) + fbox_pos

            # construct axes
            cross_product_pairs = np.array(list(product(axes_of_box.T, fbox_axes.T)))
            cross_axes = np.cross(cross_product_pairs[:,0], cross_product_pairs[:,1]).T
            self.axes_of_collision_projection[:, :3] = axes_of_box
            self.axes_of_collision_projection[:, 3:6] = fbox_axes
            self.axes_of_collision_projection[:, 6:] = cross_axes

            # projection
            box_projs = box_vertices.dot(self.axes_of_collision_projection)
            fbox_projs = fbox_vertices.dot(self.axes_of_collision_projection)
            min_box_projs, max_box_projs = box_projs.min(axis=0), box_projs.max(axis=0)
            min_fbox_projs, max_fbox_projs = fbox_projs.min(axis=0), fbox_projs.max(axis=0)

            # check if no separating planes exist
            if np.all([min_box_projs <= max_fbox_projs, max_box_projs >= min_fbox_projs]):
                return True
        
        return False

    def publish_joints(self, joints):
        joint_state = JointState()
        joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        joint_state.header.stamp = rospy.Time.now()

        if len(joints) == 7:
            joints = np.concatenate([joints, [0, 0]])
        joint_state.position = joints

        self.pub_joint_state.publish(joint_state)

    def get_bounding_collision_poses(self, joints):
        fk = self.fk(joints)

        box_poses_world = []
        for i, link in enumerate(self.links_of_collision_bounding_boxes):
            link_transform = fk[link - 1]
            box_pose_world = link_transform.dot(self.poses_bounding_boxes[i])
            box_poses_world.append(box_pose_world)

        return box_poses_world

    def publish_collision_boxes(self, joints):
        box_poses_world = self.get_bounding_collision_poses(joints)

        for i, pose in enumerate(box_poses_world):
            self.bounding_boxes_data[i, :3] = pose[:3, 3]
            q = quaternion.from_rotation_matrix(pose[:3, :3])

            for j, k in enumerate('wxyz'):
                self.bounding_boxes_data[i, 3 + j] = getattr(q, k)

        self.pub_bounding_boxes.publish_collision_boxes(self.bounding_boxes_data)



