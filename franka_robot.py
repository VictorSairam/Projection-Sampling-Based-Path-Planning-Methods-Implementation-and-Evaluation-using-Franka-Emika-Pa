import numpy as np
import quaternion
from itertools import product

import rospy
from sensor_msgs.msg import JointState
from bounding_box_ros_publisher import BoundingBoxesPublisher


class PandaRobot:

    lower_joint_limits = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper_joint_limits = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    home_joints = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])
    denavit_harteberg_parameters = np.array([[0, 0.333, 0, 0],
                                [0, 0, -np.pi/2, 0],
                                [0, 0.316, np.pi/2, 0],
                                [0.0825, 0, np.pi/2, 0],
                                [-0.0825, 0.384, -np.pi/2, 0],
                                [0, 0, np.pi/2, 0],
                                [0.088, 0, np.pi/2, 0],
                                [0, 0.107, 0, 0],
                                [0, 0.1034, 0, 0]])
    num_dof = 7

    _dh_alpha_rot = np.array([
                        [1, 0, 0, 0],
                        [0, -1, -1, 0],
                        [0, -1, -1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    _dh_a_trans = np.array([
                        [1, 0, 0, -1],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    _dh_d_trans = np.array([
                        [1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, -1],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)
    _dh_theta_rot = np.array([
                        [-1, -1, 0, 0],
                        [-1, -1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]
                        ], dtype=np.float32)

    shape_of_collision_boxes = np.array([
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
    links_of_collision_boxes = [1, 1, 1, 1, 1, 3, 4, 5, 5, 5, 7, 7]
    raw_poses_of_collision_boxes = np.array([
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
        self._joint_state_publisher = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._bounding_boxes_publisher = BoundingBoxesPublisher('franka_collision_boxes')

        self._data_of_collision_boxes = np.zeros((len(self.shape_of_collision_boxes), 10))
        self._data_of_collision_boxes[:, -3:] = self.shape_of_collision_boxes

        # Precompute things and preallocate np memory for collision checking
        self._poses_of_bounding_boxes = []
        for pose in self.raw_poses_of_collision_boxes:
            T = np.eye(4)
            T[:3, 3] = pose[:3]
            T[:3, :3] = quaternion.as_rotation_matrix(quaternion.quaternion(*pose[3:]))
            self._poses_of_bounding_boxes.append(T)

        self._bounding_box_hdiags = []
        self.bounding_box_vertices_offset = []
        self._vertex_offset_signs = np.array(list(product([1, -1],[1,-1], [1,-1])))
        for sizes in self.shape_of_collision_boxes:
            hsizes = sizes/2

            self.bounding_box_vertices_offset.append(self._vertex_offset_signs * hsizes)
            self._bounding_box_hdiags.append(np.linalg.norm(sizes/2))
        self.bounding_box_vertices_offset = np.array(self.bounding_box_vertices_offset)
        self._bounding_box_hdiags = np.array(self._bounding_box_hdiags)

        self.collision_projection_ax = np.zeros((3, 15))
        self._box_vertices_offset = np.ones([8, 3])
        self._box_transform = np.eye(4)

    def forw_kine(self, joints):
        '''
        Calculate the position of each joint using the denavit_harteberg_parameters
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 4x4 transformation matrices from the base to the position of each joint.
        '''
        forw_kine = np.zeros((len(self.denavit_harteberg_parameters), 4, 4))
        prev_transform = np.eye(4)

        for i in range(len(self.denavit_harteberg_parameters)):
            a, d, alpha, theta = self.denavit_harteberg_parameters[i]

            if i < self.num_dof:
                theta = theta + joints[i]

            ca, sa = np.cos(alpha), np.sin(alpha)
            ct, st = np.cos(theta), np.sin(theta)
            self._dh_alpha_rot[1, 1] = ca
            self._dh_alpha_rot[1, 2] = -sa
            self._dh_alpha_rot[2, 1] = sa
            self._dh_alpha_rot[2, 2] = ca

            self._dh_a_trans[0, 3] = a
            self._dh_d_trans[2, 3] = d

            self._dh_theta_rot[0, 0] = ct
            self._dh_theta_rot[0, 1] = -st
            self._dh_theta_rot[1, 0] = st
            self._dh_theta_rot[1, 1] = ct

            transformation_matrix = self._dh_alpha_rot.dot(self._dh_a_trans).dot(self._dh_d_trans).dot(self._dh_theta_rot)

            forw_kine[i] = prev_transform.dot(transformation_matrix)
            prev_transform = forw_kine[i]

        return forw_kine

    def end_effector(self, joints):
        '''
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the [x, y, z, roll, pitch, yaw] location of the end-effector.
        '''
        fk = self.forw_kine(joints)
        end_effector_frame = fk[-1,:,:]

        x, y, z = end_effector_frame[:-1,3]
        roll = np.arctan2(end_effector_frame[2,1], end_effector_frame[2,2])
        pitch = np.arcsin(-end_effector_frame[2,0])
        yaw = np.arctan2(end_effector_frame[1,0], end_effector_frame[0,0])

        return np.array([x, y, z, roll, pitch, yaw])

    def jacobian(self, joints):
        '''
        Calculate the jacobians analytically using our forward kinematics
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 6 x num_dof end-effector jacobian.
        '''
        jacobian = np.zeros((6,self.num_dof))
        fk = self.forw_kine(joints)
        end_effector_pose = fk[-1,:3,3]

        for i in range(self.num_dof):
            joint_pos = fk[i,:3,3]
            joint_axis = fk[i,:3,2]
            jacobian[:3,i] = np.cross(joint_axis, (end_effector_pose - joint_pos)).T
            jacobian[3:,i] = joint_axis.T

        return jacobian

    def inv_kinea(self, desired_ee_pos, current_joints):
        '''
        Arguments: desired_ee_pos which is a np array of [x, y, z, r, p, y] which represent the desired end-effector position of the robot
                   current_joints which represents the current location of the robot
        Returns: A numpy array that contains the joints required in order to achieve the desired end-effector position.
        '''
        joints = current_joints.copy()
        curr_end_effector_position = self.end_effector(joints)
        error_in_end_effector = desired_ee_pos - curr_end_effector_position
        alpha = 0.1

        while np.linalg.norm(error_in_end_effector) > 1e-3:
            jacob = self.jacobian(joints)
            joints += alpha * jacob.T.dot(error_in_end_effector.T)
            
            curr_end_effector_position = self.end_effector(joints)
            error_in_end_effector = desired_ee_pos - curr_end_effector_position

        return joints

    def is_self_collision(self, joints):
        '''
        Arguments: joints represents the current location of the robot
        Returns: A boolean where True means the arm has self-collision and false means that there are no collisions.
        '''

        poses_of_franks_boxes = self.get_collision_boxes_poses(joints)
        for i, pose_of_main_boxes in enumerate(poses_of_franks_boxes):
            pose_of_mbox = pose_of_main_boxes[:3, 3]
            ax_of_mbox = pose_of_main_boxes[:3, :3]

            offsets_mbox_vertices = self.bounding_box_vertices_offset[i]
            mbox_vertices = offsets_mbox_vertices.dot(ax_of_mbox.T) + pose_of_mbox

            for j, pose_of_other_box in enumerate(poses_of_franks_boxes):
                if i - 4 <= j <= i + 4:
                    continue

                pose_of_otherbox = pose_of_other_box[:3, 3]
                axes_of_otherbox = pose_of_other_box[:3, :3]
                offset_in_otherbox_vertices = self.bounding_box_vertices_offset[j]
                vertices_of_otherbox = offset_in_otherbox_vertices.dot(axes_of_otherbox.T) + pose_of_otherbox

                # construct axes
                pairs_of_cross_prods = np.array(list(product(ax_of_mbox.T, axes_of_otherbox.T)))
                cross_axes = np.cross(pairs_of_cross_prods[:, 0], pairs_of_cross_prods[:, 1]).T
                self.collision_projection_ax[:, :3] = ax_of_mbox
                self.collision_projection_ax[:, 3:6] = axes_of_otherbox
                self.collision_projection_ax[:, 6:] = cross_axes

                # projection
                mbox_projs = mbox_vertices.dot(self.collision_projection_ax)
                obox_projs = vertices_of_otherbox.dot(self.collision_projection_ax)
                min_mbox_projs, max_mbox_projs = mbox_projs.min(axis=0), mbox_projs.max(axis=0)
                min_obox_projs, max_obox_projs = obox_projs.min(axis=0), obox_projs.max(axis=0)

                # check if no separating planes exist
                if np.all([min_mbox_projs <= max_obox_projs, max_mbox_projs >= min_obox_projs]):
                    return True
        return False

    def is_bounding_box_colliding(self, joints, box):
        '''
        Arguments: joints represents the current location of the robot
                   box contains the position of the center of the box [x, y, z, r, p, y] and the length, width, and height [l, w, h]
        Returns: A boolean where True means the box is in collision with the arm and false means that there are no collisions.
        '''
        pos_of_box, rpy_of_box, hsizes_of_boxes = box[:3], box[3:6], box[6:]/2
        box_q = quaternion.from_euler_angles(rpy_of_box)
        box_axes = quaternion.as_rotation_matrix(box_q)

        self._box_vertices_offset[:,:] = self._vertex_offset_signs * hsizes_of_boxes
        box_vertices = (box_axes.dot(self._box_vertices_offset.T) + np.expand_dims(pos_of_box, 1)).T

        box_hdiag = np.linalg.norm(hsizes_of_boxes)
        min_col_dists = box_hdiag + self._bounding_box_hdiags

        poses_of_franks_boxes = self.get_collision_boxes_poses(joints)
        for i, franka_box_pose in enumerate(poses_of_franks_boxes):
            fbox_pos = franka_box_pose[:3, 3]
            fbox_axes = franka_box_pose[:3, :3]

            # coarse collision check
            if np.linalg.norm(fbox_pos - pos_of_box) > min_col_dists[i]:
                continue

            fbox_vertex_offsets = self.bounding_box_vertices_offset[i]
            fbox_vertices = fbox_vertex_offsets.dot(fbox_axes.T) + fbox_pos

            # construct axes
            pairs_of_cross_prods = np.array(list(product(box_axes.T, fbox_axes.T)))
            cross_axes = np.cross(pairs_of_cross_prods[:,0], pairs_of_cross_prods[:,1]).T
            self.collision_projection_ax[:, :3] = box_axes
            self.collision_projection_ax[:, 3:6] = fbox_axes
            self.collision_projection_ax[:, 6:] = cross_axes

            # projection
            box_projs = box_vertices.dot(self.collision_projection_ax)
            fbox_projs = fbox_vertices.dot(self.collision_projection_ax)
            minimum_box_projections, maximum_box_projections = box_projs.min(axis=0), box_projs.max(axis=0)
            min_fbox_projs, max_fbox_projs = fbox_projs.min(axis=0), fbox_projs.max(axis=0)

            # check if no separating planes exist
            if np.all([minimum_box_projections <= max_fbox_projs, maximum_box_projections >= min_fbox_projs]):
                return True
        
        return False

    def publish_joints(self, joints):
        joint_state = JointState()
        joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        joint_state.header.stamp = rospy.Time.now()

        if len(joints) == 7:
            joints = np.concatenate([joints, [0, 0]])
        joint_state.position = joints

        self._joint_state_publisher.publish(joint_state)

    def get_collision_boxes_poses(self, joints):
        fk = self.forw_kine(joints)

        world_pose_of_boxes = []
        for i, link in enumerate(self.links_of_collision_boxes):
            transformation_of_link = fk[link - 1]
            box_pose_world = transformation_of_link.dot(self._poses_of_bounding_boxes[i])
            world_pose_of_boxes.append(box_pose_world)

        return world_pose_of_boxes

    def publish_collision_boxes(self, joints):
        world_pose_of_boxes = self.get_collision_boxes_poses(joints)

        for i, pose in enumerate(world_pose_of_boxes):
            self._data_of_collision_boxes[i, :3] = pose[:3, 3]
            q = quaternion.from_rotation_matrix(pose[:3, :3])

            for j, k in enumerate('wxyz'):
                self._data_of_collision_boxes[i, 3 + j] = getattr(q, k)

        self._bounding_boxes_publisher.box_publisher(self._data_of_collision_boxes)
