import numpy as np
import quaternion
from itertools import product

import rospy
from sensor_msgs.msg import JointState
from collision_boxes_publisher import CollisionBoxesPublisher


class PandaArm:

    denavit_params = np.array([[0, 0.333, 0, 0],
                                [0, 0, -np.pi/2, 0],
                                [0, 0.316, np.pi/2, 0],
                                [0.0825, 0, np.pi/2, 0],
                                [-0.0825, 0.384, -np.pi/2, 0],
                                [0, 0, np.pi/2, 0],
                                [0.088, 0, np.pi/2, 0],
                                [0, 0.107, 0, 0],
                                [0, 0.1034, 0, 0]])
    lower_joint_limits = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    upper_joint_limits = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    poses_home = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])


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
    _collision_box_poses_raw = np.array([
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
        self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self._collision_boxes_pub = CollisionBoxesPublisher('franka_collision_boxes')

        self._collision_boxes_data = np.zeros((len(self.shape_of_bounding_boxes), 10))
        self._collision_boxes_data[:, -3:] = self.shape_of_bounding_boxes

        # Precompute things and preallocate np memory for collision checking
        self._collision_box_poses = []
        for pose in self._collision_box_poses_raw:
            T = np.eye(4)
            T[:3, 3] = pose[:3]
            T[:3, :3] = quaternion.as_rotation_matrix(quaternion.quaternion(*pose[3:]))
            self._collision_box_poses.append(T)

        self._collision_box_hdiags = []
        self._collision_box_vertices_offset = []
        self._vertex_offset_signs = np.array(list(product([1, -1],[1,-1], [1,-1])))
        for sizes in self.shape_of_bounding_boxes:
            hsizes = sizes/2

            self._collision_box_vertices_offset.append(self._vertex_offset_signs * hsizes)
            self._collision_box_hdiags.append(np.linalg.norm(sizes/2))
        self._collision_box_vertices_offset = np.array(self._collision_box_vertices_offset)
        self._collision_box_hdiags = np.array(self._collision_box_hdiags)

        self._collision_proj_axes = np.zeros((3, 15))
        self._box_vertices_offset = np.ones([8, 3])
        self._box_transform = np.eye(4)

    def forward_kinematics(self, joints):
        '''
        Calculate the position of each joint using the denavit_params
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 4x4 transformation matrices from the base to the position of each joint.
        '''
        forward_kinematics = np.zeros((len(self.denavit_params), 4, 4))
        previous_transformation = np.eye(4)

        for i in range(len(self.denavit_params)):
            a, d, alpha, theta = self.denavit_params[i]

            if i < self.num_dof:
                theta = theta + joints[i]

            ca, sa = np.cos(alpha), np.sin(alpha)
            ct, st = np.cos(theta), np.sin(theta)
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

            forward_kinematics[i] = previous_transformation.dot(transform)
            previous_transformation = forward_kinematics[i]

        return forward_kinematics

    def ee(self, joints):
        '''
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the [x, y, z, roll, pitch, yaw] location of the end-effector.
        '''
        fk = self.forward_kinematics(joints)
        ee_frame = fk[-1,:,:]

        x, y, z = ee_frame[:-1,3]
        roll = np.arctan2(ee_frame[2,1], ee_frame[2,2])
        pitch = np.arcsin(-ee_frame[2,0])
        yaw = np.arctan2(ee_frame[1,0], ee_frame[0,0])

        return np.array([x, y, z, roll, pitch, yaw])

    def jacobian(self, joints):
        '''
        Calculate the jacobians analytically using your forward kinematics
        Arguments: array of joint positions (rad)
        Returns: A numpy array that contains the 6 x num_dof end-effector jacobian.
        '''
        jacobian = np.zeros((6,self.num_dof))
        fk = self.forward_kinematics(joints)
        ee_pos = fk[-1,:3,3]

        for i in range(self.num_dof):
            joint_pos = fk[i,:3,3]
            joint_axis = fk[i,:3,2]
            jacobian[:3,i] = np.cross(joint_axis, (ee_pos - joint_pos)).T
            jacobian[3:,i] = joint_axis.T

        return jacobian

    def inverse_kinematics(self, desired_ee_pos, current_joints):
        '''
        Arguments: desired_ee_pos which is a np array of [x, y, z, r, p, y] which represent the desired end-effector position of the robot
                   current_joints which represents the current location of the robot
        Returns: A numpy array that contains the joints required in order to achieve the desired end-effector position.
        '''
        joints = current_joints.copy()
        current_ee_pos = self.ee(joints)
        ee_error = desired_ee_pos - current_ee_pos
        alpha = 0.1

        while np.linalg.norm(ee_error) > 1e-3:
            jacob = self.jacobian(joints)
            joints += alpha * jacob.T.dot(ee_error.T)
            
            current_ee_pos = self.ee(joints)
            ee_error = desired_ee_pos - current_ee_pos

        return joints

    def check_self_collision(self, joints):
        '''
        Arguments: joints represents the current location of the robot
        Returns: A boolean where True means the arm has self-collision and false means that there are no collisions.
        '''

        franka_box_poses = self.get_collision_boxes_poses(joints)
        for i, main_box_pose in enumerate(franka_box_poses):
            mbox_pos = main_box_pose[:3, 3]
            mbox_axes = main_box_pose[:3, :3]

            mbox_vertex_offsets = self._collision_box_vertices_offset[i]
            mbox_vertices = mbox_vertex_offsets.dot(mbox_axes.T) + mbox_pos

            for j, other_box_pose in enumerate(franka_box_poses):
                if i - 4 <= j <= i + 4:
                    continue

                obox_pos = other_box_pose[:3, 3]
                obox_axes = other_box_pose[:3, :3]
                obox_vertex_offsets = self._collision_box_vertices_offset[j]
                obox_vertices = obox_vertex_offsets.dot(obox_axes.T) + obox_pos

                # construct axes
                cross_product_pairs = np.array(list(product(mbox_axes.T, obox_axes.T)))
                cross_axes = np.cross(cross_product_pairs[:, 0], cross_product_pairs[:, 1]).T
                self._collision_proj_axes[:, :3] = mbox_axes
                self._collision_proj_axes[:, 3:6] = obox_axes
                self._collision_proj_axes[:, 6:] = cross_axes

                # projection
                mbox_projs = mbox_vertices.dot(self._collision_proj_axes)
                obox_projs = obox_vertices.dot(self._collision_proj_axes)
                min_mbox_projs, max_mbox_projs = mbox_projs.min(axis=0), mbox_projs.max(axis=0)
                min_obox_projs, max_obox_projs = obox_projs.min(axis=0), obox_projs.max(axis=0)

                # check if no separating planes exist
                if np.all([min_mbox_projs <= max_obox_projs, max_mbox_projs >= min_obox_projs]):
                    return True
        return False

    def check_box_collision(self, joints, box):
        '''
        Arguments: joints represents the current location of the robot
                   box contains the position of the center of the box [x, y, z, r, p, y] and the length, width, and height [l, w, h]
        Returns: A boolean where True means the box is in collision with the arm and false means that there are no collisions.
        '''
        box_pos, box_rpy, box_hsizes = box[:3], box[3:6], box[6:]/2
        box_q = quaternion.from_euler_angles(box_rpy)
        box_axes = quaternion.as_rotation_matrix(box_q)

        self._box_vertices_offset[:,:] = self._vertex_offset_signs * box_hsizes
        box_vertices = (box_axes.dot(self._box_vertices_offset.T) + np.expand_dims(box_pos, 1)).T

        box_hdiag = np.linalg.norm(box_hsizes)
        min_col_dists = box_hdiag + self._collision_box_hdiags

        franka_box_poses = self.get_collision_boxes_poses(joints)
        for i, franka_box_pose in enumerate(franka_box_poses):
            fbox_pos = franka_box_pose[:3, 3]
            fbox_axes = franka_box_pose[:3, :3]

            # coarse collision check
            if np.linalg.norm(fbox_pos - box_pos) > min_col_dists[i]:
                continue

            fbox_vertex_offsets = self._collision_box_vertices_offset[i]
            fbox_vertices = fbox_vertex_offsets.dot(fbox_axes.T) + fbox_pos

            # construct axes
            cross_product_pairs = np.array(list(product(box_axes.T, fbox_axes.T)))
            cross_axes = np.cross(cross_product_pairs[:,0], cross_product_pairs[:,1]).T
            self._collision_proj_axes[:, :3] = box_axes
            self._collision_proj_axes[:, 3:6] = fbox_axes
            self._collision_proj_axes[:, 6:] = cross_axes

            # projection
            box_projs = box_vertices.dot(self._collision_proj_axes)
            fbox_projs = fbox_vertices.dot(self._collision_proj_axes)
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

        self._joint_state_pub.publish(joint_state)

    def get_collision_boxes_poses(self, joints):
        fk = self.forward_kinematics(joints)

        box_poses_world = []
        for i, link in enumerate(self.links_of_collision_bounding_boxes):
            link_transform = fk[link - 1]
            box_pose_world = link_transform.dot(self._collision_box_poses[i])
            box_poses_world.append(box_pose_world)

        return box_poses_world

    def publish_collision_boxes(self, joints):
        box_poses_world = self.get_collision_boxes_poses(joints)

        for i, pose in enumerate(box_poses_world):
            self._collision_boxes_data[i, :3] = pose[:3, 3]
            q = quaternion.from_rotation_matrix(pose[:3, :3])

            for j, k in enumerate('wxyz'):
                self._collision_boxes_data[i, 3 + j] = getattr(q, k)

        self._collision_boxes_pub.publish_boxes(self._collision_boxes_data)



# PUBLISHER NODE FOR COLLISION BOXES
class CollisionBoxesPublisher:

    def __init__(self, name):
        self._boxes_pub = rospy.Publisher(name, MarkerArray, queue_size=10)

    def publish_boxes(self, boxes):
        markers = []
        for i, box in enumerate(boxes):
            marker = Marker()
            marker.type = Marker.CUBE
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = 'panda_link0'
            marker.id = i

            marker.lifetime = rospy.Duration()

            marker.pose.position.x = box[0]
            marker.pose.position.y = box[1]
            marker.pose.position.z = box[2]

            marker.scale.x = box[-3]
            marker.scale.y = box[-2]
            marker.scale.z = box[-1]

            if len(box) == 9:
                q = quaternion.from_euler_angles(box[3], box[4], box[5])
                for k in 'wxyz':
                    setattr(marker.pose.orientation, k, getattr(q, k))
            elif len(box) == 10:
                for j, k in enumerate('wxyz'):
                    setattr(marker.pose.orientation, k, box[3 + j])
            else:
                raise ValueError('Invalid format for box!')

            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 0.6

            markers.append(marker)

        marker_array = MarkerArray(markers)
        self._boxes_pub.publish(marker_array)
