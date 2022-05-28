import argparse
import numpy as np
import rospy

from panda_arm_fk_ik import PandaArm
from pub_collision_bounding_boxes import BoundingBoxPublish
from rrt import RRT
from rrt_connect import RRTConnect
from prm import Graph_PRM
from ob_prm import OBPRM


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y','1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n','0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', '-s', type=int, default=0)
    parser.add_argument('--rrt', '-rrt', type=str2bool, const=True, nargs='?', default=False, help="Use RRT?")
    parser.add_argument('--rrtc', '-rrtc', type=str2bool, const=True, nargs='?', default=False, help="Use RRT-Connect?")
    parser.add_argument('--prm', '-prm', type=str2bool, const=True, nargs='?', default=False, help="Use PRM?")
    parser.add_argument('--obprm', '-obprm', type=str2bool, const=True, nargs='?', default=False, help="Use OBPRM?")
    parser.add_argument('--map2', '-map2', type=str2bool, const=True, nargs='?', default=False, help="Use map 2?")
    parser.add_argument('--map3', '-map3', type=str2bool, const=True, nargs='?', default=False, help="Use map 3?")
    parser.add_argument('--reuse_graph', '-reuse_graph', type=str2bool, const=True, nargs='?', default=False, help="Reuse the graph for PRM?")
    args = parser.parse_args()

    np.random.seed(args.seed)
    panda = PandaArm()

    rospy.init_node('planner')

    '''
    TODO: Replace obstacle box w/ the box specs in your workspace:
    [x, y, z, r, p, y, sx, sy, sz]
    '''
    if args.map3:
        boxes = np.array([
            # obstacle
            [0.45, -0.45, 0.7, 0, 0, 0.78, 0.6, 0.6, 0.05],
            # sides
            [-0.7, 0.7, 0.75, 0, 0, 0.78, 2, 0.01, 1.6],
            [0.7, -0.7, 0.75, 0, 0, 0.78, 2, 0.01, 1.6],
            # back
            [-0.7, -0.7, 0.75, 0, 0, 0.78, 0.01, 2, 1.6],
            # front
            [0.7, 0.7, 0.75, 0, 0, 0.78, 0.01, 2, 1.6],
            # top
            [0, 0, 1.5, 0, 0, 0.78, 2, 2, 0.01],
            # bottom
            [0, 0, -0.05, 0, 0, 0.78, 2, 2, 0.01]
        ])
    elif args.map2:
        boxes = np.array([
            # obstacle
            [0.7, 0, 0.6, 0, 0, 0, 0.45, 0.3, 0.05],
            # sides
            [0.15, 0.66, 0.65, 0, 0, 0, 1.2, 0.01, 1.5],
            [0.15, -0.66, 0.65, 0, 0, 0, 1.2, 0.01, 1.5],
            # back
            [-0.41, 0, 0.65, 0, 0, 0, 0.01, 1.4, 1.5],
            # front
            [0.75, 0, 0.65, 0, 0, 0, 0.01, 1.4, 1.5],
            # top
            [0.2, 0, 1.35, 0, 0, 0, 1.2, 1.4, 0.01],
            # bottom
            [0.2, 0, -0.05, 0, 0, 0, 1.2, 1.4, 0.01]
        ])
    else:
        boxes = np.array([
            # obstacle
            # [0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0.4, 0, 0.25, 0, 0, 0, 0.3, 0.05, 0.5],
            # sides
            [0.15, 0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
            [0.15, -0.46, 0.5, 0, 0, 0, 1.2, 0.01, 1.1],
            # back
            [-0.41, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
            # front
            [0.75, 0, 0.5, 0, 0, 0, 0.01, 1, 1.1],
            # top
            [0.2, 0, 1, 0, 0, 0, 1.2, 1, 0.01],
            # bottom
            [0.2, 0, -0.05, 0, 0, 0, 1.2, 1, 0.01]
        ])


    def is_in_collision(joints):
        if panda.is_self_collision(joints):
            return True
        for box in boxes:
            if panda.check_box_collision(joints, box):
                return True
        return False


    desired_ee_rp = panda.end_effector_pose(panda.poses_home)[3:5]


    def upright_constraint_end_effector(q):
        '''
        TODO: Implement constraint function and its gradient.

        This constraint should enforce the end-effector stays upright.
        Hint: Use the roll and pitch angle in desired_ee_rp. The end-effector is upright in its home state.

        Input:
            q - a joint configuration

        Output:
            err - a non-negative scalar that is 0 when the constraint is satisfied
            grad - a vector of length 6, where the ith element is the derivative of err w.r.t. the ith element of ee
        '''
        end_effector = panda.end_effector_pose(q)
        error_end_eff = np.sum((np.asarray(desired_ee_rp) - np.asarray(end_effector[3:5])) ** 2)
        gradient_ = np.asarray([0, 0, 0, 2 * (end_effector[3] - desired_ee_rp[0]), 2 * (end_effector[4] - desired_ee_rp[1]), 0])
        return error_end_eff, gradient_

    '''
    TODO: Determine the plan quality or the cost of the path
    '''
    def ret_plan_quality(plan):
        distance = 0
        for i in range(len(plan) - 1):
            distance += np.linalg.norm(np.array(plan[i+1]) - np.array(plan[i]))
        return distance

    '''
    TODO: Fill in start and target joint positions
    '''
    if args.map3:
        source_angles = np.array([0, 3*np.pi/8, 0, -np.pi / 8, 0, np.pi / 2, np.pi / 4])
        source_angles[0] = -np.deg2rad(45)
        goal_angles = np.array([0, 0, 0, -np.pi / 4, 0, np.pi / 4, np.pi / 4])
        goal_angles[0] = -np.deg2rad(45)
    elif args.map2:
        source_angles = np.array([0, np.pi/6, 0, -2*np.pi / 3, 0, 5*np.pi / 6, np.pi / 4])
        goal_angles = np.array([0, 0, 0, -np.pi / 4, 0, np.pi / 4, np.pi / 4])
    else:
        source_angles = panda.poses_home.copy()
        source_angles[0] = -np.deg2rad(45)
        goal_angles = source_angles.copy()
        goal_angles[0] = np.deg2rad(45)


    if args.rrt:
        print("RRT: RRT planner is selected!")
        planner = RRT(panda, is_in_collision)
    elif args.rrtc:
        print("RRTC: RRT Connect planner is selected!")
        planner = RRTConnect(panda, is_in_collision)
    elif args.prm:
        print("PRM: PRM planner is selected!")
        planner = Graph_PRM(panda, is_in_collision)
    elif args.obprm:
        print("OB_PRM: OB_PRM planner is selected!")
        planner = OBPRM(panda, is_in_collision)

    constraint = upright_constraint_end_effector
    if args.prm or args.obprm:
        plan = planner.plan_path(source_angles, goal_angles, constraint, args)
    else:
        plan = planner.plan_path(source_angles, goal_angles, constraint)

    path_quality = ret_plan_quality(plan)
    print("Quality of path: {}".format(path_quality))

    collision_boxes_publisher = CollisionBoxesPublisher('collision_boxes')
    rate = rospy.Rate(10)
    i = 0
    while not rospy.is_shutdown():
        rate.sleep()
        joints = plan[i % len(plan)]
        panda.publish_joints(joints)
        panda.publish_collision_boxes(joints)
        collision_boxes_publisher.publish_boxes(boxes)
        i += 1
