import argparse
import numpy as np
import rospy

from franka_robot import PandaRobot
from bounding_box_ros_publisher import BoundingBoxesPublisher
from rrt import RRT
from rrt_connect import RRTConnect
from prm import PRM
from ob_prm import OBPRM


def string2boolean(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

'''
Select a suitable planning algorithm and an environment using parsers. It is as shown below.
'''
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', '-s', type=int, default=0)
    parser.add_argument('--rrt', '-rrt', type=string2boolean, const=True, nargs='?', default=False, help="Use RRT?")
    parser.add_argument('--rrtc', '-rrtc', type=string2boolean, const=True, nargs='?', default=False, help="Use RRT-Connect?")
    parser.add_argument('--prm', '-prm', type=string2boolean, const=True, nargs='?', default=False, help="Use PRM?")
    parser.add_argument('--obprm', '-obprm', type=string2boolean, const=True, nargs='?', default=False, help="Use OBPRM?")
    parser.add_argument('--map2', '-map2', type=string2boolean, const=True, nargs='?', default=False, help="Use map 2?")
    parser.add_argument('--map3', '-map3', type=string2boolean, const=True, nargs='?', default=False, help="Use map 3?")
    parser.add_argument('--reuse_graph', '-reuse_graph', type=string2boolean, const=True, nargs='?', default=False, help="Reuse the graph for PRM?")
    args = parser.parse_args()

    np.random.seed(args.seed)
    pr = PandaRobot()

    rospy.init_node('planner')

    '''
    TODO: Replace obstacle box w/ the box specs in our workspace:
    [x, y, z, r, p, y, sx, sy, sz]
    '''
    if args.map3:
        obstacle_boxes = np.array([
            # obstacle
            # [0, 0, 0, 0, 0, 0, 0, 0, 0],
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
        obstacle_boxes = np.array([
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
        obstacle_boxes = np.array([
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


    def is_colliding(joints):
        if pr.is_self_collision(joints):
            return True
        for box in obstacle_boxes:
            if pr.is_bounding_box_colliding(joints, box):
                return True
        return False


    desired_end_effector_rp = pr.end_effector(pr.home_joints)[3:5]


    def upright_const_end_effector(q):
        '''
        We implement constraint function and its gradient.
        '''
        end_effector = pr.end_effector(q)
        error_q = np.sum((np.asarray(desired_end_effector_rp) - np.asarray(end_effector[3:5])) ** 2)
        grad = np.asarray([0, 0, 0, 2 * (end_effector[3] - desired_end_effector_rp[0]), 2 * (end_effector[4] - desired_end_effector_rp[1]), 0])
        return error_q, grad

    def get_cost_of_plan(plan):
        distance_path = 0
        for d_iter in range(len(plan) - 1):
            distance_path += np.linalg.norm(np.array(plan[d_iter+1]) - np.array(plan[d_iter]))
        return distance_path

    if args.map3:
        q_start_angle = np.array([0, 3*np.pi/8, 0, -np.pi / 8, 0, np.pi / 2, np.pi / 4])
        q_start_angle[0] = -np.deg2rad(45)
        q_target_angle = np.array([0, 0, 0, -np.pi / 4, 0, np.pi / 4, np.pi / 4])
        q_target_angle[0] = -np.deg2rad(45)
    elif args.map2:
        q_start_angle = np.array([0, np.pi/6, 0, -2*np.pi / 3, 0, 5*np.pi / 6, np.pi / 4])
        q_target_angle = np.array([0, 0, 0, -np.pi / 4, 0, np.pi / 4, np.pi / 4])
    else:
        q_start_angle = pr.home_joints.copy()
        q_start_angle[0] = -np.deg2rad(45)
        q_target_angle = q_start_angle.copy()
        q_target_angle[0] = np.deg2rad(45)

    if args.rrt:
        print("RRT planner has been selected!")
        planner = RRT(pr, is_colliding)
    elif args.rrtc:
        print("RRT Connect planner has been selected!")
        planner = RRTConnect(pr, is_colliding)
    elif args.prm:
        print("PRM planner has been selected!")
        planner = PRM(pr, is_colliding)
    elif args.obprm:
        print("OB_PRM planner has been selected!")
        planner = OBPRM(pr, is_colliding)

    constr = upright_const_end_effector
    if args.prm or args.obprm:
        plan = planner.plan(q_start_angle, q_target_angle, constr, args)
    else:
        plan = planner.plan(q_start_angle, q_target_angle, constr)

    path_cost = get_cost_of_plan(plan)
    print("Path cost: {}".format(path_cost))

    collision_boxes_publisher = BoundingBoxesPublisher('collision_boxes')
    rt = rospy.Rate(10)
    d_iter = 0
    while not rospy.is_shutdown():
        rt.sleep()
        joints = plan[d_iter % len(plan)]
        pr.publish_joints(joints)
        pr.publish_collision_boxes(joints)
        collision_boxes_publisher.box_publisher(obstacle_boxes)
        d_iter += 1
