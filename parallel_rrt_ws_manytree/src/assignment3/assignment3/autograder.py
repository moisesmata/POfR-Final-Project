#!/usr/bin/env python3
import sys
sys.path.append(".")

import numpy
import rclpy
import transforms3d._gohlketransforms as tf
from .sys_urdf_setup import MotionGrader
import time
from std_msgs.msg import String


def get_ur5_poses():
    '''The goal poses were generated manually using the interactive marker This
    function returns the 4 goal poses for this assignment.
    
    Returns:
        4 pairs of goal poses described by xyz translation and qxyz quaternions
    '''
   
    # no obstacle
    trans1 = (-0.699999988079071, -0.20000000298023224, 0.5)
    rot1 = (0.7073882818222046, 0.0, -0.7068251967430115, 0.0)

    # simple obstacle
    trans2 = (-0.48508575558662415, -0.6518027782440186, 0.35038912296295166)
    rot2 = (0.6323947310447693,0.316977858543396, -0.6318913698196411, 0.31672564148902893)

    # hard obstacle
    trans3 = (-0.39064934849739075, -0.6117185354232788, 0.4997536242008209)
    rot3 = (0.7073882818222046, 0.0, -0.7068251967430115, 0.0) 

    # very hard obstacle
    trans4 = (-0.5361842513084412, -0.055403269827365875, 0.09116514772176743)
    rot4 = (-0.55396968126297, -0.5500016212463379, 0.4444117844104767, 0.4394475221633911)

    return ((trans1, rot1), (trans2, rot2), (trans3, rot3), (trans4, rot4))



def perform_mp_poses(cartesian_grader):
    cc_grade = 0

    goal_poses = get_ur5_poses()
    obstacle_list = ["NONE", "SIMPLE", "HARD", "SUPER"]
    test_message_list = ["no obstacles", "simple obstacle", "hard obstacle", "bonus, super hard obstacle"]
    points_list = [3, 3, 4, 1]
    timeout_list = [30, 60, 80, 160]

    
    for idx, pose in enumerate(goal_poses):
        cartesian_grader.reset_robot()
        time.sleep(1.5)
        cartesian_grader.get_logger().warn(f'Attempting {test_message_list[idx]} test...')

        trans = tf.translation_matrix(pose[0])
        rot = tf.quaternion_matrix(pose[1])
        Tr = trans@rot

        msg = String()
        msg.data= obstacle_list[idx]
        cartesian_grader.obstacle_pub.publish(msg)
        time.sleep(2.0) 

        test_grade = cartesian_grader.go_to_pose(Tr, timeout_list[idx], points_list[idx])	

        cartesian_grader.get_logger().warn(f'Received {test_grade} points out of {points_list[idx]} on {test_message_list[idx]} test')

        cc_grade += test_grade
        time.sleep(2.0)	

    cartesian_grader.get_logger().warn("FINISHED ALL THE TESTS")


    return cc_grade


def main(args=None):
    time.sleep(2.0)
    rclpy.init(args=args)
    cg = MotionGrader()
    time.sleep(2.0)

    cc_grade = perform_mp_poses(cg)
    cg.get_logger().warn(f"Grade: {cc_grade}")
    cg.listener.dedicated_listener_thread.join()
    cg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
