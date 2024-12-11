#!/usr/bin/env python3
import numpy
import random
import sys
import time  # Added for timing

import moveit_msgs.msg
import moveit_msgs.srv
import rclpy
from rclpy.node import Node
import rclpy.duration
import transforms3d._gohlketransforms as tf
import transforms3d
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import copy
import math
import threading
import queue
import os
from openpyxl import load_workbook
from scipy.linalg import lu_factor, lu_solve
os.environ['OMP_NUM_THREADS'] = '8'
os.environ['MKL_NUM_THREADS'] = '8'



# -------------------- Multithreaded utilities --------------------
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import Pool

def row_multiply(args):
    row, matrix2 = args
    return np.dot(row, matrix2)

def multithreaded_matrix_multiply(matrix1, matrix2):
    # Using a multiprocessing Pool to map rows
    with Pool() as pool:
        results = pool.map(row_multiply, [(row, matrix2) for row in matrix1])
    return np.array(results)

def invert_matrix(matrix):
    """Invert a single matrix."""
    return np.linalg.inv(matrix)

def multithreaded_matrix_inversion(matrices):
    """
    Perform matrix inversion on a list of matrices using multithreading.
    """
    with ThreadPoolExecutor(max_workers=os.cpu_count()*2) as executor:
        results = list(executor.map(invert_matrix, matrices))

    return results
# ----------------------------------------------------------------


def convert_to_message(T):
    t = Pose()
    position, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t

class MoveArm(Node):
    def append_to_excel(self, tree_expansion_time, validation_time):
        # Path to the Excel file
        excel_path = "/home/moises/Downloads/Results.xlsx"
        # Load the workbook and select the active sheet
        workbook = load_workbook(excel_path)
        sheet = workbook.active
        
        # Helper function to append value to a cell
        def append_to_cell(cell, new_value):
            current_value = sheet[cell].value
            if current_value is None:
                sheet[cell] = str(new_value)
            else:
                sheet[cell] = f"{current_value}, {new_value}"
        
        # Append values to appropriate cells
        append_to_cell(f"B{self.row}", tree_expansion_time)
        append_to_cell(f"C{self.row}", validation_time)

        self.row += 1 
        # Save changes to the Excel file
        workbook.save(excel_path)

    def __init__(self):
        super().__init__('move_arm')

        #Loads the robot model
        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.row = 14
        self.declare_parameter('rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)
        self.base = self.robot.get_root()
        self.get_joint_info()

        self.service_cb_group1 = MutuallyExclusiveCallbackGroup()
        self.service_cb_group2 = MutuallyExclusiveCallbackGroup()
        self.q_current = []

        # Wait for moveit IK service
        self.ik_service = self.create_client(moveit_msgs.srv.GetPositionIK, '/compute_ik', callback_group=self.service_cb_group1)
        while not self.ik_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')
        self.get_logger().info('IK service ready')

        # Wait for validity check service
        self.state_valid_service = self.create_client(moveit_msgs.srv.GetStateValidity, '/check_state_validity',
                                                      callback_group=self.service_cb_group2)
        while not self.state_valid_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for state validity service...')
        self.get_logger().info('State validity service ready')

        # MoveIt parameter
        self.group_name = 'arm'
        self.get_logger().info(f'child map: \n{self.robot.child_map}')

        #Subscribe to topics
        self.sub_joint_states = self.create_subscription(JointState, '/joint_states', self.get_joint_state, 10)
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.sub_goal = self.create_subscription(Transform, '/motion_planning_goal', self.motion_planning_cb, 2,
                                                 callback_group=self.goal_cb_group)
        self.current_obstacle = "NONE"
        self.sub_obs = self.create_subscription(String, '/obstacle', self.get_obstacle, 10)

        #Set up publisher
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(0.1, self.motion_planning_timer, callback_group=self.timer_cb_group)

    
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    def motion_planning_cb(self, ee_goal):
        self.get_logger().info("Motion planner goal received.")
        if self.ee_goal is not None:
            self.get_logger().info("Motion planner busy. Please try again later.")
            return
        self.ee_goal = ee_goal

    def motion_planning_timer(self):
        if self.ee_goal is not None:
            self.get_logger().info("Calling motion planner")            
            self.motion_planning(self.ee_goal)
            self.ee_goal = None
            self.get_logger().info("Motion planner done")            


    def motion_planning(self, ee_goal: Transform):
        # Start IK timing
        ik_start = time.time()

        T = self.transform_to_matrix(ee_goal)

        # First try numeric IK with LU solver
        q_goal = self.IK(T)
        print(q_goal)
        if q_goal is None or len(q_goal) == 0:
            self.get_logger().info("Standard numeric IK failed, trying multithreaded IK...")

        ik_end = time.time()
        ik_duration = ik_end - ik_start
        self.get_logger().info(f"IK took {ik_duration:.4f} seconds")

        if q_goal is None or len(q_goal) == 0:
            self.get_logger().error("IK FAILED")
            return

        tree = [RRTBranch(parent=None, q=self.q_current)]
        max_iterations = 100000
        found = False
        validation_time = 0

        raw_path_start = time.time()
        for i in range(max_iterations):
            q_rand = self.trajectory_sample()
            nearest_point = self.find_closest_point_in_tree(tree, numpy.array(q_rand))
            q_new = self.step(nearest_point.q, q_rand)

            validation_start = time.time()
            if self.is_segment_valid(nearest_point.q, q_new):
                validation_time += time.time() - validation_start
                new_branch = RRTBranch(parent=nearest_point, q=q_new)
                tree.append(new_branch)
            
                validation_start = time.time()
                if self.is_segment_valid(q_new, q_goal):
                    validation_time += time.time() - validation_start
                    goal_branch = RRTBranch(parent=new_branch, q=q_goal)
                    tree.append(goal_branch)
                    found = True
                    break
                else:
                    validation_time += time.time() - validation_start
            else:
                validation_time += time.time() - validation_start
                
        raw_path_end = time.time()
        tree_expansion_duration = raw_path_end - raw_path_start - validation_time
        if not found:
            self.get_logger().error("RRT FAILED")
            return

        #go backwards to get raw path
        path = []
        current = tree[-1]
        while current is not None:
            path.append(current.q)
            current = current.parent
        path.reverse()

        self.get_logger().info(f"TREE EXPANSION COMPLETE (took {tree_expansion_duration:.4f} seconds)")
        self.get_logger().info(f"VALIDATION COMPLETE (took {validation_time:.4f} seconds)")
        self.append_to_excel(tree_expansion_duration, validation_time)

        shortcut_start = time.time()
        shortcut_path = [path[0]]  
        last_valid = path[0]       
        i = 1  # Start from the second waypoint
        while i < len(path):
            if self.is_segment_valid(last_valid, path[i]):
                i += 1
            else:
                shortcut_path.append(path[i - 1])
                last_valid = path[i - 1]
                i += 1
        if shortcut_path[-1] != path[-1]:
            shortcut_path.append(path[-1])

        self.get_logger().info("PATH SHORTCUT")

        #resample traj
        final_path = []
        for i in range(len(shortcut_path) - 1):
            q_start = shortcut_path[i]
            q_end = shortcut_path[i + 1]

            segment_length = self.distance(q_start, q_end)

            num_samples = max(1, math.ceil(segment_length / 0.5))
            step_size = segment_length / num_samples
            for j in range(num_samples):
                q_sample = self.step(q_start, q_end, step_size=(j * step_size))
                final_path.append(q_sample)
        final_path.append(shortcut_path[-1])

        shortcut_end = time.time()
        shortcut_duration = shortcut_end - shortcut_start
        self.get_logger().info(f"PATH SHORTCUT & RESAMPLED (took {shortcut_duration:.4f} seconds)")

        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for q in final_path:
            point = JointTrajectoryPoint()
            point.positions = q
            trajectory.points.append(point)
        
        
        self.pub.publish(trajectory)

    # New IK method using the multithreaded matrix inversion and multiplication
    def my_multithreaded_IK(self, T_goal, max_attempts=3, timeout_seconds=20, tolerance_translation=1e-1, tolerance_rotation=1e-2, max_iterations=10000, alpha=0.05):
        for attempt in range(max_attempts):
            start_time = time.time()
            if attempt == 0:
                q_current_guess = numpy.array(self.q_current)
            else:
                q_current_guess = numpy.array(self.q_current) + numpy.random.uniform(-0.1, 0.1, self.num_joints)

            for iteration in range(max_iterations):
                T_current = self.FK(q_current_guess)
                delta_x = self.pose_difference(T_current, T_goal)

                # Check error
                translation_error = numpy.linalg.norm(delta_x[:3])
                rotation_error = numpy.linalg.norm(delta_x[3:])
                if translation_error < tolerance_translation and rotation_error < tolerance_rotation:
                    return q_current_guess.tolist()

                J = self.jacobian(q_current_guess)
                JT = J.T
                A = JT.dot(J)
                b = JT.dot(delta_x)

                # We need to invert A and multiply by b using the multithreaded functions
                # A is (n x n), b is (n,)
                A_list = [A]  # multithreaded inversion expects a list
                inverted = multithreaded_matrix_inversion(A_list)
                A_inv = inverted[0]

                # Perform A_inv * b using the multithreaded multiplication
                # b is a vector, so reshape it as (n,1) for matrix multiplication
                b_col = b.reshape(-1, 1)
                q_dot_matrix = multithreaded_matrix_multiply(A_inv, b_col)
                q_dot = q_dot_matrix.flatten()  # convert back to 1D

                q_current_guess = q_current_guess + alpha * q_dot

                if time.time() - start_time > timeout_seconds:
                    break
        return None

    #helper function
    def transform_to_matrix(self, transform):
        translation = transform.translation
        rotation = transform.rotation
        trans = [translation.x, translation.y, translation.z]
        rot = transforms3d.quaternions.quat2mat([
            rotation.w,
            rotation.x,
            rotation.y,
            rotation.z
        ])[:3, :3]
        transform_matrix = numpy.eye(4)
        transform_matrix[:3, :3] = rot
        transform_matrix[:3, 3] = trans
        return transform_matrix

    def step(self, q_near, q_rand, step_size=0.1):
        direction = [qr - qn for qr, qn in zip(q_rand, q_near)]
        length = math.sqrt(sum([d**2 for d in direction]))
        if length == 0:
            return q_near
        step = [qn + (d / length) * step_size for qn, d in zip(q_near, direction)]
        #joint limits
        return [max(min(q, math.pi), -math.pi) for q in step]

    def distance(self, q1, q2):
        return math.sqrt(sum((qi - qj) ** 2 for qi, qj in zip(q1, q2)))

    def trajectory_sample(self):
        q_rand = [random.uniform(-math.pi, math.pi) for _ in range(self.num_joints)]
        return q_rand

    def find_closest_point_in_tree(self, tree, r):
        shortest_distance = numpy.linalg.norm(r - tree[0].q)
        closest_point = tree[0]
        for i in range(1, len(tree)):
            dist = numpy.linalg.norm(r - tree[i].q)
            if dist < shortest_distance:
                shortest_distance = dist
                closest_point = tree[i]
        return closest_point

    # Original IK using MoveIt
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.ik_request.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = 'base'
        req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()

        self.get_logger().info('Sending IK request...')
        res = self.ik_service.call(req)
        self.get_logger().info('IK request returned')

        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = list(res.solution.joint_state.position)
        for i in range(len(q)):
            while q[i] < -math.pi:
                q[i] += 2 * math.pi
            while q[i] > math.pi:
                q[i] -= 2 * math.pi
        return q

    def my_numeric_IK(self, T_goal, max_attempts=3, timeout_seconds=20, tolerance_translation=1e-1, tolerance_rotation=1e-2, max_iterations=10000, alpha=0.05):
        for attempt in range(max_attempts):
            start_time = time.time()
            if attempt == 0:
                q_current_guess = numpy.array(self.q_current)
            else:
                q_current_guess = numpy.array(self.q_current) + numpy.random.uniform(-0.1, 0.1, self.num_joints)

            for iteration in range(max_iterations):
                T_current = self.FK(q_current_guess)
                delta_x = self.pose_difference(T_current, T_goal)

                # Check error
                translation_error = numpy.linalg.norm(delta_x[:3])
                rotation_error = numpy.linalg.norm(delta_x[3:])
                if translation_error < tolerance_translation and rotation_error < tolerance_rotation:
                    return q_current_guess.tolist()

                J = self.jacobian(q_current_guess)
                JT = J.T
                A = JT.dot(J)
                b = JT.dot(delta_x)

                # Solve linear system A q_dot = b using LU
                q_dot = self.solve_linear_system_lu(A, b)

                q_current_guess = q_current_guess + alpha * q_dot

                if time.time() - start_time > timeout_seconds:
                    break
        return None

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions.'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.translation_matrix(joint.origin.xyz), tf.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2], 'rxyz'))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T

    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
    
        for i in range(self.num_joints):
            z_axis = joint_transforms[i][:3, :3] @ numpy.array(self.joint_axes[i])
            position_diff = b_T_ee[:3, 3] - joint_transforms[i][:3, 3]
            J[:3, i] = numpy.cross(z_axis, position_diff)
            J[3:, i] = z_axis
        return J
    
    def FK(self, q):
        _, b_T_ee = self.forward_kinematics(q)
        return b_T_ee

    def jacobian(self, q):
        joint_transforms, b_T_ee = self.forward_kinematics(q)
        J = self.get_jacobian(b_T_ee, joint_transforms)
        return J

    def pose_difference(self, T_current, T_goal):
        # Position error
        p_current = T_current[:3, 3]
        p_goal = T_goal[:3, 3]
        dp = p_goal - p_current
        
        # Orientation error
        R_current = T_current[:3, :3]
        R_goal = T_goal[:3, :3]
        
        R_error = R_goal @ R_current.T
        angle, axis = self.rotation_from_matrix(R_error)
        dr = angle * axis
        
        return numpy.concatenate((dp, dr))
    
    def solve_linear_system_lu(self, A, b):
        A = numpy.asarray(A, dtype=float)
        b = numpy.asarray(b, dtype=float)
        lu, piv = lu_factor(A)
        x = lu_solve((lu, piv), b)
        return x

    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: 
                break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints += 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
        self.get_logger().info('Num joints: %d' % (self.num_joints))

    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidity.Request()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = list(q)
        req.robot_state.joint_state.velocity = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.effort = list(numpy.zeros(self.num_joints))
        req.robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()

        res = self.state_valid_service.call(req)
        return res.valid

    def validation_worker(self, start_index, end_index, q_start, q_end, num_segs, stop_event):
        for i in range(start_index, end_index):
            if stop_event.is_set():
                return
            t = i / num_segs
            q_seg = [qs + t*(qe - qs) for qs, qe in zip(q_start, q_end)]
            if not self.is_state_valid(q_seg):
                stop_event.set()
                return

    def is_segment_valid(self, q_start, q_end):
        if self.current_obstacle == 'NONE':
            step_size = 0.05
        elif self.current_obstacle == 'SIMPLE':
            step_size = 0.02
        elif self.current_obstacle == 'HARD':
            step_size = 0.01
        elif self.current_obstacle == 'SUPER':
            step_size = 0.005
        else:
            step_size = 0.05

        direction = [qe - qs for qs, qe in zip(q_start, q_end)]
        length = math.sqrt(sum(d**2 for d in direction))
        if length == 0:
            return self.is_state_valid(q_start)

        num_segs = max(1, int(length / step_size))

        num_threads = min(num_segs, 8)
        segment_per_thread = math.ceil(num_segs / num_threads)
        
        threads = []
        stop_event = threading.Event()

        for t_id in range(num_threads):
            start_index = t_id * segment_per_thread
            end_index = min((t_id + 1)*segment_per_thread, num_segs + 1)
            thread = threading.Thread(target=self.validation_worker, 
                                      args=(start_index, end_index, q_start, q_end, num_segs, stop_event))
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        return not stop_event.is_set()


class RRTBranch(object):
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q


def main(args=None):
    rclpy.init(args=args)
    ma = MoveArm()
    ma.get_logger().info("Move arm initialization done")
    executor = MultiThreadedExecutor()
    executor.add_node(ma)
    executor.spin()
    ma.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
