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

class RRTBranch(object):
    def __init__(self, parent, q):
        self.parent = parent
        self.q = q

class MoveArm(Node):
    def __init__(self):
        super().__init__('move_arm')

        #Loads the robot model, which contains the robot's kinematics information
        self.ee_goal = None
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
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

        # Lock for threading on the shared tree
        self.tree_lock = threading.Lock()


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
        q_goal = self.IK(T)

        ik_end = time.time()  # End IK timing
        ik_duration = ik_end - ik_start
        self.get_logger().info(f"IK took {ik_duration:.4f} seconds")

        if not q_goal:
            #end if IK not possible
            self.get_logger().error("IK FAILED")
            return

        q_current = self.q_current
        max_iterations = 100000
        tree = [RRTBranch(parent=None, q=q_current)]

        # We'll use multiple threads per iteration to attempt stepping
        m = 10  # number of threads per iteration
        raw_path_start = time.time()

        for i in range(max_iterations):
            q_queue = queue.Queue()
            threads = []

            # Launch m threads, each tries one random step
            for _ in range(m):
                t = threading.Thread(target=self.worker, args=(tree, q_queue))
                t.start()
                threads.append(t)

            # Wait for all threads to finish
            for t in threads:
                t.join()

            # Check if any thread returned a result
            if not q_queue.empty():
                nearest_point, q_new = q_queue.get()

                # We have a candidate q_new
                # Check if we can add it to the tree and reach the goal
                # Lock the tree while modifying
                with self.tree_lock:
                    validation_time = 0.0
                    if self.is_segment_valid(nearest_point.q, q_new):
                        new_branch = RRTBranch(parent=nearest_point, q=q_new)
                        tree.append(new_branch)

                        if self.is_segment_valid(q_new, q_goal):
                            goal_branch = RRTBranch(parent=new_branch, q=q_goal)
                            tree.append(goal_branch)
                            # Found a path
                            amount_to_sleep = numpy.random.rand(1)[0] * 1
                            time.sleep(amount_to_sleep)
                            self.get_logger().info(f"Path found! Sleeping for {amount_to_sleep:.3f} seconds.")
                            # Reconstruct path
                            path = []
                            current = goal_branch
                            while current is not None:
                                path.append(current.q)
                                current = current.parent
                            path.reverse()

                            raw_path_end = time.time()
                            tree_expansion_duration = raw_path_end - raw_path_start - validation_time
                            self.get_logger().info(f"TREE EXPANSION COMPLETE (took {tree_expansion_duration:.4f} seconds)")
                            self.get_logger().info(f"VALIDATION COMPLETE (took {validation_time:.4f} seconds)")

                            # Shortcut and resample
                            shortcut_start = time.time()
                            shortcut_path = self.shortcut_path(path)
                            final_path = self.resample_path(shortcut_path)
                            shortcut_end = time.time()
                            shortcut_duration = shortcut_end - shortcut_start
                            self.get_logger().info(f"PATH SHORTCUT & RESAMPLED (took {shortcut_duration:.4f} seconds)")

                            # publish trajectory
                            trajectory = JointTrajectory()
                            trajectory.joint_names = self.joint_names
                            for q_ in final_path:
                                point = JointTrajectoryPoint()
                                point.positions = q_
                                trajectory.points.append(point)
                            self.pub.publish(trajectory)
                            return  # done

        # If we exit the loop, no path found
        self.get_logger().error("RRT FAILED")


    def worker(self, tree, q_queue):
        # This function tries one step
        # It's run by each thread.
        result = self.parallel_stepping(tree)
        if result:
            # Only one thread needs to succeed
            q_queue.put(result)

    def parallel_stepping(self, tree):
        # Sample a random configuration
        q_rand = self.trajectory_sample()

        # Lock the tree for read access to find nearest
        with self.tree_lock:
            nearest_point = self.find_closest_point_in_tree(tree, numpy.array(q_rand))
            q_new = self.step(nearest_point.q, q_rand)

        # After computing q_new, let's check validity of that step
        # We don't modify the tree yet, just return a candidate
        if self.is_segment_valid(nearest_point.q, q_new):
            # valid step
            return (nearest_point, q_new)
        return None


    def shortcut_path(self, path):
        self.get_logger().info("PATH SHORTCUT")
        shortcut_path = [path[0]]  
        last_valid = path[0]
        i = 1
        while i < len(path):
            if self.is_segment_valid(last_valid, path[i]):
                i += 1
            else:
                shortcut_path.append(path[i - 1])
                last_valid = path[i - 1]
                i += 1
        if shortcut_path[-1] != path[-1]:
            shortcut_path.append(path[-1])
        return shortcut_path

    def resample_path(self, shortcut_path):
        final_path = []
        for i in range(len(shortcut_path) - 1):
            q_start = shortcut_path[i]
            q_end = shortcut_path[i + 1]

            segment_length = self.distance(q_start, q_end)

            num_samples = math.ceil(segment_length / 0.5)
            stepsize = segment_length / num_samples
            for j in range(num_samples):
                q_sample = self.step(q_start, q_end, step_size=(j*stepsize))
                final_path.append(q_sample)
        final_path.append(shortcut_path[-1])
        return final_path

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
            # default if not specified
            step_size = 0.05

        direction = [qr - qn for qr, qn in zip(q_end, q_start)]
        length = math.sqrt(sum([d**2 for d in direction]))
        num_segs = int(length / step_size)
        for i in range(num_segs + 1):
            q_seg = self.step(q_start, q_end, step_size=i * step_size)
            if not self.is_state_valid(q_seg):
                return False
        return True

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
                q[i] = q[i] + 2*math.pi
            while q[i] > math.pi:
                q[i] = q[i] - 2*math.pi
        return q

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
