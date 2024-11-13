#!/usr/bin/env python3
import numpy
import random
import sys

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

    
    def get_joint_state(self, msg):
        '''This callback provides you with the current joint positions of the robot 
        in member variable q_current.
        '''
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    
    def get_obstacle(self, msg):
        '''This callback provides you with the name of the current obstacle which
        exists in the RVIZ environment. Options are "None", "Simple", "Hard",
        or "Super". '''
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
        '''Callback function for /motion_planning_goal. This is where you will
        implement your RRT motion planning which is to generate a joint
        trajectory for your manipulator. You are welcome to add other functions
        to this class (i.e. an is_segment_valid" function will likely come in 
        handy multiple times in the motion planning process and it will be 
        easiest to make this a seperate function and then call it from motion
        planning). You may also create trajectory shortcut and trajectory 
        sample functions if you wish, which will also be called from the 
        motion planning function.

        Args: 
            ee_goal: Transform() object describing the desired base to 
            end-effector transformation 
        '''

        T = self.transform_to_matrix(ee_goal)
        q_goal = self.IK(T)
        if not q_goal:
            #end if IK not possible
            self.get_logger().error("IK FAILED")
            return

        tree = [RRTBranch(parent=None, q=self.q_current)]
        max_iterations = 100000
        found = False

        for i in range(max_iterations):
            
            q_rand = self.trajectory_sample()
            nearest_point = self.find_closest_point_in_tree(tree, numpy.array(q_rand))

            #step towards the random configuration
            q_new = self.step(nearest_point.q, q_rand)

            if self.is_segment_valid(nearest_point.q, q_new):
                #add to tree and check if possible to get to goal
                new_branch = RRTBranch(parent=nearest_point, q=q_new)
                tree.append(new_branch)
            
                if self.is_segment_valid(q_new, q_goal):
                    goal_branch = RRTBranch(parent=new_branch, q=q_goal)
                    tree.append(goal_branch)
                    found = True
                    break

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
        self.get_logger().info("RAW PATH ACQUIRED")


        #shortcut the path
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

            num_samples = math.ceil(segment_length / 0.5) # this should be the solution to arm randomly stopping
            stepsize =  segment_length/num_samples
            for j in range(num_samples):
                q_sample = self.step(q_start, q_end, step_size=(j*stepsize))
                final_path.append(q_sample)
        final_path.append(shortcut_path[-1])
        self.get_logger().info("PATH RESAMPLED")
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for q in final_path:
            point = JointTrajectoryPoint()
            point.positions = q
            trajectory.points.append(point)
        
        self.pub.publish(trajectory)

    #helper function
    def transform_to_matrix(self, transform):
        translation = transform.translation
        rotation = transform.rotation

        #get translation
        trans = [translation.x, translation.y, translation.z]

        #get rotation
        rot = transforms3d.quaternions.quat2mat([
            rotation.w,
            rotation.x,
            rotation.y,
            rotation.z
            
        ])[:3, :3]

        #get 4x4 transform mat
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
        q_rand = []
        for i in range(self.num_joints):
            q_rand.append(random.uniform(-math.pi, math.pi))
        return q_rand
        

    ### END HELPERS

    def find_closest_point_in_tree(self, tree, r):
        shortest_distance = numpy.linalg.norm(r-tree[0].q)
        closest_point = tree[0]
        for i in range(1, len(tree)-1):
            if shortest_distance > numpy.linalg.norm(r-tree[i].q):
                shortest_distance = numpy.linalg.norm(r-tree[i].q)
                closest_point = tree[i]
        return closest_point

    
    def IK(self, T_goal):
        """ This function will perform IK for a given transform T of the 
        end-effector. It .

        Returns:
            q: returns a list q[] of values, which are the result 
            positions for the joints of the robot arm, ordered from proximal 
            to distal. If no IK solution is found, it returns an empy list
        """

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
            q = res.solution.joint_state.position
        for i in range(0,len(q)):
            while (q[i] < -math.pi): q[i] = q[i] + 2 * math.pi
            while (q[i] > math.pi): q[i] = q[i] - 2 * math.pi
        return q

    
    def get_joint_info(self):
        '''This is a function which will collect information about the robot which
        has been loaded from the parameter server. It will populate the variables
        self.num_joints (the number of joints), self.joint_names and
        self.joint_axes (the axes around which the joints rotate)
        '''
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
        self.get_logger().info('Num joints: %d' % (self.num_joints))


    
    def is_state_valid(self, q):
        """ This function checks if a set of joint angles q[] creates a valid state,
        or one that is free of collisions. The values in q[] are assumed to be values
        for the joints of the UR5 arm, ordered from proximal to distal.

        Returns:
            bool: true if state is valid, false otherwise
        """
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



class RRTBranch(object):
    '''This is a class which you can use to keep track of your tree branches.
    It is easiest to do this by appending instances of this class to a list 
    (your 'tree'). The class has a parent field and a joint position field (q). 
    
    You can initialize a new branch like this:
        RRTBranch(parent, q)
    Feel free to keep track of your branches in whatever way you want - this
    is just one of many options available to you.
    '''
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
        

