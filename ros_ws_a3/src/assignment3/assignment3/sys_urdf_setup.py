#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d._gohlketransforms as tf
import transforms3d
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
# from interactive_markers.interactive_marker_server import *
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String


def is_same_trans(trans0, trans1):
    return (abs(np.subtract(trans0, trans1)) < 2e-2).all()

def build_message(T, msg):
    Trans, Rot, _, _ = transforms3d.affines.decompose(T)
    orientation = transforms3d.quaternions.mat2quat(Rot,)
    msg.translation.x = Trans[0]
    msg.translation.y = Trans[1]
    msg.translation.z = Trans[2]
    msg.rotation.x = orientation[1]
    msg.rotation.y = orientation[2]
    msg.rotation.z = orientation[3]
    msg.rotation.w = orientation[0]
    
    return(msg)

def is_same(matrix0, matrix1):
    matrix0 = np.array(matrix0, dtype=np.float64, copy=True)
    matrix0 /= matrix0[3, 3]
    matrix1 = np.array(matrix1, dtype=np.float64, copy=True)
    matrix1 /= matrix1[3, 3]
    return np.allclose(matrix0, matrix1, 0, 2e-2)

class MotionGrader(Node):
    #Initialization
    def __init__(self):
        super().__init__('motion_grader')

        # Publisher to set robot position
        self.pub_reset = self.create_publisher(JointState, "/joint_command", 1)

        #Publisher to issue commands
        self.mp_goal_pub = self.create_publisher(Transform, '/motion_planning_goal', 1) ##
        self.mp_goal_msg = Transform()

        self.obstacle_pub = self.create_publisher(String, "obstacles",2)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self, spin_thread=True)

        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        self.robot = URDF.from_xml_string(robot_desription_text)

        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        link = self.robot.get_root()
        self.get_logger().info("Grader initialized...")
        
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
            
        self.end_effector = link
        self.joint_state_sub = self.create_subscription(JointState, "/joint_states", self.get_joint_state, 10)

    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    def reset_robot(self):
        cmd = JointState()
        joint_vals = [1.30, 
                      -1.3707345617019364, 
                      -0.000384263032252421, 
                      -0.06531771795915448, 
                      -0.0003382902263050885, 
                      -0.2635960246366345] 
        [cmd.position.append(i) for i in joint_vals]
        self.pub_reset.publish(cmd)
        time.sleep(1.0)

    def go_to_pose(self, T, timeout, points):
        m = Transform()
        msg = build_message(T, m)

        self.mp_goal_pub.publish(msg)
        time.sleep(timeout)
        try:
            t = self.buffer.lookup_transform('base','tool0', rclpy.time.Time())
            trans = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
            rot = transforms3d.quaternions.quat2mat([t.transform.rotation.w, 
                                                     t.transform.rotation.x, 
                                                     t.transform.rotation.y, 
                                                     t.transform.rotation.z])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn("TF Exception!")
            return 0
        
        TR = transforms3d.affines.compose(trans, np.eye(3), (1,1,1))@transforms3d.affines.compose((0,0,0), rot, (1,1,1))

        if is_same(T,TR): 
            return points

        else:
            self.get_logger().error("Failed to reach goal pose in time... \n")
        return 0
