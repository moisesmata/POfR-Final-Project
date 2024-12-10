#!/usr/bin/env python3

import numpy
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
#mport moveit_commander
import rclpy
from rclpy.node import Node
import sensor_msgs.msg
import std_msgs.msg
import sys
import transforms3d._gohlketransforms as tf
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from copy import deepcopy
from urdf_parser_py.urdf import URDF
#from obstacle_generator import ObstacleGenerator
#from obstacle_generator import convert_to_message

def convert_to_pose_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.translation_from_matrix(T)
    orientation = tf.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[1]
    t.orientation.y = orientation[2]
    t.orientation.z = orientation[3]
    t.orientation.w = orientation[0]        
    return t

def convert_to_trans_message(T):
    t = geometry_msgs.msg.Transform()
    position = tf.translation_from_matrix(T)
    orientation = tf.quaternion_from_matrix(T)
    t.translation.x = position[0]
    t.translation.y = position[1]
    t.translation.z = position[2]
    t.rotation.x = orientation[1]
    t.rotation.y = orientation[2]
    t.rotation.z = orientation[3]
    t.rotation.w = orientation[0]        
    return t

def convert_from_message(msg):
    R = tf.quaternion_matrix((msg.orientation.w,
                              msg.orientation.x,
                              msg.orientation.y,
                              msg.orientation.z,
                              ))
    T = tf.translation_matrix((msg.position.x, 
                               msg.position.y, 
                               msg.position.z))
    return numpy.dot(T,R)

class MarkerControl(Node):

    def __init__(self):
        print("Marker Control initializing...")
        super().__init__('marker_control')
        #self.og = ObstacleGenerator()

        # Publisher to send commands
        self.pub_command = self.create_publisher(geometry_msgs.msg.Transform, "/motion_planning_goal", 1)        

        # Publisher to command obstacle generator
        self.obs_command = self.create_publisher(std_msgs.msg.String, "obstacles", 1)        

        #Loads the robot model, which contains the robot's kinematics information
        self.declare_parameter(
            'rd_file', rclpy.Parameter.Type.STRING)
        robot_desription = self.get_parameter('rd_file').value
        with open(robot_desription, 'r') as file:
            robot_desription_text = file.read()
        # print(robot_desription_text)
        self.robot = URDF.from_xml_string(robot_desription_text)
        self.base = self.robot.get_root()
        #self.base='base_link'

        # Create interactive marker
        self.init_marker()
        print("Initialization done.")


    def control_marker_feedback(self, feedback):
        pass

    def move_arm_cb(self, feedback):
        T = convert_from_message(feedback.pose)
        msg = convert_to_trans_message(T)
        self.pub_command.publish(msg)
        self.get_logger().info('Goal published')

    def no_obs_cb(self, feedback):
        msg = std_msgs.msg.String()
        msg.data='NONE'
        self.obs_command.publish(msg)

    def simple_obs_cb(self, feedback):
        msg = std_msgs.msg.String()
        msg.data='SIMPLE'
        self.obs_command.publish(msg)

    def complex_obs_cb(self, feedback):
        msg = std_msgs.msg.String()
        msg.data='HARD'
        self.obs_command.publish(msg)

    def super_obs_cb(self, feedback):
        msg = std_msgs.msg.String()
        msg.data='SUPER'
        self.obs_command.publish(msg)

    def init_marker(self):

        self.server = InteractiveMarkerServer(self, "control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = 'base'
        control_marker.name = "move_arm_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1.0
        move_control.orientation.z = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1.0
        move_control.orientation.x = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1.0
        move_control.orientation.z = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1.0
        move_control.orientation.y = 1.0
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        box = Marker()        
        box.type = Marker.CUBE
        box.scale.x = 0.15
        box.scale.y = 0.03
        box.scale.z = 0.03
        box.color.r = 0.5
        box.color.g = 0.5
        box.color.b = 0.5
        box.color.a = 1.0
        menu_control.markers.append(box)
        box2 = deepcopy(box)
        box2.scale.x = 0.03
        box2.scale.z = 0.1
        box2.pose.position.z=0.05
        menu_control.markers.append(box2)
        control_marker.controls.append(menu_control)

        control_marker.scale = 0.25        
        self.server.insert(marker=control_marker, feedback_callback=self.control_marker_feedback)

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Move Arm", callback=self.move_arm_cb)
        obs_entry = self.menu_handler.insert("Obstacles")
        self.menu_handler.insert("No Obstacle", callback=self.no_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Simple Obstacle", callback=self.simple_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Hard Obstacle", callback=self.complex_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Super-hard Obstacle", callback=self.super_obs_cb, parent=obs_entry)
        self.menu_handler.apply(self.server, "move_arm_marker",)

        self.server.applyChanges()

        Ttrans = tf.translation_matrix((-0.7,-0.2,0.5))
        Rtrans = tf.rotation_matrix(1.57,(0,-1,0))
        self.server.setPose("move_arm_marker", convert_to_pose_message(numpy.dot(Ttrans,Rtrans)))
        self.server.applyChanges()

def main(args=None):
    # moveit_commander.roscpp_initialize(sys.argv)
    rclpy.init(args=args)
    mc = MarkerControl()
    mc.get_logger().info("Marker control initialization done")
    rclpy.spin(mc)

    mc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


