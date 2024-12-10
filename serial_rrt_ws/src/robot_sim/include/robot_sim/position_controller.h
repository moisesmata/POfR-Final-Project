#pragma once

//#include <boost/shared_ptr.hpp>
#include <memory>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
//#include <baxter_core_msgs/JointCommand.h>
// #include "baxter_core_msgs/msg/joint_command.hpp"
// #include <sensor_msgs/JointState.h>
#include "sensor_msgs/msg/joint_state.hpp"

#include "robot_sim/robot.h"

namespace robot_sim {

class PositionController : public rclcpp::Node
{
 public:
 PositionController(std::shared_ptr<Robot> robot) : Node("position_controller"), robot_(robot) {}

  bool init();

 private:
//   ros::NodeHandle root_nh_;
  std::shared_ptr<Robot> robot_;

  //  ros::Subscriber baxter_command_sub_; 
  //  void baxter_callback(const baxter_core_msgs::JointCommand::ConstPtr&);

//   ros::Subscriber generic_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr generic_command_sub_;
  void generic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

class PositionCommand : public rclcpp::Node
{
 public:
 PositionCommand(std::shared_ptr<Robot> robot) : Node("position_command"), robot_(robot) {}

  bool init();

 private:
//   ros::NodeHandle root_nh_;
  std::shared_ptr<Robot> robot_;

//   ros::Subscriber generic_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr generic_command_sub_;
  void generic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} //namespace robot_sim
