#pragma once

//#include <boost/shared_ptr.hpp>

#include <memory>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
// #include <sensor_msgs/JointState.h>
#include "sensor_msgs/msg/joint_state.hpp"

#include "robot_sim/robot.h"

namespace robot_sim {

class VelocityController : public rclcpp::Node
{
 public:
 VelocityController(std::shared_ptr<Robot> robot)
  : Node("velocity_controller"), robot_(robot) {}

  bool init();

 private:
//   ros::NodeHandle root_nh_;
  std::shared_ptr<Robot> robot_;
//   ros::Subscriber velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velocity_sub_;
 
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

} //namespace robot_sim
