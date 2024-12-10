#pragma once

//#include <boost/shared_ptr.hpp>
#include <vector>
#include <memory>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"

#include "robot_sim/robot.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace robot_sim {

class JointStatePublisher : public rclcpp::Node
{
 public:
//  JointStatePublisher(std::shared_ptr<Robot> robot) :
//   root_nh_(""), robot_(robot)
//   {}
  JointStatePublisher(std::shared_ptr<Robot> robot) : Node("joint_state_publisher_"), robot_(robot) {}

  bool init(double rate);

 private:
  // ros::NodeHandle root_nh_;
  std::shared_ptr<Robot> robot_;
  // ros::Publisher state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  // ros::Timer state_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  
//   void callback(const ros::TimerEvent&);
  void callback();
};

} //namespace robot_sim
