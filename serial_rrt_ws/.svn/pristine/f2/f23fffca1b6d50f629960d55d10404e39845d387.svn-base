#pragma once

//#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <memory>

// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
// #include <trajectory_msgs/JointTrajectory.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "robot_sim/robot.h"

namespace robot_sim {

class TrajectoryExecuter : public rclcpp::Node
{
 public:
 TrajectoryExecuter(std::shared_ptr<Robot> robot) : Node("trajectory_executor"), robot_(robot) {}

  bool init();

 private:
//   ros::NodeHandle root_nh_;
  std::shared_ptr<Robot> robot_;
//   ros::Subscriber trajectory_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  mutable boost::mutex mutex_;
//   trajectory_msgs::JointTrajectory current_trajectory_;
  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  size_t current_point_;
  bool new_trajectory_;
  bool executing_;
  //! Max velocity in rad/s
  double max_velocity_;
  std::vector<double> current_vels_;
//   ros::Timer timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  void callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void timerCallback();

  void startNewSegment();
  bool targetReached();
  void stopRobot();
};

} //namespace robot_sim
