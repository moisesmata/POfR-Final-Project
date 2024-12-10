#include "robot_sim/joint_state_publisher.h" 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace robot_sim {

bool JointStatePublisher::init(double rate)
{
  if (!robot_) return false;

  state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  state_timer_ = this->create_wall_timer(std::chrono::duration<double>(rate), std::bind(&JointStatePublisher::callback, this));
  return true;
}

void JointStatePublisher::callback()
{
  assert(robot_);
  sensor_msgs::msg::JointState msg;
  msg.position = robot_->getJointValues();
  msg.name = robot_->getJointNames();
  msg.velocity.resize(msg.position.size(), 0.0);
  msg.effort.resize(msg.position.size(), 0.0);
  if (msg.position.size() != msg.name.size()) {
    RCLCPP_ERROR(this->get_logger(), "Mismatch between sim robot and joint state publisher");
    return;
  }
  msg.header.stamp = this->now();
  state_publisher_->publish(msg);
}

}  // namespace robot_sim

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<JointStatePublisher>());
//   rclcpp::shutdown();
//   return 0;
// }