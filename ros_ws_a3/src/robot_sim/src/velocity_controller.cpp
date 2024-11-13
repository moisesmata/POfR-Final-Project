#include "robot_sim/velocity_controller.h"

using std::placeholders::_1;

namespace robot_sim {

bool VelocityController::init()
{
  velocity_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_velocities", 10, std::bind(&VelocityController::callback, this, _1));
  return true;
}

void VelocityController::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() == 0)
  {
    if (msg->velocity.size() != robot_->getNumJoints())
    {
      RCLCPP_ERROR(this->get_logger(), ("Received joint positions (" + std::to_string(msg->position.size()) +
     ") do not match robot(" + std::to_string(robot_->getNumJoints()) +
     ")").c_str());

      return;
    }
    robot_->setVelocities(msg->velocity);
  } else {
    robot_->setVelocities(msg->name, msg->velocity);
  }
}

}
