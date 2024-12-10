#include "robot_sim/position_controller.h"
using std::placeholders::_1;

namespace robot_sim {

bool PositionController::init()
{
  //baxter_command_sub_ = root_nh_.subscribe("/robot/limb/left/joint_command", 
  //					   10, &PositionController::baxter_callback, this);
  generic_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_positions", 
					    10, std::bind(&PositionController::generic_callback, this, _1));
  return true;
}
  /*
void PositionController::baxter_callback(const baxter_core_msgs::JointCommand::ConstPtr &msg)
{
  robot_->setTargetValues(msg->names, msg->command);
}
  */
void PositionController::generic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() != robot_->getNumJoints())
  {
    RCLCPP_ERROR(this->get_logger(), ("Received joint positions (" + std::to_string(msg->position.size()) +
     ") do not match robot(" + std::to_string(robot_->getNumJoints()) +
     ")").c_str());
    return;
  }
  robot_->setTargetValues(msg->position);
}

bool PositionCommand::init()
{
  generic_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_command", 
					    10, std::bind(&PositionCommand::generic_callback, this, _1));
  return true;
}

void PositionCommand::generic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.size() == 0)
  {
    if (msg->position.size() != robot_->getNumJoints())
    {
      RCLCPP_ERROR(this->get_logger(), ("Received joint positions (" + std::to_string(msg->position.size()) +
     ") do not match robot(" + std::to_string(robot_->getNumJoints()) +
     ")").c_str());
      return;
    }
    robot_->setJointValues(msg->position);
  } else {
    robot_->setJointValues(msg->name, msg->position);
  }
}

}
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
