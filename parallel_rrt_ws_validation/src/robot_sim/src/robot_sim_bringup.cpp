//#include <boost/shared_ptr.hpp>

#include <map>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>

#include "robot_sim/joint_state_publisher.h"
#include "robot_sim/robot.h"
#include "robot_sim/velocity_controller.h"
#include "robot_sim/position_controller.h"
#include "robot_sim/trajectory_executer.h"

void setTargetValues(std::shared_ptr<robot_sim::Robot> robot, int num_joints)
{
  static bool zeros = false;
  std::vector<double> vals;
  if (zeros) vals.resize(num_joints, 0.0);
  else vals.resize(num_joints, 1.0);
  robot->setTargetValues(vals);
  zeros = !zeros;
}

void setVelocities(std::shared_ptr<robot_sim::Robot> robot, int num_joints)
{
  static int count = 0;
  std::vector<double> vals;
  if (count == 0) vals.resize(num_joints, 1.0);
  else if (count == 1) vals.resize(num_joints, 0.0);
  else if (count == 2) vals.resize(num_joints, -1.0);
  else if (count == 3) vals.resize(num_joints, 0.0);
  robot->setVelocities(vals);
  if (++count > 3) count = 0;
}

int main(int argc, char **argv)
{
//   ros::init(argc, argv, "robot_sim");
  rclcpp::init(argc, argv);

//   ros::NodeHandle priv_nh("~");
  auto node_ = std::make_shared<rclcpp::Node>("robot_sim");
  auto private_node = std::make_shared<rclcpp::Node>("robot_sim_private");

  urdf::Model model;
  std::map<std::string, size_t> name_map;
  size_t i=0;
  private_node->declare_parameter("rd_file", rclcpp::PARAMETER_STRING);
  rclcpp::Parameter rd_file = private_node->get_parameter("rd_file");
  if (!model.initFile(rd_file.value_to_string().c_str()))
  {
    RCLCPP_ERROR(private_node->get_logger(), "Robot sim bringup: failed to read urdf from parameter server");
    return 0;
  }
  for (std::map<std::string, std::shared_ptr<urdf::Joint> >::iterator it = model.joints_.begin();
       it != model.joints_.end(); it++)
  {
    if (it->second->type != urdf::Joint::FIXED)
    {
      name_map[it->first]=i;
      i++;
    }
  }
  RCLCPP_INFO(private_node->get_logger(), ("Robot model read with " + std::to_string(name_map.size()) + " non-fixed joints.").c_str());

  size_t num_joints = name_map.size();

  std::shared_ptr<robot_sim::Robot> robot(new robot_sim::Robot(name_map));
  robot->init();
  std::shared_ptr<robot_sim::JointStatePublisher> publisher(new robot_sim::JointStatePublisher(robot));
  // double rate;
//   priv_nh.param<double>("joint_pub_rate", rate, 0.01);
  private_node->declare_parameter("joint_pub_rate", 0.04);
  rclcpp::Parameter rate = private_node->get_parameter("joint_pub_rate");

  if (!publisher->init(rate.as_double()))
  {
    RCLCPP_ERROR(private_node->get_logger(), "Failed to initialize publisher");
    return 0;
  }
  std::shared_ptr<robot_sim::VelocityController> vel_controller(new robot_sim::VelocityController(robot));
  if (!vel_controller->init())
  {
    RCLCPP_ERROR(private_node->get_logger(),"Failed to initialize velocity controller");
    return 0;
  }
  std::shared_ptr<robot_sim::PositionController> pos_controller(new robot_sim::PositionController(robot));
  if (!pos_controller->init())
  {
    RCLCPP_ERROR(private_node->get_logger(),"Failed to initialize position controller");
    return 0;
  }
  std::shared_ptr<robot_sim::PositionCommand> pos_command(new robot_sim::PositionCommand(robot));
  if (!pos_command->init())
  {
    RCLCPP_ERROR(private_node->get_logger(),"Failed to initialize position command");
    return 0;
  }
  std::shared_ptr<robot_sim::TrajectoryExecuter> traj_executer(new robot_sim::TrajectoryExecuter(robot));
  if (!traj_executer->init())
  {
    RCLCPP_ERROR(private_node->get_logger(),"Failed to initialize trajectory executer");
    return 0;
  }

  // This is a horrible hack, but I could not figure out how to have array parameters in ROS2
  // One should really pass the starting joint values as an array parameter
  private_node->declare_parameter("franka",false);
  rclcpp::Parameter franka = private_node->get_parameter("franka");  
  if(franka.as_bool())
  {
    std::vector<double> joint_vals;
    joint_vals.resize(9,0.0);
    joint_vals[2] = 0.40956412024912;
    joint_vals[3] = -0.1567813184088162;
    joint_vals[4] = -0.1761983075785612;
    joint_vals[5] = -2.1737492358276174;
    joint_vals[6] = 0.14403150645703755;
    joint_vals[7] = -3.920040654930425;
    joint_vals[8] = -2.455894637318861;
    robot->setJointValues(joint_vals);
  }
  private_node->declare_parameter("ur5",false);
  rclcpp::Parameter ur5 = private_node->get_parameter("ur5");  
  if(ur5.as_bool())
  {
    std::vector<double> joint_vals;
    joint_vals.resize(6,0.0);
    joint_vals[0] = 1.502014986366423;
    joint_vals[1] = -1.3707345617019364;
    joint_vals[2] = -0.000384263032252421;
    joint_vals[3] = -0.06531771795915448;
    joint_vals[4] = -0.0003382902263050885;
    joint_vals[5] = -0.2635960246366345;      
    robot->setJointValues(joint_vals);
  }

  
  RCLCPP_INFO(private_node->get_logger(),"Simulated robot running");

  // bool cycle;
  private_node->declare_parameter("cycle_move", false);
  rclcpp::Parameter cycle = private_node->get_parameter("cycle_move");
//   ros::NodeHandle root_nh("");
  rclcpp::TimerBase::SharedPtr timer;
      if (cycle.as_bool()) {
	  timer = node_->create_wall_timer(std::chrono::seconds(1), [robot, num_joints]() {
    setVelocities(robot, num_joints);
  });
  }

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(publisher);
  executor.add_node(vel_controller);
  executor.add_node(pos_controller);
  executor.add_node(pos_command);
  executor.add_node(traj_executer);
  executor.add_node(private_node);
  executor.add_node(node_);
  executor.spin();
  rclcpp::shutdown();					    
}
