#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/Pose.h"
// #include "shape_msgs/SolidPrimitive.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/srv/get_position_ik.hpp>

using moveit::planning_interface::MoveGroupInterface;

#include <chrono> 
using namespace std::chrono_literals;
using std::placeholders::_1;

class ObstacleController : public rclcpp::Node
{
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  moveit::planning_interface::MoveGroupInterface* move_group_;
  moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  
public:
  ObstacleController() : Node("obstacle_controller")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>("obstacles",
								     10, std::bind(&ObstacleController::topic_callback, this, _1));
    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    
  }

  void set_group(moveit::planning_interface::MoveGroupInterface* move_group,
		 moveit::planning_interface::PlanningSceneInterface* planning_scene_interface)
  {
    move_group_ = move_group;
    planning_scene_interface_ = planning_scene_interface;
  }

private:
  void obstacle_none()
  {    
    std::vector<std::string> obs;
    obs.push_back("obs1");
    obs.push_back("obs2");
    obs.push_back("obs3");
    obs.push_back("obs4");
    planning_scene_interface_->removeCollisionObjects(obs);
    RCLCPP_ERROR(this->get_logger(), "Removed all obstacles");
  }
  void obstacle_simple()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "obs1";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 1.0;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.35;
    box_pose.position.z = 0.2;   
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Simple obstacle added");
  }
  void obstacle_hard()
  {

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "obs1";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 2;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.35;
    box_pose.position.z = 0.3-.5;   
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group_->getPlanningFrame();
    collision_object2.id = "obs2";
    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive.BOX_X] = 0.1;
    primitive2.dimensions[primitive.BOX_Y] = 1;
    primitive2.dimensions[primitive.BOX_Z] = 0.1;
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.4;
    box_pose2.position.y = 0.3;
    box_pose2.position.z = 0.8;   
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object.ADD;
    collision_objects.push_back(collision_object2);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_ERROR(this->get_logger(), "Hard obstacle");
  }
  void obstacle_super()
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = "obs1";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.8;
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.35;
    box_pose.position.z = 0.4;   
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    moveit_msgs::msg::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group_->getPlanningFrame();
    collision_object2.id = "obs2";
    shape_msgs::msg::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[primitive.BOX_X] = 0.1;
    primitive2.dimensions[primitive.BOX_Y] = 0.5;
    primitive2.dimensions[primitive.BOX_Z] = 0.1;
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.4;
    box_pose2.position.y = 0.0;
    box_pose2.position.z = 0.8;   
    collision_object2.primitives.push_back(primitive2);
    collision_object2.primitive_poses.push_back(box_pose2);
    collision_object2.operation = collision_object.ADD;
    collision_objects.push_back(collision_object2);

    moveit_msgs::msg::CollisionObject collision_object3;
    collision_object3.header.frame_id = move_group_->getPlanningFrame();
    collision_object3.id = "obs3";
    shape_msgs::msg::SolidPrimitive primitive3;
    primitive3.type = primitive.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[primitive.BOX_X] = 0.1;
    primitive3.dimensions[primitive.BOX_Y] = 0.1;
    primitive3.dimensions[primitive.BOX_Z] = 0.8;
    geometry_msgs::msg::Pose box_pose3;
    box_pose3.orientation.w = 1.0;
    box_pose3.position.x = 0.4;
    box_pose3.position.y = -0.25;
    box_pose3.position.z = 0.4;   
    collision_object3.primitives.push_back(primitive3);
    collision_object3.primitive_poses.push_back(box_pose3);
    collision_object3.operation = collision_object.ADD;
    collision_objects.push_back(collision_object3);

    moveit_msgs::msg::CollisionObject collision_object4;
    collision_object4.header.frame_id = move_group_->getPlanningFrame();
    collision_object4.id = "obs4";
    shape_msgs::msg::SolidPrimitive primitive4;
    primitive4.type = primitive.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[primitive.BOX_X] = 0.1;
    primitive4.dimensions[primitive.BOX_Y] = 0.5;
    primitive4.dimensions[primitive.BOX_Z] = 0.1;
    geometry_msgs::msg::Pose box_pose4;
    box_pose4.orientation.w = 1.0;
    box_pose4.position.x = 0.4;
    box_pose4.position.y = -0.0;
    box_pose4.position.z = 0.3;   
    collision_object4.primitives.push_back(primitive4);
    collision_object4.primitive_poses.push_back(box_pose4);
    collision_object4.operation = collision_object.ADD;
    collision_objects.push_back(collision_object4);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Super hard obstacle");
  }

  void topic_callback(const std_msgs::msg::String & msg)
  {
    obstacle_none();
    rclcpp::sleep_for(std::chrono::nanoseconds(1s));
    if (msg.data=="NONE"){
    } else if (msg.data=="SIMPLE") {
      obstacle_simple();
    } else if (msg.data=="HARD") {
      obstacle_hard();
    } else if (msg.data=="SUPER") {      
      obstacle_super();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown obstacle controller command: '%s'", msg.data.c_str());
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleController>();
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  node->set_group(&move_group, &planning_scene_interface);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
