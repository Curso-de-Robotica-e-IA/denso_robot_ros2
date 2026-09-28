#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{

bool load_descriptions(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>("/move_group/get_parameters");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    return false;
  }
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"robot_description", "robot_description_semantic"};
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return false;
  }
  const auto response = future.get();
  if (response->values.size() != 2 || response->values[0].string_value.empty() ||
    response->values[1].string_value.empty())
  {
    return false;
  }
  node->declare_parameter("robot_description", response->values[0].string_value);
  node->declare_parameter("robot_description_semantic", response->values[1].string_value);
  const std::string key = "robot_description_kinematics.right_arm.";
  node->declare_parameter(key + "kinematics_solver", "vs050/IKFastKinematicsPlugin");
  node->declare_parameter(key + "link_prefix", "right_");
  node->declare_parameter<std::vector<double>>(
    key + "solution_weights", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_right_to_holder_pose");
  const auto target = node->declare_parameter<std::vector<double>>("target", {});
  const auto plan_only = node->declare_parameter<bool>("plan_only", false);
  if (target.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter target must contain right_joint_1 through right_joint_6");
    rclcpp::shutdown();
    return 2;
  }
  if (!load_descriptions(node)) {
    RCLCPP_ERROR(node->get_logger(), "Could not load MoveIt descriptions");
    rclcpp::shutdown();
    return 2;
  }

  std::map<std::string, double> current;
  const auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&current](sensor_msgs::msg::JointState::SharedPtr message) {
      for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
        current[message->name[i]] = message->position[i];
      }
    });
  for (int i = 0; i < 50 && current.size() < 12; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "right_arm");
  group->setPlanningTime(8.0);
  group->setNumPlanningAttempts(10);
  group->setMaxVelocityScalingFactor(0.15);
  group->setMaxAccelerationScalingFactor(0.15);
  moveit_msgs::msg::RobotState start;
  start.joint_state.name = group->getActiveJoints();
  for (const auto & joint : start.joint_state.name) {
    if (!current.count(joint)) {
      RCLCPP_ERROR(node->get_logger(), "Missing current joint state for %s", joint.c_str());
      rclcpp::shutdown();
      return 2;
    }
    start.joint_state.position.push_back(current.at(joint));
  }
  group->setStartState(start);
  group->setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    rclcpp::shutdown();
    return 1;
  }
  if (!plan_only && group->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Execution failed");
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "%s right-holder pose", plan_only ? "Planned" : "Reached");
  rclcpp::shutdown();
  return 0;
}
