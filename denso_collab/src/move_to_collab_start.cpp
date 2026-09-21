#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{

using JointTarget = std::map<std::string, double>;

// Retain the original pose for Gazebo/simulation.
const JointTarget kLeftSimulationStart{
  {"left_joint_1", -0.10105070580797723},
  {"left_joint_2", 1.4752485930548636},
  {"left_joint_3", -1.9513752321972542},
  {"left_joint_4", 0.11348479374630084},
  {"left_joint_5", 2.0434395165858494},
  {"left_joint_6", 0.05183980864340618},
};

const JointTarget kRightSimulationStart{
  {"right_joint_1", 0.0},
  {"right_joint_2", 0.0},
  {"right_joint_3", 1.57},
  {"right_joint_4", 0.0},
  {"right_joint_5", 0.0},
  {"right_joint_6", 0.0},
};

// Measured from /joint_states with both physical robots and the mounted tool
// in the real collaboration start configuration on 2026-09-17. Keep this
// independent from Gazebo: the real setup has a different right-arm pose.
const JointTarget kLeftRealStart{
  {"left_joint_1", -0.10118400929841607},
  {"left_joint_2", 1.4750073758643738},
  {"left_joint_3", -1.9512535367483903},
  {"left_joint_4", 0.11342110134927567},
  {"left_joint_5", 2.0434983105402837},
  {"left_joint_6", 0.05199140259140652},
};

const JointTarget kRightRealStart{
  {"right_joint_1", 0.00246695269676765},
  {"right_joint_2", -0.031657528509989916},
  {"right_joint_3", 1.5874174590497536},
  {"right_joint_4", -0.22802864096418166},
  {"right_joint_5", 0.01094100406012989},
  {"right_joint_6", 0.22795242129378354},
};

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
  for (const auto & prefix : {std::string("left_"), std::string("right_")}) {
    const std::string group = prefix == "left_" ? "left_arm" : "right_arm";
    const std::string key = "robot_description_kinematics." + group + ".";
    node->declare_parameter(key + "kinematics_solver", "vs050/IKFastKinematicsPlugin");
    node->declare_parameter(key + "link_prefix", prefix);
    node->declare_parameter<std::vector<double>>(
      key + "solution_weights", {4.0, 1.0, 1.0, 4.0, 3.0, 10.0});
  }
  return true;
}

bool move_to(
  moveit::planning_interface::MoveGroupInterface & group, const JointTarget & target,
  const std::map<std::string, double> & current, bool plan_only, const rclcpp::Logger & logger)
{
  moveit_msgs::msg::RobotState start;
  start.joint_state.name = group.getActiveJoints();
  for (const auto & joint : start.joint_state.name) {
    const auto it = current.find(joint);
    if (it == current.end()) {
      RCLCPP_ERROR(logger, "Missing current joint state for %s", joint.c_str());
      return false;
    }
    start.joint_state.position.push_back(it->second);
  }
  group.setStartState(start);
  group.setJointValueTarget(target);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed for %s", group.getName().c_str());
    return false;
  }
  if (!plan_only && group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed for %s", group.getName().c_str());
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_to_collab_start");
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);
  const bool sim = node->declare_parameter<bool>("sim", true);
  if (!load_descriptions(node)) {
    RCLCPP_ERROR(node->get_logger(), "Could not load MoveIt descriptions");
    rclcpp::shutdown();
    return 2;
  }

  std::map<std::string, double> current;
  const auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&current](sensor_msgs::msg::JointState::SharedPtr message) {
      for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
        current[message->name[i]] = message->position[i];
      }
    });
  for (int i = 0; i < 50 && current.size() < 12; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (current.size() < 12) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for /joint_states");
    rclcpp::shutdown();
    return 2;
  }

  auto right = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "right_arm");
  auto left = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "left_arm");
  for (const auto & group : {right, left}) {
    group->setPlanningTime(8.0);
    group->setNumPlanningAttempts(10);
    group->setMaxVelocityScalingFactor(0.15);
    group->setMaxAccelerationScalingFactor(0.15);
  }
  const JointTarget & right_target = sim ? kRightSimulationStart : kRightRealStart;
  const JointTarget & left_target = sim ? kLeftSimulationStart : kLeftRealStart;
  RCLCPP_INFO(
    node->get_logger(), "Using the %s collaboration start pose",
    sim ? "simulation" : "real-robot");
  const bool success =
    move_to(*right, right_target, current, plan_only, node->get_logger()) &&
    move_to(*left, left_target, current, plan_only, node->get_logger());
  RCLCPP_INFO(node->get_logger(), success ? "Collab start pose reached" : "Collab start pose failed");
  rclcpp::shutdown();
  return success ? 0 : 1;
}
