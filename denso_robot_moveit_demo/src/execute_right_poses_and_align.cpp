#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstdlib>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{

struct PoseTarget
{
  std::string label;
  geometry_msgs::msg::Pose pose;
};

struct Options
{
  std::string input_json;
  std::string move_group_node = "/move_group";
  std::string right_planning_group = "right_arm";
  std::string right_target_link = "right_J6";
  std::string right_reference_frame = "world";
  double planning_time = 8.0;
  int num_planning_attempts = 10;
  double velocity_scaling = 0.8;
  double acceleration_scaling = 0.1;
  bool plan_only = false;
};

std::array<double, 4> normalized_quaternion(const boost::property_tree::ptree & node)
{
  std::array<double, 4> q{
    node.get<double>("x"), node.get<double>("y"), node.get<double>("z"), node.get<double>("w")};
  double squared_norm = 0.0;
  for (const double value : q) {
    if (!std::isfinite(value)) {
      throw std::runtime_error("Quaternion contains a non-finite value");
    }
    squared_norm += value * value;
  }
  if (squared_norm <= 1e-12) {
    throw std::runtime_error("Quaternion has zero norm");
  }
  const double norm = std::sqrt(squared_norm);
  for (double & value : q) {
    value /= norm;
  }
  return q;
}

std::vector<PoseTarget> load_poses(const std::string & path)
{
  boost::property_tree::ptree root;
  boost::property_tree::read_json(path, root);
  const auto poses = root.get_child_optional("poses");
  if (!poses) {
    throw std::runtime_error("Missing required 'poses' array");
  }

  std::vector<PoseTarget> result;
  std::size_t index = 0;
  for (const auto & item : *poses) {
    const auto & entry = item.second;
    PoseTarget target;
    target.label = entry.get<std::string>("label", "pose_" + std::to_string(index));
    const auto & position = entry.get_child("position");
    target.pose.position.x = position.get<double>("x");
    target.pose.position.y = position.get<double>("y");
    target.pose.position.z = position.get<double>("z");
    if (!std::isfinite(target.pose.position.x) || !std::isfinite(target.pose.position.y) ||
      !std::isfinite(target.pose.position.z))
    {
      throw std::runtime_error("Position contains a non-finite value");
    }
    const auto q = normalized_quaternion(entry.get_child("orientation"));
    target.pose.orientation.x = q[0];
    target.pose.orientation.y = q[1];
    target.pose.orientation.z = q[2];
    target.pose.orientation.w = q[3];
    result.push_back(target);
    ++index;
  }
  if (result.size() != 3) {
    throw std::runtime_error("The input must contain exactly 3 poses");
  }
  return result;
}

bool load_descriptions_from_move_group(
  const rclcpp::Node::SharedPtr & node, const std::string & move_group_node, std::string & error)
{
  const std::string service_name = move_group_node + "/get_parameters";
  const auto client = node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    error = "Timed out waiting for " + service_name;
    return false;
  }
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"robot_description", "robot_description_semantic"};
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    error = "Could not get robot descriptions from " + service_name;
    return false;
  }
  const auto response = future.get();
  if (!response || response->values.size() != 2 ||
    response->values[0].type != rclcpp::PARAMETER_STRING ||
    response->values[1].type != rclcpp::PARAMETER_STRING ||
    response->values[0].string_value.empty() || response->values[1].string_value.empty())
  {
    error = "robot_description or robot_description_semantic is missing on " + move_group_node;
    return false;
  }
  node->declare_parameter<std::string>("robot_description", response->values[0].string_value);
  node->declare_parameter<std::string>("robot_description_semantic", response->values[1].string_value);
  return true;
}

bool valid_group(
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group,
  const std::string & group_name)
{
  const auto groups = group->getJointModelGroupNames();
  return std::find(groups.begin(), groups.end(), group_name) != groups.end();
}

bool plan_and_execute(
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group,
  const rclcpp::Logger & logger, const std::string & description, bool plan_only)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed for %s", description.c_str());
    group->clearPoseTargets();
    return false;
  }
  if (!plan_only && group->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    group->stop();
    RCLCPP_ERROR(logger, "Execution failed for %s", description.c_str());
    group->clearPoseTargets();
    return false;
  }
  group->stop();
  group->clearPoseTargets();
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("execute_right_poses_and_align");
  const auto logger = node->get_logger();
  Options options;
  options.input_json = node->declare_parameter<std::string>("input_json", "");
  options.move_group_node = node->declare_parameter<std::string>("move_group_node", options.move_group_node);
  options.right_planning_group = node->declare_parameter<std::string>("right_planning_group", options.right_planning_group);
  options.right_target_link = node->declare_parameter<std::string>("right_target_link", options.right_target_link);
  options.right_reference_frame = node->declare_parameter<std::string>("right_reference_frame", options.right_reference_frame);
  options.planning_time = node->declare_parameter<double>("planning_time", options.planning_time);
  options.num_planning_attempts = node->declare_parameter<int>("num_planning_attempts", options.num_planning_attempts);
  options.velocity_scaling = node->declare_parameter<double>("velocity_scaling", options.velocity_scaling);
  options.acceleration_scaling = node->declare_parameter<double>("acceleration_scaling", options.acceleration_scaling);
  options.plan_only = node->declare_parameter<bool>("plan_only", options.plan_only);

  if (options.input_json.empty()) {
    RCLCPP_ERROR(logger, "input_json is required");
    rclcpp::shutdown();
    return 2;
  }
  std::vector<PoseTarget> poses;
  try {
    poses = load_poses(options.input_json);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Invalid input_json: %s", ex.what());
    rclcpp::shutdown();
    return 2;
  }

  std::string description_error;
  if (!load_descriptions_from_move_group(node, options.move_group_node, description_error)) {
    RCLCPP_ERROR(logger, "%s", description_error.c_str());
    rclcpp::shutdown();
    return 2;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_group;
  try {
    right_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options.right_planning_group);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Could not create MoveIt interfaces: %s", ex.what());
    rclcpp::shutdown();
    return 2;
  }
  if (!valid_group(right_group, options.right_planning_group) ||
    !right_group->getRobotModel()->hasLinkModel(options.right_target_link))
  {
    RCLCPP_ERROR(logger, "Invalid MoveIt group or target link configuration");
    rclcpp::shutdown();
    return 2;
  }
  right_group->setPlanningTime(options.planning_time);
  right_group->setNumPlanningAttempts(options.num_planning_attempts);
  right_group->setMaxVelocityScalingFactor(options.velocity_scaling);
  right_group->setMaxAccelerationScalingFactor(options.acceleration_scaling);

  RCLCPP_INFO(logger, "Executing 3 right-arm poses in reference frame '%s'", options.right_reference_frame.c_str());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped right_target;
    right_target.header.frame_id = options.right_reference_frame;
    right_target.header.stamp = node->now();
    right_target.pose = poses[i].pose;
    RCLCPP_INFO(logger, "[%zu/3] Right pose '%s'", i + 1, poses[i].label.c_str());
    if (!right_group->setPoseTarget(right_target, options.right_target_link) ||
      !plan_and_execute(right_group, logger, "right pose '" + poses[i].label + "'", options.plan_only))
    {
      RCLCPP_ERROR(logger, "Stopping sequence after failed right-arm motion");
      rclcpp::shutdown();
      return 1;
    }

    if (options.plan_only) {
      RCLCPP_INFO(logger, "[%zu/3] plan_only=true: align_tool was not started", i + 1);
      continue;
    }
    const std::string align_command =
      "ros2 run denso_robot_moveit_demo align_tool --ros-args"
      " -p target_offset_x_m:=0.0"
      " -p target_offset_y_m:=-0.04"
      " -p target_offset_z_m:=0.25"
      " -p target_pitch_deg:=90.0"
      " -p target_yaw_deg:=0.0"
      " -p velocity_scaling:=0.8";
    RCLCPP_INFO(logger, "[%zu/3] Starting align_tool and waiting for it to finish", i + 1);
    const int align_result = std::system(align_command.c_str());
    if (align_result != 0) {
      RCLCPP_ERROR(logger, "align_tool failed (system return code: %d)", align_result);
      rclcpp::shutdown();
      return 1;
    }
  }
  RCLCPP_INFO(logger, "All 3 right poses and left alignments completed");
  rclcpp::shutdown();
  return 0;
}
