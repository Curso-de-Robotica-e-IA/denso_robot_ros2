#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{

struct Options
{
  std::string input_json = "denso_robot_bringup/robot1_points_in_robot2_frame.json";
  std::string group_name = "arm";
  std::string eef_link;
  double planning_time = 5.0;
  int num_planning_attempts = 5;
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
  int start_index = 0;
  int end_index = -1;
  double dwell_seconds = 0.0;
  bool plan_only = false;
  bool skip_failed = false;
  std::string move_group_node = "/move_group";
  double rotation_x_deg = 0.0;
  double rotation_y_deg = 0.0;
  double rotation_z_deg = 0.0;
  bool use_orientation = false;
};

struct Target
{
  std::string label;
  std::array<double, 3> robot2_xyz{};
  std::array<double, 3> robot1_xyz{};
  bool has_robot1 = false;
  std::array<double, 4> robot2_xyzw{};
  bool has_orientation = false;
};

std::array<double, 3> parse_position(const boost::property_tree::ptree & node)
{
  std::array<double, 3> xyz{};
  xyz[0] = node.get<double>("x");
  xyz[1] = node.get<double>("y");
  xyz[2] = node.get<double>("z");
  for (double v : xyz) {
    if (!std::isfinite(v)) {
      throw std::runtime_error("Position contains non-finite numeric value");
    }
  }
  return xyz;
}

std::array<double, 4> normalize_quaternion(const std::array<double, 4> & q)
{
  const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm <= std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error("Quaternion has zero norm");
  }
  std::array<double, 4> out = q;
  for (double & v : out) {
    v /= norm;
  }
  return out;
}

std::array<double, 4> parse_quaternion(const boost::property_tree::ptree & node)
{
  std::array<double, 4> q{};
  q[0] = node.get<double>("x");
  q[1] = node.get<double>("y");
  q[2] = node.get<double>("z");
  q[3] = node.get<double>("w");
  for (double v : q) {
    if (!std::isfinite(v)) {
      throw std::runtime_error("Quaternion contains non-finite numeric value");
    }
  }
  return normalize_quaternion(q);
}

std::array<double, 4> multiply_quaternion(
  const std::array<double, 4> & a,
  const std::array<double, 4> & b)
{
  std::array<double, 4> out{};
  out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
  out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  return normalize_quaternion(out);
}

std::vector<Target> load_targets(const std::string & json_path)
{
  boost::property_tree::ptree root;
  boost::property_tree::read_json(json_path, root);

  const auto points_opt = root.get_child_optional("points");
  if (!points_opt) {
    throw std::runtime_error("Missing required 'points' array in JSON");
  }

  std::vector<Target> targets;
  size_t index = 0;
  for (const auto & item : *points_opt) {
    const auto & point = item.second;
    Target target;
    target.label = point.get<std::string>("label", "point_" + std::to_string(index));

    boost::optional<const boost::property_tree::ptree &> robot2_pos =
      point.get_child_optional("position_robot2_transformed");
    if (!robot2_pos) {
      robot2_pos = point.get_child_optional("position_robot2");
    }
    if (!robot2_pos) {
      throw std::runtime_error(
              "Point '" + target.label +
              "' is missing position_robot2_transformed (or legacy position_robot2)");
    }
    target.robot2_xyz = parse_position(*robot2_pos);

    if (const auto robot1_pos = point.get_child_optional("position_robot1")) {
      target.robot1_xyz = parse_position(*robot1_pos);
      target.has_robot1 = true;
    }

    boost::optional<const boost::property_tree::ptree &> robot2_ori =
      point.get_child_optional("orientation_robot2_transformed_quaternion");
    if (!robot2_ori) {
      robot2_ori = point.get_child_optional("orientation_robot2_quaternion");
    }
    if (robot2_ori) {
      target.robot2_xyzw = parse_quaternion(*robot2_ori);
      target.has_orientation = true;
    }

    targets.push_back(target);
    ++index;
  }

  if (targets.empty()) {
    throw std::runtime_error("No targets found in 'points' array");
  }

  return targets;
}

std::array<double, 4> quaternion_from_rpy_deg(double roll_deg, double pitch_deg, double yaw_deg)
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double roll = roll_deg * kDegToRad;
  const double pitch = pitch_deg * kDegToRad;
  const double yaw = yaw_deg * kDegToRad;

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  std::array<double, 4> q{};
  q[3] = cr * cp * cy + sr * sp * sy;
  q[0] = sr * cp * cy - cr * sp * sy;
  q[1] = cr * sp * cy + sr * cp * sy;
  q[2] = cr * cp * sy - sr * sp * cy;

  return normalize_quaternion(q);
}

bool load_descriptions_from_move_group(
  const rclcpp::Node::SharedPtr & node,
  const std::string & move_group_node,
  std::string & error)
{
  std::string service_name = move_group_node;
  if (!service_name.empty() && service_name.back() != '/') {
    service_name += "/";
  }
  service_name += "get_parameters";

  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    error = "Timed out waiting for service: " + service_name;
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"robot_description", "robot_description_semantic"};
  auto future = client->async_send_request(request);

  const auto rc = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));
  if (rc != rclcpp::FutureReturnCode::SUCCESS) {
    error = "Failed to query " + service_name + " for robot description parameters";
    return false;
  }

  const auto response = future.get();
  if (!response || response->values.size() != request->names.size()) {
    error = "Unexpected response size from " + service_name;
    return false;
  }

  const auto & robot_desc = response->values[0];
  const auto & semantic_desc = response->values[1];
  if (robot_desc.type != rclcpp::PARAMETER_STRING || robot_desc.string_value.empty()) {
    error = "robot_description is missing/empty on node " + move_group_node;
    return false;
  }
  if (semantic_desc.type != rclcpp::PARAMETER_STRING || semantic_desc.string_value.empty()) {
    error = "robot_description_semantic is missing/empty on node " + move_group_node;
    return false;
  }

  node->declare_parameter<std::string>("robot_description", robot_desc.string_value);
  node->declare_parameter<std::string>("robot_description_semantic", semantic_desc.string_value);
  return true;
}

bool load_options(const rclcpp::Node::SharedPtr & node, Options & options, std::string & error)
{
  options.input_json = node->declare_parameter<std::string>("input_json", options.input_json);
  options.group_name = node->declare_parameter<std::string>("group_name", options.group_name);
  options.eef_link = node->declare_parameter<std::string>("eef_link", options.eef_link);
  options.planning_time = node->declare_parameter<double>("planning_time", options.planning_time);
  options.num_planning_attempts =
    node->declare_parameter<int>("num_planning_attempts", options.num_planning_attempts);
  options.velocity_scaling =
    node->declare_parameter<double>("velocity_scaling", options.velocity_scaling);
  options.acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", options.acceleration_scaling);
  options.start_index = node->declare_parameter<int>("start_index", options.start_index);
  options.end_index = node->declare_parameter<int>("end_index", options.end_index);
  options.dwell_seconds = node->declare_parameter<double>("dwell_seconds", options.dwell_seconds);
  options.plan_only = node->declare_parameter<bool>("plan_only", options.plan_only);
  options.skip_failed = node->declare_parameter<bool>("skip_failed", options.skip_failed);
  options.move_group_node =
    node->declare_parameter<std::string>("move_group_node", options.move_group_node);
  options.rotation_x_deg = node->declare_parameter<double>("rotation_x_deg", options.rotation_x_deg);
  options.rotation_y_deg = node->declare_parameter<double>("rotation_y_deg", options.rotation_y_deg);
  options.rotation_z_deg = node->declare_parameter<double>("rotation_z_deg", options.rotation_z_deg);
  options.use_orientation = node->declare_parameter<bool>("use_orientation", options.use_orientation);

  if (options.group_name.empty()) {
    error = "group_name cannot be empty";
    return false;
  }
  if (options.move_group_node.empty()) {
    error = "move_group_node cannot be empty";
    return false;
  }
  if (options.planning_time <= 0.0) {
    error = "planning_time must be > 0";
    return false;
  }
  if (options.num_planning_attempts <= 0) {
    error = "num_planning_attempts must be > 0";
    return false;
  }
  if (options.velocity_scaling < 0.0 || options.velocity_scaling > 1.0) {
    error = "velocity_scaling must be in [0, 1]";
    return false;
  }
  if (options.acceleration_scaling < 0.0 || options.acceleration_scaling > 1.0) {
    error = "acceleration_scaling must be in [0, 1]";
    return false;
  }
  if (options.start_index < 0) {
    error = "start_index must be >= 0";
    return false;
  }
  if (options.dwell_seconds < 0.0) {
    error = "dwell_seconds must be >= 0";
    return false;
  }

  std::ifstream check_file(options.input_json);
  if (!check_file.good()) {
    error = "Cannot open input_json: " + options.input_json;
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_moveit_points_from_json");
  const auto logger = node->get_logger();

  Options options;
  std::string config_error;
  if (!load_options(node, options, config_error)) {
    RCLCPP_ERROR(logger, "Configuration error: %s", config_error.c_str());
    rclcpp::shutdown();
    return 2;
  }

  std::string description_error;
  if (!load_descriptions_from_move_group(node, options.move_group_node, description_error)) {
    RCLCPP_ERROR(logger, "Failed to load robot descriptions from move_group: %s", description_error.c_str());
    RCLCPP_ERROR(logger, "Hint: start denso_robot_bringup launch before running this node.");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<Target> targets_all;
  try {
    targets_all = load_targets(options.input_json);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to load targets from %s: %s", options.input_json.c_str(), ex.what());
    rclcpp::shutdown();
    return 2;
  }

  const int max_idx = static_cast<int>(targets_all.size()) - 1;
  const int start_idx = std::max(0, options.start_index);
  const int end_idx = (options.end_index < 0) ? max_idx : std::min(options.end_index, max_idx);
  if (start_idx > end_idx) {
    RCLCPP_ERROR(logger, "Invalid index range: start=%d end=%d", start_idx, end_idx);
    rclcpp::shutdown();
    return 2;
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  try {
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options.group_name);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to construct MoveGroupInterface: %s", ex.what());
    RCLCPP_ERROR(logger, "Check that move_group is running and robot descriptions are available.");
    rclcpp::shutdown();
    return 2;
  }
  move_group->setPlanningTime(options.planning_time);
  move_group->setNumPlanningAttempts(options.num_planning_attempts);
  move_group->setMaxVelocityScalingFactor(options.velocity_scaling);
  move_group->setMaxAccelerationScalingFactor(options.acceleration_scaling);

  const auto groups = move_group->getJointModelGroupNames();
  if (std::find(groups.begin(), groups.end(), options.group_name) == groups.end()) {
    std::ostringstream oss;
    oss << "Invalid group_name '" << options.group_name << "'. Available groups: ";
    for (const auto & g : groups) {
      oss << g << " ";
    }
    RCLCPP_ERROR(logger, "%s", oss.str().c_str());
    rclcpp::shutdown();
    return 2;
  }

  const auto robot_model = move_group->getRobotModel();
  if (!options.eef_link.empty() && !robot_model->hasLinkModel(options.eef_link)) {
    std::vector<std::string> candidates;
    for (const auto * link : robot_model->getLinkModels()) {
      const std::string & name = link->getName();
      if (name.size() >= 10 && name.substr(name.size() - 10) == "calib_link") {
        candidates.push_back(name);
      }
    }
    std::ostringstream oss;
    oss << "Invalid eef_link '" << options.eef_link << "'.";
    if (!candidates.empty()) {
      oss << " calib_link candidates: ";
      for (const auto & c : candidates) {
        oss << c << " ";
      }
    }
    RCLCPP_ERROR(logger, "%s", oss.str().c_str());
    rclcpp::shutdown();
    return 2;
  }

  const std::string selected_eef = options.eef_link.empty() ? move_group->getEndEffectorLink() : options.eef_link;

  const bool has_fixed_rotation =
    std::abs(options.rotation_x_deg) > 1e-9 ||
    std::abs(options.rotation_y_deg) > 1e-9 ||
    std::abs(options.rotation_z_deg) > 1e-9;
  const bool apply_orientation = options.use_orientation || has_fixed_rotation;

  std::array<double, 4> fixed_orientation{0.0, 0.0, 0.0, 1.0};
  if (has_fixed_rotation) {
    try {
      fixed_orientation = quaternion_from_rpy_deg(
        options.rotation_x_deg, options.rotation_y_deg, options.rotation_z_deg);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(logger, "Failed to compute fixed orientation: %s", ex.what());
      rclcpp::shutdown();
      return 2;
    }
  }

  RCLCPP_INFO(logger, "MoveIt sender configuration:");
  RCLCPP_INFO(logger, "  input_json: %s", options.input_json.c_str());
  RCLCPP_INFO(logger, "  group_name: %s", options.group_name.c_str());
  RCLCPP_INFO(logger, "  planning_frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "  eef_link: %s", selected_eef.empty() ? "<default>" : selected_eef.c_str());
  RCLCPP_INFO(logger, "  targets_to_send: %d (indices %d..%d)", end_idx - start_idx + 1, start_idx, end_idx);
  RCLCPP_INFO(logger, "  plan_only: %s", options.plan_only ? "true" : "false");
  RCLCPP_INFO(logger, "  use_orientation: %s", options.use_orientation ? "true" : "false");
  RCLCPP_INFO(
    logger,
    "  fixed_rotation_deg (r,p,y): [%.1f, %.1f, %.1f]",
    options.rotation_x_deg, options.rotation_y_deg, options.rotation_z_deg);

  int succeeded = 0;
  int failed = 0;

  for (int i = start_idx; i <= end_idx; ++i) {
    const auto & target = targets_all[static_cast<size_t>(i)];
    RCLCPP_INFO(
      logger,
      "Target #%d [%s] robot2 position: [%.9f, %.9f, %.9f]",
      i,
      target.label.c_str(),
      target.robot2_xyz[0], target.robot2_xyz[1], target.robot2_xyz[2]);
    if (target.has_robot1) {
      RCLCPP_INFO(
        logger,
        "  robot1 position: [%.9f, %.9f, %.9f]",
        target.robot1_xyz[0], target.robot1_xyz[1], target.robot1_xyz[2]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = true;

    if (apply_orientation) {
      if (options.use_orientation && !target.has_orientation) {
        ++failed;
        RCLCPP_ERROR(logger, "  result: missing orientation_robot2_transformed_quaternion");
        move_group->clearPoseTargets();
        if (!options.skip_failed) {
          break;
        }
        continue;
      }

      std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
      if (options.use_orientation) {
        orientation = target.robot2_xyzw;
      }
      if (has_fixed_rotation) {
        orientation = multiply_quaternion(fixed_orientation, orientation);
      }

      geometry_msgs::msg::Pose pose_target;
      pose_target.position.x = target.robot2_xyz[0];
      pose_target.position.y = target.robot2_xyz[1];
      pose_target.position.z = target.robot2_xyz[2];
      pose_target.orientation.x = orientation[0];
      pose_target.orientation.y = orientation[1];
      pose_target.orientation.z = orientation[2];
      pose_target.orientation.w = orientation[3];

      if (!options.eef_link.empty()) {
        ok = move_group->setPoseTarget(pose_target, options.eef_link);
      } else {
        ok = move_group->setPoseTarget(pose_target);
      }
    } else {
      if (!options.eef_link.empty()) {
        ok = move_group->setPositionTarget(
          target.robot2_xyz[0], target.robot2_xyz[1], target.robot2_xyz[2], options.eef_link);
      } else {
        ok = move_group->setPositionTarget(target.robot2_xyz[0], target.robot2_xyz[1], target.robot2_xyz[2]);
      }
    }

    if (!ok) {
      ++failed;
      RCLCPP_ERROR(logger, "  result: failed to set position target");
      move_group->clearPoseTargets();
      if (!options.skip_failed) {
        break;
      }
      continue;
    }

    const auto plan_result = move_group->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      ++failed;
      RCLCPP_ERROR(logger, "  result: planning failed");
      move_group->clearPoseTargets();
      if (!options.skip_failed) {
        break;
      }
      continue;
    }

    if (!options.plan_only) {
      const auto exec_result = move_group->execute(plan);
      move_group->stop();
      if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
        ++failed;
        RCLCPP_ERROR(logger, "  result: execution failed");
        move_group->clearPoseTargets();
        if (!options.skip_failed) {
          break;
        }
        continue;
      }
    }

    ++succeeded;
    RCLCPP_INFO(logger, "  result: success");
    move_group->clearPoseTargets();

    if (options.dwell_seconds > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(options.dwell_seconds));
    }
  }

  RCLCPP_INFO(logger, "Summary: succeeded=%d failed=%d", succeeded, failed);
  rclcpp::shutdown();
  return failed == 0 ? 0 : 1;
}
