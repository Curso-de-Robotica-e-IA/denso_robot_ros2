#include <algorithm>
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
  bool use_orientation = false;
  std::string move_group_node = "/move_group";
};

struct Target
{
  std::array<double, 3> xyz{};
  std::array<double, 4> xyzw{};
  bool has_orientation = false;
};

void print_usage(const char * prog)
{
  std::cout
    << "Usage: " << prog << " [options]\n"
    << "Options:\n"
    << "  --input-json <path>            JSON file with moveit_position_targets_xyz\n"
    << "  --group-name <name>            MoveIt planning group (default: arm)\n"
    << "  --eef-link <name>              End effector link (optional)\n"
    << "  --planning-time <sec>          Planning time per target > 0 (default: 5.0)\n"
    << "  --num-planning-attempts <n>    Planning attempts > 0 (default: 5)\n"
    << "  --velocity-scaling <v>         Velocity scaling in [0,1] (default: 0.1)\n"
    << "  --acceleration-scaling <v>     Acceleration scaling in [0,1] (default: 0.1)\n"
    << "  --start-index <n>              First target index >= 0 (default: 0)\n"
    << "  --end-index <n>                Last target index, -1 means last (default: -1)\n"
    << "  --dwell-seconds <sec>          Pause between targets >= 0 (default: 0.0)\n"
    << "  --plan-only                    Plan without execution\n"
    << "  --skip-failed                  Continue when one target fails\n"
    << "  --use-orientation              Also send quaternion targets from\n"
    << "                                 moveit_orientation_targets_xyzw\n"
    << "  --move-group-node <name>       Node name used to fetch robot_description params\n"
    << "                                 (default: /move_group)\n"
    << "  -h, --help                     Show this help\n";
}

bool parse_int(const std::string & text, int & value)
{
  try {
    size_t pos = 0;
    int parsed = std::stoi(text, &pos);
    if (pos != text.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_double(const std::string & text, double & value)
{
  try {
    size_t pos = 0;
    double parsed = std::stod(text, &pos);
    if (pos != text.size() || !std::isfinite(parsed)) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_cli(int argc, char ** argv, Options & options, std::string & error)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto need_value = [&](const std::string & flag) -> const char * {
      if (i + 1 >= argc) {
        error = "Missing value for " + flag;
        return nullptr;
      }
      return argv[++i];
    };

    if (arg == "-h" || arg == "--help") {
      print_usage(argv[0]);
      return false;
    }
    if (arg == "--input-json") {
      const char * v = need_value(arg);
      if (!v) {return false;}
      options.input_json = v;
      continue;
    }
    if (arg == "--group-name") {
      const char * v = need_value(arg);
      if (!v) {return false;}
      options.group_name = v;
      continue;
    }
    if (arg == "--eef-link") {
      const char * v = need_value(arg);
      if (!v) {return false;}
      options.eef_link = v;
      continue;
    }
    if (arg == "--planning-time") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.planning_time)) {
        error = "Invalid --planning-time";
        return false;
      }
      continue;
    }
    if (arg == "--num-planning-attempts") {
      const char * v = need_value(arg);
      if (!v || !parse_int(v, options.num_planning_attempts)) {
        error = "Invalid --num-planning-attempts";
        return false;
      }
      continue;
    }
    if (arg == "--velocity-scaling") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.velocity_scaling)) {
        error = "Invalid --velocity-scaling";
        return false;
      }
      continue;
    }
    if (arg == "--acceleration-scaling") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.acceleration_scaling)) {
        error = "Invalid --acceleration-scaling";
        return false;
      }
      continue;
    }
    if (arg == "--start-index") {
      const char * v = need_value(arg);
      if (!v || !parse_int(v, options.start_index)) {
        error = "Invalid --start-index";
        return false;
      }
      continue;
    }
    if (arg == "--end-index") {
      const char * v = need_value(arg);
      if (!v || !parse_int(v, options.end_index)) {
        error = "Invalid --end-index";
        return false;
      }
      continue;
    }
    if (arg == "--dwell-seconds") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.dwell_seconds)) {
        error = "Invalid --dwell-seconds";
        return false;
      }
      continue;
    }
    if (arg == "--plan-only") {
      options.plan_only = true;
      continue;
    }
    if (arg == "--skip-failed") {
      options.skip_failed = true;
      continue;
    }
    if (arg == "--use-orientation") {
      options.use_orientation = true;
      continue;
    }
    if (arg == "--move-group-node") {
      const char * v = need_value(arg);
      if (!v) {return false;}
      options.move_group_node = v;
      continue;
    }

    error = "Unknown argument: " + arg;
    return false;
  }

  if (options.group_name.empty()) {
    error = "--group-name cannot be empty";
    return false;
  }
  if (options.move_group_node.empty()) {
    error = "--move-group-node cannot be empty";
    return false;
  }
  if (options.planning_time <= 0.0) {
    error = "--planning-time must be > 0";
    return false;
  }
  if (options.num_planning_attempts <= 0) {
    error = "--num-planning-attempts must be > 0";
    return false;
  }
  if (options.velocity_scaling < 0.0 || options.velocity_scaling > 1.0) {
    error = "--velocity-scaling must be in [0, 1]";
    return false;
  }
  if (options.acceleration_scaling < 0.0 || options.acceleration_scaling > 1.0) {
    error = "--acceleration-scaling must be in [0, 1]";
    return false;
  }
  if (options.start_index < 0) {
    error = "--start-index must be >= 0";
    return false;
  }
  if (options.dwell_seconds < 0.0) {
    error = "--dwell-seconds must be >= 0";
    return false;
  }

  std::ifstream check_file(options.input_json);
  if (!check_file.good()) {
    error = "Cannot open --input-json: " + options.input_json;
    return false;
  }

  return true;
}

std::vector<Target> load_targets(const std::string & json_path, bool use_orientation)
{
  boost::property_tree::ptree root;
  boost::property_tree::read_json(json_path, root);

  std::vector<Target> targets;
  const auto & pos_arr = root.get_child("moveit_position_targets_xyz");

  boost::optional<const boost::property_tree::ptree &> ori_arr;
  if (use_orientation) {
    ori_arr = root.get_child_optional("moveit_orientation_targets_xyzw");
    if (!ori_arr) {
      throw std::runtime_error(
              "Missing moveit_orientation_targets_xyzw while --use-orientation is enabled");
    }
  }

  const size_t expected_ori_size = ori_arr ? ori_arr->size() : 0;
  if (ori_arr && expected_ori_size != pos_arr.size()) {
    throw std::runtime_error(
            "moveit_position_targets_xyz and moveit_orientation_targets_xyzw must have same length");
  }

  auto ori_it = ori_arr ? ori_arr->begin() : boost::property_tree::ptree::const_iterator{};

  for (const auto & item : pos_arr) {
    Target target;

    const auto & xyz_tree = item.second;
    size_t idx = 0;
    for (const auto & v : xyz_tree) {
      if (idx >= 3) {
        throw std::runtime_error("Each target must contain exactly 3 values");
      }
      target.xyz[idx] = v.second.get_value<double>();
      if (!std::isfinite(target.xyz[idx])) {
        throw std::runtime_error("Target contains non-finite numeric value");
      }
      ++idx;
    }
    if (idx != 3) {
      throw std::runtime_error("Each target must contain exactly 3 values");
    }

    if (ori_arr) {
      const auto & xyzw_tree = ori_it->second;
      ++ori_it;

      size_t q_idx = 0;
      for (const auto & qv : xyzw_tree) {
        if (q_idx >= 4) {
          throw std::runtime_error("Each orientation target must contain exactly 4 values");
        }
        target.xyzw[q_idx] = qv.second.get_value<double>();
        if (!std::isfinite(target.xyzw[q_idx])) {
          throw std::runtime_error("Orientation target contains non-finite numeric value");
        }
        ++q_idx;
      }
      if (q_idx != 4) {
        throw std::runtime_error("Each orientation target must contain exactly 4 values");
      }

      const double q_norm = std::sqrt(
        target.xyzw[0] * target.xyzw[0] +
        target.xyzw[1] * target.xyzw[1] +
        target.xyzw[2] * target.xyzw[2] +
        target.xyzw[3] * target.xyzw[3]);
      if (q_norm <= std::numeric_limits<double>::epsilon()) {
        throw std::runtime_error("Orientation target has zero quaternion norm");
      }
      target.xyzw[0] /= q_norm;
      target.xyzw[1] /= q_norm;
      target.xyzw[2] /= q_norm;
      target.xyzw[3] /= q_norm;
      target.has_orientation = true;
    }

    targets.push_back(target);
  }

  if (targets.empty()) {
    throw std::runtime_error("No targets in moveit_position_targets_xyz");
  }

  return targets;
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

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  std::string parse_error;
  if (!parse_cli(argc, argv, options, parse_error)) {
    if (!parse_error.empty()) {
      std::cerr << "Argument error: " << parse_error << "\n\n";
      print_usage(argv[0]);
      return 2;
    }
    return 0;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_moveit_xyz_targets_from_json");
  const auto logger = node->get_logger();

  std::string description_error;
  if (!load_descriptions_from_move_group(node, options.move_group_node, description_error)) {
    RCLCPP_ERROR(logger, "Failed to load robot descriptions from move_group: %s", description_error.c_str());
    RCLCPP_ERROR(logger, "Hint: start denso_robot_bringup launch before running this node, or set --move-group-node.");
    rclcpp::shutdown();
    return 2;
  }

  std::vector<Target> targets_all;
  try {
    targets_all = load_targets(options.input_json, options.use_orientation);
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
    oss << "Invalid --group-name '" << options.group_name << "'. Available groups: ";
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
    oss << "Invalid --eef-link '" << options.eef_link << "'.";
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

  RCLCPP_INFO(logger, "MoveIt sender configuration:");
  RCLCPP_INFO(logger, "  input_json: %s", options.input_json.c_str());
  RCLCPP_INFO(logger, "  group_name: %s", options.group_name.c_str());
  RCLCPP_INFO(logger, "  planning_frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "  eef_link: %s", selected_eef.empty() ? "<default>" : selected_eef.c_str());
  RCLCPP_INFO(logger, "  targets_to_send: %d (indices %d..%d)", end_idx - start_idx + 1, start_idx, end_idx);
  RCLCPP_INFO(logger, "  use_orientation: %s", options.use_orientation ? "true" : "false");
  RCLCPP_INFO(logger, "  plan_only: %s", options.plan_only ? "true" : "false");

  int succeeded = 0;
  int failed = 0;

  for (int i = start_idx; i <= end_idx; ++i) {
    const auto & target = targets_all[static_cast<size_t>(i)];
    RCLCPP_INFO(logger, "Target #%d position: [%.9f, %.9f, %.9f]", i, target.xyz[0], target.xyz[1], target.xyz[2]);
    if (options.use_orientation) {
      RCLCPP_INFO(
        logger,
        "Target #%d orientation (xyzw): [%.9f, %.9f, %.9f, %.9f]",
        i,
        target.xyzw[0], target.xyzw[1], target.xyzw[2], target.xyzw[3]);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = true;

    if (options.use_orientation) {
      if (!target.has_orientation) {
        ++failed;
        RCLCPP_ERROR(logger, "  result: missing orientation target while --use-orientation is enabled");
        move_group->clearPoseTargets();
        if (!options.skip_failed) {
          break;
        }
        continue;
      }

      geometry_msgs::msg::Pose pose_target;
      pose_target.position.x = target.xyz[0];
      pose_target.position.y = target.xyz[1];
      pose_target.position.z = target.xyz[2];
      pose_target.orientation.x = target.xyzw[0];
      pose_target.orientation.y = target.xyzw[1];
      pose_target.orientation.z = target.xyzw[2];
      pose_target.orientation.w = target.xyzw[3];

      if (!options.eef_link.empty()) {
        ok = move_group->setPoseTarget(pose_target, options.eef_link);
      } else {
        ok = move_group->setPoseTarget(pose_target);
      }
    } else {
      if (!options.eef_link.empty()) {
        ok = move_group->setPositionTarget(target.xyz[0], target.xyz[1], target.xyz[2], options.eef_link);
      } else {
        ok = move_group->setPositionTarget(target.xyz[0], target.xyz[1], target.xyz[2]);
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
