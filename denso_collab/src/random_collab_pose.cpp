#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <yaml-cpp/yaml.h>

#include "denso_collab/alignment_pose.hpp"

namespace
{

struct Limits
{
  std::string planning_mode;
  double offset_x_m;
  double offset_y_m;
  double offset_z_m;
  double tilt_x_deg;
  double tilt_y_deg;
  int attempts;
  double planning_time_sec;
  double camera_distance_m;
  double screen_width_m;
  double screen_height_m;
  double screen_margin_m;
  double screen_plane_offset_m;
  double touch_approach_distance_m;
  int touch_path_samples;
  double max_touch_condition_number;
  double calibration_x_m;
  double calibration_y_m;
};

using Anchor = std::pair<std::string, std::vector<double>>;

bool load_descriptions(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>(
    "/move_group/get_parameters");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    return false;
  }

  std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  for (int attempt = 0; attempt < 3 && !response; ++attempt) {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description", "robot_description_semantic"};
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      response = future.get();
    } else {
      client->remove_pending_request(future);
    }
  }
  if (!response || response->values.size() != 2 ||
    response->values[0].string_value.empty() || response->values[1].string_value.empty())
  {
    return false;
  }

  node->declare_parameter("robot_description", response->values[0].string_value);
  node->declare_parameter("robot_description_semantic", response->values[1].string_value);
  for (const auto & group_prefix : {
      std::pair<std::string, std::string>{"left_arm", "left_"},
      std::pair<std::string, std::string>{"right_arm", "right_"}})
  {
    const std::string key = "robot_description_kinematics." + group_prefix.first + ".";
    node->declare_parameter(key + "kinematics_solver", "vs050/IKFastKinematicsPlugin");
    node->declare_parameter(key + "link_prefix", group_prefix.second);
    node->declare_parameter<std::vector<double>>(
      key + "solution_weights", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
  }
  return true;
}

Limits load_limits(const rclcpp::Node::SharedPtr & node, const std::string & path)
{
  const YAML::Node config = YAML::LoadFile(path);
  const YAML::Node values = config["random_collab_pose"]["ros__parameters"];
  const YAML::Node screen = config["move_to_screen_standoff"]["ros__parameters"];
  if (!values) {
    throw std::runtime_error("Missing random_collab_pose.ros__parameters");
  }
  if (!screen) {
    throw std::runtime_error("Missing move_to_screen_standoff.ros__parameters");
  }
  const bool sim = node->declare_parameter<bool>("sim", false);
  Limits limits{
    node->declare_parameter<std::string>(
      "planning_mode", values["planning_mode"].as<std::string>()),
    node->declare_parameter("max_holder_offset_x_m", values["max_holder_offset_x_m"].as<double>()),
    node->declare_parameter("max_holder_offset_y_m", values["max_holder_offset_y_m"].as<double>()),
    node->declare_parameter("max_holder_offset_z_m", values["max_holder_offset_z_m"].as<double>()),
    node->declare_parameter("max_holder_tilt_x_deg", values["max_holder_tilt_x_deg"].as<double>()),
    node->declare_parameter("max_holder_tilt_y_deg", values["max_holder_tilt_y_deg"].as<double>()),
    static_cast<int>(node->declare_parameter<int64_t>(
      "max_candidate_attempts", values["max_candidate_attempts"].as<int64_t>())),
    node->declare_parameter(
      "validation_planning_timeout_sec",
      values["validation_planning_timeout_sec"].as<double>()),
    node->declare_parameter(
      "observation_distance_m",
      screen["observation_distance_m"].as<double>()),
    node->declare_parameter(
      "screen_validation_width_m", values["screen_validation_width_m"].as<double>()),
    node->declare_parameter(
      "screen_validation_height_m", values["screen_validation_height_m"].as<double>()),
    node->declare_parameter(
      "screen_validation_margin_m", values["screen_validation_margin_m"].as<double>()),
    node->declare_parameter(
      "screen_plane_offset_m",
      (sim ? screen["sim_screen_plane_offset_m"] : screen["screen_plane_offset_m"]).as<double>()),
    node->declare_parameter(
      "touch_approach_distance_m",
      (sim ? screen["sim_touch_approach_distance_m"] : screen["approach_distance_m"]).as<double>()),
    static_cast<int>(node->declare_parameter<int64_t>(
      "touch_path_samples", values["touch_path_samples"].as<int64_t>())),
    node->declare_parameter(
      "max_touch_jacobian_condition_number",
      values["max_touch_jacobian_condition_number"].as<double>()),
    node->declare_parameter(
      "calibration_offset_x_m", screen["calibration_offset_x_m"].as<double>()),
    node->declare_parameter(
      "calibration_offset_y_m", screen["calibration_offset_y_m"].as<double>())};
  if (limits.planning_mode != "dual_arm" && limits.planning_mode != "sequential" ||
    limits.offset_x_m < 0.0 || limits.offset_y_m < 0.0 || limits.offset_z_m < 0.0 ||
    limits.tilt_x_deg < 0.0 || limits.tilt_y_deg < 0.0 || limits.attempts <= 0 ||
    limits.planning_time_sec <= 0.0 || limits.camera_distance_m <= 0.0 ||
    limits.screen_width_m <= 0.0 || limits.screen_height_m <= 0.0 ||
    limits.screen_margin_m < 0.0 || limits.screen_margin_m * 2.0 >= limits.screen_width_m ||
    limits.screen_margin_m * 2.0 >= limits.screen_height_m ||
    !std::isfinite(limits.screen_plane_offset_m) || limits.screen_plane_offset_m < 0.0 ||
    limits.touch_approach_distance_m <= 0.0 || limits.touch_path_samples < 2 ||
    !std::isfinite(limits.max_touch_condition_number) ||
    limits.max_touch_condition_number <= 1.0 ||
    !std::isfinite(limits.calibration_x_m) || !std::isfinite(limits.calibration_y_m))
  {
    throw std::runtime_error(
            "Invalid random-pose or screen reachability validation limits");
  }
  return limits;
}

std::vector<Anchor> load_anchors(const std::string & path)
{
  const YAML::Node poses = YAML::LoadFile(path)["simulation"]["validated"];
  if (!poses || !poses.IsMap()) {
    throw std::runtime_error("Missing simulation.validated pose map");
  }
  std::vector<Anchor> anchors;
  for (const auto & pose : poses) {
    auto joints = pose.second.as<std::vector<double>>();
    if (joints.size() != 6) {
      throw std::runtime_error("Every validated pose must contain six joints");
    }
    for (const double joint : joints) {
      if (!std::isfinite(joint)) {
        throw std::runtime_error("Validated poses must contain finite joint values");
      }
    }
    anchors.emplace_back(pose.first.as<std::string>(), std::move(joints));
  }
  if (anchors.empty()) {
    throw std::runtime_error("No validated poses found");
  }
  return anchors;
}

geometry_msgs::msg::Transform to_transform(const Eigen::Isometry3d & value)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = value.translation().x();
  transform.translation.y = value.translation().y();
  transform.translation.z = value.translation().z();
  const Eigen::Quaterniond rotation(value.rotation());
  transform.rotation.x = rotation.x();
  transform.rotation.y = rotation.y();
  transform.rotation.z = rotation.z();
  transform.rotation.w = rotation.w();
  return transform;
}

double sample(std::mt19937 & generator, double maximum)
{
  return std::uniform_real_distribution<double>(-maximum, maximum)(generator);
}

struct ScreenPoint
{
  const char * name;
  double x;
  double y;
};

double jacobian_condition_number(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * joints,
  const moveit::core::LinkModel * tip)
{
  Eigen::MatrixXd jacobian;
  if (!state.getJacobian(joints, tip, Eigen::Vector3d::Zero(), jacobian)) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto singular_values = svd.singularValues();
  if (singular_values.size() == 0 || singular_values[singular_values.size() - 1] <= 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  return singular_values[0] / singular_values[singular_values.size() - 1];
}

bool validate_screen_reachability(
  const moveit::core::RobotState & aligned_state,
  const moveit::core::JointModelGroup * left_joints,
  const moveit::core::LinkModel * holder_link,
  const moveit::core::LinkModel * camera_link,
  const moveit::core::LinkModel * tool_link,
  const Limits & limits,
  const rclcpp::Logger & logger,
  int candidate_number,
  int candidate_count)
{
  const double half_width = limits.screen_width_m / 2.0 - limits.screen_margin_m;
  const double half_height = limits.screen_height_m / 2.0 - limits.screen_margin_m;
  const std::vector<ScreenPoint> points{
    {"upper-left", -half_width, half_height},
    {"upper-center", 0.0, half_height},
    {"upper-right", half_width, half_height},
    {"center-left", -half_width, 0.0},
    {"center", 0.0, 0.0},
    {"center-right", half_width, 0.0},
    {"lower-left", -half_width, -half_height},
    {"lower-center", 0.0, -half_height},
    {"lower-right", half_width, -half_height},
  };

  const Eigen::Isometry3d world_holder = aligned_state.getGlobalLinkTransform(holder_link);
  const Eigen::Isometry3d world_tool = aligned_state.getGlobalLinkTransform(tool_link);
  const Eigen::Isometry3d world_camera = aligned_state.getGlobalLinkTransform(camera_link);
  const Eigen::Isometry3d tool_camera = world_tool.inverse() * world_camera;
  const Eigen::Matrix3d holder_camera_rotation =
    Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d world_tool_rotation =
    world_holder.rotation() * holder_camera_rotation * tool_camera.rotation().transpose();

  double worst_condition = 0.0;
  const char * worst_point = "none";
  for (const auto & point : points) {
    moveit::core::RobotState touch_state(aligned_state);
    for (int sample_index = 0; sample_index < limits.touch_path_samples; ++sample_index) {
      const double fraction = static_cast<double>(sample_index) /
        static_cast<double>(limits.touch_path_samples - 1);
      const double z = limits.screen_plane_offset_m +
        limits.touch_approach_distance_m * (1.0 - fraction);
      Eigen::Isometry3d desired_tool = Eigen::Isometry3d::Identity();
      desired_tool.linear() = world_tool_rotation;
      desired_tool.translation() = world_holder * Eigen::Vector3d(
        point.x + limits.calibration_x_m,
        point.y + limits.calibration_y_m,
        z);
      if (!touch_state.setFromIK(left_joints, desired_tool, tool_link->getName(), 0.05)) {
        RCLCPP_INFO(
          logger,
          "Candidate %d/%d: screen %s IK failed at %.1f mm from the screen",
          candidate_number, candidate_count, point.name,
          limits.touch_approach_distance_m * (1.0 - fraction) * 1000.0);
        return false;
      }
      touch_state.update();
      if (!touch_state.satisfiesBounds(left_joints)) {
        RCLCPP_INFO(
          logger, "Candidate %d/%d: screen %s violates left-arm joint limits",
          candidate_number, candidate_count, point.name);
        return false;
      }
      const double condition = jacobian_condition_number(
        touch_state, left_joints, tool_link);
      if (condition > worst_condition) {
        worst_condition = condition;
        worst_point = point.name;
      }
      if (!std::isfinite(condition) || condition > limits.max_touch_condition_number) {
        RCLCPP_INFO(
          logger,
          "Candidate %d/%d: screen %s Jacobian condition %.2f exceeds %.2f "
          "at %.1f mm from the screen",
          candidate_number, candidate_count, point.name, condition,
          limits.max_touch_condition_number,
          limits.touch_approach_distance_m * (1.0 - fraction) * 1000.0);
        return false;
      }
    }
  }
  RCLCPP_INFO(
    logger,
    "Candidate %d/%d: %.0f x %.0f mm screen grid reachable; worst condition %.2f at %s",
    candidate_number, candidate_count,
    half_width * 2.0 * 1000.0, half_height * 2.0 * 1000.0,
    worst_condition, worst_point);
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("random_collab_pose");
  const auto logger = node->get_logger();
  const bool plan_only = node->declare_parameter<bool>("plan_only", false);

  const std::string share = ament_index_cpp::get_package_share_directory("denso_collab");
  Limits limits;
  std::vector<Anchor> anchors;
  try {
    limits = load_limits(node, share + "/config/cellphone_collab.yaml");
    anchors = load_anchors(share + "/config/right_holder_poses.yaml");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger, "Invalid collaboration config: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  if (!load_descriptions(node)) {
    RCLCPP_ERROR(logger, "Could not load MoveIt descriptions");
    rclcpp::shutdown();
    return 2;
  }

  auto right = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "right_arm");
  auto left = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "left_arm");
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> dual;
  for (const auto & group : {right, left}) {
    group->setPlanningTime(limits.planning_time_sec);
    group->setNumPlanningAttempts(1);
    group->setMaxVelocityScalingFactor(1.0);
    group->setMaxAccelerationScalingFactor(1.0);
  }
  if (limits.planning_mode == "dual_arm") {
    dual = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "dual_arm");
    dual->setPlanningTime(limits.planning_time_sec);
    dual->setNumPlanningAttempts(1);
    dual->setMaxVelocityScalingFactor(1.0);
    dual->setMaxAccelerationScalingFactor(1.0);
  }

  const auto model = right->getRobotModel();
  const auto * dual_joints = model->getJointModelGroup("dual_arm");
  const auto * right_joints = model->getJointModelGroup("right_arm");
  const auto * left_joints = model->getJointModelGroup("left_arm");
  constexpr const char * kHolderFrame = "right_cellphone_holder_tags_frame";
  constexpr const char * kCameraFrame = "left_camera_depth_optical_frame";
  constexpr const char * kToolFrame = "left_calib_link";
  if ((limits.planning_mode == "dual_arm" && !dual_joints) ||
    !right_joints || !left_joints || !model->hasLinkModel(kHolderFrame) ||
    !model->hasLinkModel(kCameraFrame) || !model->hasLinkModel(kToolFrame))
  {
    RCLCPP_ERROR(logger, "Required groups or tool links are missing from the robot model");
    rclcpp::shutdown();
    return 2;
  }

  std::map<std::string, double> joint_positions;
  const auto joint_states = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, [&joint_positions](sensor_msgs::msg::JointState::SharedPtr message) {
      for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
        joint_positions[message->name[i]] = message->position[i];
      }
    });
  const auto & required_variables = model->getVariableNames();
  for (int i = 0; i < 100 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    if (std::all_of(
        required_variables.begin(), required_variables.end(),
        [&joint_positions](const std::string & name) {return joint_positions.count(name) != 0;}))
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  (void)joint_states;
  if (!std::all_of(
      required_variables.begin(), required_variables.end(),
      [&joint_positions](const std::string & name) {return joint_positions.count(name) != 0;}))
  {
    RCLCPP_ERROR(logger, "Timed out waiting for all robot joints on /joint_states");
    rclcpp::shutdown();
    return 2;
  }
  moveit::core::RobotState current(model);
  current.setToDefaultValues();
  for (const auto & variable : required_variables) {
    current.setVariablePosition(variable, joint_positions.at(variable));
  }
  current.update();

  std::mt19937 generator(std::random_device{}());
  for (const auto & anchor : anchors) {
    moveit::core::RobotState anchor_state(current);
    anchor_state.setJointGroupPositions(right_joints, anchor.second);
    anchor_state.update();
    if (!anchor_state.satisfiesBounds(right_joints)) {
      RCLCPP_ERROR(
        logger, "Validated anchor '%s' violates right-arm joint limits",
        anchor.first.c_str());
      rclcpp::shutdown();
      return 2;
    }
  }
  std::uniform_int_distribution<std::size_t> anchor_distribution(0, anchors.size() - 1);

  moveit::planning_interface::MoveGroupInterface::Plan dual_plan;
  moveit::planning_interface::MoveGroupInterface::Plan right_plan;
  moveit::planning_interface::MoveGroupInterface::Plan left_plan;
  bool found = false;
  constexpr double kDegreesToRadians = std::acos(-1.0) / 180.0;
  for (int attempt = 1; attempt <= limits.attempts && rclcpp::ok(); ++attempt) {
    const Anchor & anchor = anchors[anchor_distribution(generator)];
    moveit::core::RobotState anchor_state(current);
    anchor_state.setJointGroupPositions(right_joints, anchor.second);
    anchor_state.update();
    const Eigen::Isometry3d anchor_holder = anchor_state.getGlobalLinkTransform(kHolderFrame);
    const double dx = sample(generator, limits.offset_x_m);
    const double dy = sample(generator, limits.offset_y_m);
    const double dz = sample(generator, limits.offset_z_m);
    const double tilt_x = sample(generator, limits.tilt_x_deg) * kDegreesToRadians;
    const double tilt_y = sample(generator, limits.tilt_y_deg) * kDegreesToRadians;

    Eigen::Isometry3d requested_holder = anchor_holder;
    requested_holder.translate(Eigen::Vector3d(dx, dy, dz));
    requested_holder.rotate(
      Eigen::AngleAxisd(tilt_x, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(tilt_y, Eigen::Vector3d::UnitY()));

    moveit::core::RobotState candidate(anchor_state);
    if (!candidate.setFromIK(right_joints, requested_holder, kHolderFrame, 0.05)) {
      RCLCPP_INFO(logger, "Candidate %d/%d: right-arm IK failed", attempt, limits.attempts);
      continue;
    }
    candidate.update();

    const auto holder = to_transform(candidate.getGlobalLinkTransform(kHolderFrame));
    const auto camera_pose = denso_collab::camera_pose_from_holder(
      holder, tf2::Vector3(0.0, 0.0, limits.camera_distance_m));
    if (!candidate.setFromIK(left_joints, camera_pose, kCameraFrame, 0.05)) {
      RCLCPP_INFO(logger, "Candidate %d/%d: left-camera IK failed", attempt, limits.attempts);
      continue;
    }
    candidate.update();
    if ((limits.planning_mode == "dual_arm" && !candidate.satisfiesBounds(dual_joints)) ||
      (limits.planning_mode == "sequential" &&
      (!candidate.satisfiesBounds(right_joints) || !candidate.satisfiesBounds(left_joints))))
    {
      RCLCPP_INFO(logger, "Candidate %d/%d: joint limits failed", attempt, limits.attempts);
      continue;
    }

    if (!validate_screen_reachability(
        candidate, left_joints, model->getLinkModel(kHolderFrame),
        model->getLinkModel(kCameraFrame), model->getLinkModel(kToolFrame),
        limits, logger, attempt, limits.attempts))
    {
      continue;
    }

    if (limits.planning_mode == "dual_arm") {
      dual->setStartState(current);
      dual->setJointValueTarget(candidate);
      if (dual->plan(dual_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(
          logger, "Candidate %d/%d: dual-arm collision-aware plan failed",
          attempt, limits.attempts);
        continue;
      }
      if (dual_plan.trajectory_.joint_trajectory.joint_names.size() !=
        dual_joints->getVariableCount())
      {
        RCLCPP_ERROR(logger, "Dual-arm plan does not contain all 12 joints");
        continue;
      }
    } else {
      right->setStartState(current);
      right->setJointValueTarget(candidate);
      if (right->plan(right_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(
          logger, "Candidate %d/%d: right-arm collision-aware plan failed",
          attempt, limits.attempts);
        continue;
      }
      left->setStartState(candidate);
      if (!left->setPoseTarget(camera_pose, kCameraFrame) ||
        left->plan(left_plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        left->clearPoseTargets();
        RCLCPP_INFO(
          logger, "Candidate %d/%d: left-camera collision-aware plan failed",
          attempt, limits.attempts);
        continue;
      }
      left->clearPoseTargets();
    }
    RCLCPP_INFO(
      logger,
      "Candidate %d/%d valid from anchor %s: offset [%.3f %.3f %.3f] m, "
      "tilt [%.2f %.2f] deg",
      attempt, limits.attempts, anchor.first.c_str(), dx, dy, dz,
      tilt_x / kDegreesToRadians, tilt_y / kDegreesToRadians);
    found = true;
    break;
  }

  if (!found) {
    RCLCPP_ERROR(
      logger, "No valid candidate found after %d attempts; no robot moved",
      limits.attempts);
    rclcpp::shutdown();
    return 1;
  }

  if (plan_only) {
    RCLCPP_INFO(logger, "Random pose and complete screen reachability validated; plan only");
    rclcpp::shutdown();
    return 0;
  }

  if (limits.planning_mode == "dual_arm") {
    RCLCPP_INFO(logger, "Combined hypothetical plan succeeded; executing both arms together");
    if (dual->execute(dual_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Dual-arm execution failed");
      rclcpp::shutdown();
      return 1;
    }
  } else {
    RCLCPP_INFO(logger, "Both hypothetical plans succeeded; executing right arm, then left alignment");
    if (right->execute(right_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Right-arm execution failed; left arm was not moved");
      rclcpp::shutdown();
      return 1;
    }
    if (left->execute(left_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Left-arm alignment execution failed");
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(logger, "One random collaboration pose completed");
  rclcpp::shutdown();
  return 0;
}
