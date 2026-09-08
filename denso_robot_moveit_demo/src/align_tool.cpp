#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{

struct Options
{
  std::string move_group_node = "/move_group";
  std::string planning_group = "left_arm";
  std::string target_frame = "right_cellphone_holder_phone_plane_frame";
  std::string camera_frame = "left_basic_camera_link";
  double distance_m = 0.3;  // Deprecated: kept as default for target_offset_z_m.
  double target_offset_x_m = 0.0;
  double target_offset_y_m = 0.0;
  double target_offset_z_m = 0.3;
  double target_roll_deg = 0.0;
  double target_pitch_deg = 0.0;
  double target_yaw_deg = 180.0;
  double planning_time = 8.0;
  int num_planning_attempts = 10;
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
  double startup_delay_sec = 3.0;
  double tf_timeout_sec = 8.0;
  bool plan_only = false;
};

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

  // A freshly-created DDS service client can discover the request endpoint before its
  // response endpoint is fully matched.  The descriptions are large enough that the first
  // response may then time out, especially when align_tool is spawned between robot poses.
  // Give discovery a brief settling period and retry transient response failures.
  constexpr int kMaxAttempts = 3;
  std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  for (int attempt = 1; attempt <= kMaxAttempts; ++attempt) {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description", "robot_description_semantic"};
    auto future = client->async_send_request(request);

    const auto rc = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));
    if (rc == rclcpp::FutureReturnCode::SUCCESS) {
      response = future.get();
      break;
    }

    client->remove_pending_request(future);
    RCLCPP_WARN(
      node->get_logger(), "Attempt %d/%d to query %s timed out",
      attempt, kMaxAttempts, service_name.c_str());
    if (attempt < kMaxAttempts) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  if (!response) {
    error = "Failed to query " + service_name + " after " +
      std::to_string(kMaxAttempts) + " attempts";
    return false;
  }

  if (response->values.size() != 2) {
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

geometry_msgs::msg::Pose build_camera_pose_from_target_tf(
  const geometry_msgs::msg::TransformStamped & target_tf,
  const tf2::Vector3 & target_offset_m,
  const tf2::Vector3 & target_rpy_deg)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto deg_to_rad = [](double deg) { return deg * kPi / 180.0; };

  tf2::Quaternion q_target;
  tf2::fromMsg(target_tf.transform.rotation, q_target);
  q_target.normalize();

  const tf2::Vector3 p_target_in_planning(
    target_tf.transform.translation.x,
    target_tf.transform.translation.y,
    target_tf.transform.translation.z);
  const tf2::Transform t_planning_target(q_target, p_target_in_planning);

  tf2::Quaternion q_target_camera;
  q_target_camera.setRPY(
    deg_to_rad(target_rpy_deg.x()),
    deg_to_rad(target_rpy_deg.y()),
    deg_to_rad(target_rpy_deg.z()));
  q_target_camera.normalize();

  const tf2::Transform t_target_camera(q_target_camera, target_offset_m);

  // Homogeneous transform composition:
  //   T_planning_camera = T_planning_target * T_target_camera
  const tf2::Transform t_planning_camera = t_planning_target * t_target_camera;

  geometry_msgs::msg::Pose pose;
  pose.position.x = t_planning_camera.getOrigin().x();
  pose.position.y = t_planning_camera.getOrigin().y();
  pose.position.z = t_planning_camera.getOrigin().z();
  pose.orientation = tf2::toMsg(t_planning_camera.getRotation());
  return pose;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("align_tool");
  const auto logger = node->get_logger();

  Options options;
  options.move_group_node = node->declare_parameter<std::string>("move_group_node", options.move_group_node);
  options.planning_group = node->declare_parameter<std::string>("planning_group", options.planning_group);
  options.target_frame = node->declare_parameter<std::string>("target_frame", options.target_frame);
  options.camera_frame = node->declare_parameter<std::string>("camera_frame", options.camera_frame);
  options.distance_m = node->declare_parameter<double>("distance_m", options.distance_m);
  options.target_offset_x_m = node->declare_parameter<double>("target_offset_x_m", options.target_offset_x_m);
  options.target_offset_y_m = node->declare_parameter<double>("target_offset_y_m", options.target_offset_y_m);
  options.target_offset_z_m = node->declare_parameter<double>("target_offset_z_m", options.distance_m);
  options.target_roll_deg = node->declare_parameter<double>("target_roll_deg", options.target_roll_deg);
  options.target_pitch_deg = node->declare_parameter<double>("target_pitch_deg", options.target_pitch_deg);
  options.target_yaw_deg = node->declare_parameter<double>("target_yaw_deg", options.target_yaw_deg);
  options.planning_time = node->declare_parameter<double>("planning_time", options.planning_time);
  options.num_planning_attempts =
    node->declare_parameter<int>("num_planning_attempts", options.num_planning_attempts);
  options.velocity_scaling = node->declare_parameter<double>("velocity_scaling", options.velocity_scaling);
  options.acceleration_scaling =
    node->declare_parameter<double>("acceleration_scaling", options.acceleration_scaling);
  options.startup_delay_sec = node->declare_parameter<double>("startup_delay_sec", options.startup_delay_sec);
  options.tf_timeout_sec = node->declare_parameter<double>("tf_timeout_sec", options.tf_timeout_sec);
  options.plan_only = node->declare_parameter<bool>("plan_only", options.plan_only);

  if (options.planning_group.empty() || options.target_frame.empty() || options.camera_frame.empty()) {
    RCLCPP_ERROR(logger, "planning_group, target_frame and camera_frame must be non-empty");
    rclcpp::shutdown();
    return 2;
  }
  if (options.planning_time <= 0.0 || options.num_planning_attempts <= 0) {
    RCLCPP_ERROR(logger, "planning_time and num_planning_attempts must be > 0");
    rclcpp::shutdown();
    return 2;
  }
  if (
    options.velocity_scaling < 0.0 || options.velocity_scaling > 1.0 ||
    options.acceleration_scaling < 0.0 || options.acceleration_scaling > 1.0)
  {
    RCLCPP_ERROR(logger, "velocity_scaling and acceleration_scaling must be in [0, 1]");
    rclcpp::shutdown();
    return 2;
  }

  std::string description_error;
  if (!load_descriptions_from_move_group(node, options.move_group_node, description_error)) {
    RCLCPP_ERROR(logger, "Failed to load robot descriptions from move_group: %s", description_error.c_str());
    rclcpp::shutdown();
    return 2;
  }

  if (options.startup_delay_sec > 0.0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(options.startup_delay_sec));
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  try {
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, options.planning_group);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to construct MoveGroupInterface: %s", ex.what());
    rclcpp::shutdown();
    return 2;
  }

  move_group->setPlanningTime(options.planning_time);
  move_group->setNumPlanningAttempts(options.num_planning_attempts);
  move_group->setMaxVelocityScalingFactor(options.velocity_scaling);
  move_group->setMaxAccelerationScalingFactor(options.acceleration_scaling);

  const auto groups = move_group->getJointModelGroupNames();
  if (std::find(groups.begin(), groups.end(), options.planning_group) == groups.end()) {
    RCLCPP_ERROR(logger, "Planning group '%s' not found in robot model", options.planning_group.c_str());
    rclcpp::shutdown();
    return 2;
  }

  const auto robot_model = move_group->getRobotModel();
  if (!robot_model->hasLinkModel(options.camera_frame)) {
    RCLCPP_ERROR(
      logger, "Camera frame '%s' is not a link in the robot model; cannot use as pose target link",
      options.camera_frame.c_str());
    rclcpp::shutdown();
    return 2;
  }

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::msg::TransformStamped target_in_planning;
  const std::string planning_frame = move_group->getPlanningFrame();
  try {
    target_in_planning = tf_buffer.lookupTransform(
      planning_frame, options.target_frame, tf2::TimePointZero,
      tf2::durationFromSec(options.tf_timeout_sec));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      logger,
      "Failed to lookup TF from '%s' to '%s': %s",
      planning_frame.c_str(), options.target_frame.c_str(), ex.what());
    rclcpp::shutdown();
    return 2;
  } 
  RCLCPP_INFO(logger, "Target frame '%s' found in planning frame '%s'", options.target_frame.c_str(), planning_frame.c_str());

  const tf2::Vector3 target_offset_m(
    options.target_offset_x_m,
    options.target_offset_y_m,
    options.target_offset_z_m);
  const tf2::Vector3 target_rpy_deg(
    options.target_roll_deg,
    options.target_pitch_deg,
    options.target_yaw_deg);

  const geometry_msgs::msg::Pose camera_target_pose =
    build_camera_pose_from_target_tf(target_in_planning, target_offset_m, target_rpy_deg);

  RCLCPP_INFO(logger, "Planning frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "Planning group: %s", options.planning_group.c_str());
  RCLCPP_INFO(logger, "Target frame: %s", options.target_frame.c_str());
  RCLCPP_INFO(logger, "Camera link target: %s", options.camera_frame.c_str());
  RCLCPP_INFO(
    logger,
    "Target offset in target frame [x y z] m: [%.6f %.6f %.6f]",
    options.target_offset_x_m, options.target_offset_y_m, options.target_offset_z_m);
  RCLCPP_INFO(
    logger,
    "Target orientation in target frame [roll pitch yaw] deg: [%.3f %.3f %.3f]",
    options.target_roll_deg, options.target_pitch_deg, options.target_yaw_deg);
  RCLCPP_INFO(
    logger,
    "Transform model: T_planning_camera = T_planning_target * T_target_camera");
  RCLCPP_INFO(
    logger,
    "Computed target pose (xyz xyzw): [%.6f %.6f %.6f] [%.6f %.6f %.6f %.6f]",
    camera_target_pose.position.x, camera_target_pose.position.y, camera_target_pose.position.z,
    camera_target_pose.orientation.x, camera_target_pose.orientation.y,
    camera_target_pose.orientation.z, camera_target_pose.orientation.w);

  if (!move_group->setPoseTarget(camera_target_pose, options.camera_frame)) {
    RCLCPP_ERROR(
      logger,
      "MoveIt rejected pose target for link '%s' in group '%s'.",
      options.camera_frame.c_str(), options.planning_group.c_str());
    move_group->clearPoseTargets();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group->plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed");
    move_group->clearPoseTargets();
    rclcpp::shutdown();
    return 1;
  }

  const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
  if (!joint_trajectory.points.empty() && joint_trajectory.points.back().positions.size() >= 6) {
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    const auto & endpoint = joint_trajectory.points.back().positions;
    RCLCPP_INFO(
      logger,
      "Planned endpoint [J1..J6] deg: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
      endpoint[0] * kRadToDeg, endpoint[1] * kRadToDeg, endpoint[2] * kRadToDeg,
      endpoint[3] * kRadToDeg, endpoint[4] * kRadToDeg, endpoint[5] * kRadToDeg);
  }

  if (!options.plan_only) {
    const auto exec_result = move_group->execute(plan);
    move_group->stop();
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Execution failed");
      move_group->clearPoseTargets();
      rclcpp::shutdown();
      return 1;
    }
  }

  move_group->clearPoseTargets();
  RCLCPP_INFO(logger, "Alignment completed");
  rclcpp::shutdown();
  return 0;
}
