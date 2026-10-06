#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{

struct Target
{
  std::string color;
  double pixel_x;
  double pixel_y;
  double radius_px;
  geometry_msgs::msg::PointStamped point;
};

bool load_descriptions_from_move_group(
  const rclcpp::Node::SharedPtr & node, const std::string & move_group_node)
{
  const auto client = node->create_client<rcl_interfaces::srv::GetParameters>(
    move_group_node + "/get_parameters");
  if (!client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for %s/get_parameters", move_group_node.c_str());
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> response;
  for (int attempt = 1; attempt <= 3; ++attempt) {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = {"robot_description", "robot_description_semantic"};
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      response = future.get();
      break;
    }
    client->remove_pending_request(future);
    RCLCPP_WARN(node->get_logger(), "move_group description request %d/3 timed out", attempt);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (!response) {
    RCLCPP_ERROR(node->get_logger(), "Could not load robot descriptions from %s", move_group_node.c_str());
    return false;
  }

  if (response->values.size() != 2 ||
    response->values[0].type != rclcpp::PARAMETER_STRING ||
    response->values[1].type != rclcpp::PARAMETER_STRING ||
    response->values[0].string_value.empty() || response->values[1].string_value.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "move_group returned an incomplete robot description");
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
      key + "solution_weights", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    node->declare_parameter(key + "kinematics_solver_search_resolution", 0.005);
    node->declare_parameter(key + "kinematics_solver_timeout", 0.05);
    node->declare_parameter(key + "kinematics_solver_attempts", 1);
  }
  return true;
}

class ScreenStandoffMover : public rclcpp::Node
{
public:
  ScreenStandoffMover()
  : Node("move_to_screen_standoff")
  {
    move_group_node_ = declare_parameter<std::string>("move_group_node", "/move_group");
    planning_group_ = declare_parameter<std::string>("planning_group", "left_arm");
    end_effector_link_ = declare_parameter<std::string>("tool_link", "left_calib_link");
    target_link_ = declare_parameter<std::string>("target_link", "left_camera_depth_optical_frame");
    holder_frame_ = declare_parameter<std::string>("holder_frame", "right_cellphone_holder_tags_frame");
    approach_distance_m_ = declare_parameter<double>("approach_distance_m");
    sim_touch_approach_distance_m_ = declare_parameter<double>("sim_touch_approach_distance_m", 0.015);
    screen_plane_offset_m_ = declare_parameter<double>("screen_plane_offset_m", 0.0045);
    const double sim_screen_plane_offset_m =
      declare_parameter<double>("sim_screen_plane_offset_m", 0.008);
    sim_ = declare_parameter<bool>("sim", false);
    observation_distance_m_ = declare_parameter<double>("observation_distance_m");
    align_before_each_target_ = declare_parameter<bool>("align_before_each_target");
    planning_time_ = declare_parameter<double>("planning_time");
    attempts_ = declare_parameter<int>("num_planning_attempts");
    velocity_ = declare_parameter<double>("velocity_scaling");
    acceleration_ = declare_parameter<double>("acceleration_scaling");
    detection_timeout_sec_ = declare_parameter<double>("detection_timeout_sec");
    touch_enabled_ = declare_parameter<bool>("touch_enabled", false);
    if (sim_) {
      screen_plane_offset_m_ = sim_screen_plane_offset_m;
      if (touch_enabled_) {
        approach_distance_m_ = sim_touch_approach_distance_m_;
      }
    }
    touch_speed_mps_ = declare_parameter<double>("touch_speed_mps", 0.005);
    touch_retract_speed_mps_ = declare_parameter<double>("touch_retract_speed_mps", 0.005);
    touch_retract_distance_m_ = declare_parameter<double>("touch_retract_distance_m", 0.005);
    touch_dwell_sec_ = declare_parameter<double>("touch_dwell_sec", 0.0);
    touch_max_travel_m_ = declare_parameter<double>("touch_max_travel_m", 0.035);
    touch_timeout_sec_ = declare_parameter<double>("touch_timeout_sec", 10.0);
    max_touch_xy_error_m_ = declare_parameter<double>("max_touch_xy_error_m", 0.005);
    if (!std::isfinite(approach_distance_m_) || approach_distance_m_ <= 0.0 ||
      !std::isfinite(observation_distance_m_) || observation_distance_m_ <= 0.0 ||
      !std::isfinite(planning_time_) || planning_time_ <= 0.0 || attempts_ < 1 ||
      !std::isfinite(velocity_) || velocity_ <= 0.0 || velocity_ > 1.0 ||
      !std::isfinite(acceleration_) || acceleration_ <= 0.0 || acceleration_ > 1.0 ||
      !std::isfinite(detection_timeout_sec_) || detection_timeout_sec_ <= 0.0 ||
      !std::isfinite(touch_speed_mps_) || touch_speed_mps_ <= 0.0 ||
      touch_speed_mps_ > (sim_ ? 0.20 : 0.20) ||
      !std::isfinite(touch_retract_speed_mps_) || touch_retract_speed_mps_ <= 0.0 ||
      touch_retract_speed_mps_ > (sim_ ? 0.40 : 0.40) ||
      !std::isfinite(touch_retract_distance_m_) || touch_retract_distance_m_ <= 0.0 ||
      touch_retract_distance_m_ > 0.05 || !std::isfinite(touch_dwell_sec_) ||
      touch_dwell_sec_ < 0.0 || touch_dwell_sec_ > 5.0 ||
      !std::isfinite(touch_max_travel_m_) || touch_max_travel_m_ <= 0.0 ||
      touch_max_travel_m_ > 0.05 || !std::isfinite(touch_timeout_sec_) ||
      touch_timeout_sec_ <= 0.0 || touch_timeout_sec_ > 30.0 ||
      !std::isfinite(sim_touch_approach_distance_m_) || sim_touch_approach_distance_m_ <= 0.0 ||
      !std::isfinite(screen_plane_offset_m_) || screen_plane_offset_m_ < 0.0 ||
      screen_plane_offset_m_ > 0.03 ||
      !std::isfinite(max_touch_xy_error_m_) || max_touch_xy_error_m_ <= 0.0 ||
      max_touch_xy_error_m_ > 0.02)
    {
      throw std::runtime_error("Invalid screen approach or touch limit");
    }
    plan_only_ = declare_parameter<bool>("plan_only", false);
    calibration_x_m_ = declare_parameter<double>("calibration_offset_x_m", 0.0);
    calibration_y_m_ = declare_parameter<double>("calibration_offset_y_m", 0.0);
    if (!std::isfinite(calibration_x_m_) || !std::isfinite(calibration_y_m_) ||
      std::abs(calibration_x_m_) > 0.02 || std::abs(calibration_y_m_) > 0.02)
    {
      throw std::runtime_error("Calibration offsets must be within +/-20 mm");
    }
  }

  void initialize()
  {
    if (!load_descriptions_from_move_group(shared_from_this(), move_group_node_)) {
      throw std::runtime_error("Cannot initialize MoveIt");
    }
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), planning_group_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(attempts_);
    move_group_->setMaxVelocityScalingFactor(velocity_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_);
    constexpr double kPi = 3.14159265358979323846;
    target_orientation_.setRPY(
      kPi, 0.0, 0.0);
    target_orientation_.normalize();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "screen_standoff_goal", 10);
    active_joints_ = move_group_->getActiveJoints();
    joint_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions joint_options;
    joint_options.callback_group = joint_group_;
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ScreenStandoffMover::receive_joint_state, this, std::placeholders::_1),
      joint_options);
    if (touch_enabled_ && !plan_only_) {
      touch_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions options;
      options.callback_group = touch_group_;
      touch_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "/touch_detected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(servo_command_mutex_);
          const bool was_touching = touch_detected_.exchange(msg->data);
          last_touch_ms_.store(steady_ms());
          if (msg->data && !was_touching) {
            geometry_msgs::msg::TwistStamped stop;
            stop.header.stamp = now();
            stop.header.frame_id = "world";
            servo_publisher_->publish(stop);
          }
        }, options);
      servo_status_subscription_ = create_subscription<std_msgs::msg::Int8>(
        "/left_servo_node/status", 10,
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          servo_status_.store(msg->data);
          last_servo_status_ms_.store(steady_ms());
        },
        options);
      servo_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/left_servo_node/delta_twist_cmds", 10);
      servo_start_client_ = create_client<std_srvs::srv::Trigger>(
        "/left_servo_node/start_servo", rmw_qos_profile_services_default, touch_group_);
      servo_pause_client_ = create_client<std_srvs::srv::Trigger>(
        "/left_servo_node/pause_servo", rmw_qos_profile_services_default, touch_group_);
      servo_unpause_client_ = create_client<std_srvs::srv::Trigger>(
        "/left_servo_node/unpause_servo", rmw_qos_profile_services_default, touch_group_);
    }
  }

  bool succeeded() const {return succeeded_;}

private:
  static int64_t steady_ms()
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool set_servo_paused(bool paused)
  {
    const auto & client = paused ? servo_pause_client_ : servo_unpause_client_;
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "MoveIt Servo %s service is unavailable",
        paused ? "pause" : "unpause");
      return false;
    }
    auto future = client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready ||
      !future.get()->success)
    {
      RCLCPP_ERROR(get_logger(), "MoveIt Servo %s failed", paused ? "pause" : "unpause");
      return false;
    }
    return true;
  }

  void finish(bool success)
  {
    succeeded_ = success;
    rclcpp::shutdown();
  }

  void receive_joint_state(const sensor_msgs::msg::JointState::SharedPtr state)
  {
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      for (std::size_t index = 0; index < state->name.size() && index < state->position.size(); ++index) {
        current_joints_[state->name[index]] = state->position[index];
      }
      last_joint_ms_.store(steady_ms());
      if (ready_) {
        return;
      }
      for (const auto & joint : active_joints_) {
        if (current_joints_.find(joint) == current_joints_.end()) {
          return;
        }
      }
      initial_joints_.clear();
      for (const auto & joint : active_joints_) {
        initial_joints_.push_back(current_joints_[joint]);
      }
      planned_joints_ = initial_joints_;
      ready_ = true;
    }
    if (align_to_holder()) {
      aligned_ = true;
      // Subscribe only after alignment so queued pre-move detections cannot be used.
      targets_subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "detected_targets", 10,
        std::bind(&ScreenStandoffMover::receive_targets, this, std::placeholders::_1));
      detection_timer_ = create_wall_timer(
        std::chrono::duration<double>(detection_timeout_sec_), [this]() {
          if (!started_) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for stable RGB targets");
            finish(false);
          }
        });
      RCLCPP_INFO(get_logger(), "Camera aligned; waiting for stable RGB targets");
    } else {
      finish(false);
    }
  }

  void receive_targets(const std_msgs::msg::Float64MultiArray::SharedPtr message)
  {
    if (!ready_ || !aligned_ || started_) {
      return;
    }
    if (message->layout.dim.size() != 1 ||
      message->layout.dim[0].label != holder_frame_)
    {
      RCLCPP_ERROR(get_logger(), "Detected target frame does not match %s", holder_frame_.c_str());
      return;
    }
    // Each row is [RGB index, pixel x, pixel y, radius px, holder x m, holder y m].
    if (message->data.empty() || message->data.size() % 18 != 0 ||
      message->layout.dim[0].size != message->data.size() / 6)
    {
      RCLCPP_ERROR(get_logger(), "Detected targets do not form complete RGB trios");
      return;
    }
    const std::vector<std::string> colors{"red", "green", "blue"};
    std::vector<Target> incoming;
    std::vector<int> counts(3, 0);
    for (std::size_t index = 0; index < message->data.size(); index += 6) {
      const auto * row = &message->data[index];
      if (!std::all_of(row, row + 6, [](double value) {return std::isfinite(value);}) ||
        row[0] < 0.0 || row[0] > 2.0 || row[0] != std::floor(row[0]) || row[3] <= 0.0)
      {
        RCLCPP_ERROR(get_logger(), "Invalid target entry in detected_targets");
        return;
      }
      const auto color = static_cast<std::size_t>(row[0]);
      ++counts[color];
      Target target{colors[color], row[1], row[2], row[3],
        geometry_msgs::msg::PointStamped()};
      target.point.header.frame_id = holder_frame_;
      target.point.point.x = row[4];
      target.point.point.y = row[5];
      incoming.push_back(target);
    }
    if (counts[0] != counts[1] || counts[1] != counts[2]) {
      RCLCPP_ERROR(get_logger(), "Detected RGB counts are unequal: red=%d green=%d blue=%d",
        counts[0], counts[1], counts[2]);
      return;
    }
    std::sort(incoming.begin(), incoming.end(), [](const Target & a, const Target & b) {
      return a.pixel_y == b.pixel_y ? a.pixel_x < b.pixel_x : a.pixel_y < b.pixel_y;
    });
    targets_ = std::move(incoming);
    for (std::size_t index = 0; index < targets_.size(); ++index) {
      const auto & target = targets_[index];
      RCLCPP_INFO(get_logger(), "Target %zu/%zu: %s pixel=(%.1f, %.1f) r=%.1f holder=(%.4f, %.4f)",
        index + 1, targets_.size(), target.color.c_str(), target.pixel_x,
        target.pixel_y, target.radius_px, target.point.point.x, target.point.point.y);
    }
    started_ = true;
    run_sequence();
  }

  geometry_msgs::msg::PoseStamped approach_pose(
    const geometry_msgs::msg::PointStamped & point) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = point.header;
    pose.pose.position.x = point.point.x;
    pose.pose.position.y = point.point.y;
    pose.pose.position.z = screen_plane_offset_m_ + approach_distance_m_;
    return pose;
  }

  bool plan_and_execute()
  {
    moveit_msgs::msg::RobotState start_state;
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      for (const auto & [joint, position] : current_joints_) {
        start_state.joint_state.name.push_back(joint);
        const auto active = std::find(active_joints_.begin(), active_joints_.end(), joint);
        start_state.joint_state.position.push_back(
          active == active_joints_.end() ? position : planned_joints_[active - active_joints_.begin()]);
      }
    }
    move_group_->setStartState(start_state);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      move_group_->clearPoseTargets();
      return false;
    }
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    auto next_joints = planned_joints_;
    if (!trajectory.points.empty()) {
      const auto & endpoint = trajectory.points.back().positions;
      for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        const auto joint = std::find(
          active_joints_.begin(), active_joints_.end(), trajectory.joint_names[index]);
        if (joint != active_joints_.end() && index < endpoint.size()) {
          next_joints[std::distance(active_joints_.begin(), joint)] = endpoint[index];
        }
      }
    }
    if (!plan_only_ && move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed");
      move_group_->stop();
      move_group_->clearPoseTargets();
      return false;
    }
    planned_joints_ = std::move(next_joints);
    move_group_->stop();
    move_group_->clearPoseTargets();
    return true;
  }

  bool move_to(const std::string & color, const geometry_msgs::msg::PointStamped & point)
  {
    auto calib_pose = approach_pose(point);
    calib_pose.pose.position.x += calibration_x_m_;
    calib_pose.pose.position.y += calibration_y_m_;
    geometry_msgs::msg::TransformStamped calib_to_target;
    try {
      calib_to_target = tf_buffer_->lookupTransform(
        end_effector_link_, target_link_, tf2::TimePointZero,
        tf2::durationFromSec(2.0));
    } catch (const tf2::TransformException & error) {
      RCLCPP_ERROR(get_logger(), "Cannot transform %s to %s: %s", end_effector_link_.c_str(),
        target_link_.c_str(), error.what());
      return false;
    }
    tf2::Transform calib_target;
    tf2::fromMsg(calib_to_target.transform, calib_target);
    tf2::Transform holder_calib(
      target_orientation_ * calib_target.inverse().getRotation(),
      tf2::Vector3(
        calib_pose.pose.position.x,
        calib_pose.pose.position.y,
        calib_pose.pose.position.z));
    const auto calib_transform = tf2::toMsg(holder_calib);
    calib_pose.pose.position.x = calib_transform.translation.x;
    calib_pose.pose.position.y = calib_transform.translation.y;
    calib_pose.pose.position.z = calib_transform.translation.z;
    calib_pose.pose.orientation = calib_transform.rotation;
    goal_publisher_->publish(calib_pose);
    const auto target_transform = tf2::toMsg(holder_calib * calib_target);
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = target_transform.translation.x;
    target_pose.position.y = target_transform.translation.y;
    target_pose.position.z = target_transform.translation.z;
    target_pose.orientation = target_transform.rotation;

    move_group_->setPoseReferenceFrame(calib_pose.header.frame_id);
    if (!move_group_->setPoseTarget(target_pose, target_link_)) {
      RCLCPP_ERROR(get_logger(), "MoveIt rejected the %s standoff pose", color.c_str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Moving to %s at %.0f mm from screen (screen plane %.1f mm from tags)",
      color.c_str(), approach_distance_m_ * 1000.0, screen_plane_offset_m_ * 1000.0);
    return plan_and_execute();
  }

  bool align_to_holder()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.z = observation_distance_m_;
    pose.orientation = tf2::toMsg(target_orientation_);
    move_group_->setPoseReferenceFrame(holder_frame_);
    if (!move_group_->setPoseTarget(pose, target_link_)) {
      RCLCPP_ERROR(get_logger(), "MoveIt rejected the holder alignment pose");
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "Aligning camera %.0f mm from %s", observation_distance_m_ * 1000.0,
      holder_frame_.c_str());
    return plan_and_execute();
  }

  bool touch_screen(const Target & target)
  {
    const auto & color = target.color;
    if (!touch_enabled_ || plan_only_) {
      RCLCPP_INFO(get_logger(), "%s reached; touch disabled", color.c_str());
      return true;
    }
    if (steady_ms() - last_touch_ms_.load() > 500 || touch_detected_.load()) {
      RCLCPP_ERROR(get_logger(), "Touch signal is missing, stale, or already active");
      return false;
    }
    if (!servo_started_) {
      if (!servo_start_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_logger(), "MoveIt Servo is unavailable; bring up use_servo:=true");
        return false;
      }
      auto future = servo_start_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready ||
        !future.get()->success)
      {
        RCLCPP_ERROR(get_logger(), "Could not start MoveIt Servo");
        return false;
      }
      servo_started_ = true;
    } else if (!set_servo_paused(false)) {
      return false;
    }

    tf2::Vector3 direction;
    double start_z;
    try {
      tf2::Transform world_holder;
      tf2::fromMsg(tf_buffer_->lookupTransform("world", holder_frame_, tf2::TimePointZero,
        tf2::durationFromSec(1.0)).transform, world_holder);
      direction = world_holder.getBasis() * tf2::Vector3(0.0, 0.0, -1.0);
      start_z = tf_buffer_->lookupTransform(holder_frame_, end_effector_link_,
        tf2::TimePointZero, tf2::durationFromSec(1.0)).transform.translation.z;
    } catch (const tf2::TransformException & error) {
      set_servo_paused(true);
      RCLCPP_ERROR(get_logger(), "Cannot establish touch direction: %s", error.what());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Touch %s: tip starts %.1f mm from holder plane",
      color.c_str(), start_z * 1000.0);

    const auto command = [this, &direction](double speed_mps) {
        std::lock_guard<std::mutex> lock(servo_command_mutex_);
        if (speed_mps > 0.0 && touch_detected_.load()) {
          speed_mps = 0.0;
        }
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = now();
        twist.header.frame_id = "world";
        twist.twist.linear.x = speed_mps * direction.x();
        twist.twist.linear.y = speed_mps * direction.y();
        twist.twist.linear.z = speed_mps * direction.z();
        servo_publisher_->publish(twist);
      };
    constexpr auto servo_command_period = std::chrono::milliseconds(10);
    const auto halt = [&command, servo_command_period]() {
        // Match Servo's 100 Hz publish period. The touch callback already sends
        // the first zero command immediately, then these four commands settle
        // the stop for 40 ms before reversing into the retract.
        for (int count = 0; count < 4; ++count) {
          command(0.0);
          std::this_thread::sleep_for(servo_command_period);
        }
      };
    const auto tip_z = [this]() {
        return tf_buffer_->lookupTransform(holder_frame_, end_effector_link_,
          tf2::TimePointZero, tf2::durationFromSec(0.2)).transform.translation.z;
      };

    bool touched = false;
    bool contact_seen = false;
    const auto started_ms = steady_ms();
    const auto deadline = started_ms + static_cast<int64_t>(touch_timeout_sec_ * 1000.0);
    const auto safe_to_move = [this, &tip_z, start_z, started_ms]() {
        const auto status = servo_status_.load();
        if (status == 2 || status == 4 || status == 5) {
          RCLCPP_ERROR(get_logger(), "Touch stopped by MoveIt Servo status %d", status);
          return false;
        }
        if (steady_ms() - started_ms > 500 &&
          steady_ms() - last_servo_status_ms_.load() > 500)
        {
          RCLCPP_ERROR(get_logger(), "Touch stopped: Servo status is stale");
          return false;
        }
        if (steady_ms() - last_touch_ms_.load() > 500) {
          RCLCPP_ERROR(get_logger(), "Touch stopped: contact signal is stale");
          return false;
        }
        if (start_z - tip_z() > touch_max_travel_m_) {
          RCLCPP_ERROR(get_logger(), "Touch stopped: travel limit reached");
          return false;
        }
        return true;
      };
    try {
      while (rclcpp::ok() && steady_ms() < deadline) {
        if (touch_detected_.load()) {
          contact_seen = true;
          halt();  // Stop the advance before any TF lookup or contact logging.
          if (steady_ms() - last_touch_ms_.load() > 500) {
            RCLCPP_ERROR(get_logger(), "Touch stopped: contact signal is stale");
            break;
          }
          const auto tip = tf_buffer_->lookupTransform(holder_frame_, end_effector_link_,
            tf2::TimePointZero, tf2::durationFromSec(0.2)).transform.translation;
          const double error = std::hypot(
            tip.x - target.point.point.x, tip.y - target.point.point.y);
          RCLCPP_INFO(get_logger(), "Touch XY error: %.1f mm", error * 1000.0);
          touched = error <= max_touch_xy_error_m_;
          break;
        }
        if (!safe_to_move()) {
          break;
        }
        command(touch_speed_mps_);
        std::this_thread::sleep_for(servo_command_period);
      }
      if (!contact_seen) {
        halt();
      }
      RCLCPP_INFO(get_logger(), "Touch probe: tip %.1f mm from holder plane, contact=%s",
        tip_z() * 1000.0, touched ? "true" : "false");
      if (contact_seen) {
        const double contact_z = tip_z();
        const double retract_target_z = std::max(
          start_z, contact_z + touch_retract_distance_m_);
        bool retract_ok = true;
        const auto dwell_deadline = steady_ms() + static_cast<int64_t>(touch_dwell_sec_ * 1000.0);
        while (touched && rclcpp::ok() && steady_ms() < dwell_deadline) {
          if (!safe_to_move()) {
            retract_ok = false;
            break;
          }
          command(0.0);
          std::this_thread::sleep_for(servo_command_period);
        }
        const auto retract_deadline = steady_ms() + static_cast<int64_t>(touch_timeout_sec_ * 1000.0);
        while (retract_ok && rclcpp::ok() && steady_ms() < retract_deadline &&
          tip_z() < retract_target_z - 0.002)
        {
          if (!safe_to_move()) {
            retract_ok = false;
            break;
          }
          if (tip_z() < contact_z - 0.003) {
            RCLCPP_ERROR(get_logger(), "Touch stopped: unexpected motion toward the phone");
            retract_ok = false;
            break;  // Unexpected motion direction: stop before moving farther into the phone.
          }
          command(-touch_retract_speed_mps_);
          std::this_thread::sleep_for(servo_command_period);
        }
        halt();
        touched = touched && retract_ok && tip_z() >= retract_target_z - 0.002;
      }
    } catch (const tf2::TransformException & error) {
      halt();
      set_servo_paused(true);
      RCLCPP_ERROR(get_logger(), "Touch transform failed: %s", error.what());
      return false;
    }
    if (!set_servo_paused(true)) {
      return false;
    }
    if (!touched) {
      RCLCPP_ERROR(get_logger(), "Touch/retract failed; no further robot movement will be planned");
      return false;
    }
    const auto paused_ms = steady_ms();
    while (rclcpp::ok() && last_joint_ms_.load() <= paused_ms &&
      steady_ms() - paused_ms < 500)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (last_joint_ms_.load() <= paused_ms) {
      RCLCPP_ERROR(get_logger(), "No fresh joint state after touch");
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(joint_mutex_);
      for (std::size_t index = 0; index < active_joints_.size(); ++index) {
        const auto joint = current_joints_.find(active_joints_[index]);
        if (joint == current_joints_.end()) {
          RCLCPP_ERROR(get_logger(), "Missing joint state after touch");
          return false;
        }
        planned_joints_[index] = joint->second;
      }
    }
    RCLCPP_INFO(get_logger(), "%s touched and tool retracted", color.c_str());
    return true;
  }

  void run_sequence()
  {
    bool complete = true;
    for (std::size_t index = 0; index < targets_.size(); ++index) {
      if (index > 0 && align_before_each_target_ && !align_to_holder()) {
        complete = false;
        break;
      }
      if (!move_to(targets_[index].color, targets_[index].point)) {
        complete = false;
        break;
      }
      if (!touch_screen(targets_[index])) {
        complete = false;
        break;
      }
    }

    if (!complete) {
      RCLCPP_ERROR(get_logger(), "Sequence stopped; inspect the robot before any recovery move");
      finish(false);
      return;
    }
    move_group_->setJointValueTarget(initial_joints_);
    if (!plan_and_execute()) {
      RCLCPP_ERROR(get_logger(), "Failed to return to the initial joint state");
      finish(false);
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Standoff sequence %s; returned to the initial joint state",
      complete ? "complete" : "stopped");
    finish(true);
  }

  bool ready_{false};
  bool aligned_{false};
  bool started_{false};
  bool plan_only_{false};
  bool touch_enabled_{false};
  bool sim_{false};
  bool servo_started_{false};
  bool succeeded_{false};
  tf2::Quaternion target_orientation_;
  std::string move_group_node_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string target_link_;
  std::string holder_frame_;
  double approach_distance_m_;
  double sim_touch_approach_distance_m_;
  double screen_plane_offset_m_;
  double observation_distance_m_;
  bool align_before_each_target_;
  double planning_time_;
  int attempts_;
  double velocity_;
  double acceleration_;
  double detection_timeout_sec_;
  double touch_speed_mps_;
  double touch_retract_speed_mps_;
  double touch_retract_distance_m_;
  double touch_dwell_sec_;
  double touch_max_travel_m_;
  double touch_timeout_sec_;
  double max_touch_xy_error_m_;
  std::atomic<bool> touch_detected_{false};
  std::atomic<int64_t> last_touch_ms_{0};
  std::atomic<int8_t> servo_status_{-1};
  std::atomic<int64_t> last_servo_status_ms_{0};
  std::atomic<int64_t> last_joint_ms_{0};
  std::mutex joint_mutex_;
  std::mutex servo_command_mutex_;
  double calibration_x_m_;
  double calibration_y_m_;
  std::vector<Target> targets_;
  std::vector<std::string> active_joints_;
  std::unordered_map<std::string, double> current_joints_;
  std::vector<double> initial_joints_;
  std::vector<double> planned_joints_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr targets_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::CallbackGroup::SharedPtr joint_group_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_publisher_;
  rclcpp::TimerBase::SharedPtr detection_timer_;
  rclcpp::CallbackGroup::SharedPtr touch_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr touch_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_unpause_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<ScreenStandoffMover>();
    node->initialize();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    return node->succeeded() ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("move_to_screen_standoff"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 1;
}
