#include <array>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>

namespace
{

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
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"robot_description", "robot_description_semantic"};
  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Could not load robot descriptions from %s", move_group_node.c_str());
    return false;
  }

  const auto response = future.get();
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
      key + "solution_weights", {4.0, 1.0, 1.0, 4.0, 3.0, 10.0});
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
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "left_calib_link");
    target_link_ = declare_parameter<std::string>("target_link", "left_camera_depth_optical_frame");
    standoff_m_ = declare_parameter<double>("standoff_m", 0.1);
    planning_time_ = declare_parameter<double>("planning_time", 8.0);
    attempts_ = declare_parameter<int>("num_planning_attempts", 10);
    velocity_ = declare_parameter<double>("velocity_scaling", 0.1);
    acceleration_ = declare_parameter<double>("acceleration_scaling", 0.1);
    plan_only_ = declare_parameter<bool>("plan_only", false);
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
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "screen_standoff_goal", 10);
    const std::array<std::string, 3> colors{"red", "green", "blue"};
    for (std::size_t index = 0; index < colors.size(); ++index) {
      subscriptions_[index] = create_subscription<geometry_msgs::msg::PointStamped>(
        "/dots/" + colors[index], 10,
        [this, index](geometry_msgs::msg::PointStamped::SharedPtr point) {
          receive_target(index, std::move(point));
        });
    }
    active_joints_ = move_group_->getActiveJoints();
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&ScreenStandoffMover::receive_joint_state, this, std::placeholders::_1));
  }

private:
  void receive_joint_state(const sensor_msgs::msg::JointState::SharedPtr state)
  {
    for (std::size_t index = 0; index < state->name.size() && index < state->position.size(); ++index) {
      current_joints_[state->name[index]] = state->position[index];
    }
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
    RCLCPP_INFO(get_logger(), "Received current robot state; waiting for colored dots");
  }

  void receive_target(
    std::size_t index, const geometry_msgs::msg::PointStamped::SharedPtr point)
  {
    if (!ready_ || started_) {
      return;
    }
    targets_[index] = *point;
    if (!targets_[0] || !targets_[1] || !targets_[2]) {
      return;
    }
    started_ = true;
    run_sequence();
  }

  geometry_msgs::msg::PoseStamped standoff_pose(
    const geometry_msgs::msg::PointStamped & point) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = point.header;
    pose.pose.position.x = point.point.x;
    pose.pose.position.y = point.point.y;
    pose.pose.position.z = standoff_m_;
    return pose;
  }

  bool plan_and_execute()
  {
    moveit_msgs::msg::RobotState start_state;
    start_state.joint_state.name = active_joints_;
    start_state.joint_state.position = planned_joints_;
    move_group_->setStartState(start_state);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed");
      move_group_->clearPoseTargets();
      return false;
    }
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (!trajectory.points.empty()) {
      const auto & endpoint = trajectory.points.back().positions;
      for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        const auto joint = std::find(
          active_joints_.begin(), active_joints_.end(), trajectory.joint_names[index]);
        if (joint != active_joints_.end() && index < endpoint.size()) {
          planned_joints_[std::distance(active_joints_.begin(), joint)] = endpoint[index];
        }
      }
    }
    if (!plan_only_ && move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed");
      move_group_->clearPoseTargets();
      return false;
    }
    move_group_->stop();
    move_group_->clearPoseTargets();
    return true;
  }

  bool move_to(const std::string & color, const geometry_msgs::msg::PointStamped & point)
  {
    auto calib_pose = standoff_pose(point);
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
    if (!have_target_orientation_) {
      try {
        const auto holder_target = tf_buffer_->lookupTransform(
          calib_pose.header.frame_id, target_link_, tf2::TimePointZero,
          tf2::durationFromSec(2.0));
        tf2::fromMsg(holder_target.transform.rotation, target_orientation_);
        have_target_orientation_ = true;
      } catch (const tf2::TransformException & error) {
        RCLCPP_ERROR(get_logger(), "Cannot read the aligned camera orientation: %s", error.what());
        return false;
      }
    }
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
    RCLCPP_INFO(get_logger(), "Moving to %s at %.0f mm standoff", color.c_str(), standoff_m_ * 1000.0);
    return plan_and_execute();
  }

  void touch_screen(const std::string & color)
  {
    RCLCPP_INFO(get_logger(), "%s reached; touch_screen is deferred", color.c_str());
  }

  void run_sequence()
  {
    const std::array<std::string, 3> colors{"red", "green", "blue"};
    bool complete = true;
    for (std::size_t index = 0; index < colors.size(); ++index) {
      if (!move_to(colors[index], *targets_[index])) {
        complete = false;
        break;
      }
      touch_screen(colors[index]);
    }

    move_group_->setJointValueTarget(initial_joints_);
    if (!plan_and_execute()) {
      RCLCPP_ERROR(get_logger(), "Failed to return to the initial joint state");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Color sequence %s; returned to the initial joint state",
      complete ? "complete" : "stopped");
  }

  bool ready_{false};
  bool started_{false};
  bool plan_only_{false};
  bool have_target_orientation_{false};
  tf2::Quaternion target_orientation_;
  std::string move_group_node_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string target_link_;
  double standoff_m_;
  double planning_time_;
  int attempts_;
  double velocity_;
  double acceleration_;
  std::array<std::optional<geometry_msgs::msg::PointStamped>, 3> targets_;
  std::vector<std::string> active_joints_;
  std::unordered_map<std::string, double> current_joints_;
  std::vector<double> initial_joints_;
  std::vector<double> planned_joints_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr, 3>
    subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
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
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("move_to_screen_standoff"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
