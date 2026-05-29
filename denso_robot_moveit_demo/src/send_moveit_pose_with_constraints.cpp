#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{

struct Options
{
  std::string group_name = "arm";
  std::string eef_link;
  double planning_time = 5.0;
  int num_planning_attempts = 5;
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
  
  // Target pose
  double target_x = 0.0;
  double target_y = 0.0;
  double target_z = 0.0;
  
  // Plane constraint: which plane (xy, xz, or yz) and the constraint value
  std::string plane_constraint_type = "xy";  // Options: "xy", "xz", "yz"
  double plane_constraint_value = 0.0;  // The z, y, or x value to constrain to
  
  // Orientation constraint tolerance
  double orientation_tolerance = 0.05;  // Radians
  
  std::string move_group_node = "/move_group";
};

struct PoseState
{
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};  // xyzw
};

void print_usage(const char * prog)
{
  std::cout
    << "Usage: " << prog << " [options]\n"
    << "Options:\n"
    << "  --group-name <name>              MoveIt planning group (default: arm)\n"
    << "  --eef-link <name>                End effector link (optional, auto-detected)\n"
    << "  --planning-time <sec>            Planning time per target > 0 (default: 5.0)\n"
    << "  --num-planning-attempts <n>      Planning attempts > 0 (default: 5)\n"
    << "  --velocity-scaling <v>           Velocity scaling in [0,1] (default: 0.1)\n"
    << "  --acceleration-scaling <v>       Acceleration scaling in [0,1] (default: 0.1)\n"
    << "  --target-x <meters>              Target pose X coordinate (default: 0.0)\n"
    << "  --target-y <meters>              Target pose Y coordinate (default: 0.0)\n"
    << "  --target-z <meters>              Target pose Z coordinate (default: 0.0)\n"
    << "  --plane-constraint-type <type>   Plane type: xy|xz|yz (default: xy)\n"
    << "                                   xy-plane: constraint value is Z coordinate\n"
    << "                                   xz-plane: constraint value is Y coordinate\n"
    << "                                   yz-plane: constraint value is X coordinate\n"
    << "  --plane-constraint-value <val>   Plane constraint value (default: 0.0)\n"
    << "  --orientation-tolerance <rad>    Orientation constraint tolerance in radians\n"
    << "                                   (default: 0.05)\n"
    << "  --move-group-node <name>         Node name for robot_description params\n"
    << "                                   (default: /move_group)\n"
    << "  -h, --help                       Show this help\n"
    << "\nExample:\n"
    << "  " << prog << " --target-x 0.5 --target-y 0.2 --target-z 0.6 \\\n"
    << "    --plane-constraint-type xy --plane-constraint-value 0.6\n"
    << "    (Move to [0.5, 0.2, 0.6] while staying in xy-plane at Z=0.6)\n";
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
    if (arg == "--target-x") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.target_x)) {
        error = "Invalid --target-x";
        return false;
      }
      continue;
    }
    if (arg == "--target-y") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.target_y)) {
        error = "Invalid --target-y";
        return false;
      }
      continue;
    }
    if (arg == "--target-z") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.target_z)) {
        error = "Invalid --target-z";
        return false;
      }
      continue;
    }
    if (arg == "--plane-constraint-type") {
      const char * v = need_value(arg);
      if (!v) {return false;}
      options.plane_constraint_type = v;
      continue;
    }
    if (arg == "--plane-constraint-value") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.plane_constraint_value)) {
        error = "Invalid --plane-constraint-value";
        return false;
      }
      continue;
    }
    if (arg == "--orientation-tolerance") {
      const char * v = need_value(arg);
      if (!v || !parse_double(v, options.orientation_tolerance)) {
        error = "Invalid --orientation-tolerance";
        return false;
      }
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
  if (options.orientation_tolerance <= 0.0) {
    error = "--orientation-tolerance must be > 0";
    return false;
  }
  
  const std::string plane_type = options.plane_constraint_type;
  if (plane_type != "xy" && plane_type != "xz" && plane_type != "yz") {
    error = "-z-constraint-type must be xy, xz, or yz";
    return false;
  }

  return true;
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

PoseState get_current_pose(
  const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group,
  const std::string & eef_link)
{
  PoseState pose;
  
  const auto current_pose = [&]() {
    if (!eef_link.empty()) {
      return move_group->getCurrentPose(eef_link);
    } else {
      return move_group->getCurrentPose();
    }
  }();

  pose.position[0] = current_pose.pose.position.x;
  pose.position[1] = current_pose.pose.position.y;
  pose.position[2] = current_pose.pose.position.z;
  pose.orientation[0] = current_pose.pose.orientation.x;
  pose.orientation[1] = current_pose.pose.orientation.y;
  pose.orientation[2] = current_pose.pose.orientation.z;
  pose.orientation[3] = current_pose.pose.orientation.w;
  
  return pose;
}

moveit_msgs::msg::Constraints create_plane_constraints(
  const Options & options,
  const PoseState & current_pose,
  const std::string & ref_frame,
  const std::string & eef_link)
{
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "use_equality_constraints";

  // Create plane constraint to keep the end effector in a plane
  // Using equality constraints with one dimension set to 0.0005
  moveit_msgs::msg::PositionConstraint plane_constraint;
  plane_constraint.header.frame_id = ref_frame;
  plane_constraint.link_name = eef_link;
  
  shape_msgs::msg::SolidPrimitive plane;
  plane.type = shape_msgs::msg::SolidPrimitive::BOX;
  plane.dimensions.resize(3);
  
  // Set up box dimensions based on plane constraint type
  // One dimension is set to 0.0005 for equality constraint (between 0.00001 and 0.001)
  // Large dimensions (1.0) for unconstrained axes
  const double equality_dim = 0.0005;  // Magic number from tutorial
  const double large_dim = 2.0;  // Large dimension for unconstrained axes
  
  if (options.plane_constraint_type == "xy") {
    // Constrain Z axis (normal to xy plane) - equality constraint
    plane.dimensions[0] = large_dim;  // X - free
    plane.dimensions[1] = large_dim;  // Y - free
    plane.dimensions[2] = equality_dim;  // Z - equality constraint
  } else if (options.plane_constraint_type == "xz") {
    // Constrain Y axis (normal to xz plane) - equality constraint
    plane.dimensions[0] = large_dim;  // X - free
    plane.dimensions[1] = equality_dim;  // Y - equality constraint
    plane.dimensions[2] = large_dim;  // Z - free
  } else if (options.plane_constraint_type == "yz") {
    // Constrain X axis (normal to yz plane) - equality constraint
    plane.dimensions[0] = equality_dim;  // X - equality constraint
    plane.dimensions[1] = large_dim;  // Y - free
    plane.dimensions[2] = large_dim;  // Z - free
  }
  
  plane_constraint.constraint_region.primitives.emplace_back(plane);
  
  // Position the box center at the constraint plane
  // For unconstrained axes, center around current pose
  // For constrained axis, use the specified plane value
  geometry_msgs::msg::Pose plane_pose;
  if (options.plane_constraint_type == "xy") {
    plane_pose.position.x = current_pose.position[0];
    plane_pose.position.y = current_pose.position[1];
    plane_pose.position.z = options.plane_constraint_value;
  } else if (options.plane_constraint_type == "xz") {
    plane_pose.position.x = current_pose.position[0];
    plane_pose.position.y = options.plane_constraint_value;
    plane_pose.position.z = current_pose.position[2];
  } else if (options.plane_constraint_type == "yz") {
    plane_pose.position.x = options.plane_constraint_value;
    plane_pose.position.y = current_pose.position[1];
    plane_pose.position.z = current_pose.position[2];
  }
  plane_pose.orientation.w = 1.0;
  
  plane_constraint.constraint_region.primitive_poses.emplace_back(plane_pose);
  plane_constraint.weight = 1.0;
  
  constraints.position_constraints.emplace_back(plane_constraint);

  // Create orientation constraint to keep current orientation
  moveit_msgs::msg::OrientationConstraint ori_constraint;
  ori_constraint.header.frame_id = ref_frame;
  ori_constraint.link_name = eef_link;
  ori_constraint.orientation.x = current_pose.orientation[0];
  ori_constraint.orientation.y = current_pose.orientation[1];
  ori_constraint.orientation.z = current_pose.orientation[2];
  ori_constraint.orientation.w = current_pose.orientation[3];
  ori_constraint.absolute_x_axis_tolerance = options.orientation_tolerance;
  ori_constraint.absolute_y_axis_tolerance = options.orientation_tolerance;
  ori_constraint.absolute_z_axis_tolerance = options.orientation_tolerance;
  ori_constraint.weight = 1.0;
  
  constraints.orientation_constraints.emplace_back(ori_constraint);

  return constraints;
}

void publish_constraint_markers(
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & pub,
  const Options & options,
  const PoseState & current_pose,
  const std::string & ref_frame)
{
  visualization_msgs::msg::MarkerArray markers;
  static int marker_id = 0;

  // Plane constraint visualization (as a semi-transparent box)
  visualization_msgs::msg::Marker plane_marker;
  plane_marker.header.frame_id = ref_frame;
  plane_marker.header.stamp = rclcpp::Time(0);
  plane_marker.id = marker_id++;
  plane_marker.type = visualization_msgs::msg::Marker::CUBE;
  plane_marker.action = visualization_msgs::msg::Marker::ADD;
  plane_marker.lifetime = rclcpp::Duration::from_seconds(0);

  // Position and scale for the plane (adjusted based on plane type)
  if (options.plane_constraint_type == "xy") {
    // XY plane - constrain Z axis (normal is Z, thin in Z direction)
    plane_marker.pose.position.x = current_pose.position[0];
    plane_marker.pose.position.y = current_pose.position[1];
    plane_marker.pose.position.z = options.plane_constraint_value;
    plane_marker.scale.x = 2.0;     // X extent (enlarged)
    plane_marker.scale.y = 2.0;     // Y extent (enlarged)
    plane_marker.scale.z = 0.01;    // Thin in Z
    plane_marker.color.r = 1.0;     // Red for xy-plane
    plane_marker.color.g = 0.0;
    plane_marker.color.b = 0.0;
  } else if (options.plane_constraint_type == "xz") {
    // XZ plane - constrain Y axis (normal is Y, thin in Y direction)
    plane_marker.pose.position.x = current_pose.position[0];
    plane_marker.pose.position.y = options.plane_constraint_value;
    plane_marker.pose.position.z = current_pose.position[2];
    plane_marker.scale.x = 2.0;     // X extent (enlarged)
    plane_marker.scale.y = 0.01;    // Thin in Y (the constrained axis)
    plane_marker.scale.z = 2.0;     // Z extent (enlarged)
    plane_marker.color.r = 0.0;     // Green for xz-plane
    plane_marker.color.g = 1.0;
    plane_marker.color.b = 0.0;
  } else if (options.plane_constraint_type == "yz") {
    // YZ plane - constrain X axis (normal is X, thin in X direction)
    plane_marker.pose.position.x = options.plane_constraint_value;
    plane_marker.pose.position.y = current_pose.position[1];
    plane_marker.pose.position.z = current_pose.position[2];
    plane_marker.scale.x = 0.01;    // Thin in X (the constrained axis)
    plane_marker.scale.y = 2.0;     // Y extent (enlarged)
    plane_marker.scale.z = 2.0;     // Z extent (enlarged)
    plane_marker.color.r = 0.0;     // Blue for yz-plane
    plane_marker.color.g = 0.0;
    plane_marker.color.b = 1.0;
  }

  plane_marker.pose.orientation.w = 1.0;
  plane_marker.color.a = 0.3;  // Semi-transparent
  markers.markers.push_back(plane_marker);

  // Orientation constraint visualization (as a sphere)
  visualization_msgs::msg::Marker ori_marker;
  ori_marker.header.frame_id = ref_frame;
  ori_marker.header.stamp = rclcpp::Time(0);
  ori_marker.id = marker_id++;
  ori_marker.type = visualization_msgs::msg::Marker::SPHERE;
  ori_marker.action = visualization_msgs::msg::Marker::ADD;
  ori_marker.lifetime = rclcpp::Duration::from_seconds(0);

  ori_marker.pose.position.x = current_pose.position[0];
  ori_marker.pose.position.y = current_pose.position[1];
  ori_marker.pose.position.z = current_pose.position[2];
  ori_marker.pose.orientation.x = current_pose.orientation[0];
  ori_marker.pose.orientation.y = current_pose.orientation[1];
  ori_marker.pose.orientation.z = current_pose.orientation[2];
  ori_marker.pose.orientation.w = current_pose.orientation[3];

  // Scale represents tolerance region
  ori_marker.scale.x = options.orientation_tolerance;
  ori_marker.scale.y = options.orientation_tolerance;
  ori_marker.scale.z = options.orientation_tolerance;
  ori_marker.color.r = 1.0;
  ori_marker.color.g = 1.0;
  ori_marker.color.b = 0.0;  // Yellow
  ori_marker.color.a = 0.2;

  markers.markers.push_back(ori_marker);

  pub->publish(markers);
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
  auto node = std::make_shared<rclcpp::Node>("send_moveit_pose_with_constraints");
  const auto logger = node->get_logger();

  // Create marker publisher for constraint visualization
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "constraint_markers", 10);

  std::string description_error;
  if (!load_descriptions_from_move_group(node, options.move_group_node, description_error)) {
    RCLCPP_ERROR(logger, "Failed to load robot descriptions from move_group: %s", description_error.c_str());
    RCLCPP_ERROR(logger, "Hint: start denso_robot_bringup launch before running this node, or set --move-group-node.");
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

  // Get current pose
  const auto current_pose = get_current_pose(move_group, selected_eef);
  const std::string planning_frame = move_group->getPlanningFrame();
  
  RCLCPP_INFO(logger, "=== MoveIt Pose with Constraints Configuration ===");
  RCLCPP_INFO(logger, "  group_name: %s", options.group_name.c_str());
  RCLCPP_INFO(logger, "  planning_frame: %s", planning_frame.c_str());
  RCLCPP_INFO(logger, "  eef_link: %s", selected_eef.c_str());
  RCLCPP_INFO(logger, "  Current pose: [%.6f, %.6f, %.6f]", current_pose.position[0], current_pose.position[1], current_pose.position[2]);
  RCLCPP_INFO(logger, "  Current orientation (xyzw): [%.6f, %.6f, %.6f, %.6f]", 
    current_pose.orientation[0], current_pose.orientation[1], current_pose.orientation[2], current_pose.orientation[3]);
  RCLCPP_INFO(logger, "  Target pose: [%.6f, %.6f, %.6f]", options.target_x, options.target_y, options.target_z);
  RCLCPP_INFO(logger, "  Plane constraint: %s-plane at %.6f (using equality constraint with dim=0.0005)", 
    options.plane_constraint_type.c_str(), options.plane_constraint_value);
  RCLCPP_INFO(logger, "  Orientation constraint tolerance: %.6f rad", options.orientation_tolerance);

  // Create constraints
  const auto constraints = create_plane_constraints(options, current_pose, planning_frame, selected_eef);
  
  // Publish constraint markers for visualization
  publish_constraint_markers(marker_pub, options, current_pose, planning_frame);
  RCLCPP_INFO(logger, "Constraint markers published to 'constraint_markers' topic");

  // Set target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = options.target_x;
  target_pose.position.y = options.target_y;
  target_pose.position.z = options.target_z;
  target_pose.orientation.x = current_pose.orientation[0];
  target_pose.orientation.y = current_pose.orientation[1];
  target_pose.orientation.z = current_pose.orientation[2];
  target_pose.orientation.w = current_pose.orientation[3];
  RCLCPP_INFO(logger, "\nNote: Ensure target matches plane constraint!");
  RCLCPP_INFO(logger, "  For %s-plane at %.6f, target %s coordinate should be %.6f",
    options.plane_constraint_type.c_str(),
    options.plane_constraint_value,
    (options.plane_constraint_type == "xy") ? "Z" : 
    (options.plane_constraint_type == "xz") ? "Y" : "X",
    options.plane_constraint_value);

  int attempt = 1;
  int total_succeeded = 0;

  for (attempt = 1; attempt <= 2; ++attempt) {
    const bool use_ompl = (attempt == 1);
    RCLCPP_INFO(logger, "\n=== Attempt %d: %s ===", attempt, use_ompl ? "WITH OMPL Constraints" : "WITHOUT OMPL Constraints");

    if (use_ompl) {
      // Enable OMPL and set constraints
      move_group->setPathConstraints(constraints);
      RCLCPP_INFO(logger, "Path constraints enabled");
    } else {
      // Clear constraints for second attempt
      move_group->clearPathConstraints();
      RCLCPP_INFO(logger, "Path constraints cleared");
    }

    // Set target pose
    move_group->setPoseTarget(target_pose, selected_eef);
    RCLCPP_INFO(logger, "Pose target set to: [%.6f, %.6f, %.6f]", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_result = move_group->plan(plan);
    
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "  Planning failed on attempt %d (error code: %d)", attempt, static_cast<int>(plan_result.val));
      move_group->clearPoseTargets();
      move_group->clearPathConstraints();
      if (attempt == 1) {
        RCLCPP_INFO(logger, "Proceeding to attempt 2 without constraints...");
      }
      continue;
    }

    RCLCPP_INFO(logger, "Planning succeeded, executing motion...");
    
    const auto exec_result = move_group->execute(plan);
    move_group->stop();
    
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(logger, "  Execution failed on attempt %d (error code: %d)", attempt, static_cast<int>(exec_result.val));
      move_group->clearPoseTargets();
      move_group->clearPathConstraints();
      continue;
    }

    RCLCPP_INFO(logger, "  Execution succeeded on attempt %d", attempt);
    move_group->clearPoseTargets();
    
    ++total_succeeded;

    // Small delay between attempts
    if (attempt == 1) {
      RCLCPP_INFO(logger, "Waiting 2 seconds before next attempt...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  }

  move_group->clearPathConstraints();

  RCLCPP_INFO(logger, "\n=== Summary ===");
  RCLCPP_INFO(logger, "Successfully completed: %d out of 2 attempts", total_succeeded);
  if (total_succeeded == 2) {
    RCLCPP_INFO(logger, "Both planning methods succeeded!");
  } else if (total_succeeded == 1) {
    RCLCPP_WARN(logger, "Only one method succeeded");
  } else {
    RCLCPP_WARN(logger, "All planning attempts failed");
  }
  
  rclcpp::shutdown();
  return total_succeeded > 0 ? 0 : 1;
}
