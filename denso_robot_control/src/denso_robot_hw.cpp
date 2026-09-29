// Copyright 2020 DENSO WAVE INCORPORATED
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "denso_robot_control/denso_robot_hw.hpp"

#include <chrono>
#include <cmath>
// #include <limits>
// #include <memory>
// #include <vector>
#include <thread>
#include <functional>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace denso_robot_control
{

hardware_interface::CallbackReturn
DensoRobotHW::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "Starting DENSO robot drivers ...");
  // TODO: do we really need this wait time ??
  // std::this_thread::sleep_for(std::chrono::seconds(5));

  // set default values for commands and states
  for (uint i = 0; i < pos_interface_.size(); i++) {
    if (std::isnan(pos_interface_[i])) {
      cmd_interface_[i] = 0;
      pos_interface_[i] = 0;
      vel_interface_[i] = 0;
      eff_interface_[i] = 0;
    }
  }

  node_name_ = info_.hardware_parameters["node_name"].c_str();
  node_namespace_ = info_.hardware_parameters["node_namespace"].c_str();
  robot_ip_address_ = info_.hardware_parameters["ip_address"];
  robot_name_ = info_.hardware_parameters["robot_name"];
  robot_joints_ = std::stoi(info_.hardware_parameters["robot_joints"]);
  ctrl_type_ = std::stoi(info_.hardware_parameters["controller_type"]);

  joint_type_.resize(robot_joints_);
  for (int i = 0; i < robot_joints_; i++) {
    std::stringstream ss;
    ss << "joint_" << i + 1;
    joint_type_[i] = std::stoi(info_.hardware_parameters[ss.str()]);
  }

  arm_group_ = std::stoi(info_.hardware_parameters["arm_group"]);

  send_format_ = std::stoi(info_.hardware_parameters["send_format"]);
  recv_format_ = std::stoi(info_.hardware_parameters["recv_format"]);

  std::string str_verbose = info_.hardware_parameters["verbose"];
  verbose_ = (str_verbose == "True" || str_verbose == "true");
  if (verbose_) {
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "*******************************************");
    RCLCPP_INFO(
      rclcpp::get_logger("DensoRobotHW"),
      "***** Verbose mode ON. List of parsed robot arguments :");
    RCLCPP_INFO(
      rclcpp::get_logger("DensoRobotHW"), "***** robot name: %s", robot_name_.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger("DensoRobotHW"),
      "***** controller type (8 = RC8 ; 9 = RC9): %d", ctrl_type_);
    RCLCPP_INFO(
      rclcpp::get_logger("DensoRobotHW"),
      "***** robot ip address: %s", robot_ip_address_.c_str());
    RCLCPP_INFO(
      rclcpp::get_logger("DensoRobotHW"), "***** number of joints:  %d", robot_joints_);
    for (int i = 0; i < robot_joints_; i++) {
      RCLCPP_INFO(
        rclcpp::get_logger("DensoRobotHW"),
        "***** joint_%d type (0 = prismatic ; 1 = revolute):  %d", i + 1, joint_type_[i]);
    }
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** arm group: %d", arm_group_);
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** send format: %d", send_format_);
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** receive format: %d", recv_format_);
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "*******************************************");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "Arguments parsed !!");
  }

  pos_interface_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  vel_interface_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  eff_interface_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  cmd_interface_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Denso robots allow exactly one command interface on each joint (POSITION type).
    // However, VELOCITY type is needed for simulation
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %ld command interfaces found. 2 expected (only 1 active).",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %s command interfaces found. Expected %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    // Denso robots allow three state interfaces on each joint
    // (POSITION, VELOCITY and EFFORT types)
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %ld state interface. 3 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %s state interface as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %s state interface as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(
        rclcpp::get_logger("DensoRobotHW"),
        "Joint '%s' has %s state interface as third state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[2].name.c_str(),
        hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** Hardware configured !!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DensoRobotHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_interface_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_interface_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &eff_interface_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DensoRobotHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_interface_[i]));

    // Denso robots allow exactly one command interface on each joint (POSITION type),
    // so VELOCITY commands are ignored
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, NULL));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
DensoRobotHW::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
  drobo_ = std::make_shared<DensoRobotControl>(
    node_name_, node_namespace_, robot_name_, robot_ip_address_, ctrl_type_,
    robot_joints_, joint_type_, arm_group_, send_format_, recv_format_, verbose_);

  auto node = rclcpp::Node::make_shared(node_name_, node_namespace_);
  drobo_->setNode(node);

  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "Initializing drivers ...");
  HRESULT hr = drobo_->Initialize(pos_interface_, cmd_interface_);
  if (FAILED(hr)) {
    RCLCPP_FATAL(
      rclcpp::get_logger("DensoRobotHW"), "Failed to initialize real controller. (%X)", hr);
    return CallbackReturn::ERROR;
  }

  SpinNode(node, drobo_);

  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "System successfully started !!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DensoRobotHW::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "Stopping robot drivers... ");
  // TODO: do we really need this wait time ??
  drobo_->Stop();
  std::this_thread::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "System successfully stopped !!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DensoRobotHW::read(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  std::unique_lock<std::mutex> lock_mode(mtx_mode_);
  // read robot current position
  drobo_->read(pos_interface_);
  return return_type::OK;
}

hardware_interface::return_type DensoRobotHW::write(
  const rclcpp::Time & /* time */,
  const rclcpp::Duration & /* period */)
{
  std::unique_lock<std::mutex> lock_mode(mtx_mode_);
  drobo_->write(cmd_interface_);
  return return_type::OK;
}

// ***************************************************************************************************
void DensoRobotHW::SpinNode(rclcpp::Node::SharedPtr & node, DensoRobotControl_Ptr drobo)
{
  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** Starting DENSO robot control thread ...");
  std::thread denso_thread([node, drobo]() {
      rclcpp::WallRate loop_rate(1000);
      while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        drobo->Update();
        loop_rate.sleep();
      }
      rclcpp::shutdown();
    });

  denso_thread.detach();
  RCLCPP_INFO(rclcpp::get_logger("DensoRobotHW"), "***** DENSO robot control thread started !!");
}
}  // namespace denso_robot_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  denso_robot_control::DensoRobotHW,
  hardware_interface::SystemInterface
)
