#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("record_right_pose");
  const auto name = node->declare_parameter<std::string>("name", "unnamed");
  const auto topic = node->declare_parameter<std::string>("joint_state_topic", "/joint_states");

  const std::array<std::string, 6> joints{
    "right_joint_1", "right_joint_2", "right_joint_3",
    "right_joint_4", "right_joint_5", "right_joint_6"};
  std::map<std::string, double> positions;
  const auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    topic, 10, [&positions](sensor_msgs::msg::JointState::SharedPtr message) {
      for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
        if (message->name[i].rfind("right_joint_", 0) == 0) {
          positions[message->name[i]] = message->position[i];
        }
      }
    });
  (void)subscription;

  for (int i = 0; rclcpp::ok() && i < 50 && positions.size() < joints.size(); ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (positions.size() < joints.size()) {
    RCLCPP_ERROR(node->get_logger(), "Timed out waiting for all right-arm joints on %s", topic.c_str());
    rclcpp::shutdown();
    return 2;
  }

  std::cout << std::setprecision(17) << name << ": [";
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << positions.at(joints[i]);
  }
  std::cout << "]\n";
  rclcpp::shutdown();
  return 0;
}
