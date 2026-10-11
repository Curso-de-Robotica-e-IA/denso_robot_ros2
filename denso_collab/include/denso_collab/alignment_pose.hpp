#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace denso_collab
{

inline double phone_rotation_radians(const std::string & orientation)
{
  if (orientation == "vertical") {
    return 0.0;
  }
  if (orientation == "horizontal") {
    return std::acos(-1.0) / 2.0;
  }
  throw std::invalid_argument("phone_orientation must be vertical or horizontal");
}

inline tf2::Quaternion camera_orientation_in_holder(double phone_rotation_rad)
{
  tf2::Quaternion keep_camera_upright;
  keep_camera_upright.setRPY(0.0, 0.0, -phone_rotation_rad);
  tf2::Quaternion face_holder;
  face_holder.setRPY(std::acos(-1.0), 0.0, 0.0);
  tf2::Quaternion orientation = keep_camera_upright * face_holder;
  orientation.normalize();
  return orientation;
}

inline geometry_msgs::msg::Pose camera_pose_from_holder(
  const geometry_msgs::msg::Transform & holder,
  const tf2::Vector3 & offset,
  double phone_rotation_rad = 0.0)
{
  tf2::Quaternion holder_rotation;
  tf2::fromMsg(holder.rotation, holder_rotation);
  holder_rotation.normalize();

  const tf2::Quaternion camera_rotation =
    camera_orientation_in_holder(phone_rotation_rad);

  const tf2::Transform world_holder(
    holder_rotation,
    tf2::Vector3(holder.translation.x, holder.translation.y, holder.translation.z));
  const tf2::Transform world_camera =
    world_holder * tf2::Transform(camera_rotation, offset);

  geometry_msgs::msg::Pose pose;
  pose.position.x = world_camera.getOrigin().x();
  pose.position.y = world_camera.getOrigin().y();
  pose.position.z = world_camera.getOrigin().z();
  pose.orientation = tf2::toMsg(world_camera.getRotation());
  return pose;
}

}  // namespace denso_collab
