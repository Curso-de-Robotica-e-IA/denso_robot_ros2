#pragma once

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace denso_collab
{

inline geometry_msgs::msg::Pose camera_pose_from_holder(
  const geometry_msgs::msg::Transform & holder,
  const tf2::Vector3 & offset)
{
  tf2::Quaternion holder_rotation;
  tf2::fromMsg(holder.rotation, holder_rotation);
  holder_rotation.normalize();

  tf2::Quaternion camera_rotation;
  camera_rotation.setRPY(std::acos(-1.0), 0.0, 0.0);
  camera_rotation.normalize();

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
