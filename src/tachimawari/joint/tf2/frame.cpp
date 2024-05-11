// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "tachimawari/joint/tf2/frame.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace tachimawari::joint
{

Frame::Frame(
  const uint8_t id, const double translation_x, const double translation_y,
  const double translation_z, const double quaternion_x, const double quaternion_y,
  const double quaternion_z, const double quaternion_w)
: id(id),
  translation_x(translation_x),
  translation_y(translation_y),
  translation_z(translation_z),
  quaternion_x(quaternion_x),
  quaternion_y(quaternion_y),
  quaternion_z(quaternion_z),
  quaternion_w(quaternion_w)
{
}

geometry_msgs::msg::TransformStamped Frame::get_transform_stamped(rclcpp::Time time_stamp)
{
  auto map_string_name = FrameId::frame_id_string.find(id);
  std::string frame_name = map_string_name->second;

  auto map_parent_name = FrameId::parent_frame.find(id);
  std::string parent_frame_name = map_parent_name->second;

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent_frame_name;
  t.header.stamp = time_stamp;
  t.child_frame_id = frame_name;

  t.transform.translation.x = translation_x;
  t.transform.translation.y = translation_y;
  t.transform.translation.z = translation_z;

  t.transform.rotation.x = quaternion_x;
  t.transform.rotation.y = quaternion_y;
  t.transform.rotation.z = quaternion_z;
  t.transform.rotation.w = quaternion_w;
  return t;
}

void Frame::update_quaternion(std::vector<Joint> current_joints)
{
  double roll_deg = get_joint_angle(ROLL, current_joints);
  double pitch_deg = get_joint_angle(PITCH, current_joints);
  double yaw_deg = get_joint_angle(YAW, current_joints);
  
  double roll_rad = roll_deg * M_PI / 180.0;
  double pitch_rad = pitch_deg * M_PI / 180.0;
  double yaw_rad = yaw_deg * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);

  quaternion_x = q.x();
  quaternion_y = q.y();
  quaternion_z = q.z();
  quaternion_w = q.w();
}

double Frame::get_joint_angle(uint8_t quaternion_axis, std::vector<Joint> current_joints)
{
  auto map = FrameId::frame_joint_map.find(id);
  std::vector<uint8_t> frame_joints = map->second;
  uint8_t id_map = map->first;
  uint8_t joint_id = frame_joints[quaternion_axis];

  if (joint_id == 0) {
    return 0.0;
  }

  for (auto joint : current_joints) {
    if (joint.get_id() == joint_id) {
      return joint.get_position();
    }
  }
}

}  // namespace tachimawari::joint
