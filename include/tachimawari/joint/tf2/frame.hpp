// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TACHIMAWARI__JOINT__TF2__FRAME_HPP_
#define TACHIMAWARI__JOINT__TF2__FRAME_HPP_

#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "keisan/angle/angle.hpp"
#include "rclcpp/time.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari/joint/tf2/frame_id.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace tachimawari::joint
{

class Frame
{
public:
  Frame(
    const uint8_t id, const double translation_x, const double translation_y,
    const double translation_z, const double const_roll, const double const_pitch,
    const double const_yaw);

  void update_quaternion(std::vector<Joint> current_joints);
  void update_quaternion(keisan::Angle<double> imu_yaw);
  geometry_msgs::msg::TransformStamped get_transform_stamped(rclcpp::Time time_stamp);

  uint8_t id;

  double translation_x;
  double translation_y;
  double translation_z;

  double quaternion_x;
  double quaternion_y;
  double quaternion_z;
  double quaternion_w;

  double const_roll;
  double const_pitch;
  double const_yaw;

private:
  enum : uint8_t {
    ROLL = 0,
    PITCH = 1,
    YAW = 2,
  };

  double get_joint_angle(uint8_t quaternion_axis, std::vector<Joint> current_joints);
};
}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__TF2__FRAME_HPP_
