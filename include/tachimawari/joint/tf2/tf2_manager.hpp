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

#ifndef TACHIMAWARI__JOINT__TF2__TF2_MANAGER_HPP_
#define TACHIMAWARI__JOINT__TF2__TF2_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "keisan/angle/angle.hpp"
#include "rclcpp/time.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari/joint/tf2/frame.hpp"
#include "tachimawari/joint/tf2/frame_id.hpp"
#include "tachimawari/joint/utils/utils.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tachimawari::joint
{

class Tf2Manager
{
public:
  explicit Tf2Manager();

  bool load_configuration();
  bool save_configuration();
  bool sync_configuration();
  void update(std::vector<Joint> current_joints, keisan::Angle<double> imu_yaw);
  std::vector<Frame> get_frames() { return frames; }

private:
  static const std::vector<std::pair<int, int>> tf2_joint_pairs;
  std::string config_path;
  std::vector<Frame> frames;
};
}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__TF2__TF2_MANAGER_HPP_
