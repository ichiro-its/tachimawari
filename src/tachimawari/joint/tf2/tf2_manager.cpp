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

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/joint/tf2/tf2_manager.hpp"

#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"

namespace tachimawari::joint
{

Tf2Manager::Tf2Manager() {}

bool Tf2Manager::load_configuration(const std::string & path)
{
  std::string ss = path + "frame_measurements.json";

  std::ifstream input(ss, std::ifstream::in);
  if (!input.is_open()) {
    throw std::runtime_error("Unable to open `" + ss + "`!");
  }

  nlohmann::json config = jitsuyo::load_config(path, "frame_measurements.json");
  if (config.empty()) {
    return false;
  }

  for (auto & item : config.items()) {
    // Get all config
    double translation_x;
    double translation_y;
    double translation_z;
    double const_roll;
    double const_pitch;
    double const_yaw;

    std::string name = item.key();

    nlohmann::json section;
    if (!jitsuyo::assign_val(item.value(), name, section)) return false;
    if (!jitsuyo::assign_val(section, "translation_x", translation_x) ||
      !jitsuyo::assign_val(section, "translation_y", translation_y) ||
      !jitsuyo::assign_val(section, "translation_z", translation_z) ||
      !jitsuyo::assign_val(section, "const_roll", const_roll) ||
      !jitsuyo::assign_val(section, "const_pitch", const_pitch) ||
      !jitsuyo::assign_val(section, "const_yaw", const_yaw)) return false;

    auto map_string_id = FrameId::frame_string_id.find(name);

    uint8_t id = map_string_id->second;
    frames.push_back(
      Frame(id, translation_x, translation_y, translation_z, const_roll, const_pitch, const_yaw));
  }

  return true;
}

void Tf2Manager::update(const std::vector<Joint> & current_joints, const keisan::Angle<double> & imu_yaw)
{
  for (auto & item : frames) {
    if (item.id == FrameId::ODOM) {
      item.update_quaternion(imu_yaw);
    } else {
      item.update_quaternion(current_joints);
    }
  }
}

}  // namespace tachimawari::joint
