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
  nlohmann::json config;
  if (!jitsuyo::load_config(path, "frame_measurements.json", config)) {
    return false;
  }

  bool valid_config = true;

  for (auto & item : config.items()) {
    // Get all config
    double translation_x;
    double translation_y;
    double translation_z;
    double const_roll;
    double const_pitch;
    double const_yaw;

    std::string name = item.key();
    
    bool valid_section = true;
    nlohmann::json section;
    if (jitsuyo::assign_val(config, name, section)) {
      nlohmann::json translation;
      nlohmann::json const_rpy;

      if (jitsuyo::assign_val(section, "translation", translation)) {
        bool valid_translation = true;
        valid_translation &= jitsuyo::assign_val(translation, "x", translation_x);
        valid_translation &= jitsuyo::assign_val(translation, "y", translation_y);
        valid_translation &= jitsuyo::assign_val(translation, "z", translation_z);
        if (!valid_translation) {
          std::cout << "Error found at section `translation`" << std::endl;
          valid_section = false;
        }
      } else {
        valid_section = false;
      }

      if (jitsuyo::assign_val(section, "const_rpy", const_rpy)) {
        bool valid_const_rpy = true;
        valid_const_rpy &= jitsuyo::assign_val(const_rpy, "roll", const_roll);
        valid_const_rpy &= jitsuyo::assign_val(const_rpy, "pitch", const_pitch);
        valid_const_rpy &= jitsuyo::assign_val(const_rpy, "yaw", const_yaw);
        if (!valid_const_rpy) {
          std::cout << "Error found at section `const_rpy`" << std::endl;
          valid_section = false;
        }
      } else {
        valid_section = false;
      }
    } else {
      valid_section = false;
    }

    if (!valid_section) {
      valid_config = false;
      continue;
    }

    auto map_string_id = FrameId::frame_string_id.find(name);

    uint8_t id = map_string_id->second;
    frames.push_back(
      Frame(id, translation_x, translation_y, translation_z, const_roll, const_pitch, const_yaw));
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `" + path + "frame_measurements.json`");
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
