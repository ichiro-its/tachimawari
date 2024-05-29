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

#include "tachimawari/joint/tf2/tf2_manager.hpp"

#include <algorithm>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

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

  nlohmann::json config = nlohmann::json::parse(input);

  for (auto & item : config.items()) {
    // Get all config
    try {
      std::string name = item.key();
      double translation_x = item.value().at("translation").at("x");
      double translation_y = item.value().at("translation").at("y");
      double translation_z = item.value().at("translation").at("z");
      double const_roll = item.value().at("const_rpy").at("roll");
      double const_pitch = item.value().at("const_rpy").at("pitch");
      double const_yaw = item.value().at("const_rpy").at("yaw");
      auto map_string_id = FrameId::frame_string_id.find(name);

      uint8_t id = map_string_id->second;
      frames.push_back(
        Frame(id, translation_x, translation_y, translation_z, const_roll, const_pitch, const_yaw));
    } catch (nlohmann::json::parse_error & ex) {
      std::cerr << "parse error at byte " << ex.byte << std::endl;
      throw ex;
    }
  }

  return true;
}

void Tf2Manager::update(std::vector<Joint> current_joints, keisan::Angle<double> imu_yaw)
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
