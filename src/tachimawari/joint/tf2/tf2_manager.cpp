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

bool Tf2Manager::load_configuration()
{
  config_path = "/home/ichiro/ros2-ws-cp/src/tachimawari/data/";
  std::string ss = config_path + utils::get_host_name() + ".json";

  if (utils::is_file_exist(ss) == false) {
    if (save_configuration() == false) {
      return false;
    }
  }

  std::ifstream input(ss, std::ifstream::in);
  if (input.is_open() == false) {
    return false;
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
    }
  }

  return true;
}

bool Tf2Manager::save_configuration()
{
  config_path = "/home/ichiro/ros2-ws-cp/src/tachimawari/data/";
  std::string ss = config_path + utils::get_host_name() + ".json";

  if (utils::is_file_exist(ss) == false) {
    if (utils::create_file(ss) == false) {
      return false;
    }
  }

  nlohmann::json config = nlohmann::json::array();

  for (auto & item : frames) {
    auto iterator = FrameId::frame_id_string.find(item.id);
    std::string name = iterator->second;

    nlohmann::json frame;
    frame["name"] = name;
    frame["translation"]["x"] = item.translation_x;
    frame["translation"]["y"] = item.translation_y;
    frame["translation"]["z"] = item.translation_z;
    frame["const_rpy"]["roll"] = item.const_roll;
    frame["const_rpy"]["pitch"] = item.const_pitch;
    frame["const_rpy"]["yaw"] = item.const_yaw;

    config.push_back(frame);
  }

  std::ofstream output(ss, std::ofstream::out);
  if (output.is_open() == false) {
    return false;
  }

  output << config.dump(2);
  output.close();

  return true;
}

bool Tf2Manager::sync_configuration()
{
  if (!load_configuration()) {
    return false;
  }

  if (!save_configuration()) {
    return false;
  }

  return true;
}

void Tf2Manager::update(std::vector<Joint> current_joints, keisan::Angle<double> imu_yaw)
{
  for (auto & item : frames) {
    if (item.id == 0) {
      item.update_quaternion(imu_yaw);
    } else {
      item.update_quaternion(current_joints);
    }
  }
}

}  // namespace tachimawari::joint
