// Copyright (c) 2023 Ichiro ITS
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

#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include "tachimawari/control/control.hpp"
#include "tachimawari/node/tachimawari_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the mode! [sdk / cm740]" << std::endl;
    return 1;
  }

  bool read_all = false;
  std::string target_joint_str;
  uint8_t target_joint;

  if (argc < 3) {
    read_all = true;
  } else {
    target_joint_str = argv[2];
    target_joint = std::stoi(target_joint_str);
  }

  std::string mode = argv[1];
  std::shared_ptr<tachimawari::control::ControlManager> controller;

  if (mode == "sdk") {
    controller = std::make_shared<tachimawari::control::DynamixelSDK>("/dev/ttyUSB0");
  } else if (mode == "cm740") {
    controller = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
  } else {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 1;
  }

  if (!controller->connect()) {
    controller->set_port("/dev/ttyUSB1");

    if (!controller->connect()) {
      std::cout << "failed to connect controller" << std::endl;
      return 1;
    }
  }

  auto joint_manager = std::make_shared<tachimawari::joint::JointManager>(controller);
  std::vector<tachimawari::joint::Joint> joints;

  if (!joint_manager->set_joints(joints)) {
    std::cerr << "failed to sync write" << std::endl;
  }

  auto node = std::make_shared<rclcpp::Node>("tachimawari_node");
  tachimawari::TachimawariNode tachimawari_node(node, controller);
  joints = joint_manager->get_current_joints();

  double max = std::numeric_limits<double>::min();
  double min = std::numeric_limits<double>::max();

  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok()) {
    rcl_rate.sleep();
    rclcpp::spin_some(node);

    if (!read_all) {
      for (const auto &joint : joints) {
        if (target_joint == joint.get_id()) {
          min = std::min(joint.get_position(), min);
          max = std::max(joint.get_position(), max);

          std::cout << "id " << static_cast<int>(joint.get_id()) << ": " << joint.get_position() << std::endl;
          std::cout << "minimum: " << min << std::endl;
          std::cout << "maximum: " << max << std::endl;
        }
      }
    } else {
      for (const auto &joint : joints) {
        std::cout << "id " << static_cast<int>(joint.get_id()) << ": " << joint.get_position() << std::endl;
      }
    }
  }
}
