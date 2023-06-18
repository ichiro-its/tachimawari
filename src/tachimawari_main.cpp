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

#include <memory>
#include <string>

#include "tachimawari/control/control.hpp"
#include "tachimawari/node/tachimawari_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  std::string mode = argv[1];
  std::shared_ptr<tachimawari::control::ControlManager> controller;

  if (mode == "sdk") {
    controller = std::make_shared<tachimawari::control::DynamixelSDK>("/dev/ttyUSB0");
  } else if (mode == "cm740") {
    controller = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
  } else {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  if (!controller->connect()) {
    controller->set_port("/dev/ttyUSB1");

    if (!controller->connect()) {
      std::cout << "failed to connect controller\n";
      return 1;
    }
  }

  auto node = std::make_shared<rclcpp::Node>("tachimawari_node");
  auto tachimawari_node = std::make_shared<tachimawari::TachimawariNode>(node, controller);

  tachimawari_node->run_joint_manager();
  tachimawari_node->run_imu_provider();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
