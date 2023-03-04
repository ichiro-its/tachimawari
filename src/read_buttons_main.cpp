// Copyright (c) 2021 Ichiro ITS
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

#include "tachimawari/control/control.hpp"
#include "tachimawari/joint/joint.hpp"
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

  // controller->bulk_read_proccess(tachimawari::control::DynamixelSDK::MARIN_CORE, 64, 16);

  

  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok()) {
    rcl_rate.sleep();

    if (!controller->bulk_read_packet()) {
      std::cout << "bulk read packet failed\n";
      return 1;
    }

    // controller->get_data_bulk();
    std::cout<< "success\n";

    int start = controller->get_bulk_data(tachimawari::control::CM740::CONTROLLER, 40, 2);
    int stop  = controller->get_bulk_data(tachimawari::control::CM740::CONTROLLER, 44, 2);

    int yaw_r   = controller->get_data(tachimawari::control::DynamixelSDK::MARIN_CORE, 64u, 2);
    int pitch_r = controller->get_data(tachimawari::control::DynamixelSDK::MARIN_CORE, 66u, 2) - 360;
    int roll_r  = controller->get_data(tachimawari::control::DynamixelSDK::MARIN_CORE, 68u, 2) - 360;

    std::cout << "start: " << start << ", stop: " << stop << "\n";
    std::cout << "yaw: " << yaw_r << ", pitch: " << pitch_r << ", roll: " << roll_r << "\n";
  }

  return 0;
}
