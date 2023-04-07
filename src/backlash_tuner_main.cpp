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
#include <sstream>

#include "tachimawari/control/control.hpp"
#include "tachimawari/node/tachimawari_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
  bool read_all = false;
  std::string target_joint_str;
  int target_joint;
  double max = -1000, min = 1000;
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    std::cerr << "Please specify the mode! [sdk / cm740]" << std::endl;
  }

  if (argc < 3)
  {
    read_all = true;
  }
  else
  {
    target_joint_str = argv[2];
    target_joint = stoi(target_joint_str);
  }

  std::string mode = argv[1];
  std::shared_ptr<tachimawari::control::ControlManager> controller;

  if (mode == "sdk")
  {
    controller = std::make_shared<tachimawari::control::DynamixelSDK>("/dev/ttyUSB0");
  }
  else if (mode == "cm740")
  {
    controller = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
  }
  else
  {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  if (!controller->connect())
  {
    controller->set_port("/dev/ttyUSB1");

    if (!controller->connect())
    {
      std::cout << "failed to connect controller\n";
      return 1;
    }
  }

  // init_joints_main.cpp
  auto joint_manager = std::make_shared<tachimawari::joint::JointManager>(controller);
  std::vector<tachimawari::joint::Joint> joints;

  if (!joint_manager->set_joints(joints))
  {
    std::cout << "failed to sync write\n";
  }

  // spin
  auto node = std::make_shared<rclcpp::Node>("tachimawari_node");
  tachimawari::TachimawariNode tachimawari_node(node, controller);
  joints = joint_manager->get_current_joints();

  // spin and read only one servo
  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok())
  {
    rcl_rate.sleep();
    rclcpp::spin_some(node);

    // read_joints_main.cpp
    if (!read_all)
    {
      for (const auto &joint : joints)
      {
        if (target_joint == static_cast<int>(joint.get_id()))
        {
          min = joint.get_position() > min ? min : joint.get_position();
          max = joint.get_position() < max ? max : joint.get_position();
          std::cout << "id " << static_cast<int>(joint.get_id()) << ": " << joint.get_position() << "\n";
          std::cout << "minimum: " << min << "\n";
          std::cout << "maximum: " << max << "\n";
        }
      }
    }
    else
    {
      // spin and read all servos
      for (const auto &joint : joints)
      {
        std::cout << "id " << static_cast<int>(joint.get_id()) << ": " << joint.get_position() << "\n";
      }
    }
  }
}
