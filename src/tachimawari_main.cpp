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

#include <memory>

#include "tachimawari/control/controller/controller.hpp"
#include "tachimawari/node/tachimawari_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto cm740 = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
  if (!cm740->connect()) {
    cm740->set_port("/dev/ttyUSB1");

    if (!cm740->connect()) {
      std::cout << "failed to connect CM740\n";
      return 1;
    }
  }

  auto node = std::make_shared<rclcpp::Node>("tachimawari_node");
  auto tachimawari_node = std::make_shared<tachimawari::TachimawariNode>(node);

  tachimawari_node->set_joint_manager(cm740);
  tachimawari_node->set_imu_provider(cm740);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
