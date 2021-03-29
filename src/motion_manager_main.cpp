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

#include <rclcpp/rclcpp.hpp>

#include <tachimawari/motion_manager.hpp>

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string port_name = "tty/ACM0";

  // change the port name
  if (argc > 1) {
    port_name = argv[1];
  }

  // init node
  auto motion_manager = std::make_shared<tachimawari::MotionManager>("motion_manager", port_name);

  // open the port
  if (motion_manager->start()) {
    rclcpp::spin(motion_manager);
  }

  // close the port
  motion_manager->stop();

  rclcpp::shutdown();
}
