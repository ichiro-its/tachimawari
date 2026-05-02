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

#include <iostream>
#include <memory>
#include <string>

#include "tachimawari/control/control.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " [sdk / cm740] [port(optional)]" << std::endl;
    return 1;
  }

  std::string mode = argv[1];
  std::string port = "/dev/ttyUSB0";
  if (argc >= 3) {
    port = argv[2];
  }

  std::shared_ptr<tachimawari::control::ControlManager> controller;

  if (mode == "sdk") {
    controller = std::make_shared<tachimawari::control::DynamixelSDK>(port);
  } else if (mode == "cm740") {
    controller = std::make_shared<tachimawari::control::CM740>(port);
  } else {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 1;
  }

  std::cout << "Using port: " << port << std::endl;

  if (!controller->connect()) {
    std::cout << "Failed to connect on " << port << std::endl;
    return 1;
  }

  {
    using tachimawari::joint::JointId;

    for (const auto & [key, value] : JointId::by_name) {
      std::cout << "ping " << key << ": ";
      if (controller->ping(value)) {
        std::cout << "\033[32m" << "success" << "\033[0m" << "\n";
      } else {
        std::cout << "\033[31m" << "failed" << "\033[0m" << "\n";
      }
    }
  }

  return 0;
}
