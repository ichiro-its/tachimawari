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

#include <signal.h>

#include <chrono>
#include <memory>
#include <string>

#include "musen/musen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/control/control.hpp"
#include "tachimawari/node/rviz_server_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  int port = 5000;
  if (argc > 2) {
    for (int i = 2; i < argc; i++) {
      if ((std::string)argv[i] == "-port") {
        if (argc < (i + 1)) {
          std::cerr << "Please specify the port!" << std::endl;
        }
        port = atoi(argv[i + 1]);
      }
    }
  }
  std::string mode = argv[1];
  int mode_id = 0;

  if (mode == "sdk") {
    mode_id = 1;
  } else if (mode == "cm740") {
    mode_id = 2;
  } else if (mode == "dummy") {
    mode_id = 3;
  } else {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  auto node = std::make_shared<rclcpp::Node>("rviz_server");

  auto rviz_server = std::make_shared<tachimawari::RvizServerNode>(node, port, mode_id);
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
