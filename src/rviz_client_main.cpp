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

#include <cstdlib>
#include <iostream>

#include "musen/musen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/node/rviz_client_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int port = 5000;
  std::string addr = "127.0.0.1";

  for (int i = 0; i < argc; i++) {
    if ((std::string)argv[i] == "-port") {
      if (argc < (i + 1)) {
        std::cerr << "Please specify the port!" << std::endl;
      }
      port = atoi(argv[i + 1]);
    }
    if ((std::string)argv[i] == "-addr") {
      if (argc < (i + 1)) {
        std::cerr << "Please specify the addr!" << std::endl;
      }
      addr = argv[i + 1];
    }
  }

  musen::Address server_address(addr, port);

  auto node = std::make_shared<rclcpp::Node>("rviz_client");
  std::cout << "Connecting to server" << std::endl;
  auto client = musen::Client(server_address);
  auto rvizclient = std::make_shared<tachimawari::RvizClientNode>(node, client);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
