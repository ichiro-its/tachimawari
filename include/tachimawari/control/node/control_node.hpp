// Copyright (c) 2021-2023 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TACHIMAWARI__CONTROL__NODE__CONTROL_NODE_HPP_
#define TACHIMAWARI__CONTROL__NODE__CONTROL_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari_interfaces/msg/packet.hpp"
#include "tachimawari_interfaces/msg/status.hpp"

namespace tachimawari::control
{

class ControlNode
{
public:
  using Status = tachimawari_interfaces::msg::Status;
  using Packet = tachimawari_interfaces::msg::Packet;

  static std::string get_node_prefix();
  static std::string status_topic();
  static std::string write_packet_topic();

  ControlNode(rclcpp::Node::SharedPtr node, std::shared_ptr<ControlManager> control_manager);

  void update();

private:
  std::shared_ptr<ControlManager> control_manager;

  rclcpp::Publisher<Status>::SharedPtr status_publisher;

  rclcpp::Subscription<Packet>::SharedPtr write_packet_subscriber;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__NODE__CONTROL_NODE_HPP_
