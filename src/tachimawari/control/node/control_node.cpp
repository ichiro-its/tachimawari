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

#include "tachimawari/control/node/control_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari/control/controller/module/cm740_address.hpp"
#include "tachimawari/control/manager/control_manager.hpp"

namespace tachimawari::control
{

std::string ControlNode::get_node_prefix() { return "control"; }

std::string ControlNode::status_topic() { return get_node_prefix() + "/status"; }

std::string ControlNode::write_packet_topic() { return get_node_prefix() + "/write_packet"; }

ControlNode::ControlNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<ControlManager> control_manager)
: control_manager(control_manager)
{
  status_publisher = node->create_publisher<Status>(status_topic(), 10);

  write_packet_subscriber = node->create_subscription<Packet>(
    write_packet_topic(), 10, [this](const Packet::SharedPtr message) {
      this->control_manager->write_packet(
        message->id, message->address, message->value, message->length);
    });
}

void ControlNode::update()
{
  auto status_msg = Status();

  // Button read is now from kansei
  // status_msg.button =
  //   control_manager->get_bulk_data(ControlManager::CONTROLLER, CM740Address::BUTTON, 1);
  status_msg.led_panel =
    control_manager->get_bulk_data(ControlManager::CONTROLLER, CM740Address::LED_PANNEL, 1);

  status_publisher->publish(status_msg);
}

}  // namespace tachimawari::control
