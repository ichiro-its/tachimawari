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

#include "tachimawari/node/rviz_server_node.hpp"

#include <chrono>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/control.hpp"
#include "tachimawari/joint/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

using namespace std::chrono_literals;

namespace tachimawari
{

RvizServerNode::RvizServerNode(const rclcpp::Node::SharedPtr node, const int port, const int mode)
: node(node), server(port)
{
  switch (mode) {
    case 1:  // sdk
      control_manager = std::make_shared<tachimawari::control::DynamixelSDK>("/dev/ttyUSB0");
      if (!control_manager->connect()) {
        control_manager->set_port("/dev/ttyUSB1");
        if (!control_manager->connect()) {
          RCLCPP_ERROR(rclcpp::get_logger("RvizServerNode"), "can`t connect to sdk");
        }
      }
      joint_manager = std::make_shared<tachimawari::joint::JointManager>(control_manager);
      node_timer = node->create_wall_timer(
        8ms, [this]() {
          auto new_session = server.accept();
          if (new_session != nullptr) {
            this->sessions.push_back(new_session);
          }
          for (auto session : this->sessions) {
            auto data = read_joint();
            session->send<rviz_transfer_message>(data);
          }
        });
      break;

    case 2:  // cm740
      control_manager = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
      if (!control_manager->connect()) {
        control_manager->set_port("/dev/ttyUSB1");
        if (!control_manager->connect()) {
          RCLCPP_ERROR(rclcpp::get_logger("RvizServerNode"), "can`t connect to CM740");
        }
      }
      joint_manager = std::make_shared<tachimawari::joint::JointManager>(control_manager);
      node_timer = node->create_wall_timer(
        8ms, [this]() {
          auto new_session = server.accept();
          if (new_session != nullptr) {
            this->sessions.push_back(new_session);
          }
          for (auto session : this->sessions) {
            auto data = read_joint();
            session->send<rviz_transfer_message>(data);
          }
        });
      break;

    case 3:  // dummy
      rviz_transfer_message dummy[10];
      for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 20; ++j) {
          dummy[i].data[j].id = j;
          dummy[i].data[j].position = 2048;
        }
      }
      dummy[0].data[0].id = 0;
      dummy[0].data[0].position = 2568;
      dummy[0].data[1].id = 1;
      dummy[0].data[1].position = 2568;
      dummy[0].data[2].id = 2;
      dummy[0].data[2].position = 2568;
      dummy[0].data[1].id = 1;
      dummy[0].data[1].position = 2098;
      dummy[0].data[2].id = 2;
      dummy[0].data[2].position = 2098;

      node_timer = node->create_wall_timer(
        8ms, [this, dummy]() {
          auto new_session = server.accept();
          if (new_session != nullptr) {
            this->sessions.push_back(new_session);
          }
          for (auto session : this->sessions) {
            for (int i = 0; i < 3; ++i) {
              auto data = dummy[i];
              for (int j = 0; j < 20; ++j) {
                RCLCPP_INFO(
                  rclcpp::get_logger("rviz_server"), "[size of buffer : %d] %d : %d", sizeof(data),
                  data.data[j].id + 1, data.data[j].position);
              }
              session->send<rviz_transfer_message>(data);
            }
          }
        });
      break;

    default:
      RCLCPP_ERROR(rclcpp::get_logger("RvizServerNode"), "invalid mode");
      break;
  }
}

rviz_transfer_message RvizServerNode::read_joint()
{
  auto joints = joint_manager->get_current_joints();
  std::vector<tachimawari::joint::Joint> new_joints(joints);
  for (auto & joint : new_joints) {
    float value = tachimawari::joint::Joint::CENTER_VALUE;

    int current_value = control_manager->read_packet(
      joint.get_id(), tachimawari::joint::protocol_1::MX28Address::PRESENT_POSITION_L, 2);

    value = (current_value == -1) ? value : current_value;

    joint.set_position_value(value);
  }
  for (const auto & joint : new_joints) {
    for (auto & current_joint : joints) {
      if (current_joint.get_id() == joint.get_id()) {
        current_joint.set_position(joint.get_position());
        current_joint.set_pid_gain(
          joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);

        break;
      }
    }
  }
  rviz_transfer_message msg;
  int pos = 0;
  for (const auto joint : joints) {
    rviz_transfer_message_item item;
    item.id = static_cast<int>(joint.get_id());
    item.position = joint.get_position_value();
    msg.data[pos] = item;
    pos++;
    RCLCPP_INFO(
      rclcpp::get_logger("rviz_server"), "[size of buffer : %i] %i : %i", sizeof(item), item.id,
      item.position);
  }
  return msg;
}

}  // namespace tachimawari
