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

#include <chrono>
#include <memory>
#include <string>
#include <list>

#include "tachimawari/node/rviz_server_node.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"
#include "tachimawari/joint/joint.hpp"

using namespace std::chrono_literals;

namespace tachimawari
{

  RvizServerNode::RvizServerNode(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<tachimawari::control::ControlManager> controller,
      int port)
      : node(node), jointmanager(std::make_shared<tachimawari::joint::JointManager>(controller)),
        controlmanager(controller), server(port), sessions()
  {
    node_timer = node->create_wall_timer(
        50ms,
        [this]()
        {
          auto new_session = server.accept();
          if (new_session != nullptr)
          {
            sessions.push_back(new_session);
          }
          for (auto session : sessions)
          {
            auto joints = jointmanager->get_current_joints();
            std::vector<tachimawari::joint::Joint> new_joints(joints);
            for (auto &joint : new_joints)
            {
              float value = tachimawari::joint::Joint::CENTER_VALUE;
              int current_value = controlmanager->read_packet(
                  joint.get_id(), tachimawari::joint::protocol_1::MX28Address::PRESENT_POSITION_L, 2);
              value = (current_value == -1) ? value : current_value;
              joint.set_position_value(value);
            }
            for (const auto &joint : new_joints)
            {
              for (auto &current_joint : joints)
              {
                if (current_joint.get_id() == joint.get_id())
                {
                  current_joint.set_position(joint.get_position());
                  current_joint.set_pid_gain(
                      joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);
                  break;
                }
              }
            }

            rviz_transfer_message msg;
            int pos = 0;
            for (auto joint : joints)
            {
              rviz_transfer_message_item item;
              item.id = joint.get_id();
              item.position = joint.get_position();
              msg.data[pos] = item;
              pos++;
              RCLCPP_INFO(rclcpp::get_logger("rviz_server"), " %i : %i", joint.get_id(), joint.get_pid_gain());
            }
            if (pos != 20 - 1)
            {
              rviz_transfer_message_item nulldata;
              nulldata.id = 0;
              nulldata.position = 0;
              msg.data[pos] = nulldata;
              pos++;
            }
            session->send<rviz_transfer_message>(msg);
          }
        });
  }

  // dummy
  RvizServerNode::RvizServerNode(
      rclcpp::Node::SharedPtr node,
      int port)
      : node(node), jointmanager(nullptr), controlmanager(nullptr), server(port)
  {
    rviz_transfer_message dummy[10];
    for (int i = 0; i < 10; i++)
    {
      for (int j = 0; j < 20; j++)
      {
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

    node_timer_dummy = node->create_wall_timer(
        1ms,
        [this, dummy]()
        {
          auto new_session = this->server.accept();
          if (new_session != nullptr)
          {
            this->sessions.push_back(new_session);
          }

          for (auto session : this->sessions)
          {
            for (int i = 0; i < 3; i++)
            {
              auto data = dummy[i];
              for (int j = 0; j <= 20; j++)
              {
                RCLCPP_INFO(rclcpp::get_logger("rviz_server"), "%d : %d", data.data[j].id, data.data[j].position);
              }
              session->send<rviz_transfer_message>(data);
            }
          }
        });
  }
}
