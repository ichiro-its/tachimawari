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

#ifndef TACHIMAWARI__NODE__RVIZ_SERVER_NODE_HPP_
#define TACHIMAWARI__NODE__RVIZ_SERVER_NODE_HPP_

#include <list>
#include <memory>
#include <string>

#include "musen/musen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
namespace tachimawari
{
typedef struct
{
  int id;
  int position;
} rviz_transfer_message_item;

typedef struct
{
  rviz_transfer_message_item data[20];
} rviz_transfer_message;

class RvizServerNode
{
public:
  RvizServerNode(const rclcpp::Node::SharedPtr node, const int port, const int mode);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;
  rclcpp::TimerBase::SharedPtr node_timer_dummy;
  std::list<std::shared_ptr<musen::Session>> sessions;
  std::shared_ptr<tachimawari::joint::JointManager> joint_manager;
  std::shared_ptr<tachimawari::control::ControlManager> control_manager;
  musen::Server server;
  rviz_transfer_message read_joint();
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__NODE__RVIZ_SERVER_NODE_HPP_
