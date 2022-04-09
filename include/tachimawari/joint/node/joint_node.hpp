// Copyright (c) 2021 Ichiro ITS
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

#ifndef TACHIMAWARI__JOINT__NODE__JOINT_NODE_HPP_
#define TACHIMAWARI__JOINT__NODE__JOINT_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari/joint/utils/middleware.hpp"
#include "tachimawari_interfaces/msg/control_joints.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/msg/set_torques.hpp"
#include "tachimawari_interfaces/srv/get_joints.hpp"

namespace tachimawari::joint
{

class JointNode
{
public:
  using ControlJoints = tachimawari_interfaces::msg::ControlJoints;
  using GetJoints = tachimawari_interfaces::srv::GetJoints;
  using SetJoints = tachimawari_interfaces::msg::SetJoints;
  using SetTorques = tachimawari_interfaces::msg::SetTorques;

  JointNode(rclcpp::Node::SharedPtr node, std::shared_ptr<JointManager> joint_manager);

  void update();

private:
  std::string get_node_prefix() const;

  std::shared_ptr<JointManager> joint_manager;

  Middleware middleware;

  rclcpp::Subscription<ControlJoints>::SharedPtr control_joints_subscriber;

  rclcpp::Service<GetJoints>::SharedPtr get_joints_server;
  rclcpp::Subscription<SetJoints>::SharedPtr set_joints_subscriber;

  rclcpp::Subscription<SetTorques>::SharedPtr set_torques_subscriber;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__NODE__JOINT_NODE_HPP_
