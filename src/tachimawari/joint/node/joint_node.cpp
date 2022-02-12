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

#include <memory>
#include <string>
#include <vector>

#include "tachimawari/joint/node/joint_node.hpp"

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/srv/get_joints.hpp"

namespace tachimawari::joint
{

JointNode::JointNode(rclcpp::Node::SharedPtr node, std::shared_ptr<JointManager> joint_manager)
: joint_manager(joint_manager)
{
  set_joints_subscriber = node->create_subscription<tachimawari_interfaces::msg::SetJoints>(
    get_node_prefix() + "/set_joints", 10,
    [this](const tachimawari_interfaces::msg::SetJoints::SharedPtr message) {
      std::vector<Joint> joints;

      for (const auto & joint : message->joints) {
        joints.push_back(Joint(joint.id, joint.position));
      }

      this->joint_manager->set_joints(joints);
    }
  );

  {
    using tachimawari_interfaces::srv::GetJoints;

    get_joints_server = node->create_service<tachimawari_interfaces::srv::GetJoints>(
      get_node_prefix() + "/get_joints",
      [this](std::shared_ptr<GetJoints::Request> request,
      std::shared_ptr<GetJoints::Response> response) {
        {
          using tachimawari_interfaces::msg::Joint;

          std::vector<Joint> joints_msg;
          for (const auto & joint : this->joint_manager->get_current_joints()) {
            auto joint_msg = Joint();
            joint_msg.id = joint.get_id();
            joint_msg.position = joint.get_position();

            joints_msg.push_back(joint_msg);
          }

          response->joints = joints_msg;
        }
      }
    );
  }
}

std::string JointNode::get_node_prefix() const
{
  return "joint";
}

}  // namespace tachimawari::joint
