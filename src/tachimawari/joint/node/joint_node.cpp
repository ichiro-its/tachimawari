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
#include "tachimawari_interfaces/msg/set_torques.hpp"
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

  set_torques_subscriber = node->create_subscription<tachimawari_interfaces::msg::SetTorques>(
    get_node_prefix() + "/set_torques", 10,
    [this](const tachimawari_interfaces::msg::SetTorques::SharedPtr message) {
      std::vector<Joint> joints;
      std::transform(
        message->ids.begin(), message->ids.end(),
        std::back_inserter(joints), [](uint8_t id) -> Joint {return Joint(id, 0);});

      this->joint_manager->torque_enable(joints, message->torque_enable);
    }
  );

  {
    using tachimawari_interfaces::srv::GetJoints;

    get_joints_server = node->create_service<tachimawari_interfaces::srv::GetJoints>(
      get_node_prefix() + "/get_joints",
      [this](GetJoints::Request::SharedPtr request, GetJoints::Response::SharedPtr response) {
        {
          using tachimawari_interfaces::msg::Joint;

          const auto & current_joints = this->joint_manager->get_current_joints();
          auto & joints = response->joints;

          joints.resize(current_joints.size());
          for (size_t i = 0; i < joints.size() && i < current_joints.size(); ++i) {
            joints[i].id = current_joints[i].get_id();
            joints[i].position = current_joints[i].get_position();
          }
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
