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

#include "tachimawari/node/tachimawari_node.hpp"

#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/imu/node/imu_node.hpp"
#include "tachimawari/imu/node/imu_provider.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari/joint/node/joint_node.hpp"

using namespace std::chrono_literals;

namespace tachimawari
{

TachimawariNode::TachimawariNode(rclcpp::Node::SharedPtr node)
: node(node), control_manager(nullptr), joint_node(nullptr), imu_node(nullptr)
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (this->control_manager) {
        if (this->imu_node) {
          this->control_manager->bulk_read_packet();

          this->imu_node->update_imu();
        }

        if (this->joint_node) {
          this->joint_node->update();
          
          this->joint_node->publish_current_joints();
        }
      }
    }
  );
}

void TachimawariNode::set_joint_manager(
  std::shared_ptr<control::ControlManager> control_manager)
{
  joint_node = std::make_shared<joint::JointNode>(
    node,
    std::make_shared<joint::JointManager>(control_manager));
}

void TachimawariNode::set_imu_provider(
  std::shared_ptr<control::ControlManager> control_manager)
{
  imu_node = std::make_shared<imu::ImuNode>(
    node,
    std::make_shared<imu::ImuProvider>(control_manager));
}

}  // namespace tachimawari
