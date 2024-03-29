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

#include "tachimawari/imu/node/imu_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "keisan/keisan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/imu/node/imu_provider.hpp"

namespace tachimawari::imu
{

std::string ImuNode::get_node_prefix() {return "imu";}

std::string ImuNode::unit_topic() {return get_node_prefix() + "/unit";}

ImuNode::ImuNode(rclcpp::Node::SharedPtr node, std::shared_ptr<ImuProvider> imu_provider)
: imu_provider(imu_provider)
{
  unit_publisher = node->create_publisher<Unit>(unit_topic(), 10);
}

void ImuNode::update()
{
  auto unit_msg = Unit();

  auto gyro = imu_provider->get_gyro();
  auto accelero = imu_provider->get_accelero();

  unit_msg.gyro.roll = gyro[0];
  unit_msg.gyro.pitch = gyro[1];
  unit_msg.gyro.yaw = gyro[2];

  unit_msg.accelero.x = accelero[0];
  unit_msg.accelero.y = accelero[1];
  unit_msg.accelero.z = accelero[2];

  unit_publisher->publish(unit_msg);
}

}  // namespace tachimawari::imu
