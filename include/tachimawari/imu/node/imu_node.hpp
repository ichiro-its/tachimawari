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

#ifndef TACHIMAWARI__IMU__NODE__IMU_NODE_HPP_
#define TACHIMAWARI__IMU__NODE__IMU_NODE_HPP_

#include <memory>
#include <string>

#include "kansei_interfaces/msg/unit.hpp"
#include "tachimawari/imu/node/imu_provider.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tachimawari::imu
{

class ImuNode
{
public:
  using Unit = kansei_interfaces::msg::Unit;

  static std::string get_node_prefix();
  static std::string unit_topic();

  ImuNode(rclcpp::Node::SharedPtr node, std::shared_ptr<ImuProvider> imu_provider);

  void update_imu();

private:
  std::shared_ptr<ImuProvider> imu_provider;

  rclcpp::Publisher<Unit>::SharedPtr unit_publisher;
};

}  // namespace tachimawari::imu

#endif  // TACHIMAWARI__IMU__NODE__IMU_NODE_HPP_
