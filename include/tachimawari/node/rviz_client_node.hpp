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

#ifndef TACHIMAWARI__NODE__RVIZ_CLIENT_NODE_HPP_
#define TACHIMAWARI__NODE__RVIZ_CLIENT_NODE_HPP_

#include "musen/musen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

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

class RvizClientNode
{
public:
  RvizClientNode(const rclcpp::Node::SharedPtr & node, const musen::Client & client);

private:
  void register_joint(
    sensor_msgs::msg::JointState & joint_state_msg, const std::string & name, const double & pos);
  double val2rad(int val);
  rclcpp::Node::SharedPtr node;
  musen::Client client;
  rclcpp::TimerBase::SharedPtr node_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state;
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__NODE__RVIZ_CLIENT_NODE_HPP_
