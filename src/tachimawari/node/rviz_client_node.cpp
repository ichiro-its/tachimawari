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

#include "tachimawari/node/rviz_client_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari_interfaces/msg/current_joints.hpp"

#define _USE_MATH_DEFINES

using namespace std::chrono_literals;

const char joint_id[20][22] = {
  "right_shoulder_pitch",  // 1
  "left_shoulder_pitch",   // 2
  "right_shoulder_roll",   // 3
  "left_shoulder_roll",    // 4
  "right_elbow",           // 5
  "left_elbow",            // 6
  "right_hip_roll",        // 7
  "left_hip_roll",         // 8
  "right_hip_yaw",         // 9
  "left_hip_yaw",          // 10
  "right_hip_pitch",       // 11
  "left_hip_pitch",        // 12
  "right_knee",            // 13
  "left_knee",             // 14
  "right_ankle_roll",      // 15
  "left_ankle_roll",       // 16
  "right_ankle_pitch",     // 17
  "left_ankle_pitch",      // 18
  "head_pan",              // 19
  "head_tilt"              // 20
};

float direction[20] = {
  1,   // 1
  -1,  // 2
  -1,  // 3
  -1,  // 4
  -1,  // 5
  1,   // 6
  -1,  // 7
  -1,  // 8
  -1,  // 9
  -1,  // 10
  -1,  // 11 //
  1,   // 12 //
  -1,  // 13 //
  1,   // 14 //
  1,   // 15
  1,   // 16
  1,   // 17 //
  1,   // 18 //
  1,   // 19
  1,   // 20
};

namespace tachimawari
{

RvizClientNode::RvizClientNode(const rclcpp::Node::SharedPtr & node, const musen::Client & client)
: node(node), client(client)
{
  this->joint_state =
    this->node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  node_timer = node->create_wall_timer(1ms, [this]() {
    sensor_msgs::msg::JointState joint_state_msg;

    auto data = this->client.receive<rviz_transfer_message>();

    if (data.has_value()) {
      for (int i = 0; i < 20; ++i) {
        register_joint(
          joint_state_msg, joint_id[data->data[i].id - 1],
          val2rad(data->data[i].position * direction[data->data[i].id - 1]));
        RCLCPP_INFO(
          rclcpp::get_logger("rviz_server"), "[size of buffer: %d] %d : %d", sizeof(data),
          data->data[i].id, data->data[i].position);
      }
    }
    joint_state_msg.header.stamp = this->node->get_clock()->now();
    geometry_msgs::msg::TransformStamped world_transform;
    world_transform.header.set__frame_id("world");
    world_transform.set__child_frame_id("robot");
    world_transform.header.stamp = this->node->get_clock()->now();
    world_transform.transform.translation.x = 0.5;
    world_transform.transform.translation.y = 0;
    world_transform.transform.translation.z = 0;
    transform_broadcaster->sendTransform(world_transform);
    register_joint(joint_state_msg, "body_to_robot", 0);
    joint_state->publish(joint_state_msg);
    joint_state_msg.position.clear();
    joint_state_msg.name.clear();
  });
}

double RvizClientNode::val2rad(int val)
{
  return (val - 2048.0) * (360.0 / 4096.0) * (M_PI / 180.0);
}

void RvizClientNode::register_joint(
  sensor_msgs::msg::JointState & joint_state_msg, const std::string & name, const double & pos)
{
  joint_state_msg.header.frame_id = "world";
  joint_state_msg.name.push_back(name);
  joint_state_msg.position.push_back(pos);
}

}  // namespace tachimawari
