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

#include "tachimawari/joint/node/joint_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/node/joint_manager.hpp"
#include "tachimawari_interfaces/msg/control_joints.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/msg/set_torques.hpp"

namespace tachimawari::joint
{

std::string JointNode::get_node_prefix() { return "joint"; }

std::string JointNode::control_joints_topic() { return get_node_prefix() + "/control_joints"; }

std::string JointNode::set_joints_topic() { return get_node_prefix() + "/set_joints"; }

std::string JointNode::set_torques_topic() { return get_node_prefix() + "/set_torques"; }

std::string JointNode::status_topic() { return "measurement/status"; }

std::string JointNode::current_joints_topic() { return get_node_prefix() + "/current_joints"; }

JointNode::JointNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<JointManager> joint_manager, std::string path)
: joint_manager(joint_manager),
  middleware(),
  tf2_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(node)),
  tf2_manager(std::make_shared<Tf2Manager>()),
  imu_yaw(keisan::make_degree(0))
{
  tf2_manager->load_configuration(path);

  control_joints_subscriber = node->create_subscription<ControlJoints>(
    control_joints_topic(), 10, [this](const ControlJoints::SharedPtr message) {
      this->middleware.set_rules(message->control_type, message->ids);
    });

  set_joints_subscriber = node->create_subscription<SetJoints>(
    set_joints_topic(), 10, [this](const SetJoints::SharedPtr message) {
      if (this->middleware.validate(message->control_type)) {
        this->joint_manager->set_joints(
          this->middleware.filter_joints(message->control_type, message->joints));
      }
    });

  set_torques_subscriber = node->create_subscription<SetTorques>(
    set_torques_topic(), 10, [this](const SetTorques::SharedPtr message) {
      std::vector<Joint> joints;
      std::transform(
        message->ids.begin(), message->ids.end(), std::back_inserter(joints),
        [](uint8_t id) -> Joint { return Joint(id, 0); });

      this->joint_manager->torque_enable(joints, message->torque_enable);
    });

  status_subscriber =
    node->create_subscription<Status>(status_topic(), 10, [this](const Status::SharedPtr message) {
      imu_yaw = keisan::make_degree(message->orientation.yaw);
    });

  current_joints_publisher = node->create_publisher<CurrentJoints>(current_joints_topic(), 10);
}

void JointNode::update() { middleware.update(); }

void JointNode::publish_current_joints()
{
  const auto & current_joints = this->joint_manager->get_current_joints();
  auto msg_joints = CurrentJoints();
  auto & joints = msg_joints.joints;

  joints.resize(current_joints.size());
  for (size_t i = 0; i < joints.size() && i < current_joints.size(); ++i) {
    joints[i].id = current_joints[i].get_id();
    joints[i].position = current_joints[i].get_position();
  }

  current_joints_publisher->publish(msg_joints);
}

void JointNode::publish_frame_tree()
{
  const auto & current_joints = this->joint_manager->get_current_joints();
  tf2_manager->update(current_joints, imu_yaw);

  rclcpp::Time now = rclcpp::Clock().now();
  for (auto & frame : tf2_manager->get_frames()) {
    tf2_broadcaster->sendTransform(frame.get_transform_stamped(now));
  }
}

}  // namespace tachimawari::joint
