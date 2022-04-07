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

#include <algorithm>
#include <memory>
#include <vector>

#include "tachimawari/joint/node/joint_manager.hpp"

#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::joint
{

JointManager::JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager), is_each_joint_updated(false)
{
  torque_enable(true);

  for (auto id : JointId::list) {
    current_joints.push_back(Joint(id, 0.0));
  }
}

void JointManager::update_current_joints(const std::vector<Joint> & joints)
{
  for (const auto & joint : joints) {
    for (auto & current_joint : current_joints) {
      if (current_joint.get_id() == joint.get_id()) {
        current_joint.set_position(joint.get_position());
        current_joint.set_pid_gain(
          joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);

        break;
      }
    }
  }

  is_each_joint_updated = true;
}

void JointManager::update_current_joints_from_control_manager(const std::vector<Joint> & joints)
{
  std::vector<Joint> new_joints(joints);
  for (auto & joint : new_joints) {
    float value = Joint::CENTER_VALUE;

    int current_value = control_manager->read_packet(
      joint.get_id(), protocol_1::MX28Address::PRESENT_POSITION_L, 2);

    value = (current_value == -1) ? value : current_value;

    joint.set_position_value(value);
  }

  update_current_joints(new_joints);
}

const std::vector<Joint> & JointManager::get_current_joints()
{
  if (!is_each_joint_updated) {
    update_current_joints_from_control_manager(current_joints);
  }

  return current_joints;
}

bool JointManager::torque_enable(bool enable)
{
  return control_manager->write_packet(
    tachimawari::control::ControlManager::BROADCAST,
    protocol_1::MX28Address::TORQUE_ENABLE, enable);
}

bool JointManager::torque_enable(const std::vector<Joint> & joints, bool enable)
{
  if (std::any_of(
      joints.begin(), joints.end(), [&](Joint joint) {
        return !control_manager->write_packet(
          joint.get_id(), protocol_1::MX28Address::TORQUE_ENABLE, enable);
      }))
  {
    return false;
  }

  if (enable) {
    update_current_joints_from_control_manager(joints);
  }

  return true;
}

bool JointManager::set_joints(const std::vector<Joint> & joints)
{
  if (joints.size()) {
    update_current_joints(joints);

    return control_manager->sync_write_packet(joints);
  }

  return false;
}

}  // namespace tachimawari::joint
