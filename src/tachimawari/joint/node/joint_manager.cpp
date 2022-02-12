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
#include <vector>

#include "tachimawari/joint/node/joint_manager.hpp"

#include "tachimawari/control/packet/protocol_1/model/packet_id.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::joint
{

JointManager::JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager), is_joints_uptodate(false)
{
  torque_enable(true);

  for (auto id : JointId::list) {
    current_joints.push_back(Joint(id, 0.0));
  }
}

void JointManager::update_current_joints(const std::vector<Joint> & joints)
{
  for (const auto & joint : joints) {
    for (size_t index = 0; index < current_joints.size(); ++index) {
      if (current_joints[index].get_id() == joint.get_id()) {
        current_joints[index].set_position(joint.get_position());
        current_joints[index].set_pid_gain(
          joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);
      }
    }
  }

  is_joints_uptodate = true;
}

std::vector<Joint> JointManager::get_current_joints()
{
  if (!is_joints_uptodate) {
    if (control_manager->bulk_read_packet(current_joints)) {
      for (size_t index = 0; index < current_joints.size(); ++index) {
        float position = 0.0;

        if (control_manager->get_protocol_version() == 1.0) {
          int current_position = control_manager->get_bulk_data(
            current_joints[index].get_id(), protocol_1::MX28Address::PRESENT_POSITION_L, 2);
          position = (current_position == -1) ? 0.0 : current_position;
        }

        current_joints[index].set_position(position);
      }
    }
  }

  return current_joints;
}

bool JointManager::torque_enable(bool enable)
{
  return control_manager->write_packet(
    tachimawari::control::packet::protocol_1::PacketId::CONTROLLER,
    protocol_1::MX28Address::TORQUE_ENABLE, enable);
}

bool JointManager::torque_enable(const std::vector<Joint> & joints, bool enable)
{
  if (joints.size()) {
    for (const auto & joint : joints) {
      return control_manager->write_packet(
        joint.get_id(), protocol_1::MX28Address::TORQUE_ENABLE, enable);
    }
  }

  return false;
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
