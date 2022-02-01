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
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari
{

namespace joint
{

JointManager::JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager)
{
  torque_enable(true);
}

bool JointManager::torque_enable(const bool & enable)
{
  control_manager->write_packet(
    tachimawari::control::packet::protocol_1::PacketId::CONTROLLER,
    protocol_1::MX28Address::TORQUE_ENABLE, static_cast<int>(enable));
}

bool JointManager::torque_enable(const std::vector<Joint> & joints, const bool & enable)
{
  if (joints.size()) {
    for (auto & joint : joints) {
      control_manager->write_packet(
        joint.get_id(), protocol_1::MX28Address::TORQUE_ENABLE,
        static_cast<int>(enable));
    }
  }
}

bool JointManager::set_joints(const std::vector<Joint> & joints)
{
  if (joints.size()) {
    control_manager->sync_write_packet(joints);
  }
}

}  // namespace joint

}  // namespace tachimawari
