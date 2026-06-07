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

#include "tachimawari/joint/node/joint_manager.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::joint
{

JointManager::JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager), is_each_joint_updated(false), connectivity_poll_index(0)
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

    int current_value =
      control_manager->read_packet(joint.get_id(), protocol_1::MX28Address::PRESENT_POSITION_L, 2);

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

bool JointManager::is_warming_up(uint8_t id) const
{
  auto entry = warm_up_state.find(id);
  if (entry == warm_up_state.end()) {
    return false;
  }

  return (std::chrono::steady_clock::now() - entry->second.started_at) < TORQUE_WARM_UP_DURATION;
}

void JointManager::mark_torque_enabled(const std::vector<uint8_t> & ids)
{
  auto now = std::chrono::steady_clock::now();

  for (auto id : ids) {
    int value = control_manager->read_packet(id, protocol_1::MX28Address::PRESENT_POSITION_L, 2);

    Joint snapshot(id);
    snapshot.set_position_value(value == -1 ? Joint::CENTER_VALUE : value);

    warm_up_state[id] = WarmUpState{now, snapshot.get_position()};
  }
}

Joint JointManager::apply_resume_ramp(const Joint & joint) const
{
  auto entry = warm_up_state.find(joint.get_id());
  if (entry == warm_up_state.end()) {
    return joint;
  }

  auto ramp_elapsed =
    (std::chrono::steady_clock::now() - entry->second.started_at) - TORQUE_WARM_UP_DURATION;

  if (ramp_elapsed >= RESUME_RAMP_DURATION) {
    return joint;
  }

  float blend = std::chrono::duration<float, std::milli>(ramp_elapsed).count() /
    std::chrono::duration<float, std::milli>(RESUME_RAMP_DURATION).count();

  float start_position = entry->second.start_position;

  Joint ramped = joint;
  ramped.set_position(start_position + (joint.get_position() - start_position) * blend);

  return ramped;
}

bool JointManager::torque_enable(bool enable)
{
  if (enable) {
    mark_torque_enabled(std::vector<uint8_t>(JointId::list.begin(), JointId::list.end()));
  }

  return control_manager->write_packet(
    tachimawari::control::ControlManager::BROADCAST, protocol_1::MX28Address::TORQUE_ENABLE,
    enable);
}

bool JointManager::torque_enable(const std::vector<Joint> & joints, bool enable)
{
  if (enable) {
    std::vector<uint8_t> ids;
    ids.reserve(joints.size());
    for (const auto & joint : joints) {
      ids.push_back(joint.get_id());
    }
    
    mark_torque_enabled(ids);
  }

  if (std::any_of(joints.begin(), joints.end(), [&](Joint joint) {
        return !control_manager->write_packet(
          joint.get_id(), protocol_1::MX28Address::TORQUE_ENABLE, enable);
      })) {
    return false;
  }

  if (enable) {
    update_current_joints_from_control_manager(joints);
  }

  return true;
}

bool JointManager::is_connected(uint8_t id) const
{
  auto entry = connectivity.find(id);
  if (entry == connectivity.end()) {
    return true;
  }

  return entry->second.connected;
}

bool JointManager::set_joints(const std::vector<Joint> & joints)
{
  std::vector<Joint> ready_joints;
  for (const auto & joint : joints) {
    if (is_connected(joint.get_id()) && !is_warming_up(joint.get_id())) {
      ready_joints.push_back(apply_resume_ramp(joint));
    }
  }

  if (ready_joints.size()) {
    update_current_joints(ready_joints);

    return control_manager->sync_write_packet(ready_joints);
  }

  return false;
}

void JointManager::update_connectivity()
{
  if (JointId::list.empty()) {
    return;
  }

  uint8_t id = JointId::list[connectivity_poll_index];
  connectivity_poll_index = (connectivity_poll_index + 1) % JointId::list.size();

  bool read_ok =
    control_manager->read_packet(id, protocol_1::MX28Address::PRESENT_POSITION_L, 2) != -1;

  auto & state = connectivity[id];

  if (read_ok == state.connected) {
    state.mismatch_count = 0;
    return;
  }

  if (++state.mismatch_count < CONNECTIVITY_DEBOUNCE_COUNT) {
    return;
  }

  state.connected = read_ok;
  state.mismatch_count = 0;

  if (state.connected) {
    for (const auto & joint : current_joints) {
      if (joint.get_id() == id) {
        torque_enable(std::vector<Joint>{joint}, true);
        break;
      }
    }
  }
}

}  // namespace tachimawari::joint
