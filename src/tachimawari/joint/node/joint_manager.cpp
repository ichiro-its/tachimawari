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
#include <chrono>
#include <memory>
#include <vector>

#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::joint
{

JointManager::JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager), running(true)
{
  torque_enable(true);

  for (auto id : JointId::list) {
    control_manager->write_packet(id, protocol_1::MX28Address::RETURN_DELAY_TIME, 0);
    current_joints.push_back(Joint(id, 0.0));
  }

  read_thread = std::thread(&JointManager::read_loop, this);
}

JointManager::~JointManager()
{
  running = false;
  if (read_thread.joinable()) {
    read_thread.join();
  }
}

void JointManager::update_current_joints(const std::vector<Joint> & joints)
{
  for (const auto & joint : joints) {
    for (auto & current_joint : current_joints) {
      if (current_joint.get_id() == joint.get_id()) {
        current_joint.set_position(joint.get_position());
        current_joint.set_velocity(joint.get_velocity());
        current_joint.set_pid_gain(
          joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);

        break;
      }
    }
  }
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
    joint.set_velocity(compute_velocity_from_differential(joint.get_id(), value));
  }

  update_current_joints(new_joints);
}

float JointManager::compute_velocity_from_differential(uint8_t id, int new_position)
{
  auto it = joint_read_state.find(id);
  auto now = std::chrono::steady_clock::now();

  if (it == joint_read_state.end()) {
    // First reading — store initial state, velocity is 0
    joint_read_state[id] = {new_position, now, 0.0f};
    return 0.0f;
  }

  auto & state = it->second;
  int raw_delta = new_position - state.position;

  // Handle wraparound: if delta is more than half a rotation, it wrapped
  if (raw_delta > 2048) {
    raw_delta -= 4096;
  } else if (raw_delta < -2048) {
    raw_delta += 4096;
  }

  if (raw_delta != 0) {
    double delta_sec = std::chrono::duration<double>(now - state.time).count();
    if (delta_sec > 0) {
      state.velocity = static_cast<float>(Joint::value_to_angle(raw_delta).degree() / delta_sec);
    }
    state.position = new_position;
    state.time = now;
  }
  // If raw_delta == 0: position unchanged, keep the last computed velocity and don't update time

  return state.velocity;
}

void JointManager::read_loop()
{
  while (running) {
    std::vector<Joint> snapshot;
    {
      std::lock_guard<std::mutex> lock(joints_mutex);
      snapshot = current_joints;
    }

    std::vector<Joint> updated_joints;
    for (auto & joint : snapshot) {
      int current_value =
        control_manager->read_packet(joint.get_id(), protocol_1::MX28Address::PRESENT_POSITION_L, 2);

      // Skip this joint entirely if the read failed
      if (current_value == -1) {
        continue;
      }

      joint.set_position_value(current_value);
      joint.set_velocity(compute_velocity_from_differential(joint.get_id(), current_value));
      updated_joints.push_back(joint);
    }

    if (!updated_joints.empty()) {
      std::lock_guard<std::mutex> lock(joints_mutex);
      update_current_joints(updated_joints);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

std::vector<Joint> JointManager::get_current_joints()
{
  std::lock_guard<std::mutex> lock(joints_mutex);
  return current_joints;
}

bool JointManager::torque_enable(bool enable)
{
  return control_manager->write_packet(
    tachimawari::control::ControlManager::BROADCAST, protocol_1::MX28Address::TORQUE_ENABLE,
    enable);
}

bool JointManager::torque_enable(const std::vector<Joint> & joints, bool enable)
{
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

bool JointManager::set_joints(const std::vector<Joint> & joints)
{
  if (joints.size()) {
    {
      std::lock_guard<std::mutex> lock(joints_mutex);
      update_current_joints(joints);
    }

    return control_manager->sync_write_packet(joints);
  }

  return false;
}

}  // namespace tachimawari::joint
