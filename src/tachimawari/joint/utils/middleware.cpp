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
#include <chrono>
#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/utils/middleware.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

namespace tachimawari::joint
{

Middleware::Middleware(double time_limit, std::chrono::milliseconds time_unit)
: action_control(time_limit, time_unit.count() / 1000.0),
  head_control(time_limit, time_unit.count() / 1000.0),
  walking_control(time_limit, time_unit.count() / 1000.0)
{
  reset_ids();
}

void Middleware::reset_ids()
{
  control_rule = DEFAULT;

  action_ids.clear();
  std::copy(JointId::list.begin(), JointId::list.end(), std::back_inserter(action_ids));

  head_ids.clear();
  std::copy(JointId::head_ids.begin(), JointId::head_ids.end(), std::back_inserter(head_ids));

  walking_ids.clear();
  std::copy(JointId::body_ids.begin(), JointId::body_ids.end(), std::back_inserter(walking_ids));
}

void Middleware::set_rules(int control_type, const std::vector<uint8_t> & ids)
{
  control_rule = control_type;

  std::vector<uint8_t> excluded_ids(ids);
  if (ids.size()) {
    switch (control_type) {
      case FOR_ACTION: action_ids = ids; break;
      case FOR_HEAD: head_ids = ids; break;
      case FOR_WALKING: walking_ids = ids; break;
    }
  } else {
    switch (control_type) {
      case FOR_ACTION:
        excluded_ids.assign(action_ids.begin(), action_ids.end());
        break;

      case FOR_HEAD:
        excluded_ids.assign(head_ids.begin(), head_ids.end());
        break;

      case FOR_WALKING:
        excluded_ids.assign(walking_ids.begin(), walking_ids.end());
        break;
    }
  }

  for (const auto control : {FOR_ACTION, FOR_HEAD, FOR_WALKING}) {
    if (control_type != control) {
      std::vector<uint8_t> * ids = nullptr;

      switch (control_type) {
        case FOR_ACTION: ids = &action_ids; break;
        case FOR_HEAD: ids = &head_ids; break;
        case FOR_WALKING: ids = &walking_ids; break;
      }

      if (ids != nullptr) {
        ids->erase(
          std::remove_if(
            ids->begin(), ids->end(), [&](uint8_t id) {
              return std::find(excluded_ids.begin(), excluded_ids.end(), id) != excluded_ids.end();
            }), ids->end());
      }
    }
  }
}

bool Middleware::validate(int control_type)
{
  if (control_rule == DEFAULT && control_type != FORCE) {
    if (control_type == FOR_ACTION) {
      ++action_control;
    } else if (!action_control.is_controlling()) {
      if (control_type == FOR_HEAD) {
        ++head_control;
      } else if (control_type == FOR_WALKING) {
        ++walking_control;
      }
    } else {
      return false;
    }
  }

  return true;
}

std::vector<Joint> Middleware::filter_joints(
  int control_type,
  const std::vector<tachimawari_interfaces::msg::Joint> & joints_message)
{
  if (control_rule == DEFAULT || control_type == FORCE) {
    std::vector<Joint> joints;
    std::transform(
      joints_message.begin(), joints_message.end(),
      std::back_inserter(joints),
      [](tachimawari_interfaces::msg::Joint joint) -> Joint {
        return Joint(joint.id, joint.position);
      });

    if (control_type == FOR_ACTION) {
      return joints;
    } else if (!action_control.is_controlling() || control_type == FORCE) {
      return joints;
    }
  } else {
    std::vector<Joint> joints;
    std::vector<uint8_t> * ids = nullptr;

    switch (control_type) {
      case FOR_ACTION: ids = &action_ids; break;
      case FOR_HEAD: ids = &head_ids; break;
      case FOR_WALKING: ids = &walking_ids; break;
    }

    if (ids != nullptr) {
      for (const auto & joint : joints_message) {
        if (std::find(ids->begin(), ids->end(), joint.id) != ids->end()) {
          joints.push_back(Joint(joint.id, joint.position));
        }
      }
    }

    return joints;
  }

  return {};
}

void Middleware::update()
{
  action_control.update();
  head_control.update();
  walking_control.update();

  if (!action_control.is_controlling() && control_rule == FOR_ACTION) {
    reset_ids();
  }
}

}  // namespace tachimawari::joint
