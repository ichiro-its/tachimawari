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

#include <chrono>
#include <string>

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/utils/middleware.hpp"
#include "tachimawari_interfaces/msg/joint.hpp"

namespace tachimawari::joint
{

Middleware::Middleware(double time_limit, std::chrono::milliseconds time_unit)
: time_limit(time_limit), time_unit(time_unit.count() / 1000.0), is_action_controlling(false), action_value(0), action_counter(0), action_timer(0), is_head_controlling(false), head_value(0), head_counter(0), head_timer(0), is_walking_controlling(false), walking_value(0), walking_counter(0), walking_timer(0)
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
    if (control_type == FOR_ACTION) {
      action_ids = ids;
    } else if (control_type == FOR_HEAD) {
      head_ids = ids;
    } else if (control_type == FOR_WALKING) {
      walking_ids = ids;
    }
  } else {
    if (control_type == FOR_ACTION) {
      excluded_ids.assign(action_ids.begin(), action_ids.end());
    } else if (control_type == FOR_HEAD) {
      excluded_ids.assign(head_ids.begin(), head_ids.end());
    } else if (control_type == FOR_WALKING) {
      excluded_ids.assign(walking_ids.begin(), walking_ids.end());
    }
  }

  if (control_type != FOR_ACTION) {
    action_ids.erase(std::remove_if(action_ids.begin(), action_ids.end(), [&](uint8_t id) {
      return std::find(excluded_ids.begin(), excluded_ids.end(), id) != excluded_ids.end();
    }), action_ids.end());
  }
  if (control_type != FOR_HEAD) {
    head_ids.erase(std::remove_if(head_ids.begin(), head_ids.end(), [&](uint8_t id) {
      return std::find(excluded_ids.begin(), excluded_ids.end(), id) != excluded_ids.end();
    }), head_ids.end());
  }
  if (control_type != FOR_WALKING) {
    walking_ids.erase(std::remove_if(walking_ids.begin(), walking_ids.end(), [&](uint8_t id) {
      return std::find(excluded_ids.begin(), excluded_ids.end(), id) != excluded_ids.end();
    }), walking_ids.end());
  }
}

bool Middleware::validate(int control_type)
{
  if (control_rule == DEFAULT && control_type != FORCE) {
    if (control_type == FOR_ACTION) {
      action_counter++;
    } else if (!is_action_controlling) {
      if (control_type == FOR_HEAD) {
        head_counter++;
      } else if (control_type == FOR_WALKING) {
        walking_counter++;
      }
    } else {
      return false;
    }
  }

  return true;
}

std::vector<Joint> Middleware::filter_joints(int control_type, const std::vector<tachimawari_interfaces::msg::Joint> & joints_message)
{
  if (control_rule == DEFAULT || control_type ==  FORCE) {
    std::vector<Joint> joints;
    std::transform(
      joints_message.begin(), joints_message.end(),
      std::back_inserter(joints),
      [](tachimawari_interfaces::msg::Joint joint) -> Joint {
        return Joint(joint.id, joint.position);
      });

    if (control_type == FOR_ACTION) {
      return joints;
    } else if (!is_action_controlling || control_type ==  FORCE) {
      return joints;
    }
  } else {
    std::vector<Joint> joints;

    if (control_type == FOR_ACTION) {
      for (const auto & joint : joints_message) {
        if (std::find(action_ids.begin(), action_ids.end(), joint.id) != action_ids.end()) {
          joints.push_back(Joint(joint.id, joint.position));
        }
      }
    } else if (control_type == FOR_HEAD) {
      for (const auto & joint : joints_message) {
        if (std::find(head_ids.begin(), head_ids.end(), joint.id) != head_ids.end()) {
          joints.push_back(Joint(joint.id, joint.position));
        }
      }
    } else if (control_type == FOR_WALKING) {
      for (const auto & joint : joints_message) {
        if (std::find(walking_ids.begin(), walking_ids.end(), joint.id) != walking_ids.end()) {
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
  if (action_counter == action_value) {
    action_timer += time_unit;

    if (action_timer > time_limit) {
      is_action_controlling = false;
      action_counter = 0;
      action_value = 0;
    }
  } else {
    action_timer = 0.0;
    action_value = action_counter;
    is_action_controlling = true;

    if (control_rule == FOR_ACTION) {
      reset_ids();
    }
  }

  if (head_counter == head_value) {
    head_timer += time_unit;

    if (head_timer > time_limit) {
      is_head_controlling = false;
      head_counter = 0;
      head_value = 0;
    }
  } else {
    head_timer = 0.0;
    head_value = head_counter;
    is_head_controlling = true;
  }

  if (walking_counter == walking_value) {
    walking_timer += time_unit;

    if (walking_timer > time_limit) {
      is_walking_controlling = false;
      walking_counter = 0;
      walking_value = 0;
    }
  } else {
    walking_timer = 0.0;
    walking_value = walking_counter;
    is_walking_controlling = true;
  }
}

}  // namespace tachimawari::joint
