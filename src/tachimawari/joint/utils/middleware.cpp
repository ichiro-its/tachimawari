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

#include "tachimawari/joint/utils/middleware.hpp"

namespace tachimawari::joint
{

Middleware::Middleware(double time_limit, std::chrono::milliseconds time_unit)
: time_limit(time_limit), time_unit(time_unit.count() / 1000.0), is_action_controlling(false), action_value(0), action_counter(0), action_timer(0), is_head_controlling(false), head_value(0), head_counter(0), head_timer(0), is_walking_controlling(false), walking_value(0), walking_counter(0), walking_timer(0)
{
}

bool Middleware::validate(int control_type)
{
  if (control_type == FOR_ACTION) {
    action_counter++;

    return true;
  } else if (control_type == FOR_HEAD && !is_action_controlling) {
    head_counter++;

    return true;
  } else if (control_type == FOR_WALKING && !is_action_controlling) {
    walking_counter++;

    return true;
  }

  return false;
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
