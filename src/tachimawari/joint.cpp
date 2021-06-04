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

#include <tachimawari/joint.hpp>

#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>

namespace tachimawari
{

const std::map<std::string, uint8_t> Joint::ids = {
  // head motors
  {"neck_yaw", 19},
  {"neck_pitch", 20},

  // left arm motors
  {"left_shoulder_pitch", 2},
  {"left_shoulder_roll", 4},
  {"left_elbow", 6},

  // right arm motors
  {"right_shoulder_pitch", 1},
  {"right_shoulder_roll", 3},
  {"right_elbow", 5},

  // left leg motors
  {"left_hip_yaw", 8},
  {"left_hip_roll", 10},
  {"left_hip_pitch", 12},
  {"left_knee", 14},
  {"left_ankle_roll", 16},
  {"left_ankle_pitch", 18},

  // right leg motors
  {"right_hip_yaw", 7},
  {"right_hip_roll", 9},
  {"right_hip_pitch", 11},
  {"right_knee", 13},
  {"right_ankle_roll", 15},
  {"right_ankle_pitch", 17},
};

Joint::Joint(std::string joint_name, float present_position)
: id(Joint::ids.at(joint_name)), name(joint_name), position(present_position)
{
}

void Joint::set_target_position(float target_position, float speed)
{
  goal_position = (target_position / 360 * 4096) - 1;  // will be placed in utility
  additional_position = (position - goal_position) * speed;
}

void Joint::set_present_position(float present_position)
{
  position = present_position;
}

void Joint::set_pid_gain(float p, float i, float d)
{
  p_gain = p;
  i_gain = i;
  d_gain = d;
}

void Joint::set_target_position(float present_position, float target_position, float speed)
{
  position = (present_position / 360 * 4096) - 1;  // will be placed in utility
  goal_position = (target_position / 360 * 4096) - 1;  // will be placed in utility
  additional_position = (goal_position - position) * speed;
}

void Joint::set_simulator_target_position(
  float target_position,
  float speed)
{
  goal_position = target_position;
  additional_position = (goal_position - position) * speed;
}

void Joint::interpolate()
{
  bool goal_position_is_reached = (additional_position >= 0 && position + additional_position >=
    goal_position) || (additional_position <= 0 && position + additional_position < goal_position);

  if (goal_position_is_reached) {
    position = goal_position;
    additional_position = 0.0;
  } else {
    position = position + additional_position;
  }
}

uint8_t Joint::get_id()
{
  return id;
}

std::string Joint::get_joint_name()
{
  return name;
}

float Joint::get_position()
{
  return position;
}

float Joint::get_goal_position()
{
  return goal_position;
}

std::vector<uint16_t> Joint::get_pid_gain()  // temporary
{
  std::vector<uint16_t> pid_gain;

  pid_gain.push_back(p_gain);
  pid_gain.push_back(i_gain);
  pid_gain.push_back(d_gain);

  return pid_gain;
}

}  // namespace tachimawari
