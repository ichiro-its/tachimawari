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

#include <string>
#include <vector>

#include "keisan/angle/angle.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::joint
{

int Joint::angle_to_value(double angle)
{
  return angle * TO_VALUE_RATIO;
}

double Joint::value_to_angle(int value)
{
  return value * TO_ANGLE_RATIO;
}

Joint::Joint(uint8_t joint_id, float position)
: id(joint_id), position(keisan::make_degree(position)), p_gain(30.0), i_gain(30.0), d_gain(30.0)
{
}

Joint::Joint(uint8_t joint_id, keisan::Angle<float> position)
: Joint(joint_id, position.degree())
{
}

void Joint::set_position(float position)
{
  this->position = keisan::make_degree(position);
}

void Joint::set_position(keisan::Angle<float> position)
{
  this->position = position;
}

void Joint::set_pid_gain(float p, float i, float d)
{
  p_gain = p;
  i_gain = i;
  d_gain = d;
}

uint8_t Joint::get_id() const
{
  return id;
}

float Joint::get_position() const
{
  return position.degree();
}

std::vector<float> Joint::get_pid_gain() const
{
  return {p_gain, i_gain, d_gain};
}

void Joint::set_position_value(int value)
{
  position = keisan::make_degree((value - CENTER_VALUE) * TO_ANGLE_RATIO);
}

int Joint::get_position_value() const
{
  return (position.degree() * TO_VALUE_RATIO) + CENTER_VALUE;
}

}  // namespace tachimawari::joint
