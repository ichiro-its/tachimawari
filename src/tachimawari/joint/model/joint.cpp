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

#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::joint
{

Joint::Joint(uint8_t joint_id, const float & position)
: id(joint_id), position(position), p_gain(30.0), i_gain(30.0), d_gain(30.0)
{
}

void Joint::set_position(const float & position)
{
  this->position = position;
}

void Joint::set_pid_gain(const float & p, const float & i, const float & d)
{
  p_gain = p;
  i_gain = i;
  d_gain = d;
}

uint8_t Joint::get_id() const
{
  return id;
}

const float & Joint::get_position() const
{
  return position;
}

std::vector<float> Joint::get_pid_gain() const
{
  return {p_gain, i_gain, d_gain};
}

}  // namespace tachimawari::joint
