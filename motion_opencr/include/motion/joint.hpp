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

#ifndef MOTION__JOINT_HPP_
#define MOTION__JOINT_HPP_

#include <map>
#include <string>
#include <vector>

namespace motion
{

class Joint
{
public:
  explicit Joint(std::string joint_name, float present_position = 30.0);

  void set_goal_position(float goal_position);
  void set_pid_gain(float p, float i, float d);

  uint8_t get_id();
  uint16_t get_position();
  // temporary
  std::vector<uint8_t> get_pid_gain();

private:
  uint8_t id;

  uint8_t p_gain = 0.5;
  uint8_t i_gain = 0.0;
  uint8_t d_gain = 0.0;
  uint16_t position;

  static const std::map<std::string, uint8_t> ids;
};

}  // namespace motion

#endif  // MOTION__JOINT_HPP_
