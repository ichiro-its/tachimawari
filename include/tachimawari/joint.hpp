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

#ifndef TACHIMAWARI__JOINT_HPP_
#define TACHIMAWARI__JOINT_HPP_

#include <map>
#include <string>
#include <vector>

namespace tachimawari
{

class Joint
{
public:
  explicit Joint(const std::string & joint_name, const float & present_position = 30.0);

  void set_target_position(const float & target_position, const float & speed = 1.0);
  void set_present_position(const float & present_position);
  void set_pid_gain(const float & p, const float & i, const float & d);

  void interpolate();

  uint8_t get_id() const;
  std::string get_joint_name() const;
  float get_position() const;
  float get_goal_position() const;
  std::vector<uint16_t> get_pid_gain() const;  // temporary

private:
  uint8_t id;
  std::string name;

  float p_gain = 850.0;
  float i_gain = 0.0;
  float d_gain = 0.0;

  float goal_position;
  float position;
  float additional_position;

  static const std::map<std::string, uint8_t> ids;
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__JOINT_HPP_
