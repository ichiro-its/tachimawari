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

namespace motion
{

class Joint
{
public:
  explicit Joint(std::string joint_name, float present_position = 30.0);

  void setGoalPosition(float goal_position);
  void setPIDGain(float p, float i, float d);

  uint8_t getId();
  float getPosition();

private:
  uint8_t id;

  float p_gain = 0.5;
  float i_gain = 0.0;
  float d_gain = 0.0;
  float position;

  const std::map<std::string, uint8_t> ids;
};

}  // namespace motion

#endif  // MOTION__JOINT_HPP_
