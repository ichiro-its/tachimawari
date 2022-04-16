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

#ifndef TACHIMAWARI__JOINT__MODEL__JOINT_HPP_
#define TACHIMAWARI__JOINT__MODEL__JOINT_HPP_

#include <string>
#include <vector>

#include "keisan/angle/angle.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

namespace tachimawari::joint
{

class Joint
{
public:
  enum
  {
    CENTER_VALUE = 2048
  };

  static int angle_to_value(const keisan::Angle<double> & angle);
  static keisan::Angle<double> value_to_angle(int value);

  explicit Joint(uint8_t joint_id, float position = 0.0);
  explicit Joint(uint8_t joint_id, keisan::Angle<float> position);

  uint8_t get_id() const;

  void set_position_value(int value);
  int get_position_value() const;

  void set_position(float position);
  void set_position(keisan::Angle<float> position);
  float get_position() const;

  void set_pid_gain(float p, float i, float d);
  std::vector<float> get_pid_gain() const;

private:
  uint8_t id;

  float p_gain;
  float i_gain;
  float d_gain;

  keisan::Angle<float> position;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__MODEL__JOINT_HPP_
