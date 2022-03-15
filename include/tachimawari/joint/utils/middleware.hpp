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

#ifndef TACHIMAWARI__JOINT__UTILS__MIDDLEWARE_HPP_
#define TACHIMAWARI__JOINT__UTILS__MIDDLEWARE_HPP_

#include <chrono>
#include <string>

using namespace std::chrono_literals;

namespace tachimawari::joint
{

class Middleware
{
public:
  enum
  {
    DEFAULT,
    FOR_WALKING,
    FOR_HEAD,
    FOR_ACTION
  };

  Middleware(double time_limit = 0.5, std::chrono::milliseconds time_unit = 8ms);

  bool validate(int control_type);

  void update();

private:
  double time_limit;
  double time_unit;

  bool is_action_controlling;
  int action_value;
  int action_counter;
  double action_timer;

  bool is_head_controlling;
  int head_value;
  int head_counter;
  double head_timer;

  bool is_walking_controlling;
  int walking_value;
  int walking_counter;
  double walking_timer;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__UTILS__MIDDLEWARE_HPP_
