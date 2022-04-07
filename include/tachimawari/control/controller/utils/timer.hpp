// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__UTILS__TIMER_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__UTILS__TIMER_HPP_

namespace tachimawari::control
{

class Timer
{
public:
  explicit Timer(double transfer_time);

  void set_timeout(int length, double additional_time = 40.0);

  bool is_timeout();

private:
  double get_current_time();

  double start_time;
  double wait_time;
  double transfer_time;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__UTILS__TIMER_HPP_
