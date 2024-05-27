// Copyright (c) 2021-2023 Ichiro ITS
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

#ifndef TACHIMAWARI__IMU__NODE__IMU_PROVIDER_HPP_
#define TACHIMAWARI__IMU__NODE__IMU_PROVIDER_HPP_

#include <memory>

#include "keisan/keisan.hpp"
#include "tachimawari/control/manager/control_manager.hpp"
namespace tachimawari::imu
{

class ImuProvider
{
public:
  explicit ImuProvider(std::shared_ptr<tachimawari::control::ControlManager> control_manager);

  const keisan::Vector<3> get_gyro() const;
  const keisan::Vector<3> get_accelero() const;

private:
  std::shared_ptr<tachimawari::control::ControlManager> control_manager;
};

}  // namespace tachimawari::imu

#endif  // TACHIMAWARI__IMU__NODE__IMU_PROVIDER_HPP_
