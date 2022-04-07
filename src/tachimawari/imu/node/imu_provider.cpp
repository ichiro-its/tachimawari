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

#include <memory>
#include <vector>

#include "tachimawari/imu/node/imu_provider.hpp"

#include "keisan/keisan.hpp"
#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/control/controller/module/cm740_address.hpp"

namespace tachimawari::imu
{

ImuProvider::ImuProvider(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager)
{
}

const keisan::Vector<3> & ImuProvider::get_gyro() const
{
  if (control_manager->get_protocol_version() == 1.0) {
    {
      using tachimawari::control::CM740Address;
      using tachimawari::control::ControlManager;

      int gyro_x = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::GYRO_X_L, 2);
      int gyro_y = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::GYRO_Y_L, 2);
      int gyro_z = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::GYRO_Z_L, 2);

      return keisan::Vector<3>(
        (gyro_x == -1) ? 0 : gyro_x,
        (gyro_y == -1) ? 0 : gyro_y,
        (gyro_z == -1) ? 0 : gyro_z
      );
    }
  }

  return keisan::Vector<3>::zero();
}

const keisan::Vector<3> & ImuProvider::get_accelero() const
{
  if (control_manager->get_protocol_version() == 1.0) {
    {
      using tachimawari::control::CM740Address;
      using tachimawari::control::ControlManager;

      int accel_x = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::ACCEL_X_L, 2);
      int accel_y = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::ACCEL_Y_L, 2);
      int accel_z = control_manager->get_bulk_data(
        ControlManager::CONTROLLER,
        CM740Address::ACCEL_Z_L, 2);

      return keisan::Vector<3>(
        (accel_x == -1) ? 0 : accel_x,
        (accel_y == -1) ? 0 : accel_y,
        (accel_z == -1) ? 0 : accel_z
      );
    }
  }

  return keisan::Vector<3>::zero();
}

}  // namespace tachimawari::imu
