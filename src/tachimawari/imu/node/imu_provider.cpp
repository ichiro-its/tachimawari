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

#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/control/packet/protocol_1/model/packet_id.hpp"
#include "tachimawari/control/controller/module/cm740_address.hpp"

namespace tachimawari
{

namespace imu
{

ImuProvider::ImuProvider(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager)
{
}

float * ImuProvider::get_gyro() const
{
  float gyro[3] = {0, 0, 0};

  if (control_manager->get_protocol_version() == 1.0) {
    {
      using namespace tachimawari::control;

      int gyro_x = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::GYRO_X_L, 2);
      int gyro_y = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::GYRO_Y_L, 2);
      int gyro_z = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::GYRO_Z_L, 2);
      
      gyro[0] = gyro_x == -1 ? 0 : static_cast<double>(gyro_x);
      gyro[1] = gyro_y == -1 ? 0 : static_cast<double>(gyro_y);
      gyro[2] = gyro_z == -1 ? 0 : static_cast<double>(gyro_z);
    }
  }

  return gyro;
}

float * ImuProvider::get_accelero() const
{
  float accel[3] = {0, 0, 0};

  if (control_manager->get_protocol_version() == 1.0) {
    {
      using namespace tachimawari::control;

      int accel_x = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::ACCEL_X_L, 2);
      int accel_y = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::ACCEL_Y_L, 2);
      int accel_z = control_manager->get_bulk_data(packet::protocol_1::PacketId::CONTROLLER,
        CM740Address::ACCEL_Z_L, 2);
      
      accel[0] = accel_x == -1 ? 0 : static_cast<float>(accel_x);
      accel[1] = accel_y == -1 ? 0 : static_cast<float>(accel_y);
      accel[2] = accel_z == -1 ? 0 : static_cast<float>(accel_z);
    }
  }

  return accel;
}

}  // namespace imu

}  // namespace tachimawari
