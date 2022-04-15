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

#ifndef TACHIMAWARI__CONTROL__SDK__MODULE__DYNAMIXEL_SDK_HPP_
#define TACHIMAWARI__CONTROL__SDK__MODULE__DYNAMIXEL_SDK_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/control/sdk/packet/model/group_bulk_read.hpp"
#include "tachimawari/control/sdk/packet/protocol_1/group_bulk_read.hpp"
#include "tachimawari/control/sdk/packet/protocol_1/group_sync_write.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari::control
{

class DynamixelSDK : public ControlManager
{
public:
  enum : int
  {
    SUCCESS = 0,
    TX_FAIL = -1001
  };

  explicit DynamixelSDK(
    const std::string & port_name, int baudrate = 1000000,
    float protocol_version = 1.0);
  ~DynamixelSDK();

  bool connect() override;
  void disconnect() override;

  bool ping(uint8_t id) override;

  bool write_packet(
    uint8_t id, uint8_t address, int value,
    int data_length = 1) override;

  int read_packet(
    uint8_t id, uint8_t address, int data_length = 1) override;

  bool sync_write_packet(
    const std::vector<joint::Joint> & joints,
    bool with_pid = false) override;

  bool bulk_read_packet() override;

  int get_bulk_data(
    uint8_t id, uint8_t address,
    int data_length = 1) override;

private:
  bool send_bulk_read_packet(sdk::GroupBulkRead packet);

  dynamixel::PortHandler * port_handler;
  dynamixel::PacketHandler * packet_handler;

  std::shared_ptr<std::map<uint8_t, sdk::GroupBulkRead>> bulk_data;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__SDK__MODULE__DYNAMIXEL_SDK_HPP_
