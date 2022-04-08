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

#include <string>
#include <vector>

#include "tachimawari/control/sdk/utils/protocol_2/group_sync_write.hpp"

#include "tachimawari/joint/protocol_2/mx28_address.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari::control::sdk::protocol_2
{

GroupSyncWrite::GroupSyncWrite(
  dynamixel::PortHandler * port_handler,
  dynamixel::PacketHandler * packet_handler)
: port_handler(port_handler), packet_handler(packet_handler)
{
}

dynamixel::GroupSyncWrite GroupSyncWrite::create(
  const std::vector<tachimawari::joint::Joint> & joints,
  uint8_t starting_address)
{
  // check does the request need pid to be included
  bool is_pid = starting_address == tachimawari::joint::protocol_2::MX28Address::POSITION_D_GAIN;
  int data_length = is_pid ? 3 : 4;

  dynamixel::GroupSyncWrite group_sync_write(port_handler, packet_handler,
    starting_address, data_length);

  std::vector<uint8_t> param_data;

  for (const auto & joint : joints) {
    if (is_pid) {
      param_data.push_back(joint.get_pid_gain()[2]);
      param_data.push_back(joint.get_pid_gain()[1]);
      param_data.push_back(joint.get_pid_gain()[0]);
    } else {
      param_data.push_back(DXL_LOBYTE(DXL_LOWORD(joint.get_position_value())));
      param_data.push_back(DXL_HIBYTE(DXL_LOWORD(joint.get_position_value())));
      param_data.push_back(DXL_LOBYTE(DXL_HIWORD(joint.get_position_value())));
      param_data.push_back(DXL_HIBYTE(DXL_HIWORD(joint.get_position_value())));
    }

    if (!group_sync_write.addParam(joint.get_id(), param_data.data())) {
      // TODO(maroqijalil): will be used for logging
      // addparam failed
    }
  }

  return group_sync_write;
}

}  // namespace tachimawari::control::sdk::protocol_2
