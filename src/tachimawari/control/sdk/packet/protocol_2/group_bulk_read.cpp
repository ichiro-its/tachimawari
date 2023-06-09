// Copyright (c) 2021-2023 Ichiro ITS
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/sdk/packet/model/group_bulk_read.hpp"
#include "tachimawari/control/sdk/packet/protocol_2/group_bulk_read.hpp"

#include "tachimawari/joint/protocol_2/mx28_address.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari::control::sdk::protocol_2
{

tachimawari::control::sdk::GroupBulkRead GroupBulkRead::create(
  dynamixel::PortHandler * port_handler,
  dynamixel::PacketHandler * packet_handler,
  const std::vector<tachimawari::joint::Joint> & joints)
{
  tachimawari::control::sdk::GroupBulkRead group_bulk_read(
    port_handler, packet_handler);

  for (const auto & joint : joints) {
    group_bulk_read.add(
      joint.get_id(), tachimawari::joint::protocol_2::GOAL_POSITION, 4u);
  }

  return group_bulk_read;
}

}  // namespace tachimawari::control::sdk::protocol_2
