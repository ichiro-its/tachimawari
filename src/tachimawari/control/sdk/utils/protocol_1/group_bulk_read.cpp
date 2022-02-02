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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/sdk/utils/protocol_1/group_bulk_read.hpp"

#include "tachimawari/joint/protocol_1/mx28_address.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari
{

namespace control
{

namespace sdk
{

namespace protocol_1
{

void GroupBulkRead::insert_all(
  std::shared_ptr<std::map<uint8_t, GroupBulkRead>> bulk_data,
  GroupBulkRead group_bulk_read)
{
  for (auto & id : group_bulk_read.get_parameters_id()) {
    if (bulk_data->find(id) != bulk_data->end()) {
      bulk_data->insert({id, group_bulk_read});
    }
  }
}

GroupBulkRead::GroupBulkRead(
  dynamixel::PortHandler * port_handler,
  dynamixel::PacketHandler * packet_handler)
: group_bulk_read(port_handler, packet_handler), parameters_id({})
{
}

void GroupBulkRead::add(
  const uint8_t & id, const uint16_t & starting_address,
  const uint16_t & data_length)
{
  if (group_bulk_read.addParam(id, starting_address, data_length)) {
    parameters_id.push_back(id);
  } else {
    // add param failed
  }
}

void GroupBulkRead::add(
  const std::vector<tachimawari::joint::Joint> & joints)
{
  for (auto & joint : joints) {
    if (group_bulk_read.addParam(
        joint.get_id(),
        tachimawari::joint::protocol_1::GOAL_POSITION_L,
        static_cast<uint16_t>(2)))
    {
      parameters_id.push_back(joint.get_id());
    } else {
      // addparam failed
    }
  }
}

int GroupBulkRead::send()
{
  return group_bulk_read.txRxPacket();
}

int GroupBulkRead::get(
  const uint8_t & id, const uint8_t & address,
  const int & data_length)
{
  bool is_available = group_bulk_read.isAvailable(
    id, static_cast<uint16_t>(address), static_cast<uint16_t>(data_length));
  if (is_available) {
    uint32_t result = group_bulk_read.getData(
      id, static_cast<uint16_t>(address), static_cast<uint16_t>(data_length));

    return static_cast<int>(result);
  } else {
    // data is not found
  }

  return -1;
}

std::vector<uint8_t> GroupBulkRead::get_parameters_id() const
{
  return parameters_id;
}

bool GroupBulkRead::is_parameters_filled() const
{
  return parameters_id.size() > 0;
}

GroupBulkRead::~GroupBulkRead()
{
  group_bulk_read.clearParam();
}


}  // namespace protocol_1

}  // namespace sdk

}  // namespace control

}  // namespace tachimawari
