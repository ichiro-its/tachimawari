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

#include "tachimawari/control/sdk/packet/model/group_bulk_read.hpp"

#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "tachimawari/control/sdk/module/marin_core_address.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::control::sdk
{

void GroupBulkRead::insert_all(
  std::shared_ptr<std::map<uint8_t, std::vector<uint8_t>>> bulk_data,
  std::shared_ptr<sdk::GroupBulkRead> sdk_group_bulk_read)
{
  for (auto param : sdk_group_bulk_read->get_parameters()) {
    std::vector<uint8_t> data(param.starting_address + param.length, 0);
    auto param_data = sdk_group_bulk_read->get_from_param(param);

    if (bulk_data->find(param.id) != bulk_data->end()) {
      data = bulk_data->at(param.id);
    }

    for (size_t i = 0; i < param_data.size(); ++i) {
      data[param.starting_address + i] = param_data[i];
    }

    if (bulk_data->find(param.id) != bulk_data->end()) {
      (*bulk_data)[param.id] = data;
    } else {
      bulk_data->insert({param.id, data});
    }
  }
}

GroupBulkRead::GroupBulkRead(
  dynamixel::PortHandler * port_handler, dynamixel::PacketHandler * packet_handler)
: group_bulk_read(std::make_shared<dynamixel::GroupBulkRead>(port_handler, packet_handler)),
  parameters({})
{
}

bool GroupBulkRead::add(uint8_t id, uint16_t starting_address, uint16_t data_length)
{
  if (group_bulk_read->addParam(id, starting_address, data_length)) {
    parameters.push_back(BulkReadParam{id, starting_address, data_length});

    return true;
  } else {
    // TODO(maroqijalil): will be used for logging
    // add param failed
  }

  return false;
}

void GroupBulkRead::clear_param() { group_bulk_read->clearParam(); }

int GroupBulkRead::send() { return group_bulk_read->txRxPacket(); }

int GroupBulkRead::get(uint8_t id, uint16_t address, uint16_t data_length)
{
  bool is_available = group_bulk_read->isAvailable(id, address, data_length);

  if (is_available) {
    uint32_t result = group_bulk_read->getData(id, address, data_length);

    return static_cast<int>(result);
  } else {
    // TODO(maroqijalil): will be used for logging
    // data is not found
  }

  return -1;
}

std::vector<uint8_t> GroupBulkRead::get_from_param(BulkReadParam param)
{
  std::vector<uint8_t> result;

  bool is_available = group_bulk_read->isAvailable(param.id, param.starting_address, param.length);

  if (is_available) {
    for (size_t i = 0; i < param.length; ++i) {
      result.push_back(group_bulk_read->getData(param.id, param.starting_address + i, 1));
    }
  } else {
    // TODO(maroqijalil): will be used for logging
    // data is not found
  }

  return result;
}

std::vector<BulkReadParam> GroupBulkRead::get_parameters() const { return parameters; }

bool GroupBulkRead::is_parameters_filled() const { return parameters.size() > 0; }

GroupBulkRead::~GroupBulkRead() { group_bulk_read->clearParam(); }

}  // namespace tachimawari::control::sdk
