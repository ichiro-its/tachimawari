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

#ifndef TACHIMAWARI__CONTROL__SDK__PACKET__MODEL__GROUP_BULK_READ_HPP_
#define TACHIMAWARI__CONTROL__SDK__PACKET__MODEL__GROUP_BULK_READ_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::control::sdk
{

struct BulkReadParam
{
  uint8_t id;
  uint16_t starting_address;
  uint16_t length;
};

class GroupBulkRead
{
public:
  static void insert_all(
    std::shared_ptr<std::unordered_map<uint8_t, std::vector<uint8_t>>> & bulk_data,
    const std::shared_ptr<sdk::GroupBulkRead> & sdk_group_bulk_read);

  GroupBulkRead(dynamixel::PortHandler * port_handler, dynamixel::PacketHandler * packet_handler);
  ~GroupBulkRead();

  bool add(uint8_t id, uint16_t starting_address, uint16_t data_length);

  void clear_param();

  int send();

  int get(uint8_t id, uint16_t address, uint16_t data_length);

  std::vector<uint8_t> get_from_param(BulkReadParam param);

  std::vector<BulkReadParam> get_parameters() const;

  bool is_parameters_filled() const;

protected:
  std::shared_ptr<dynamixel::GroupBulkRead> group_bulk_read;

  std::vector<BulkReadParam> parameters;
};

}  // namespace tachimawari::control::sdk

#endif  // TACHIMAWARI__CONTROL__SDK__PACKET__MODEL__GROUP_BULK_READ_HPP_
