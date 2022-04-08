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

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__STATUS__BULK_READ_DATA_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__STATUS__BULK_READ_DATA_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/controller/packet/protocol_1/instruction/bulk_read_packet.hpp"
#include "tachimawari/control/controller/packet/protocol_1/model/packet.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::control::protocol_1
{

class BulkReadData : public Packet
{
public:
  enum
  {
    MAX_LENGTH = 255
  };

  static void insert_all(
    std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data,
    const BulkReadPacket & bulk_read_packet);

  static int validate(
    std::shared_ptr<std::vector<uint8_t>> rxpacket,
    int packet_length);

  static int update_all(
    std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data,
    std::vector<uint8_t> rxpacket, int packet_length, int data_number);

  explicit BulkReadData(uint8_t id, int data_length = MAX_LENGTH);

  void set_starting_address(uint8_t start_address);

  bool is_valid(std::vector<uint8_t> rxpacket);

  bool is_filled() const;

  int get(uint8_t address) const;
  int get(uint16_t address) const;

private:
  uint8_t start_address;

  std::vector<uint8_t> data;
  std::vector<bool> marker;
};

}  // namespace tachimawari::control::protocol_1

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__STATUS__BULK_READ_DATA_HPP_
