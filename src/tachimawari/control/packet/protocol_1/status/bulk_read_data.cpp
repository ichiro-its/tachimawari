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

#include "tachimawari/control/packet/protocol_1/status/bulk_read_data.hpp"

#include "tachimawari/control/packet/protocol_1/model/packet_index.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/bulk_read_packet.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/instruction.hpp"
#include "tachimawari/control/packet/protocol_1/utils/word.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::control::packet::protocol_1
{

void BulkReadData::insert_all(
  std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data,
  const BulkReadPacket & bulk_read_packet)
{
  auto parameters = bulk_read_packet.get_parameters();

  for (size_t i = 1; i < parameters.size(); i += 3) {
    if (bulk_data->find(parameters[i + 1]) != bulk_data->end()) {
      bulk_data->insert(
        {parameters[i + 1], BulkReadData(
            parameters[i + 1], parameters[i],
            parameters[i + 2])});
    }
  }
}

int BulkReadData::validate(
  std::shared_ptr<std::vector<uint8_t>> rxpacket,
  int packet_length)
{
  int header_place = 0;
  for (header_place = 0; header_place < (packet_length - 1); ++header_place) {
    if (rxpacket->at(header_place) == 0xFF && rxpacket->at(header_place + 1) == 0xFF) {
      break;
    } else if (header_place == (packet_length - 2) && rxpacket->at(packet_length - 1) == 0xFF) {
      break;
    }
  }

  if (header_place == 0) {
    return packet_length;
  } else {
    for (int i = 0; i < (packet_length - header_place); ++i) {
      rxpacket->at(i) = rxpacket->at(i + header_place);
    }

    return packet_length - header_place;
  }
}

int BulkReadData::update_all(
  std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data,
  std::shared_ptr<std::vector<uint8_t>> rxpacket, int packet_length, int data_number)
{
  while (true) {
    int header_place = 0;
    for (header_place = 0; header_place < (packet_length - 1); ++header_place) {
      if (rxpacket->at(header_place) == 0xFF && rxpacket->at(header_place + 1) == 0xFF) {
        break;
      }
    }

    if (header_place == 0) {
      uint8_t packet_id = rxpacket->at(PacketIndex::ID);

      if (bulk_data->find(packet_id) != bulk_data->end()) {
        int length = rxpacket->at(PacketIndex::LENGTH) + 4;

        if (bulk_data->at(packet_id).is_valid(*rxpacket.get())) {
          int bulk_data_length = rxpacket->at(PacketIndex::LENGTH) + 4;

          for (int i = 0; i < packet_length - bulk_data_length; ++i) {
            rxpacket->at(i) = rxpacket->at(i + bulk_data_length);
          }

          data_number--;
          packet_length -= bulk_data_length;
        } else {
          // so RX_CORRUPT
          for (int i = 0; i < packet_length - 2; ++i) {
            rxpacket->at(i) = rxpacket->at(i + 2);
          }

          packet_length -= 2;
        }
      }

      if (data_number <= 0 || packet_length <= 0) {
        return data_number;
      }
    } else {
      for (int i = 0; i < (packet_length - header_place); ++i) {
        rxpacket->at(i) = rxpacket->at(i + header_place);
      }

      packet_length -= header_place;
    }
  }
}

BulkReadData::BulkReadData(
  uint8_t id, uint8_t starting_address,
  uint8_t data_length)
: Packet(id, 0xFF), start_address(starting_address), data_length(data_length)
{
}

bool BulkReadData::is_valid(std::vector<uint8_t> rxpacket)
{
  info = rxpacket[PacketIndex::ERROR];

  int packet_length = rxpacket[PacketIndex::LENGTH] + 4;
  for (int i = PacketIndex::PARAMETER; i < packet_length - 1; ++i) {
    parameters.push_back(rxpacket[i]);
  }

  calculate_checksum();
  if (checksum == rxpacket[packet_length - 1]) {
    return true;
  } else {
    info = 0xFF;
    parameters.clear();
    checksum = 0x00;

    return false;
  }
}

bool BulkReadData::is_filled()
{
  return parameters.size() != 0;
}

int BulkReadData::get(uint8_t address)
{
  int index = static_cast<int>(address) - start_address;

  if (index < 0 || index == data_length) {
    return -1;
  } else {
    return parameters[index];
  }
}

int BulkReadData::get(uint16_t address)
{
  int index = static_cast<int>(address) - start_address;

  if (index < 0 || index == data_length - 1) {
    return -1;
  } else {
    return Word::make_word(parameters[index], parameters[index + 1]);
  }
}

}  // namespace tachimawari::control::packet::protocol_1
