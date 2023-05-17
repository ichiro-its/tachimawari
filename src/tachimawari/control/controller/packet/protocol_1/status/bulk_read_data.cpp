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

#include "tachimawari/control/controller/packet/protocol_1/status/bulk_read_data.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/controller/packet/protocol_1/instruction/bulk_read_packet.hpp"
#include "tachimawari/control/controller/packet/protocol_1/instruction/instruction.hpp"
#include "tachimawari/control/controller/packet/protocol_1/model/packet_index.hpp"
#include "tachimawari/control/controller/packet/protocol_1/utils/word.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::control::protocol_1
{

void BulkReadData::insert_all(
  std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data,
  const BulkReadPacket & bulk_read_packet)
{
  auto parameters = bulk_read_packet.get_parameters();

  for (size_t i = 1; i < parameters.size(); i += 3) {
    if (bulk_data->find(parameters[i + 1]) == bulk_data->end()) {
      bulk_data->insert({parameters[i + 1], BulkReadData(parameters[i + 1])});
    }

    bulk_data->at(parameters[i + 1]).set_starting_address(parameters[i + 2]);
  }
}

int BulkReadData::validate(std::shared_ptr<std::vector<uint8_t>> rxpacket, int packet_length)
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
    for (size_t i = 0; i < (packet_length - header_place); ++i) {
      rxpacket->at(i) = rxpacket->at(i + header_place);
    }

    return packet_length - header_place;
  }
}

int BulkReadData::update_all(
  std::shared_ptr<std::map<uint8_t, BulkReadData>> bulk_data, std::vector<uint8_t> rxpacket,
  int packet_length, int data_number)
{
  while (true) {
    int header_place = 0;
    for (header_place = 0; header_place < (packet_length - 1); ++header_place) {
      if (rxpacket[header_place] == 0xFF && rxpacket[header_place + 1] == 0xFF) {
        break;
      }
    }

    if (header_place == 0) {
      uint8_t packet_id = rxpacket[PacketIndex::ID];

      bool is_found = bulk_data->find(packet_id) != bulk_data->end();
      if (is_found) {
        int length = rxpacket[PacketIndex::LENGTH] + 4;

        if (bulk_data->at(packet_id).is_valid(rxpacket)) {
          int bulk_data_length = rxpacket[PacketIndex::LENGTH] + 4;

          for (size_t i = 0; i < packet_length - bulk_data_length; ++i) {
            rxpacket[i] = rxpacket[i + bulk_data_length];
          }

          data_number--;
          packet_length -= bulk_data_length;
        } else {
          is_found = false;
        }
      }

      if (data_number <= 0 || packet_length <= 0) {
        return data_number;
      }

      if (!is_found && packet_length >= 2) {
        // so RX_CORRUPT
        for (size_t i = 0; i < packet_length - 2; ++i) {
          rxpacket[i] = rxpacket[i + 2];
        }

        packet_length -= 2;
      }
    } else {
      for (size_t i = 0; i < (packet_length - header_place); ++i) {
        rxpacket[i] = rxpacket[i + header_place];
      }

      packet_length -= header_place;
    }

    if (packet_length < 2) {
      break;
    }
  }
}

BulkReadData::BulkReadData(uint8_t id, int data_length)
: Packet(id, 0xFF), data(data_length, 0x00), marker(data_length, false)
{
}

void BulkReadData::set_starting_address(uint8_t start_address)
{
  this->start_address = start_address;
}

bool BulkReadData::is_valid(std::vector<uint8_t> rxpacket)
{
  info = rxpacket[PacketIndex::ERROR];

  int packet_length = rxpacket[PacketIndex::LENGTH] + 4;
  for (size_t i = PacketIndex::PARAMETER; i < packet_length - 1; ++i) {
    parameters.push_back(rxpacket[i]);
  }

  calculate_checksum();
  if (checksum == rxpacket[packet_length - 1]) {
    for (size_t i = PacketIndex::PARAMETER; i < packet_length - 1; ++i) {
      data[static_cast<size_t>(start_address) + (i - PacketIndex::PARAMETER)] = rxpacket[i];
      marker[static_cast<size_t>(start_address) + (i - PacketIndex::PARAMETER)] = true;
    }

    return true;
  } else {
    info = 0xFF;
    parameters.clear();
    checksum = 0x00;

    return false;
  }
}

bool BulkReadData::is_filled() const {return parameters.size() != 0;}

int BulkReadData::get(uint16_t address, int length) const
{
  size_t index = static_cast<size_t>(address);
  if (index < data.size() && marker[index]) {
    if (length == 1) {
      return data[index];
    } else if (length == 2 && marker[index + 1]) {
      return Word::make_word(data[index], data[index + 1]);
    }
  }

  return -1;
}

}  // namespace tachimawari::control::protocol_1
