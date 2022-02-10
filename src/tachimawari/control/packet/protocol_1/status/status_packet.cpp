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
#include <string>
#include <vector>

#include "tachimawari/control/packet/protocol_1/status/status_packet.hpp"

#include "tachimawari/control/packet/protocol_1/model/packet_index.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/instruction.hpp"
#include "tachimawari/control/packet/protocol_1/utils/word.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::control::packet::protocol_1
{

int StatusPacket::validate(
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
    StatusPacket status_packet(rxpacket, packet_length);

    if (status_packet.is_valid()) {
      return packet_length;
    } else {
      return 0;
    }
  } else {
    for (size_t i = 0; i < (packet_length - header_place); ++i) {
      rxpacket->at(i) = rxpacket->at(i + header_place);
    }

    return packet_length - header_place;
  }
}

StatusPacket::StatusPacket(
  std::shared_ptr<std::vector<uint8_t>> rxpacket,
  int packet_length)
: Packet(rxpacket->at(PacketIndex::ID), rxpacket->at(PacketIndex::ERROR)),
  rxpacket_length(packet_length), rxpacket(rxpacket)
{
}

bool StatusPacket::is_valid()
{
  for (size_t i = PacketIndex::PARAMETER; i < rxpacket_length - 1; ++i) {
    parameters.push_back(rxpacket->at(i));
  }

  calculate_checksum();
  return checksum == rxpacket->at(rxpacket_length - 1);
}

uint8_t StatusPacket::get_data_length() const
{
  return rxpacket->at(PacketIndex::LENGTH);
}

std::shared_ptr<std::vector<uint8_t>> StatusPacket::get_raw_packet()
{
  return rxpacket;
}

bool StatusPacket::is_success() const
{
  return rxpacket != nullptr;
}

}  // namespace tachimawari::control::packet::protocol_1
