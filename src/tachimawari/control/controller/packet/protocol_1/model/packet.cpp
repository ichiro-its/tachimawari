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

#include <string>
#include <vector>

#include "tachimawari/control/controller/packet/protocol_1/model/packet.hpp"

#include "tachimawari/control/controller/packet/protocol_1/instruction/instruction.hpp"

namespace tachimawari::control::protocol_1
{

Packet::Packet(uint8_t packet_id, uint8_t instruction)
: info(instruction), checksum(0x00), packet_id(packet_id)
{
  packet.push_back(0xFF);
  packet.push_back(0xFF);
}

uint8_t Packet::get_packet_id() const
{
  return packet_id;
}

uint8_t Packet::get_info() const
{
  return info;
}

uint8_t Packet::get_data_length() const
{
  return static_cast<uint8_t>(parameters.size() + 2);
}

int Packet::get_expected_length() const
{
  if (info == Instruction::READ) {
    return parameters[1] + 6;
  } else {
    return 6;
  }
}

void Packet::calculate_checksum()
{
  checksum = packet_id + get_data_length() + info;
  for (auto parameter : parameters) {
    checksum += parameter;
  }
  checksum = ~checksum;
}

const std::vector<uint8_t> & Packet::get_parameters() const
{
  return parameters;
}

const std::vector<uint8_t> & Packet::get_packet()
{
  packet.push_back(packet_id);

  packet.push_back(get_data_length());

  packet.push_back(info);
  packet.insert(packet.end(), parameters.begin(), parameters.end());

  calculate_checksum();
  packet.push_back(checksum);

  return packet;
}

}  // namespace tachimawari::control::protocol_1
