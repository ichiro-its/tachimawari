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

#include "tachimawari/control/packet/protocol_1/model/packet.hpp"

#include "tachimawari/control/packet/protocol_1/model/packet_id.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/insctruction.hpp"

namespace tachimawari
{

namespace packet
{
  
namespace protocol_1
{

Packet::Packet(const PacketId & pakcet_id, const Instruction & instruction)
: packet_id(packet_id), info(instruction)
{
  packet.push_back(0xFF);
  packet.push_back(0xFF);
}

void Packet::set_parameters(const std::vector<uint8_t> & parameters)
{
  this->parameters = parameters;
}

const std::vector<uint8_t> & Packet::get_packet() const
{
  packet.push_back(packet_id);
  
  uint8_t data_length = static_cast<uint8_t>(parameters.size() + 2);
  packet.push_back(data_length);

  packet.push_back(info);
  packet.insert(packet.end(), parameters.begin(), parameters.end());

  uint8_t checksum = packet_id + data_length + info;
  for (auto & parameter : parameters) {
    checksum += parameter;
  }
  packet.push_back(~checksum);

  return packet;
}

}  // namespace protocol_1

}  // namespace packet

}  // namespace tachimawari
