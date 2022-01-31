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

#include "tachimawari/control/packet/protocol_1/instruction/bulk_read_packet.hpp"

#include "tachimawari/control/packet/protocol_1/model/packet_id.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/insctruction.hpp"
#include "tachimawari/control/packet/protocol_1/utils/word.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari
{

namespace control
{

namespace packet
{
  
namespace protocol_1
{

BulkReadPacket::BulkReadPacket()
: Packet(PacketId::BROADCAST, Instruction::BULK_READ)
{
  parameters.push_back(0x00);
}

bool BulkReadPacket::is_parameters_filled()
{
  return parameters.size() > 1;
}

const int & BulkReadPacket::get_expected_length() const
{
  int length = 0;
  for (int i = 1; i < parameters.size(); i += 3) {
    length += (parameters[i] + 6);
  }

  return length;
}

const int & BulkReadPacket::get_data_number() const
{
  int data_number = parameters.size() - 1;
  return data_number / 3;
}

void BulkReadPacket::add(const uint8_t & id, const uint8_t & starting_address,
  const uint8_t & data_length)
{
  parameters.push_back(data_length);
  parameters.push_back(id);
  parameters.push_back(starting_address);
}

void BulkReadPacket::add(const std::vector<tachimawari::joint::Joint> & joints)
{
}

}  // namespace protocol_1

}  // namespace packet

}  // namespace control

}  // namespace tachimawari
