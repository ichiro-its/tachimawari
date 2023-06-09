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

#include "tachimawari/control/controller/packet/protocol_1/instruction/read_packet.hpp"

#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/control/controller/packet/protocol_1/instruction/instruction.hpp"
#include "tachimawari/control/controller/packet/protocol_1/model/packet.hpp"

namespace tachimawari::control::protocol_1
{

bool ReadPacket::is_match(const Packet & instruction_packet, const Packet & status_packet)
{
  return (instruction_packet.get_packet_id() == status_packet.get_packet_id()) &&
         (static_cast<size_t>(instruction_packet.get_parameters()[1]) ==
         status_packet.get_parameters().size());
}

ReadPacket::ReadPacket()
: Packet(tachimawari::control::ControlManager::CONTROLLER, Instruction::READ)
{
}

void ReadPacket::create(uint8_t id, uint16_t address, uint8_t data_length)
{
  packet_id = id;
  parameters.push_back(address);
  parameters.push_back(data_length);
}

}  // namespace tachimawari::control::protocol_1
