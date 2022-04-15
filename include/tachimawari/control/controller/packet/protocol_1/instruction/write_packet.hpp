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

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__INSTRUCTION__WRITE_PACKET_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__INSTRUCTION__WRITE_PACKET_HPP_

#include <string>
#include <vector>

#include "tachimawari/control/controller/packet/protocol_1/model/packet.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari::control::protocol_1
{

class WritePacket : public Packet
{
public:
  WritePacket();

  void create(uint8_t id, uint8_t address, uint8_t value);

  void create(uint8_t id, uint8_t address, uint16_t value);
};

}  // namespace tachimawari::control::protocol_1

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__INSTRUCTION__WRITE_PACKET_HPP_
