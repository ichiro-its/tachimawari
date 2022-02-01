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

#ifndef TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__STATUS__STATUS_PACKET_HPP_
#define TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__STATUS__STATUS_PACKET_HPP_

#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/packet/protocol_1/model/packet.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

namespace tachimawari
{

namespace control
{

namespace packet
{

namespace protocol_1
{

class StatusPacket : public Packet
{
public:
  static int validate(
    std::shared_ptr<std::vector<uint8_t>> rxpacket,
    const int & packet_length);

  StatusPacket(std::shared_ptr<std::vector<uint8_t>> rxpacket, const int & packet_length);

  bool is_valid();

  bool is_success();

  std::shared_ptr<std::vector<uint8_t>> get_raw_packet();

  uint8_t get_data_length() const override;

private:
  std::shared_ptr<std::vector<uint8_t>> rxpacket;

  int rxpacket_length;
};

}  // namespace protocol_1

}  // namespace packet

}  // namespace control

}  // namespace tachimawari

#endif  // TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__STATUS__STATUS_PACKET_HPP_
