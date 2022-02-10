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

#ifndef TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__BULK_READ_PACKET_HPP_
#define TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__BULK_READ_PACKET_HPP_

#include <vector>

#include "tachimawari/control/packet/protocol_1/model/packet.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::control::packet::protocol_1
{

class BulkReadPacket : public Packet
{
public:
  BulkReadPacket();

  int get_expected_length() const override;

  int get_data_number() const;

  bool is_parameters_filled() const;

  void add(
    uint8_t id, uint8_t starting_address,
    uint8_t data_length);

  void add(const std::vector<tachimawari::joint::Joint> & joints);
};

}  // namespace tachimawari::control::packet::protocol_1

#endif  // TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__BULK_READ_PACKET_HPP_
