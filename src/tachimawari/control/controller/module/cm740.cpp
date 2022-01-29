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

#include "tachimawari/control/controller/module/cm740.hpp"

#include "tachimawari/control/controller/module/cm740_address.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/write_packet.hpp"
#include "tachimawari/control/packet/protocol_1/status/status_packet.hpp"
#include "tachimawari/control/packet/protocol_1/utils/word.hpp"

#include "errno.h"  // NOLINT
#include "fcntl.h"  // NOLINT
#include "stdio.h"  // NOLINT
#include "string.h"  // NOLINT
#include "termios.h"  // NOLINT
#include "unistd.h"  // NOLINT
#include "linux/serial.h"
#include "sys/ioctl.h"
#include "sys/time.h"

namespace tachimawari
{

namespace control
{

CM740::CM740(const std::string & port_name, const int & baudrate = 1000000,
  const float & protocol_version = 1.0)
: ControlManager(port_name, protocol_version, baudrate), byte_transfer_time(0.0),
  platform(std::make_shared<Linux>())
{
}

bool CM740::connect()
{
  if (platform->open_port(port_name, baudrate)) {
    byte_transfer_time = (1000.0 / baudrate) * 12.0;

    return dxl_power_on();
  }

  return false;
}

bool CM740::dxl_power_on()
{
  if (write_packet(CM740Address::DXL_POWER, 1)) {
    write_packet(CM740Address::LED_HEAD_L, packet::protocol_1::Word::make_color(255, 128, 0), 2);

    return true;
  }

  return false;
}

packet::protocol_1::StatusPacket CM740::send_packet(packet::protocol_1::Packet packet)
{
  std::vector<uint8_t> txpacket = packet.get_packet();
  auto rxpacket = std::make_shared<std::vector<uint8_t>>(1024, 0x00);

  if (platform->write_port(txpacket) == txpacket.size()) {
    int expected_length = 6;
    if (packet.get_info() == packet::protocol_1::Instruction::READ) {
      expected_length = packet.get_parameters()[1];
    }

    // set packet timeout;

    int get_length = 0;
    while (true) {
      get_length += platform->read_port(rxpacket, expected_length - get_length, get_length);

      if (get_length == expected_length) {
        packet::protocol_1::StatusPacket status_packet(rxpacket);

        if (status_packet.is_valid(get_length)) {
          return status_packet;
        } else {
          if (status_packet.is_headers_matched()) {
            // so RX_CORRUPT
            break;
          } else {
            get_length -= status_packet.get_header_place();
            rxpacket = status_packet.get_raw_packet();
          }
        }
      } else {
        // is packet timeout ? so RX_TIMEOUT
        // or RX_CORRUPT if the rx length is not zero
      }
    }
  } else {
    // so TX_FAIL
  }

  return packet::protocol_1::StatusPacket(nullptr);
}

bool CM740::write_packet(const uint8_t & address, const int & value, const int & data_length)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::WritePacket instruction_packet;

    if (data_length == 1) {
      instruction_packet.create(address, static_cast<uint8_t>(value));
    } else if (data_length == 2) {
      instruction_packet.create(address, static_cast<uint16_t>(value));
    }

    auto status_packet = send_packet(instruction_packet);
    return status_packet.is_success();
  }

  return false;
}

}  // namespace control

}  // namespace tachimawari
