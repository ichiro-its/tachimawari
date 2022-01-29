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

#include "tachimawari/control/packet/protocol_1/instruction/write_packet.hpp"

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
  
}

std::vector<uint8_t> CM740::send_packet(std::vector<uint8_t> packet)
{

  unsigned char result[1024] = { 0, };

  // if not, TX_FAIL
  if (platform->write_port(packet) == packet.size()) {

  }

  return std::vector<uint8_t>(result, result + sizeof result / sizeof result[0]);
}

bool CM740::write_packet(const uint8_t & address, const int & value, const int & data_length)
{
  packet::protocol_1::WritePacket write_packet;

  if (data_length == 1) {
    write_packet.create(address, static_cast<uint8_t>(value));
  } else if (data_length == 2) {
    write_packet.create(address, static_cast<uint16_t>(value));
  }

}

}  // namespace tachimawari
