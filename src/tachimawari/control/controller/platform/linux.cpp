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

#include "tachimawari/control/controller/platform/linux.hpp"

#include <memory>
#include <string>
#include <vector>

#include "errno.h"  // NOLINT
#include "fcntl.h"  // NOLINT
#include "linux/serial.h"
#include "stdio.h"   // NOLINT
#include "string.h"  // NOLINT
#include "sys/ioctl.h"
#include "sys/time.h"
#include "tachimawari/control/controller/packet/protocol_1/instruction/write_packet.hpp"
#include "termios.h"  // NOLINT
#include "unistd.h"   // NOLINT

namespace tachimawari::control
{

Linux::Linux() : port_name(""), baudrate(1000000), socket_fd(-1) {}

bool Linux::open_port(const std::string & port_name, int baudrate)
{
  close_port();

  if ((socket_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    close_port();
    return false;
  }

  struct termios newtio;
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  tcsetattr(socket_fd, TCSANOW, &newtio);

  struct serial_struct serinfo;
  if (ioctl(socket_fd, TIOCGSERIAL, &serinfo) >= 0) {
    serinfo.flags &= ~ASYNC_SPD_MASK;
    serinfo.flags |= ASYNC_SPD_CUST;
    serinfo.flags |= ASYNC_LOW_LATENCY;
    serinfo.custom_divisor = serinfo.baud_base / baudrate;

    if (ioctl(socket_fd, TIOCSSERIAL, &serinfo) < 0) {
      close_port();
      return false;
    }
  } else {
    close_port();
    return false;
  }

  tcflush(socket_fd, TCIFLUSH);

  return true;
}

void Linux::close_port()
{
  if (socket_fd != -1) {
    close(socket_fd);
  }

  socket_fd = -1;
}

void Linux::clear_port() { tcflush(socket_fd, TCIFLUSH); }

int Linux::write_port(const std::vector<uint8_t> & packet)
{
  clear_port();

  return write(socket_fd, packet.data(), packet.size());
}

int Linux::read_port(
  std::shared_ptr<std::vector<uint8_t>> packet, int packet_length, int packet_index)
{
  unsigned char rxpacket[(packet_index + packet_length) * 2] = {0x00};

  int result_length = read(socket_fd, &rxpacket[packet_index], packet_length);

  if (result_length != 0) {
    for (size_t i = packet_index; i < (packet_index + packet_length) * 2; ++i) {
      packet->at(i) = rxpacket[i];
    }
  }

  return result_length;
}

}  // namespace tachimawari::control
