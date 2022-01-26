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

#include "tachimawari/control/controller/cm740.hpp"

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

CM740::CM740(
  const std::string & port_name, const int & baudrate, const float & protocol_version)
: ControlManager(port_name, protocol_version, baudrate)
{
}

bool CM740::connect()
{
  struct termios newtio;
  struct serial_struct serinfo;

  close_port();

  if ((socket_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
    close_port();
    return false;
  }

  // You must set 38400bps!
  memset(&newtio, 0, sizeof(newtio));
  newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;
  tcsetattr(socket_fd, TCSANOW, &newtio);

  // Set non-standard baudrate
  if (ioctl(socket_fd, TIOCGSERIAL, &serinfo) < 0) {
    close_port();
    return false;
  }

  serinfo.flags &= ~ASYNC_SPD_MASK;
  serinfo.flags |= ASYNC_SPD_CUST;
  serinfo.flags |= ASYNC_LOW_LATENCY;
  serinfo.custom_divisor = serinfo.baud_base / baudrate;

  if (ioctl(socket_fd, TIOCSSERIAL, &serinfo) < 0) {
    close_port();
    return false;
  }

  tcflush(socket_fd, TCIFLUSH);

  byte_transfer_time = (1000.0 / baudrate) * 12.0;

  return true;
}

void CM740::close_port()
{
  if (socket_fd != -1) {
    close(socket_fd);
  }
  socket_fd = -1;
}

}  // namespace tachimawari
