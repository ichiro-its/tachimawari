// Copyright 2020-2021 Ichiro ITS
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

#ifndef MOTION__SERIAL_HPP_
#define MOTION__SERIAL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <string>
#include <vector>

namespace motion
{
class Serial : public rclcpp::Node
{
public:
  explicit Serial(std::string node_name, std::string port_name = "/dev/ttyUSB0");
  ~Serial();

  bool open();
  void close();
  bool broadcastPing();
  bool broadcastPing(std::vector<uint8_t> ids);
  bool ping(uint8_t id);

private:
  dynamixel::PortHandler * port_handler;
  dynamixel::PacketHandler * packet_handler;

  int baudrate = 1000000;  // 57600
};
}  // namespace motion

#endif  // MOTION__SERIAL_HPP_
