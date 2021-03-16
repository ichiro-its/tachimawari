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

#include <motion_opencr/motion_manager.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <iostream>

namespace motion
{

MotionManager::MotionManager()
{
  MotionManager("/dev/ttyACM0", 2.0F);
}

MotionManager::MotionManager(std::string port, float protocol_version)
: port_handler(dynamixel::PortHandler::getPortHandler(port.c_str())), packet_handler(dynamixel::PacketHandler::getPacketHandler(protocol_version))
{
}

void MotionManager::start()
{
  int baudrate = 57600;

  // Open port
  std::cout << "open the port\n";
  if (port_handler->openPort()) {
    std::cout << "succeeded to open the port!\n";
  } else {
    std::cout << "failed to open the port!\n" <<
      "try again!\n";
    return;
  }

  // Set baudrate
  if (port_handler->setBaudRate(baudrate)) {
    std::cout << "succeeded to set the baudrate!\n";
  } else {
    std::cout << "failed to set the baudrate!\n" <<
      "try again!\n";
    stop();
    return;
  }

  std::cout << "\033[H\033[J";
}

void MotionManager::stop()
{
  // Close port
  port_handler->closePort();
}

void MotionManager::insert_motion(uint8_t id, std::shared_ptr<Motion> motion)
{
  motion_list.insert({ id, motion });
}

void MotionManager::delete_motion(uint8_t id)
{
  motion_list.erase(id);
}

}  // namespace motion
