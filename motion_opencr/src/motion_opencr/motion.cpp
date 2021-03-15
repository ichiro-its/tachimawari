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

#include <motion_opencr/motion.hpp>

#include <iostream>

namespace motion
{

Motion::Motion(std::string pose_name)
: name(pose_name)
{
  port_handler = dynamixel::PortHandler::getPortHandler("/dev/ttyACM0");
  packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0F);
}

void Motion::start()
{
  int baudrate = 57600;

  // Open port
  std::cout << "open the port\n";
  if (port_handler->openPort()) {
    std::cout << "succeeded to open the port!\n";
  } else {
    std::cout << "failed to open the port!\n" <<
      "try again!\n";
    stop();
  }

  // Set baudrate
  if (port_handler->setBaudRate(baudrate)) {
    std::cout << "succeeded to set the baudrate!\n";
  } else {
    std::cout << "failed to set the baudrate!\n" <<
      "try again!\n";
    stop();
  }

  std::cout << "\033[H\033[J";
}

void Motion::stop()
{
  // Close port
  port_handler->closePort();
}

void Motion::run_motion()
{
  run_motion(0);
}

void Motion::run_motion(uint8_t id)
{
  for (auto i = id; i < poses.size(); i++) {
    start_pose(i);
  }
}

void Motion::start_pose(uint8_t id)
{

}

void Motion::insert_pose(Pose pose)
{
  poses.push_back(pose);
}

void Motion::insert_pose(uint8_t id, Pose pose)
{
  poses.insert(poses.begin() + id, pose);
}

void Motion::delete_pose(uint8_t id)
{
  poses.erase(poses.begin() + id);
}

void Motion::set_name(std::string new_name)
{
  name = new_name;
}

std::string Motion::get_name()
{
  return name;
}

}  // namespace motion
