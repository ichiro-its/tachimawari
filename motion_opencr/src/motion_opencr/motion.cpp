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

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  std::string joint_name = "right_shoulder_pitch";

  uint8_t addr_mx_torque_enable = 64;
  uint8_t addr_mx_goal_position = 116;
  uint8_t addr_mx_present_position = 132;

  uint8_t torque_enable = 1;
  uint32_t dxl_present_position = 0;
  int32_t dxl_moving_treshold = 0;

  // Open port
  std::cout << "open the port\n";
  if (port_handler->openPort()) {
    std::cout << "succeeded to open the port!\n";
  } else {
    std::cout << "failed to open the port!\n" <<
      "try again!\n";
    return 0;
  }

  // Set baudrate
  if (port_handler->setBaudRate(baudrate)) {
    std::cout << "succeeded to set the baudrate!\n";
  } else {
    std::cout << "failed to set the baudrate!\n" <<
      "try again!\n";
    return 0;
  }

  std::cout << "\033[H\033[J";

  // Enable Torque
  dxl_comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), addr_mx_torque_enable, torque_enable, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to enable torque. " <<
      packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    return 0;
  } else if (dxl_error != 0) {
    std::cout << "failed to enable torque. " <<
      packet_handler->getRxPacketError(dxl_error) << "\n";
    return 0;
  } else {
    std::cout << "dynamixel has been successfully connected\n";
  }
}

void Motion::stop()
{
  uint8_t addr_mx_torque_enable = 64;
  uint8_t torque_disable = 0;
  uint8_t dxl_error = 0;

  // Disable Torque
  dxl_comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), addr_mx_torque_enable, torque_disable, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to disable torque. " <<
      packet_handler->getTxRxResult(dxl_comm_result) << "\n";
  } else if (dxl_error != 0) {
    std::cout << "failed to disable torque. " <<
      packet_handler->getRxPacketError(dxl_error) << "\n";
  }

  // Close port
  port_handler->closePort();
}

void Motion::start_pose(uint8_t id)
{

}

void Motion::insert_pose(Pose pose)
{
  insert_pose(poses.size(), pose);
}

void Motion::insert_pose(uint8_t id, Pose pose)
{
  
}

void Motion::delete_pose(uint8_t id)
{
  
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
