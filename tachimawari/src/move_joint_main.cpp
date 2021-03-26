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

#include <tachimawari/joint.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <iostream>
#include <string>
#include <vector>
#include <array>

using Joint = tachimawari::Joint;

int main(int argc, char * argv[])
{
  std::string port_name = "/dev/ttyACM0";
  int baudrate = 57600;

  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;

  std::string joint_name = "right_shoulder_pitch";

  uint8_t addr_mx_torque_enable = 64;
  uint8_t addr_mx_goal_position = 116;
  uint8_t addr_mx_present_position = 132;

  uint8_t torque_enable = 1;
  uint8_t torque_disable = 0;
  uint32_t dxl_present_position = 0;
  int32_t dxl_moving_treshold = 0;

  if (argc > 1) {
    port_name = argv[1];
  }
  if (argc > 2) {
    joint_name = argv[2];
  }

  std::cout << "set the port name as " << port_name << "\n";
  dynamixel::PortHandler * port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
  dynamixel::PacketHandler * packet_handler = dynamixel::PacketHandler::getPacketHandler(2.0F);

  Joint joint(joint_name);

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

  // Write Goal Position
  joint.set_target_position(90.0);
  dxl_comm_result = packet_handler->write4ByteTxRx(
    port_handler, joint.get_id(), addr_mx_goal_position, joint.get_position(), &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to write the goal position. " <<
      packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    return 0;
  } else if (dxl_error != 0) {
    std::cout << "failed to write the goal position. " <<
      packet_handler->getRxPacketError(dxl_error) << "\n";
    return 0;
  } else {
    std::cout << "succeeded to write the goal position\n";
  }

  // Read present position
  do {
    dxl_comm_result = packet_handler->read4ByteTxRx(
      port_handler, joint.get_id(), addr_mx_present_position,
      static_cast<uint32_t *>(&dxl_present_position), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      std::cout << "error. " << packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    } else if (dxl_error != 0) {
      std::cout << "error. " << packet_handler->getRxPacketError(dxl_error) << "\n";
    } else {
      std::cout << "success, present position: " << dxl_present_position << "\n";
    }
  } while ((abs(joint.get_position() - dxl_present_position) > dxl_moving_treshold));

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

  return 0;
}
