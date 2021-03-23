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
#include <motion_opencr/joint.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

namespace motion
{

MotionManager::MotionManager()
{
  MotionManager("/dev/ttyACM0", 2.0F);
}

MotionManager::MotionManager(std::string port, float protocol_version)
: port_handler(dynamixel::PortHandler::getPortHandler(port.c_str())), packet_handler(
    dynamixel::PacketHandler::getPacketHandler(protocol_version))
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

bool MotionManager::torque_enable(std::vector<Joint> joints)
{
  bool torque_enable_state = false;

  for (auto joint : joints) {
    torque_enable_state = torque_enable(joint);
  }

  return torque_enable_state;
}

bool MotionManager::torque_enable(Joint joint)
{
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  uint8_t torque_enable = 1;

  // Enable Torque
  dxl_comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), static_cast<uint8_t>(MXAddress::TORQUE_ENABLE),
    torque_enable, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
      static_cast<int>(joint.get_id()) << "]. " << packet_handler->getTxRxResult(dxl_comm_result) <<
      "\n";
    return false;
  } else if (dxl_error != 0) {
    std::cout << "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
      static_cast<int>(joint.get_id()) << "]. " << packet_handler->getRxPacketError(dxl_error) <<
      "\n";
    return false;
  } else {
    std::cout << "[ID:" << std::setfill('0') << std::setw(2) <<
      static_cast<int>(joint.get_id()) << "]. has been successfully connected\n";
  }

  std::cout << "\033[H\033[J";

  return true;
}

bool MotionManager::torque_disable(std::vector<Joint> joints)
{
  bool torque_disable_state = false;

  for (auto joint : joints) {
    torque_disable_state = torque_disable(joint);
  }

  return torque_disable_state;
}

bool MotionManager::torque_disable(Joint joint)
{
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;
  uint8_t torque_disable = 0;

  // Disable Torque
  dxl_comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), static_cast<uint8_t>(MXAddress::TORQUE_ENABLE),
    torque_disable, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to disable torque [ID:" << std::setfill('0') << std::setw(2) <<
      static_cast<int>(joint.get_id()) << "]. " << packet_handler->getTxRxResult(dxl_comm_result) <<
      "\n";
    return false;
  } else if (dxl_error != 0) {
    std::cout << "failed to disable torque [ID:" << std::setfill('0') << std::setw(2) <<
      static_cast<int>(joint.get_id()) << "]. " << packet_handler->getRxPacketError(dxl_error) <<
      "\n";
    return false;
  }

  std::cout << "\033[H\033[J";

  return true;
}

bool MotionManager::sync_write_joints(std::vector<Joint> joints, MXAddress address, int byte_length)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite group_sync_write(port_handler, packet_handler, static_cast<uint8_t>(address), byte_length);

  int dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint8_t dxl_error = 0;
  uint8_t param_data[4];

  // Add parameter storage for Dynamixel value
  for (auto joint : joints) {
    // Allocate value into byte array
    param_data[0] = DXL_LOBYTE(DXL_LOWORD(joint.get_position()));
    param_data[1] = DXL_HIBYTE(DXL_LOWORD(joint.get_position()));
    param_data[2] = DXL_LOBYTE(DXL_HIWORD(joint.get_position()));
    param_data[3] = DXL_HIBYTE(DXL_HIWORD(joint.get_position()));

    // Add Dynamixel data value to the Syncwrite storage
    dxl_addparam_result = group_sync_write.addParam(joint.get_id(), param_data);
    if (dxl_addparam_result != true)
    {
      std::cout << "[ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. group_sync_write addparam failed\n";
    }
  }

  // Syncwrite data
  dxl_comm_result = group_sync_write.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to synwrite data. " << packet_handler->getTxRxResult(dxl_comm_result) << "\n";
    return false;
  }

  // Clear syncwrite parameter storage
  group_sync_write.clearParam();

  return true;
}

bool MotionManager::sync_read_joints(std::vector<Joint> joints, MXAddress address, int byte_length)
{
  // Initialize Groupsyncread instance
  dynamixel::GroupSyncRead group_sync_read(port_handler, packet_handler, static_cast<uint8_t>(address), byte_length);

  std::vector<Joint> data_result;

  int dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint8_t dxl_error = 0;
  int32_t dxl_data_result = 0;

  // Add parameter storage for Dynamixel value
  for (auto joint : joints) {
    dxl_addparam_result = group_sync_read.addParam(joint.get_id());
    if (dxl_addparam_result != true) {
      std::cout << "[ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. group_sync_read addparam failed\n";
      return false;
    }
  }

  // Syncread data
  dxl_comm_result = group_sync_read.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) {
    std::cout << "failed to syncread data " << packet_handler->getTxRxResult(dxl_comm_result) << "\n";
  }

  // Check if groupsyncread data of Dynamixel is available
  for (int index = 0; index < joints.size(); index++) {
    dxl_getdata_result = group_sync_read.isAvailable(joints.at(index).get_id(), static_cast<uint8_t>(address),
      byte_length);
    if (dxl_getdata_result != true) {
      std::cout << "[ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joints.at(index).get_id()) << "]. group_sync_read getdata failed\n";
    } else {
      // Get Dynamixel#1 present position value
      dxl_data_result = group_sync_read.getData(joints.at(index).get_id(), static_cast<uint8_t>(address), byte_length);
      std::cout << "[ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joints.at(index).get_id()) << "]. data: " << std::setfill('0') <<
        std::setw(4) << dxl_data_result << "\n";
      joints.at(index).set_present_position(dxl_data_result);
    }
  }

  // Clear syncread parameter storage
  group_sync_read.clearParam();

  return true;
}

void MotionManager::move_joint(Joint joint, float speed)
{
}

void MotionManager::move_joint(std::vector<Joint> joints, float speed)
{
  for (auto joint : joints) {
    move_joint(joint, speed);
  }
}

}  // namespace motion
