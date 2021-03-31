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

#include <tachimawari/motion_manager.hpp>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>

#include <tachimawari_interfaces/srv/set_joints.hpp>
#include <tachimawari_interfaces/msg/joint.hpp>

#include <tachimawari/joint.hpp>

#include <iostream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace tachimawari
{

MotionManager::MotionManager(std::string node_name, std::string port, float protocol_version)
: rclcpp::Node(node_name), port_handler(dynamixel::PortHandler::getPortHandler(port.c_str())),
  packet_handler(dynamixel::PacketHandler::getPacketHandler(protocol_version))
{
  {
    using SetJoints = tachimawari_interfaces::srv::SetJoints;
    using Joint = tachimawari::Joint;

    set_joints_service = this->create_service<SetJoints>(
      node_name + "/set_joints",
      [this](std::shared_ptr<SetJoints::Request> request,
      std::shared_ptr<SetJoints::Response> response) {
        (void)request;
        std::vector<Joint> joints;
        std::vector<tachimawari_interfaces::msg::Joint>
        joints_message = request->joints;

        for (int index = 0; index < static_cast<int>(joints_message.size()); index++) {
          Joint joint(joints_message.at(index).name.c_str());

          joint.set_target_position(
            joints_message.at(index).position, joints_message.at(index).speed);
          joints.push_back(joint);
        }

        response->status = move_joints(joints);
      }
    );
  }
}

MotionManager::~MotionManager()
{
  stop();
}

bool MotionManager::start()
{
  int baudrate = 57600;

  // Open port
  if (port_handler->openPort()) {
    RCLCPP_INFO_STREAM(get_logger(), "succeeded to open the port!");
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "failed to open the port!, try again!");
    return false;
  }

  // Set baudrate
  if (port_handler->setBaudRate(baudrate)) {
    RCLCPP_INFO_STREAM(get_logger(), "succeeded to set the baudrate!");
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "failed to set the baudrate!, try again!");
    stop();
    return false;
  }

  return true;
}

void MotionManager::stop()
{
  // disable toruqe
  torque_disable(joints);

  // Close port
  port_handler->closePort();
}

bool MotionManager::torque_enable(std::vector<Joint> joints)
{
  if (torque_enabled) {
    return torque_enabled;
  }

  bool torque_enable_state = false;

  for (auto joint : joints) {
    torque_enable_state = torque_enable(joint);
  }

  torque_enabled = true;
  return torque_enable_state;
}

bool MotionManager::torque_enable(Joint joint)
{
  int comm_result = COMM_TX_FAIL;
  uint8_t comm_error = 0;
  uint8_t torque_enable = 1;

  // Enable Torque
  comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), static_cast<uint8_t>(MXAddress::TORQUE_ENABLE),
    torque_enable, &comm_error);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. " << packet_handler->getTxRxResult(comm_result));
    return false;
  } else if (comm_error != 0) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. " << packet_handler->getRxPacketError(comm_error));
    return false;
  }

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
  int comm_result = COMM_TX_FAIL;
  uint8_t comm_error = 0;
  uint8_t torque_disable = 0;

  // Disable Torque
  comm_result = packet_handler->write1ByteTxRx(
    port_handler, joint.get_id(), static_cast<uint8_t>(MXAddress::TORQUE_ENABLE),
    torque_disable, &comm_error);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. " << packet_handler->getTxRxResult(comm_result));
    return false;
  } else if (comm_error != 0) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to enable torque [ID:" << std::setfill('0') << std::setw(2) <<
        static_cast<int>(joint.get_id()) << "]. " << packet_handler->getRxPacketError(comm_error));
    return false;
  }

  return true;
}

bool MotionManager::sync_write_joints(
  std::vector<Joint> joints, MXAddress start_address,
  int data_length)
{
  // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite group_sync_write(port_handler, packet_handler,
    static_cast<uint8_t>(start_address), data_length);

  uint8_t param_data[4];
  bool addparam_state = false;

  // Add parameter storage for Dynamixel value
  for (auto joint : joints) {
    // Allocate value into byte array
    param_data[0] = DXL_LOBYTE(DXL_LOWORD(joint.get_position()));
    param_data[1] = DXL_HIBYTE(DXL_LOWORD(joint.get_position()));
    param_data[2] = DXL_LOBYTE(DXL_HIWORD(joint.get_position()));
    param_data[3] = DXL_HIBYTE(DXL_HIWORD(joint.get_position()));

    // Add Dynamixel data value to the Syncwrite storage
    addparam_state = group_sync_write.addParam(joint.get_id(), param_data);
    if (addparam_state != true) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "[ID:" << std::setfill('0') << std::setw(2) <<
          static_cast<int>(joint.get_id()) << "]. group_sync_write addparam failed");
    }
  }

  // Syncwrite data
  int comm_result = COMM_TX_FAIL;
  comm_result = group_sync_write.txPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to synwrite data. " << packet_handler->getTxRxResult(comm_result));
    return false;
  }

  // Clear syncwrite parameter storage
  group_sync_write.clearParam();

  return true;
}

bool MotionManager::sync_read_joints(
  std::vector<Joint> & joints, MXAddress start_address,
  int data_length)
{
  // Initialize Groupsyncread instance
  dynamixel::GroupSyncRead group_sync_read(port_handler, packet_handler,
    static_cast<uint8_t>(start_address), data_length);

  // Add parameter storage for Dynamixel value
  bool addparam_state = false;
  for (auto joint : joints) {
    addparam_state = group_sync_read.addParam(joint.get_id());
    if (addparam_state != true) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "[ID:" << std::setfill('0') << std::setw(2) <<
          static_cast<int>(joint.get_id()) << "]. syncread addparam failed");
    }
  }

  // Syncread data
  int comm_result = COMM_TX_FAIL;
  comm_result = group_sync_read.txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to syncread data " << packet_handler->getTxRxResult(comm_result));
    return false;
  }

  bool getdata_state = false;
  int32_t data_result = 0;

  for (int index = 0; index < static_cast<int>(joints.size()); index++) {
    // Check if groupsyncread data of Dynamixel is available
    getdata_state = group_sync_read.isAvailable(
      joints.at(index).get_id(), static_cast<uint8_t>(start_address), data_length);
    if (getdata_state != true) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "[ID:" << std::setfill('0') << std::setw(2) <<
          static_cast<int>(joints.at(index).get_id()) << "]. syncread getdata failed");
    } else {
      // Get Dynamixel present position value
      data_result = group_sync_read.getData(
        joints.at(index).get_id(), static_cast<uint8_t>(start_address), data_length);
      joints.at(index).set_present_position(data_result);
    }
  }

  // Clear syncread parameter storage
  group_sync_read.clearParam();

  return true;
}

bool MotionManager::bulk_read_joints(
  std::vector<Joint> & joints, MXAddress start_address,
  int data_length)
{
  // Initialize GroupBulkRead instance
  dynamixel::GroupBulkRead group_bulk_read(port_handler, packet_handler);

  // Add parameter storage for Dynamixel data
  bool addparam_state = false;
  for (auto joint : joints) {
    addparam_state = group_bulk_read.addParam(
      joint.get_id(), static_cast<uint16_t>(start_address), data_length);
    if (addparam_state != true) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "[ID:" << std::setfill('0') << std::setw(2) <<
          static_cast<int>(joint.get_id()) << "]. bulkread addparam failed");
      return false;
    }
  }

  // Syncread data
  int comm_result = COMM_TX_FAIL;
  comm_result = group_bulk_read.txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "failed to bulkread data " << packet_handler->getTxRxResult(comm_result));
    return false;
  }

  bool getdata_state = false;
  int32_t data_result = 0;

  for (int index = 0; index < static_cast<int>(joints.size()); index++) {
    // Check if groupsyncread data of Dynamixel is available
    getdata_state = group_bulk_read.isAvailable(
      joints.at(index).get_id(), static_cast<uint8_t>(start_address), data_length);
    if (getdata_state != true) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "[ID:" << std::setfill('0') << std::setw(2) <<
          static_cast<int>(joints.at(index).get_id()) << "]. bulkread getdata failed");
    } else {
      // Get Dynamixel present position value
      data_result = group_bulk_read.getData(
        joints.at(index).get_id(), static_cast<uint8_t>(start_address), data_length);
      joints.at(index).set_present_position(data_result);
    }
  }

  // Clear syncread parameter storage
  group_bulk_read.clearParam();

  return true;
}

bool MotionManager::init_joints_present_position(std::vector<Joint> joints)
{
  bool init_state = true;

  if (!init_joints_state) {
    init_state = sync_read_joints(joints);
    this->joints = joints;
    init_joints_state = true;
  } else {
    for (int index = 0; index < static_cast<int>(joints.size()); index++) {
      joints.at(index).set_present_position(this->joints.at(index).get_goal_position());
    }
    this->joints = joints;
  }

  return init_state;
}

bool MotionManager::move_joint(Joint /*joint*/)
{
  return true;
}

bool MotionManager::move_joints(std::vector<Joint> joints)
{
  bool move_joints_state = true;
  rclcpp::Rate rcl_rate(8ms);

  if (torque_enable(joints)) {
    if (init_joints_present_position(joints)) {
      while (true) {
        rcl_rate.sleep();

        for (int index = 0; index < static_cast<int>(joints.size()); index++) {
          joints.at(index).interpolate();
        }

        move_joints_state = sync_write_joints(joints);

        if (!move_joints_state) {
          break;
        }
      }
    }
  }

  return move_joints_state;
}

}  // namespace tachimawari
