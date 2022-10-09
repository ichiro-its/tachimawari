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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/sdk/module/dynamixel_sdk.hpp"

#include "tachimawari/control/controller/module/cm740_address.hpp"
#include "tachimawari/control/controller/packet/protocol_1/utils/word.hpp"
#include "tachimawari/control/sdk/packet/model/group_bulk_read.hpp"
#include "tachimawari/control/sdk/packet/protocol_1/group_bulk_read.hpp"
#include "tachimawari/control/sdk/packet/protocol_1/group_sync_write.hpp"
#include "tachimawari/control/sdk/packet/protocol_2/group_sync_write.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"
#include "tachimawari/joint/protocol_2/mx28_address.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari::control
{

DynamixelSDK::DynamixelSDK(
  const std::string & port_name, int baudrate,
  float protocol_version)
: ControlManager(port_name, protocol_version, baudrate),
  port_handler(dynamixel::PortHandler::getPortHandler(port_name.c_str())),
  packet_handler(dynamixel::PacketHandler::getPacketHandler(protocol_version)),
  bulk_data(std::make_shared<std::map<uint8_t, sdk::GroupBulkRead>>())
{
}

void DynamixelSDK::set_port(const std::string & port_name)
{
  port_handler = dynamixel::PortHandler::getPortHandler(port_name.c_str());
}

bool DynamixelSDK::connect()
{
  if (!port_handler->openPort()) {
    return false;
  }

  if (!port_handler->setBaudRate(baudrate)) {
    disconnect();

    return false;
  }

  if (protocol_version == 1.0) {
    write_packet(
      CONTROLLER, CM740Address::LED_HEAD_L,
      protocol_1::Word::make_color(255, 128, 0), 2);
  }

  return true;
}

bool DynamixelSDK::send_bulk_read_packet(sdk::GroupBulkRead group_bulk_read)
{
  int result = group_bulk_read.send();
  if (result == SUCCESS) {
    sdk::GroupBulkRead::insert_all(bulk_data, group_bulk_read);
  } else {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getTxRxResult(result);

    return false;
  }

  return true;
}

bool DynamixelSDK::ping(uint8_t id)
{
  uint8_t error = 0;
  uint16_t model_number;

  int result = packet_handler->ping(port_handler, id, &model_number, &error);
  if (result != SUCCESS) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getTxRxResult(result);

    return false;
  } else if (error != 0) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getRxPacketError(error);

    return false;
  }

  return true;
}

bool DynamixelSDK::write_packet(
  uint8_t id, uint8_t address, int value,
  int data_length)
{
  uint8_t error = 0;
  uint16_t model_number;
  int result = TX_FAIL;

  if (data_length == 1) {
    result = packet_handler->write1ByteTxRx(
      port_handler, id, address,
      static_cast<uint8_t>(value), &error);
  } else if (data_length == 2) {
    result = packet_handler->write2ByteTxRx(
      port_handler, id, address,
      static_cast<uint16_t>(value), &error);
  } else if (data_length == 4) {
    result = packet_handler->write4ByteTxRx(
      port_handler, id, address,
      static_cast<uint32_t>(value), &error);
  }

  if (result != SUCCESS) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getTxRxResult(result);

    return false;
  } else if (error != 0) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getRxPacketError(error);

    return false;
  }

  return true;
}

int DynamixelSDK::read_packet(
  uint8_t id, uint8_t address, int data_length)
{
  int value = -1;
  uint8_t error = 0;
  uint16_t model_number;
  int result = TX_FAIL;

  if (data_length == 1) {
    uint8_t result_val = 0;

    result = packet_handler->read1ByteTxRx(
      port_handler, id, address,
      &result_val, &error);

    value = (result == SUCCESS) ? result_val : value;
  } else if (data_length == 2) {
    uint16_t result_val = 0;

    result = packet_handler->read2ByteTxRx(
      port_handler, id, address,
      &result_val, &error);

    value = (result == SUCCESS) ? result_val : value;
  } else if (data_length == 4) {
    uint32_t result_val = 0;

    result = packet_handler->read4ByteTxRx(
      port_handler, id, address,
      &result_val, &error);

    value = (result == SUCCESS) ? result_val : value;
  }

  if (result != SUCCESS) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getTxRxResult(result);
  } else if (error != 0) {
    // TODO(maroqijalil): will be used for logging
    // packet_handler->getRxPacketError(error);
  }

  return value;
}

bool DynamixelSDK::sync_write_packet(
  const std::vector<joint::Joint> & joints,
  bool with_pid)
{
  int result = TX_FAIL;

  if (protocol_version == 1.0) {
    auto group_sync_write = sdk::protocol_1::GroupSyncWrite(port_handler, packet_handler).create(
      joints, with_pid ? tachimawari::joint::protocol_1::MX28Address::D_GAIN :
      tachimawari::joint::protocol_1::MX28Address::GOAL_POSITION_L);

    result = group_sync_write.txPacket();
    if (result != SUCCESS) {
      // TODO(maroqijalil): will be used for logging
      // packet_handler->getTxRxResult(result);
    }

    group_sync_write.clearParam();
  } else if (protocol_version == 2.0) {
    using sdk::protocol_2::GroupSyncWrite;
    using tachimawari::joint::protocol_2::MX28Address;

    if (with_pid) {
      auto pid_sync_write = GroupSyncWrite(port_handler, packet_handler).create(
        joints, MX28Address::POSITION_D_GAIN);

      result = pid_sync_write.txPacket();
      if (result != SUCCESS) {
        // TODO(maroqijalil): will be used for logging
        // packet_handler->getTxRxResult(result);
      }

      pid_sync_write.clearParam();
    }

    auto position_sync_write = GroupSyncWrite(port_handler, packet_handler).create(
      joints, MX28Address::GOAL_POSITION);

    result = position_sync_write.txPacket();
    if (result != SUCCESS) {
      // TODO(maroqijalil): will be used for logging
      // packet_handler->getTxRxResult(result);
    }

    position_sync_write.clearParam();
  }

  return result == SUCCESS;
}

bool DynamixelSDK::bulk_read_packet()
{
  if (protocol_version == 1.0) {
    sdk::GroupBulkRead group_bulk_read(port_handler, packet_handler);

    if (ping(CONTROLLER)) {
      group_bulk_read.add(CONTROLLER, CM740Address::DXL_POWER, 30u);
    }

    if (group_bulk_read.is_parameters_filled()) {
      return send_bulk_read_packet(group_bulk_read);
    } else {
      return false;
    }
  }

  return false;
}

int DynamixelSDK::get_bulk_data(
  uint8_t id, uint8_t address,
  int data_length)
{
  if (bulk_data->find(id) != bulk_data->end()) {
    return bulk_data->at(id).get(id, address, data_length);
  } else {
    // data is not found
  }

  return -1;
}

void DynamixelSDK::disconnect()
{
  write_packet(CONTROLLER, CM740Address::LED_HEAD_L, protocol_1::Word::make_color(0, 255, 0), 2);

  port_handler->closePort();
}

DynamixelSDK::~DynamixelSDK()
{
  disconnect();
}

}  // namespace tachimawari::control
