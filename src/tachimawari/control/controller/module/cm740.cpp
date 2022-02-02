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

#include "tachimawari/control/controller/module/cm740.hpp"

#include "tachimawari/control/controller/module/cm740_address.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/bulk_read_packet.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/instruction.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/sync_write_packet.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/write_packet.hpp"
#include "tachimawari/control/packet/protocol_1/model/packet_id.hpp"
#include "tachimawari/control/packet/protocol_1/status/bulk_read_data.hpp"
#include "tachimawari/control/packet/protocol_1/status/status_packet.hpp"
#include "tachimawari/control/packet/protocol_1/utils/word.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

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

namespace control
{

CM740::CM740(
  const std::string & port_name, const int & baudrate,
  const float & protocol_version)
: ControlManager(port_name, protocol_version, baudrate), byte_transfer_time(0.0),
  platform(std::make_shared<Linux>()),
  bulk_data(std::make_shared<std::map<uint8_t, packet::protocol_1::BulkReadData>>())
{
}

bool CM740::connect()
{
  if (platform->open_port(port_name, baudrate)) {
    byte_transfer_time = (1000.0 / baudrate) * 12.0;

    return dxl_power_on();
  }

  return false;
}

bool CM740::dxl_power_on()
{
  if (write_packet(CM740Address::DXL_POWER, 1)) {
    if (protocol_version == 1.0) {
      write_packet(CM740Address::LED_HEAD_L, packet::protocol_1::Word::make_color(255, 128, 0), 2);
    }

    return true;
  }

  return false;
}

packet::protocol_1::StatusPacket CM740::send_packet(packet::protocol_1::Packet packet)
{
  {
    using StatusPacket = packet::protocol_1::StatusPacket;

    std::vector<uint8_t> txpacket = packet.get_packet();

    if (platform->write_port(txpacket) == txpacket.size()) {
      // set packet timeout;
      int get_length = 0;
      int expected_length = packet.get_expected_length();
      auto rxpacket = std::make_shared<std::vector<uint8_t>>(MAX_PACKET_SIZE, 0x00);

      while (true) {
        get_length += platform->read_port(rxpacket, expected_length - get_length, get_length);

        if (get_length == expected_length) {
          int new_get_length = StatusPacket::validate(rxpacket, get_length);

          if (new_get_length == get_length) {
            return StatusPacket(rxpacket, get_length);
          } else if (new_get_length != 0) {
            // is packet timeout ? so RX_TIMEOUT
            get_length = new_get_length;
          } else {
            // so RX_CORRUPT
            return StatusPacket(nullptr, 0);
          }
        } else {
          // is packet timeout ? so RX_TIMEOUT
          if (get_length > expected_length) {
            break;
          }
        }
      }
    } else {
      // so TX_FAIL
    }

    return StatusPacket(nullptr, 0);
  }
}

bool CM740::send_bulk_read_packet(packet::protocol_1::BulkReadPacket packet)
{
  {
    using BulkReadData = packet::protocol_1::BulkReadData;

    std::vector<uint8_t> txpacket = packet.get_packet();

    if (platform->write_port(txpacket) == txpacket.size()) {
      int data_number = packet.get_data_number();
      BulkReadData::insert_all(bulk_data, packet);

      // set packet timeout;
      int get_length = 0;
      int expected_length = packet.get_expected_length();
      auto rxpacket = std::make_shared<std::vector<uint8_t>>(MAX_PACKET_SIZE, 0x00);

      while (true) {
        get_length += platform->read_port(rxpacket, expected_length - get_length, get_length);

        if (get_length == expected_length) {
          int new_get_length = BulkReadData::validate(rxpacket, get_length);

          if (new_get_length == get_length) {
            data_number = BulkReadData::update_all(bulk_data, rxpacket, get_length, data_number);

            if (data_number == 0) {
              return true;
            } else {
              // is packet timeout ? so RX_TIMEOUT
              // or RX_CORRUPT / data is inclompete if data number more than 0
              return false;
            }
          } else {
            // is packet timeout ? so RX_TIMEOUT
            get_length = new_get_length;
          }
        } else {
          // is packet timeout ? so RX_TIMEOUT
          if (get_length > expected_length) {
            break;
          }
        }
      }
    } else {
      // so TX_FAIL
    }

    return false;
  }
}

bool CM740::ping(const uint8_t & id)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::Packet instruction_packet(id, packet::protocol_1::Instruction::PING);
    auto status_packet = send_packet(instruction_packet);

    return status_packet.is_success();
  }

  return false;
}

bool CM740::write_packet(
  const uint8_t & address, const int & value,
  const int & data_length)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::WritePacket instruction_packet;

    if (data_length == 1) {
      instruction_packet.create(address, static_cast<uint8_t>(value));
    } else if (data_length == 2) {
      instruction_packet.create(address, static_cast<uint16_t>(value));
    }

    auto status_packet = send_packet(instruction_packet);
    return status_packet.is_success();
  }

  return false;
}

bool CM740::write_packet(
  const uint8_t & id, const uint8_t & address, const int & value,
  const int & data_length)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::WritePacket instruction_packet;

    if (data_length == 1) {
      instruction_packet.create(id, address, static_cast<uint8_t>(value));
    } else if (data_length == 2) {
      instruction_packet.create(id, address, static_cast<uint16_t>(value));
    }

    auto status_packet = send_packet(instruction_packet);
    return status_packet.is_success();
  }

  return false;
}

bool CM740::sync_write_packet(const std::vector<joint::Joint> & joints, const bool & with_pid)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::SyncWritePacket instruction_packet;

    instruction_packet.create(joints, with_pid ?
      tachimawari::joint::protocol_1::MX28Address::D_GAIN :
      tachimawari::joint::protocol_1::MX28Address::GOAL_POSITION_L);

    std::vector<uint8_t> txpacket = instruction_packet.get_packet();
    return platform->write_port(txpacket) == txpacket.size();
  }

  return false;
}

bool CM740::bulk_read_packet()
{
  if (protocol_version == 1.0) {
    packet::protocol_1::BulkReadPacket instruction_packet;

    if (ping(packet::protocol_1::PacketId::CONTROLLER)) {
      instruction_packet.add(
        packet::protocol_1::PacketId::CONTROLLER, CM740Address::DXL_POWER,
        static_cast<uint8_t>(30));
    }

    if (instruction_packet.is_parameters_filled()) {
      return send_bulk_read_packet(instruction_packet);
    } else {
      return false;
    }
  }

  return false;
}

bool CM740::bulk_read_packet(const std::vector<joint::Joint> & joints)
{
  if (protocol_version == 1.0) {
    packet::protocol_1::BulkReadPacket instruction_packet;

    if (joints.size()) {
      instruction_packet.add(joints);
    }

    if (instruction_packet.is_parameters_filled()) {
      return send_bulk_read_packet(instruction_packet);
    } else {
      return false;
    }
  }

  return false;
}

int CM740::get_bulk_data(
  const uint8_t & id, const uint8_t & address,
  const int & data_length)
{
  if (bulk_data->find(id) != bulk_data->end()) {
    if (data_length == 1) {
      return bulk_data->at(id).get(static_cast<uint8_t>(address));
    } else if (data_length == 2) {
      return bulk_data->at(id).get(static_cast<uint16_t>(address));
    }
  } else {
    return -1;
  }

  return -1;
}

void CM740::disconnect()
{
  if (protocol_version == 1.0) {
    write_packet(CM740Address::LED_HEAD_L, packet::protocol_1::Word::make_color(0, 255, 0), 2);
  }

  platform->close_port();
}

CM740::~CM740()
{
  disconnect();
}

}  // namespace control

}  // namespace tachimawari
