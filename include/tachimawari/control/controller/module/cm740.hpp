// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/control/controller/platform/linux.hpp"
#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/control/packet/protocol_1/instruction/bulk_read_packet.hpp"
#include "tachimawari/control/packet/protocol_1/status/bulk_read_data.hpp"
#include "tachimawari/control/packet/protocol_1/status/status_packet.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari
{

namespace control
{

class CM740 : public ControlManager
{
public:
  enum
  {
    MAX_PACKET_SIZE = 1024
  };

  explicit CM740(
    const std::string & port_name, const int & baudrate = 1000000,
    const float & protocol_version = 1.0);
  
  // will be used if there is another option
  // for serial comunication
  void set_platform(/* platform */);

  bool connect() override;
  void disconnect() override;

  bool ping(const uint8_t & id) override;

  bool write_packet(const uint8_t & address, const int & value,
    const int & data_length = 1) override;

  bool write_packet(const uint8_t & id, const uint8_t & address, const int & value,
    const int & data_length = 1) override;

  bool sync_write_packet(const std::vector<joint::Joint> & joints,
    const bool & with_pid = false) override;

  bool bulk_read_packet() override;

  bool bulk_read_packet(const std::vector<joint::Joint> & joints) override;

  int get_bulk_data(const uint8_t & id, const uint8_t & address,
    const int & data_length = 1) override;

private:
  bool dxl_power_on();

  packet::protocol_1::StatusPacket send_packet(packet::protocol_1::Packet packet);

  bool send_bulk_read_packet(packet::protocol_1::BulkReadPacket packet);

  std::shared_ptr<Linux> platform;

  double byte_transfer_time;

  std::shared_ptr<std::map<uint8_t, packet::protocol_1::BulkReadData>> bulk_data;
};

}  // namespace control

}  // namespace tachimawari

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_HPP_
