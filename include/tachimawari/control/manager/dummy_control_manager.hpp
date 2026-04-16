// Copyright (c) 2021-2023 Ichiro ITS
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

#ifndef TACHIMAWARI__CONTROL__MANAGER__DUMMY_CONTROL_MANAGER_HPP_
#define TACHIMAWARI__CONTROL__MANAGER__DUMMY_CONTROL_MANAGER_HPP_

#include <string>
#include <vector>

#include "tachimawari/control/manager/control_manager.hpp"

namespace tachimawari::control
{

class DummyControlManager : public ControlManager
{
public:
  explicit DummyControlManager(
    const std::string & port_name = "dummy", int baudrate = 1000000, float protocol_version = 1.0);

  void set_port(const std::string & port_name) override;

  bool connect() override;
  bool ping(uint8_t id) override;
  bool write_packet(uint8_t id, uint16_t address, int value, int data_length = 1) override;
  int read_packet(uint8_t id, uint16_t address, int data_length = 1) override;
  bool sync_write_packet(const std::vector<joint::Joint> & joints, bool with_pid = true) override;
  bool send_bulk_read_packet() override;
  bool add_default_bulk_read_packet() override;
  int get_data(uint8_t id, uint16_t address, int data_lenghth = 1) override;
  int get_bulk_data(uint8_t id, uint16_t address, int data_length = 1) override;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__MANAGER__DUMMY_CONTROL_MANAGER_HPP_
