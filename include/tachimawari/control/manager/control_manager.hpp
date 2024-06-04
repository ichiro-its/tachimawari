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

#ifndef TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_
#define TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_

#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::control
{

class ControlManager
{
public:
  enum : uint8_t {MARIN_CORE = 190, CONTROLLER = 200, BROADCAST = 254};

  explicit ControlManager(const std::string & port_name, float protocol_version, int baudrate);
  virtual ~ControlManager() {}

  float get_protocol_version() const;

  virtual void set_port(const std::string & port_name) {}

  virtual bool connect();
  virtual void disconnect() {}

  virtual bool ping(uint8_t id) {}

  virtual bool write_packet(uint8_t id, uint16_t address, int value, int data_length = 1) {}

  virtual int read_packet(uint8_t id, uint16_t address, int data_length = 1) {}

  virtual bool sync_write_packet(const std::vector<joint::Joint> & joints, bool with_pid = true) {}

  virtual bool send_bulk_read_packet() {}

  virtual bool add_default_bulk_read_packet() {}

  virtual int get_data(uint8_t id, uint16_t address, int data_lenghth = 1) {}

  virtual int get_bulk_data(uint8_t id, uint16_t address, int data_length = 1) {}

protected:
  std::string port_name;
  float protocol_version;
  int baudrate;
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_
