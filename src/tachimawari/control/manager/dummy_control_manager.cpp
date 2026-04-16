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

#include "tachimawari/control/manager/dummy_control_manager.hpp"

namespace tachimawari::control
{

DummyControlManager::DummyControlManager(
  const std::string & port_name, int baudrate, float protocol_version)
: ControlManager(port_name, protocol_version, baudrate)
{
}

void DummyControlManager::set_port(const std::string & port_name)
{
  this->port_name = port_name;
}

bool DummyControlManager::connect() { return true; }

bool DummyControlManager::ping(uint8_t) { return true; }

bool DummyControlManager::write_packet(uint8_t, uint16_t, int, int)
{
  return true;
}

int DummyControlManager::read_packet(uint8_t, uint16_t, int)
{
  return -1;
}

bool DummyControlManager::sync_write_packet(
  const std::vector<joint::Joint> &, bool)
{
  return true;
}

bool DummyControlManager::send_bulk_read_packet() { return true; }

bool DummyControlManager::add_default_bulk_read_packet() { return true; }

int DummyControlManager::get_data(uint8_t, uint16_t, int)
{
  return 0;
}

int DummyControlManager::get_bulk_data(uint8_t, uint16_t, int)
{
  return 0;
}

}  // namespace tachimawari::control
