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

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__CM740_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__CM740_HPP_

#include <vector>

#include "tachimawari/control/manager/control_manager.hpp"

namespace tachimawari
{

class CM740 : public ControlManager
{
public:
  explicit CM740(
    const std::string & port_name, const int & baudrate = 1000000,
    const float & protocol_version = 1.0);

  bool connect() override;
  void disconnect() override;

  bool sync_write_joints(const std::vector<joint::Joint> & joints,
    const bool & with_pid) override;

  bool bulk_read_joints(const std::vector<joint::Joint> & joints) override;
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__CM740_HPP_
