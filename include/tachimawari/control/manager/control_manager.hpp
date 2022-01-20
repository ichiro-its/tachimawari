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

#ifndef TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_
#define TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_

#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari
{

class ControlManager
{
public:
  explicit ControlManager(
    const std::string & port_name, const float & protocol_version, const int & baudrate);
  virtual ~ControlManager() {}

  virtual bool connect();
  virtual void disconnect();

  virtual bool sync_write_joints(const std::vector<joint::Joint> & joints,
    const bool & with_pid);

  virtual bool bulk_read_joints(const std::vector<joint::Joint> & joints);

protected:
  std::string port_name;
  float protocol_version;
  int baudrate;
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__CONTROL__MANAGER__CONTROL_MANAGER_HPP_
