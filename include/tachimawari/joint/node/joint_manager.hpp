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

#ifndef TACHIMAWARI__JOINT__NODE__JOINT_MANAGER_HPP_
#define TACHIMAWARI__JOINT__NODE__JOINT_MANAGER_HPP_

#include <memory>
#include <vector>

#include "tachimawari/control/manager/control_manager.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace tachimawari::joint
{

class JointManager
{
public:
  explicit JointManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager);

  bool torque_enable(bool enable);
  bool torque_enable(const std::vector<Joint> & joints, bool enable);

  bool set_joints(const std::vector<Joint> & joints);

  const std::vector<Joint> & get_current_joints();

private:
  void update_current_joints(const std::vector<Joint> & joints);
  void update_current_joints_from_control_manager(const std::vector<Joint> & joints);

  std::shared_ptr<tachimawari::control::ControlManager> control_manager;

  std::vector<Joint> current_joints;
  bool is_each_joint_updated;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT__NODE__JOINT_MANAGER_HPP_
