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

#ifndef MOTION_OPENCR__MOTION_MANAGER_HPP_
#define MOTION_OPENCR__MOTION_MANAGER_HPP_

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <motion_opencr/motion.hpp>

#include <map>
#include <memory>
#include <string>

namespace motion
{

class MotionManager
{
public:
  explicit MotionManager();
  explicit MotionManager(std::string port, float protocol_version);

  void start();
  void stop();

  void insert_motion(uint8_t id, std::shared_ptr<Motion> motion);
  void delete_motion(uint8_t id);

  void run_motion(uint8_t motion_id);
  void run_pose(uint8_t motion_id, uint8_t pose_id);

private:
  dynamixel::PortHandler * port_handler;
  dynamixel::PacketHandler * packet_handler;

  std::map<uint8_t, std::shared_ptr<Motion>> motion_list;
};

}  // namespace motion

#endif  // MOTION_OPENCR__MOTION_MANAGER_HPP_
