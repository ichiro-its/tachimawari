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

#ifndef TACHIMAWARI__CONTROL__SDK__UTILS__PROTOCOL_2__GROUP_SYNC_WRITE_HPP_
#define TACHIMAWARI__CONTROL__SDK__UTILS__PROTOCOL_2__GROUP_SYNC_WRITE_HPP_

#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/protocol_2/mx28_address.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace tachimawari::control::sdk::protocol_2
{

class GroupSyncWrite
{
public:
  GroupSyncWrite(
    dynamixel::PortHandler * port_handler,
    dynamixel::PacketHandler * packet_handler);

  dynamixel::GroupSyncWrite create(
    const std::vector<tachimawari::joint::Joint> & joints,
    uint8_t starting_address =
    tachimawari::joint::protocol_2::MX28Address::GOAL_POSITION);

private:
  dynamixel::PortHandler * port_handler;
  dynamixel::PacketHandler * packet_handler;
};

}  // namespace tachimawari::control::sdk::protocol_2

#endif  // TACHIMAWARI__CONTROL__SDK__UTILS__PROTOCOL_2__GROUP_SYNC_WRITE_HPP_
