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

#include "tachimawari/joint/tf2/frame_id.hpp"

#include <map>
#include <string>
#include <vector>

#include "tachimawari/joint/model/joint_id.hpp"

namespace tachimawari::joint
{

const std::array<uint8_t, 9> FrameId::frame_ids = {
  BASE_LINK, TORSO, GAZE, RIGHT_THIGH, RIGHT_CALF, RIGHT_FOOT, LEFT_THIGH, LEFT_CALF, LEFT_FOOT,
};

const std::map<uint8_t, std::string> FrameId::frame_id_string = {
  {FrameId::ODOM, "odom"},           {FrameId::BASE_LINK, "base_link"},
  {FrameId::TORSO, "torso"},         {FrameId::GAZE, "gaze"},
  {FrameId::RIGHT_THIGH, "r_thigh"}, {FrameId::RIGHT_CALF, "r_calf"},
  {FrameId::RIGHT_FOOT, "r_sole"},   {FrameId::LEFT_THIGH, "l_thigh"},
  {FrameId::LEFT_CALF, "l_calf"},    {FrameId::LEFT_FOOT, "l_sole"}};

const std::map<std::string, uint8_t> FrameId::frame_string_id = {
  {"odom", FrameId::ODOM},           {"base_link", FrameId::BASE_LINK},
  {"torso", FrameId::TORSO},         {"gaze", FrameId::GAZE},
  {"r_thigh", FrameId::RIGHT_THIGH}, {"r_calf", FrameId::RIGHT_CALF},
  {"r_sole", FrameId::RIGHT_FOOT},   {"l_thigh", FrameId::LEFT_THIGH},
  {"l_calf", FrameId::LEFT_CALF},    {"l_sole", FrameId::LEFT_FOOT}};

const std::map<uint8_t, std::vector<uint8_t>> FrameId::frame_joint_map = {
  {ODOM, {0, 0, 0}},
  {BASE_LINK, {0, 0, 0}},
  {TORSO, {0, 0, 0}},
  {GAZE, {0, JointId::NECK_PITCH, JointId::NECK_YAW}},
  {RIGHT_THIGH, {JointId::RIGHT_HIP_ROLL, JointId::RIGHT_HIP_PITCH, JointId::RIGHT_HIP_YAW}},
  {RIGHT_CALF, {0, JointId::RIGHT_KNEE, 0}},
  {RIGHT_FOOT, {JointId::RIGHT_ANKLE_ROLL, JointId::RIGHT_ANKLE_PITCH, 0}},
  {LEFT_THIGH, {JointId::LEFT_HIP_ROLL, JointId::LEFT_HIP_PITCH, JointId::LEFT_HIP_YAW}},
  {LEFT_CALF, {0, JointId::LEFT_KNEE, 0}},
  {LEFT_FOOT, {JointId::LEFT_ANKLE_ROLL, JointId::LEFT_ANKLE_PITCH, 0}},
};

const std::map<uint8_t, std::string> FrameId::parent_frame = {
  {ODOM, "world"},        {BASE_LINK, "odom"},        {GAZE, "torso"},
  {TORSO, "base_link"},   {RIGHT_THIGH, "base_link"}, {RIGHT_CALF, "r_thigh"},
  {RIGHT_FOOT, "r_calf"}, {LEFT_THIGH, "base_link"},  {LEFT_CALF, "l_thigh"},
  {LEFT_FOOT, "l_calf"},
};

}  // namespace tachimawari::joint
