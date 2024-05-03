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
  {FrameId::BASE_LINK, "BASE_LINK"},
  {FrameId::TORSO, "TORSO"},
  {FrameId::GAZE, "GAZE"},
  {FrameId::RIGHT_THIGH, "RIGHT_THIGH"},
  {FrameId::RIGHT_CALF, "RIGHT_CALF"},
  {FrameId::RIGHT_FOOT, "RIGHT_FOOT"},
  {FrameId::LEFT_THIGH, "LEFT_THIGH"},
  {FrameId::LEFT_CALF, "LEFT_CALF"},
  {FrameId::LEFT_FOOT, "LEFT_FOOT"}};

const std::map<std::string, uint8_t> FrameId::frame_string_id = {
  {"BASE_LINK", FrameId::BASE_LINK},
  {"TORSO", FrameId::TORSO},
  {"GAZE", FrameId::GAZE},
  {"RIGHT_THIGH", FrameId::RIGHT_THIGH},
  {"RIGHT_CALF", FrameId::RIGHT_CALF},
  {"RIGHT_FOOT", FrameId::RIGHT_FOOT},
  {"LEFT_THIGH", FrameId::LEFT_THIGH},
  {"LEFT_CALF", FrameId::LEFT_CALF},
  {"LEFT_FOOT", FrameId::LEFT_FOOT}};

const std::map<uint8_t, std::vector<uint8_t>> FrameId::frame_joint_map = {
  {GAZE, {0, JointId::NECK_PITCH, JointId::NECK_YAW}},
  {RIGHT_THIGH, {JointId::RIGHT_HIP_ROLL, JointId::RIGHT_HIP_PITCH, JointId::RIGHT_HIP_YAW}},
  {RIGHT_CALF, {0, JointId::RIGHT_KNEE, 0}},
  {RIGHT_FOOT, {JointId::RIGHT_ANKLE_ROLL, JointId::RIGHT_ANKLE_PITCH, 0}},
  {LEFT_THIGH, {JointId::LEFT_HIP_ROLL, JointId::LEFT_HIP_PITCH, JointId::LEFT_HIP_YAW}},
  {LEFT_CALF, {0, JointId::LEFT_KNEE, 0}},
  {LEFT_FOOT, {JointId::LEFT_ANKLE_ROLL, JointId::LEFT_ANKLE_PITCH, 0}},
};

const std::map<uint8_t, uint8_t> FrameId::parent_frame = {
  {GAZE, TORSO},
  {RIGHT_THIGH, BASE_LINK},
  {RIGHT_CALF, RIGHT_THIGH},
  {RIGHT_FOOT, RIGHT_CALF},
  {LEFT_THIGH, BASE_LINK},
  {LEFT_CALF, LEFT_THIGH},
  {LEFT_FOOT, LEFT_CALF},
};

}  // namespace tachimawari::joint
