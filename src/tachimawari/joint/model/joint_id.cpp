// Copyright (c) 2021-2023 Ichiro ITS
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

#include "tachimawari/joint/model/joint_id.hpp"

#include <map>
#include <string>
#include <vector>

namespace tachimawari::joint
{

const std::array<uint8_t, 24> JointId::list = {
  RIGHT_SHOULDER_PITCH,
  LEFT_SHOULDER_PITCH,
  RIGHT_SHOULDER_ROLL,
  LEFT_SHOULDER_ROLL,
  RIGHT_ELBOW,
  LEFT_ELBOW,
  RIGHT_HIP_YAW,
  LEFT_HIP_YAW,
  RIGHT_HIP_ROLL,
  LEFT_HIP_ROLL,
  RIGHT_HIP_PITCH,
  LEFT_HIP_PITCH,
  RIGHT_KNEE,
  LEFT_KNEE,
  RIGHT_ANKLE_PITCH,
  LEFT_ANKLE_PITCH,
  RIGHT_ANKLE_ROLL,
  LEFT_ANKLE_ROLL,
  NECK_YAW,
  NECK_PITCH,
  RIGHT_GRIPPER,
  LEFT_GRIPPER,
  RIGHT_SHOULDER_YAW,
  LEFT_SHOULDER_YAW,
};

const std::map<std::string, uint8_t> JointId::by_name = {
  {"neck_yaw", NECK_YAW},
  {"neck_pitch", NECK_PITCH},
  {"left_shoulder_pitch", LEFT_SHOULDER_PITCH},
  {"left_shoulder_roll", LEFT_SHOULDER_ROLL},
  {"left_shoulder_yaw", LEFT_SHOULDER_YAW},
  {"left_elbow", LEFT_ELBOW},
  {"left_gripper", LEFT_GRIPPER},
  {"right_shoulder_pitch", RIGHT_SHOULDER_PITCH},
  {"right_shoulder_roll", RIGHT_SHOULDER_ROLL},
  {"right_shoulder_yaw", RIGHT_SHOULDER_YAW},
  {"right_elbow", RIGHT_ELBOW},
  {"right_gripper", RIGHT_GRIPPER},
  {"left_hip_yaw", LEFT_HIP_YAW},
  {"left_hip_roll", LEFT_HIP_ROLL},
  {"left_hip_pitch", LEFT_HIP_PITCH},
  {"left_knee", LEFT_KNEE},
  {"left_ankle_roll", LEFT_ANKLE_ROLL},
  {"left_ankle_pitch", LEFT_ANKLE_PITCH},
  {"right_hip_yaw", RIGHT_HIP_YAW},
  {"right_hip_roll", RIGHT_HIP_ROLL},
  {"right_hip_pitch", RIGHT_HIP_PITCH},
  {"right_knee", RIGHT_KNEE},
  {"right_ankle_roll", RIGHT_ANKLE_ROLL},
  {"right_ankle_pitch", RIGHT_ANKLE_PITCH}};

const std::map<uint8_t, std::string> JointId::by_id = {
  {NECK_YAW, "neck_yaw"},
  {NECK_PITCH, "neck_pitch"},
  {LEFT_SHOULDER_PITCH, "left_shoulder_pitch"},
  {LEFT_SHOULDER_ROLL, "left_shoulder_roll"},
  {LEFT_SHOULDER_YAW, "left_shoulder_yaw"},
  {LEFT_ELBOW, "left_elbow"},
  {LEFT_GRIPPER, "left_gripper"},
  {RIGHT_SHOULDER_PITCH, "right_shoulder_pitch"},
  {RIGHT_SHOULDER_ROLL, "right_shoulder_roll"},
  {RIGHT_SHOULDER_YAW, "right_shoulder_yaw"},
  {RIGHT_ELBOW, "right_elbow"},
  {RIGHT_GRIPPER, "right_gripper"},
  {LEFT_HIP_YAW, "left_hip_yaw"},
  {LEFT_HIP_ROLL, "left_hip_roll"},
  {LEFT_HIP_PITCH, "left_hip_pitch"},
  {LEFT_KNEE, "left_knee"},
  {LEFT_ANKLE_ROLL, "left_ankle_roll"},
  {LEFT_ANKLE_PITCH, "left_ankle_pitch"},
  {RIGHT_HIP_YAW, "right_hip_yaw"},
  {RIGHT_HIP_ROLL, "right_hip_roll"},
  {RIGHT_HIP_PITCH, "right_hip_pitch"},
  {RIGHT_KNEE, "right_knee"},
  {RIGHT_ANKLE_ROLL, "right_ankle_roll"},
  {RIGHT_ANKLE_PITCH, "right_ankle_pitch"}};

const std::array<uint8_t, 2> JointId::head_ids = {
  NECK_YAW,
  NECK_PITCH,
};

const std::array<uint8_t, 22> JointId::body_ids = {
  RIGHT_SHOULDER_PITCH, LEFT_SHOULDER_PITCH, RIGHT_SHOULDER_ROLL, LEFT_SHOULDER_ROLL,
  RIGHT_SHOULDER_YAW,   LEFT_SHOULDER_YAW,   RIGHT_ELBOW,         LEFT_ELBOW,
  RIGHT_GRIPPER,        LEFT_GRIPPER,        RIGHT_HIP_YAW,       LEFT_HIP_YAW,
  RIGHT_HIP_ROLL,       LEFT_HIP_ROLL,       RIGHT_HIP_PITCH,     LEFT_HIP_PITCH,
  RIGHT_KNEE,           LEFT_KNEE,           RIGHT_ANKLE_PITCH,   LEFT_ANKLE_PITCH,
  RIGHT_ANKLE_ROLL,     LEFT_ANKLE_ROLL,
};

}  // namespace tachimawari::joint
