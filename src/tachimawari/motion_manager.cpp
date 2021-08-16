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

#include <tachimawari/motion_manager.hpp>
#include <tachimawari/joint.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace tachimawari
{

MotionManager::MotionManager()
{
  std::vector<std::string> ids = {
    // right leg motors
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",

    // left leg motors
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",

    // right arm motors
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow",

    // left arm motors
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow",
  };

  std::vector<tachimawari::Joint> joints;
  for (auto id : ids) {
    tachimawari::Joint joint(id);
    joints.push_back(joint);
  }
}

}  // namespace tachimawari
