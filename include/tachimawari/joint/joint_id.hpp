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

#ifndef TACHIMAWARI__JOINT__JOINT_ID_HPP_
#define TACHIMAWARI__JOINT__JOINT_ID_HPP_

namespace tachimawari
{

enum JointId : uint8_t
{
  // head motors
  NECK_YAW = 19,
  NECK_PITCH = 20,

  // left arm motors
  LEFT_SHOULDER_PITCH = 2,
  LEFT_SHOULDER_ROLL = 4,
  LEFT_ELBOW = 6,

  // right arm motors
  RIGHT_SHOULDER_PITCH = 1,
  RIGHT_SHOULDER_ROLL = 3,
  RIGHT_ELBOW = 5,

  // left leg motors
  LEFT_HIP_YAW = 8,
  LEFT_HIP_ROLL = 10,
  LEFT_HIP_PITCH = 12,
  LEFT_KNEE = 14,
  LEFT_ANKLE_ROLL = 16,
  LEFT_ANKLE_PITCH = 18,

  // right leg motors
  RIGHT_HIP_YAW = 7,
  RIGHT_HIP_ROLL = 9,
  RIGHT_HIP_PITCH = 11,
  RIGHT_KNEE = 13,
  RIGHT_ANKLE_ROLL = 15,
  RIGHT_ANKLE_PITCH = 17,
};

}  // namespace tachimawari

#endif  // TACHIMAWARI__JOINT__JOINT_ID_HPP_
