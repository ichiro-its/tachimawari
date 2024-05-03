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

#ifndef TACHIMAWARI__JOINT_TF2__FRAME_ID_HPP_
#define TACHIMAWARI__JOINT_TF2__FRAME_ID_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

namespace tachimawari::joint
{

class FrameId
{
public:
  enum : uint8_t {
    BASE_LINK = 0,
    TORSO = 1,
    GAZE = 2,
    RIGHT_THIGH = 3,
    RIGHT_CALF = 4,
    RIGHT_FOOT = 5,
    LEFT_THIGH = 6,
    LEFT_CALF = 7,
    LEFT_FOOT = 8,
  };

  static const std::array<uint8_t, 9> frame_ids;
  static const std::map<uint8_t, std::string> frame_id_string;
  static const std::map<std::string, uint8_t> frame_string_id;
  static const std::map<uint8_t, std::vector<uint8_t>> frame_joint_map;
  static const std::map<uint8_t, uint8_t> parent_frame;
};

}  // namespace tachimawari::joint

#endif  // TACHIMAWARI__JOINT_TF2__FRAME_ID_HPP_