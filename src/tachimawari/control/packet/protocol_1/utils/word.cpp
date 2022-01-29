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

#include <string>

#include "tachimawari/control/packet/protocol_1/utils/word.hpp"

namespace tachimawari
{

namespace control
{

namespace packet
{
  
namespace protocol_1
{

uint8_t Word::get_low_byte(int word)
{
  uint16_t temp;
  temp = word & 0x00FF;
  return static_cast<uint8_t>(temp);
}

uint8_t Word::get_high_byte(int word)
{
  uint16_t temp;
  temp = word & 0xFF00;
  return static_cast<uint8_t>((temp >> 8));
}

int Word::make_color(int red, int green, int blue)
{
  int r = (red & 0xFF) >> 3;
  int g = (green & 0xFF) >> 3;
  int b = (blue & 0xFF) >> 3;

  return static_cast<int>(((b << 10) | (g << 5) | r));
}

}  // namespace protocol_1

}  // namespace packet

}  // namespace control

}  // namespace tachimawari
