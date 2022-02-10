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

namespace tachimawari::control::packet::protocol_1
{

uint8_t Word::get_low_byte(int word)
{
  return static_cast<uint8_t>(word & 0x00FF);
}

uint8_t Word::get_high_byte(int word)
{
  return static_cast<uint8_t>(((word & 0xFF00) >> 8));
}

uint16_t Word::make_word(uint8_t lowbyte, uint8_t highbyte)
{
  uint8_t word;

  word = highbyte;
  word = word << 8;
  word = word | lowbyte;

  return static_cast<uint16_t>(word);
}

uint16_t Word::make_color(uint8_t red, uint8_t green, uint8_t blue)
{
  int r = (red & 0xFF) >> 3;
  int g = (green & 0xFF) >> 3;
  int b = (blue & 0xFF) >> 3;

  return static_cast<uint16_t>(((b << 10) | (g << 5) | r));
}

}  // namespace tachimawari::control::packet::protocol_1
