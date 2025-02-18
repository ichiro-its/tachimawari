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

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__UTILS__WORD_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__UTILS__WORD_HPP_

#include <cstdint>
#include <string>

namespace tachimawari::control::protocol_1
{

class Word
{
public:
  static uint8_t get_low_byte(int word);
  static uint8_t get_high_byte(int word);

  static uint16_t make_word(uint8_t lowbyte, uint8_t highbyte);

  static uint16_t make_color(uint8_t red, uint8_t green, uint8_t blue);
};

}  // namespace tachimawari::control::protocol_1

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__PACKET__PROTOCOL_1__UTILS__WORD_HPP_
