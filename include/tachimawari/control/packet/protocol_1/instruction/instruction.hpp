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

#ifndef TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__INSTRUCTION_HPP_
#define TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__INSTRUCTION_HPP_

#include <string>

namespace tachimawari::control::packet::protocol_1
{

enum Instruction : uint8_t
{
  PING = 1,
  READ = 2,
  WRITE = 3,
  REG_WRITE = 4,
  ACTION = 5,
  RESET = 6,
  SYNC_WRITE = 131,
  BULK_READ = 146,
};

}  // namespace tachimawari::control::packet::protocol_1

#endif  // TACHIMAWARI__CONTROL__PACKET__PROTOCOL_1__INSTRUCTION__INSTRUCTION_HPP_
