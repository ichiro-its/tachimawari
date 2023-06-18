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

#ifndef TACHIMAWARI__JOINT__PROTOCOL_1__MX28_ADDRESS_HPP_
#define TACHIMAWARI__JOINT__PROTOCOL_1__MX28_ADDRESS_HPP_

#include <string>

namespace tachimawari::joint::protocol_1
{

enum MX28Address : uint8_t
{
  // EEPROM Area
  MODEL_NUMBER_L            = 0,
  MODEL_NUMBER_H            = 1,
  VERSION                   = 2,
  ID                        = 3,
  BAUD_RATE                 = 4,
  RETURN_DELAY_TIME         = 5,
  CW_ANGLE_LIMIT_L          = 6,
  CW_ANGLE_LIMIT_H          = 7,
  CCW_ANGLE_LIMIT_L         = 8,
  CCW_ANGLE_LIMIT_H         = 9,
  SYSTEM_DATA2              = 10,
  HIGH_LIMIT_TEMPERATURE    = 11,
  LOW_LIMIT_VOLTAGE         = 12,
  HIGH_LIMIT_VOLTAGE        = 13,
  MAX_TORQUE_L              = 14,
  MAX_TORQUE_H              = 15,
  RETURN_LEVEL              = 16,
  ALARM_LED                 = 17,
  ALARM_SHUTDOWN            = 18,
  OPERATING_MODE            = 19,
  LOW_CALIBRATION_L         = 20,
  LOW_CALIBRATION_H         = 21,
  HIGH_CALIBRATION_L        = 22,
  HIGH_CALIBRATION_H        = 23,

  // RAM Area
  TORQUE_ENABLE             = 24,
  LED                       = 25,
  D_GAIN                    = 26,
  I_GAIN                    = 27,
  P_GAIN                    = 28,
  RESERVED                  = 29,
  GOAL_POSITION_L           = 30,
  GOAL_POSITION_H           = 31,
  MOVING_SPEED_L            = 32,
  MOVING_SPEED_H            = 33,
  TORQUE_LIMIT_L            = 34,
  TORQUE_LIMIT_H            = 35,
  PRESENT_POSITION_L        = 36,
  PRESENT_POSITION_H        = 37,
  PRESENT_SPEED_L           = 38,
  PRESENT_SPEED_H           = 39,
  PRESENT_LOAD_L            = 40,
  PRESENT_LOAD_H            = 41,
  PRESENT_VOLTAGE           = 42,
  PRESENT_TEMPERATURE       = 43,
  REGISTERED_INSTRUCTION    = 44,
  PAUSE_TIME                = 45,
  MOVING                    = 46,
  LOCK                      = 47,
  PUNCH_L                   = 48,
  PUNCH_H                   = 49,
  RESERVED4                 = 50,
  RESERVED5                 = 51,
  POT_L                     = 52,
  POT_H                     = 53,
  PWM_OUT_L                 = 54,
  PWM_OUT_H                 = 55,
  P_ERROR_L                 = 56,
  P_ERROR_H                 = 57,
  I_ERROR_L                 = 58,
  I_ERROR_H                 = 59,
  D_ERROR_L                 = 60,
  D_ERROR_H                 = 61,
  P_ERROR_OUT_L             = 62,
  P_ERROR_OUT_H             = 63,
  I_ERROR_OUT_L             = 64,
  I_ERROR_OUT_H             = 65,
  D_ERROR_OUT_L             = 66,
  D_ERROR_OUT_H             = 67,
  MAXNUM_ADDRESS
};

}  // namespace tachimawari::joint::protocol_1

#endif  // TACHIMAWARI__JOINT__PROTOCOL_1__MX28_ADDRESS_HPP_
