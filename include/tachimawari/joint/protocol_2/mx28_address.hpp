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

#ifndef TACHIMAWARI__JOINT__PROTOCOL_2__MX28_ADDRESS_HPP
#define TACHIMAWARI__JOINT__PROTOCOL_2__MX28_ADDRESS_HPP

#include <string>

namespace tachimawari
{

namespace joint
{

namespace protocol_2
{

enum MXP2Address : uint8_t
{
  // EEPROM Area
  MODEL_NUMBER              = 0,
  MODEL_INFORMATION         = 2,
  FIRMWARE_VERSION          = 6,
  ID                        = 7,
  BAUD_RATE                 = 8,
  RETURN_DELAY_TIME         = 9,
  DRIVE_MODE                = 10,
  OPERATING_MODE            = 11,
  SECONDARY_ID              = 12,
  PROTOCOL_TYPE             = 13,
  HORNING_OFFSET            = 20,
  MOVING_THRESHOLD          = 24,
  TEMPERATURE_LIMIT         = 31,
  MAX_VOLTAGE_LIMIT         = 32,
  MIN_VOLTAGE_LIMIT         = 34,
  PWM_LIMIT                 = 36,
  CURRENT_LIMIT             = 38,
  ACCELERATION_LIMIT        = 40,
  VELOCITY_LIMIT            = 44,
  MAX_POSITION_LIMIT        = 48,
  MIN_POSITION_LIMIT        = 52,
  SHUTDOWN                  = 63,

  // RAM Area
  TORQUE_ENABLE             = 64,
  LED                       = 65,
  STATUS_RETURN_VALUE       = 68,
  REGISTERED_INSTRUCTION    = 69,
  HARDWARE_ERROR_STATUS     = 70,
  VELOCITY_I_GAIN           = 76,
  VELOCITY_P_GAIN           = 78,
  POSITION_D_GAIN           = 80,
  POSITION_I_GAIN           = 82,
  POSITION_P_GAIN           = 84,
  FEEDFORWARD_2ND_GAIN      = 88,
  FEEDFORWARD_1ST_GAIN      = 90,
  BUS_WATCHDOG              = 98,
  GOAL_PWM                  = 100,
  GOAL_CURRENT              = 102,
  GOAL_VELOCITY             = 104,
  PROFILE_ACCELERATION      = 108,
  PROFILE_VELOCITY          = 112,
  GOAL_POSITION             = 116,
  REALTIME_TICK             = 120,
  MOVING                    = 122,
  MOVING_STATUS             = 123,
  PRESENT_PWM               = 124,
  PRESENT_CURRENT           = 126,
  PRESENT_VELOCITY          = 128,
  PRESENT_POSITION          = 132,
  VELOCITY_TRAJECTORY       = 136,
  POSITION_TRAJECTORY       = 140,
  PRESENT_INPUT_VOLTAGE     = 144,
  PRESENT_TEMPERATURE       = 146
};

}  // namespace protocol_2

}  // namespace joint

}  // namespace tachimawari

#endif  // TACHIMAWARI__JOINT__PROTOCOL_2__MX28_ADDRESS_HPP
