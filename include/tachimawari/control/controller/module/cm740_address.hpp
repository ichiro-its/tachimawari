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

#ifndef TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_ADDRESS_HPP_
#define TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_ADDRESS_HPP_

#include <string>

namespace tachimawari::control
{

enum CM740Address : uint8_t
{
  MODEL_NUMBER_L    = 0,
  MODEL_NUMBER_H    = 1,
  VERSION           = 2,
  ID                = 3,
  BAUD_RATE         = 4,
  RETURN_DELAY_TIME = 5,
  RETURN_LEVEL      = 16,
  DXL_POWER         = 24,
  LED_PANNEL        = 25,
  LED_HEAD_L        = 26,
  LED_HEAD_H        = 27,
  LED_EYE_L         = 28,
  LED_EYE_H         = 29,
  BUTTON            = 30,
  GYRO_Z_L          = 38,
  GYRO_Z_H          = 39,
  GYRO_Y_L          = 40,
  GYRO_Y_H          = 41,
  GYRO_X_L          = 42,
  GYRO_X_H          = 43,
  ACCEL_X_L         = 44,
  ACCEL_X_H         = 45,
  ACCEL_Y_L         = 46,
  ACCEL_Y_H         = 47,
  ACCEL_Z_L         = 48,
  ACCEL_Z_H         = 49,
  VOLTAGE           = 50,
  LEFT_MIC_L        = 51,
  LEFT_MIC_H        = 52,
  ADC2_L            = 53,
  ADC2_H            = 54,
  ADC3_L            = 55,
  ADC3_H            = 56,
  ADC4_L            = 57,
  ADC4_H            = 58,
  ADC5_L            = 59,
  ADC5_H            = 60,
  ADC6_L            = 61,
  ADC6_H            = 62,
  ADC7_L            = 63,
  ADC7_H            = 64,
  ADC8_L            = 65,
  ADC8_H            = 66,
  RIGHT_MIC_L       = 67,
  RIGHT_MIC_H       = 68,
  ADC10_L           = 69,
  ADC10_H           = 70,
  ADC11_L           = 71,
  ADC11_H           = 72,
  ADC12_L           = 73,
  ADC12_H           = 74,
  ADC13_L           = 75,
  ADC13_H           = 76,
  ADC14_L           = 77,
  ADC14_H           = 78,
  ADC15_L           = 79,
  ADC15_H           = 80,
  MAXNUM_ADDRESS
};

}  // namespace tachimawari::control

#endif  // TACHIMAWARI__CONTROL__CONTROLLER__MODULE__CM740_ADDRESS_HPP_
