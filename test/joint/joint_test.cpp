// Copyright (c) 2021 ICHIRO ITS
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

#include <vector>

#include "gtest/gtest.h"
#include "tachimawari/control/control.hpp"
#include "keisan/angle/angle.hpp"

namespace tmw = tachimawari::joint;
namespace ksn = keisan;

TEST(JointTest, MakeDegree)
{
  EXPECT_FLOAT_EQ(ksn::make_degree(100.0f).degree(), 100.0f);
  EXPECT_DOUBLE_EQ(ksn::make_degree(-75.0).degree(), -75.0);
  EXPECT_DOUBLE_EQ(ksn::make_degree(25.0l).degree(), 25.0l);
}

TEST(JointTest, AngleToValue)
{
  EXPECT_EQ(tmw::Joint::angle_to_value(ksn::Angle<double>(ksn::make_degree(100.0))), 1137);
  EXPECT_EQ(tmw::Joint::angle_to_value(ksn::Angle<double>(ksn::make_degree(-75.0))), -853);
  EXPECT_EQ(tmw::Joint::angle_to_value(ksn::Angle<double>(ksn::make_degree(25.0))), 284);
}

TEST(JointTest, ValueToAngle)
{
  EXPECT_EQ(tmw::Joint::value_to_angle(4096), ksn::Angle<double>(ksn::make_degree(360.0)));
  EXPECT_EQ(tmw::Joint::value_to_angle(-1024), ksn::Angle<double>(ksn::make_degree(-90.0)));
  EXPECT_EQ(tmw::Joint::value_to_angle(2048), ksn::Angle<double>(ksn::make_degree(180.0)));
}

TEST(JointTest, GetPosition)
{
  tmw::Joint joint(1, 0.0);

  joint.set_position_value(2018);
  EXPECT_EQ(joint.get_position_value(), 2018);

  joint.set_position(0.0);
  EXPECT_EQ(joint.get_position(), 0.0);
}

TEST(JointTest, PIDTest)
{
  tmw::Joint joint(1, 0.0);
  std::vector<float> vector{0.0, 1.0, 2.0};

  joint.set_pid_gain(0.0, 1.0, 2.0);
  EXPECT_EQ(joint.get_pid_gain(), vector);
}
