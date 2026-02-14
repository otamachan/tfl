// Copyright 2026 Tamaki Nishino
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "tfl/transform_buffer.hpp"

using tfl::TimeNs;
using tfl::TransformBuffer;
using tfl::TransformData;

namespace
{

TransformData make_identity(TimeNs stamp)
{
  TransformData d;
  d.stamp_ns = stamp;
  return d;
}

TransformData make_translation(TimeNs stamp, double x, double y, double z)
{
  TransformData d;
  d.stamp_ns = stamp;
  d.translation[0] = x;
  d.translation[1] = y;
  d.translation[2] = z;
  return d;
}

}  // namespace

TEST(TransformBuffer, AutoRegisterFrames)
{
  TransformBuffer buf;
  EXPECT_EQ(buf.frame_count(), 0u);

  buf.set_transform("child", "parent", make_identity(1000));
  EXPECT_EQ(buf.frame_count(), 2u);
}

TEST(TransformBuffer, SimpleParentChild)
{
  TransformBuffer buf;
  buf.set_transform("child", "parent", make_translation(1000, 1.0, 0.0, 0.0));

  auto result = buf.lookup_transform("parent", "child", 1000);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 1.0);
  EXPECT_DOUBLE_EQ(result->translation[1], 0.0);
  EXPECT_DOUBLE_EQ(result->translation[2], 0.0);
}

TEST(TransformBuffer, Chain)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("c", "b", make_translation(1000, 0.0, 2.0, 0.0));

  auto result = buf.lookup_transform("a", "c", 1000);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 1.0);
  EXPECT_DOUBLE_EQ(result->translation[1], 2.0);
  EXPECT_DOUBLE_EQ(result->translation[2], 0.0);
}

TEST(TransformBuffer, CanTransform)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));

  EXPECT_TRUE(buf.can_transform("a", "b", 1000));

  // "c" is not connected to "a"
  buf.set_transform("c", "d", make_identity(1000));
  EXPECT_FALSE(buf.can_transform("a", "c", 1000));
}

TEST(TransformBuffer, Interpolation)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 0.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(3000, 10.0, 0.0, 0.0));

  auto result = buf.lookup_transform("a", "b", 2000);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], 5.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 0.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 0.0, 1e-9);
}

TEST(TransformBuffer, StaticFrame)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(0, 5.0, 0.0, 0.0), true);

  EXPECT_TRUE(buf.can_transform("a", "b", 999));
  auto result = buf.lookup_transform("a", "b", 999);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 5.0);
}

TEST(TransformBuffer, IdentityTransform)
{
  TransformBuffer buf;
  buf.set_transform("a", "root", make_identity(1000));

  auto result = buf.lookup_transform("a", "a", 1000);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 0.0);
  EXPECT_DOUBLE_EQ(result->translation[1], 0.0);
  EXPECT_DOUBLE_EQ(result->translation[2], 0.0);
  EXPECT_DOUBLE_EQ(result->rotation[3], 1.0);
}

TEST(TransformBuffer, LookupFailsForUnknownFrame)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_identity(1000));

  auto result = buf.lookup_transform("a", "unknown", 1000);
  EXPECT_FALSE(result.has_value());
}

TEST(TransformBuffer, ClearResetsData)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 2.0, 3.0));
  ASSERT_TRUE(buf.lookup_transform("a", "b", 1000).has_value());

  buf.clear();

  EXPECT_FALSE(buf.lookup_transform("a", "b", 1000).has_value());
  EXPECT_FALSE(buf.can_transform("a", "b", 1000));

  // Frame registrations preserved: can insert and lookup again
  buf.set_transform("b", "a", make_translation(5000, 4.0, 5.0, 6.0));
  auto result = buf.lookup_transform("a", "b", 5000);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 4.0);
}
