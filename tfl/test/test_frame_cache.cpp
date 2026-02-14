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

#include "tfl/frame_transform_buffer.hpp"

using tfl::FrameID;
using tfl::FrameTransformBuffer;
using tfl::TimeNs;
using tfl::TransformData;

namespace
{

TransformData make_data(TimeNs stamp, FrameID parent)
{
  TransformData d;
  d.stamp_ns = stamp;
  d.parent_id = parent;
  d.translation[0] = static_cast<double>(stamp);
  return d;
}

constexpr TimeNs kDuration = 10'000'000'000LL;

}  // namespace

TEST(FrameTransformBuffer, EmptyGetDataReturnsZero)
{
  FrameTransformBuffer cache;
  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(100, d1, d2), 0);
}

TEST(FrameTransformBuffer, InsertOneGetData)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(1000, d1, d2), 1);
  EXPECT_EQ(d1.stamp_ns, 1000);
  EXPECT_EQ(d1.parent_id, 1u);
}

TEST(FrameTransformBuffer, LatestTimeZero)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);
  cache.insert(make_data(2000, 1), kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(0, d1, d2), 1);
  EXPECT_EQ(d1.stamp_ns, 2000);
}

TEST(FrameTransformBuffer, ExactMatch)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);
  cache.insert(make_data(2000, 1), kDuration);
  cache.insert(make_data(3000, 1), kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(2000, d1, d2), 1);
  EXPECT_EQ(d1.stamp_ns, 2000);
}

TEST(FrameTransformBuffer, InterpolationBracket)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);
  cache.insert(make_data(3000, 1), kDuration);

  TransformData d1, d2;
  uint8_t n = cache.get_data(2000, d1, d2);
  EXPECT_EQ(n, 2);
  EXPECT_EQ(d1.stamp_ns, 1000);  // older
  EXPECT_EQ(d2.stamp_ns, 3000);  // newer
}

TEST(FrameTransformBuffer, OutOfRangeReturnsZero)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);
  cache.insert(make_data(2000, 1), kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(500, d1, d2), 0);
  EXPECT_EQ(cache.get_data(3000, d1, d2), 0);
}

TEST(FrameTransformBuffer, Pruning)
{
  constexpr TimeNs short_duration = 1000;
  FrameTransformBuffer cache(2000, false);

  cache.insert(make_data(100, 1), short_duration);
  cache.insert(make_data(200, 1), short_duration);
  cache.insert(make_data(2000, 1), short_duration);

  TransformData d1, d2;
  // stamp=100 is outside [2000 - 1000, 2000], should be pruned
  EXPECT_EQ(cache.get_data(100, d1, d2), 0);
  // stamp=200 is also outside
  EXPECT_EQ(cache.get_data(200, d1, d2), 0);
  // stamp=2000 should still be there
  EXPECT_EQ(cache.get_data(2000, d1, d2), 1);
}

TEST(FrameTransformBuffer, StaticFrame)
{
  FrameTransformBuffer cache(1, true);
  TransformData d;
  d.parent_id = 5;
  d.translation[0] = 1.0;
  cache.insert(d, kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(999, d1, d2), 1);
  EXPECT_EQ(d1.stamp_ns, 999);  // stamp overwritten to requested time
  EXPECT_EQ(d1.parent_id, 5u);
  EXPECT_DOUBLE_EQ(d1.translation[0], 1.0);
}

TEST(FrameTransformBuffer, GetLatestStamp)
{
  FrameTransformBuffer cache;
  EXPECT_EQ(cache.get_latest_stamp(), 0);

  cache.insert(make_data(1000, 1), kDuration);
  EXPECT_EQ(cache.get_latest_stamp(), 1000);

  cache.insert(make_data(2000, 1), kDuration);
  EXPECT_EQ(cache.get_latest_stamp(), 2000);
}

TEST(FrameTransformBuffer, GetParent)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 42), kDuration);
  EXPECT_EQ(cache.get_parent(1000), 42u);
  EXPECT_EQ(cache.get_parent(0), 42u);
}

TEST(FrameTransformBuffer, IsStatic)
{
  FrameTransformBuffer dynamic_cache;
  EXPECT_FALSE(dynamic_cache.is_static());

  FrameTransformBuffer static_cache(1, true);
  EXPECT_TRUE(static_cache.is_static());
}

TEST(FrameTransformBuffer, Clear)
{
  FrameTransformBuffer cache;
  cache.insert(make_data(1000, 1), kDuration);
  cache.insert(make_data(2000, 1), kDuration);

  TransformData d1, d2;
  EXPECT_EQ(cache.get_data(1000, d1, d2), 1);
  EXPECT_EQ(cache.get_latest_stamp(), 2000);

  cache.clear();

  EXPECT_EQ(cache.get_data(1000, d1, d2), 0);
  EXPECT_EQ(cache.get_data(0, d1, d2), 0);
  EXPECT_EQ(cache.get_latest_stamp(), 0);

  // Can insert again after clear
  cache.insert(make_data(5000, 2), kDuration);
  EXPECT_EQ(cache.get_data(5000, d1, d2), 1);
  EXPECT_EQ(d1.stamp_ns, 5000);
  EXPECT_EQ(d1.parent_id, 2u);
}
