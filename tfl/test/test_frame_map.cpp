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

#include "tfl/frame_map.hpp"

using tfl::FrameMap;
using tfl::INVALID_FRAME;

TEST(FrameMap, InsertAndFind)
{
  FrameMap map;
  EXPECT_TRUE(map.insert("foo", 1));
  EXPECT_EQ(map.find("foo"), 1u);
}

TEST(FrameMap, DuplicateInsert)
{
  FrameMap map;
  EXPECT_TRUE(map.insert("foo", 1));
  EXPECT_FALSE(map.insert("foo", 2));
  EXPECT_EQ(map.find("foo"), 1u);
}

TEST(FrameMap, NotFound)
{
  FrameMap map;
  EXPECT_EQ(map.find("missing"), INVALID_FRAME);
}

TEST(FrameMap, MultipleEntries)
{
  FrameMap map;
  for (uint32_t i = 1; i <= 100; ++i) {
    EXPECT_TRUE(map.insert("frame_" + std::to_string(i), i));
  }
  EXPECT_EQ(map.size(), 100u);

  for (uint32_t i = 1; i <= 100; ++i) {
    EXPECT_EQ(map.find("frame_" + std::to_string(i)), i);
  }
  EXPECT_EQ(map.find("frame_0"), INVALID_FRAME);
  EXPECT_EQ(map.find("frame_101"), INVALID_FRAME);
}

TEST(FrameMap, SizeAndCapacity)
{
  FrameMap map(64);
  EXPECT_EQ(map.capacity(), 64u);
  EXPECT_EQ(map.size(), 0u);

  map.insert("a", 1);
  map.insert("b", 2);
  EXPECT_EQ(map.size(), 2u);
}
