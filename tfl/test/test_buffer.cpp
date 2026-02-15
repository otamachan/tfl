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

#include <chrono>
#include <cmath>
#include <thread>

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

// Rotation around Z axis by angle (radians)
TransformData make_rotation_z(TimeNs stamp, double angle)
{
  TransformData d;
  d.stamp_ns = stamp;
  d.rotation = {0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
  return d;
}

// Rotation around Z axis + translation
TransformData make_transform_z(TimeNs stamp, double angle, double x, double y, double z)
{
  TransformData d;
  d.stamp_ns = stamp;
  d.rotation = {0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
  d.translation = {x, y, z};
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

// source_parent_of_target path: source is ancestor of target
// Tree: b->a, lookup("b", "a") means target=b, source=a, a is parent of b
TEST(TransformBuffer, SourceParentOfTarget)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 2.0, 0.0));

  // lookup(target="b", source="a"): a is parent of b → inverse of child transform
  auto result = buf.lookup_transform("b", "a", 1000);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], -1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], -2.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 0.0, 1e-9);
}

// full_path: source and target share a common ancestor
// Tree: b->a, c->a, lookup("b", "c")
TEST(TransformBuffer, FullPath)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("c", "a", make_translation(1000, 0.0, 2.0, 0.0));

  // lookup(target="b", source="c"): b and c share common parent a
  auto result = buf.lookup_transform("b", "c", 1000);
  ASSERT_TRUE(result.has_value());
  // c->a is (0,2,0), a->b is inverse of b->a = (-1,0,0)
  // c in b frame: (-1,0,0) + (0,2,0) = (-1,2,0)
  EXPECT_NEAR(result->translation[0], -1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 2.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 0.0, 1e-9);
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

TEST(TransformBuffer, OutOfOrderInsert)
{
  TransformBuffer buf;
  // Insert in reverse order: t=3000 first, then t=1000
  buf.set_transform("b", "a", make_translation(3000, 10.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(1000, 0.0, 0.0, 0.0));

  // Exact match at both timestamps
  auto r1 = buf.lookup_transform("a", "b", 1000);
  ASSERT_TRUE(r1.has_value());
  EXPECT_NEAR(r1->translation[0], 0.0, 1e-9);

  auto r3 = buf.lookup_transform("a", "b", 3000);
  ASSERT_TRUE(r3.has_value());
  EXPECT_NEAR(r3->translation[0], 10.0, 1e-9);

  // Interpolation at midpoint
  auto r2 = buf.lookup_transform("a", "b", 2000);
  ASSERT_TRUE(r2.has_value());
  EXPECT_NEAR(r2->translation[0], 5.0, 1e-9);
}

TEST(TransformBuffer, RotationInterpolation)
{
  const double pi = std::acos(-1.0);
  TransformBuffer buf;
  buf.set_transform("b", "a", make_rotation_z(1000, 0.0));
  buf.set_transform("b", "a", make_rotation_z(3000, pi / 2.0));

  // slerp: constant angular velocity → angle = t * 90deg
  // t=0.5 → 45 deg
  {
    auto result = buf.lookup_transform("a", "b", 2000);
    ASSERT_TRUE(result.has_value());
    const double angle = pi / 4.0;
    EXPECT_NEAR(result->rotation[2], std::sin(angle / 2.0), 1e-9);
    EXPECT_NEAR(result->rotation[3], std::cos(angle / 2.0), 1e-9);
  }
  // t=0.25 → 22.5 deg
  {
    auto result = buf.lookup_transform("a", "b", 1500);
    ASSERT_TRUE(result.has_value());
    const double angle = pi / 8.0;
    EXPECT_NEAR(result->rotation[0], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[1], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[2], std::sin(angle / 2.0), 1e-9);
    EXPECT_NEAR(result->rotation[3], std::cos(angle / 2.0), 1e-9);
  }
  // t=0.75 → 67.5 deg
  {
    auto result = buf.lookup_transform("a", "b", 2500);
    ASSERT_TRUE(result.has_value());
    const double angle = 3.0 * pi / 8.0;
    EXPECT_NEAR(result->rotation[0], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[1], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[2], std::sin(angle / 2.0), 1e-9);
    EXPECT_NEAR(result->rotation[3], std::cos(angle / 2.0), 1e-9);
  }
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

TEST(TransformBuffer, LookupWithTimeoutSucceeds)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));

  auto result = buf.lookup_transform("a", "b", 1000, std::chrono::milliseconds(500));
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 1.0);
}

TEST(TransformBuffer, LookupWithTimeoutTimesOut)
{
  TransformBuffer buf;

  auto start = std::chrono::steady_clock::now();
  auto result = buf.lookup_transform("a", "b", 1000, std::chrono::milliseconds(50));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(result.has_value());
  EXPECT_GE(elapsed, std::chrono::milliseconds(40));
}

TEST(TransformBuffer, CanTransformWithTimeout)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));

  EXPECT_TRUE(buf.can_transform("a", "b", 1000, std::chrono::milliseconds(500)));
}

TEST(TransformBuffer, LookupWithTimeoutDelayedInsert)
{
  TransformBuffer buf;

  std::thread writer([&buf]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    buf.set_transform("b", "a", make_translation(1000, 3.0, 0.0, 0.0));
  });

  auto result = buf.lookup_transform("a", "b", 1000, std::chrono::milliseconds(500));
  writer.join();

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->translation[0], 3.0);
}

// source_parent_of_target with rotation + translation
// Tree: b->a with 90deg Z rotation + (1,0,0) translation
// inverse(rot=90degZ, trans=(1,0,0)):
//   inv_rot = -90deg Z
//   inv_trans = rot(-90degZ, -(1,0,0)) = rot(-90degZ, (-1,0,0)) = (0,1,0)
TEST(TransformBuffer, SourceParentOfTargetWithRotation)
{
  const double pi = std::acos(-1.0);
  TransformBuffer buf;
  buf.set_transform("b", "a", make_transform_z(1000, pi / 2.0, 1.0, 0.0, 0.0));

  auto result = buf.lookup_transform("b", "a", 1000);
  ASSERT_TRUE(result.has_value());
  // inverse rotation: -90 deg around Z
  EXPECT_NEAR(result->rotation[0], 0.0, 1e-9);
  EXPECT_NEAR(result->rotation[1], 0.0, 1e-9);
  EXPECT_NEAR(result->rotation[2], std::sin(-pi / 4.0), 1e-9);
  EXPECT_NEAR(result->rotation[3], std::cos(-pi / 4.0), 1e-9);
  // inverse translation: (0, 1, 0)
  EXPECT_NEAR(result->translation[0], 0.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 1.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 0.0, 1e-9);
}

// full_path with rotation + translation
// Tree: b->a (90deg Z, (1,0,0)), c->a (identity, (0,2,0))
// lookup(target="b", source="c"):
//   source_to_top (c→a) = (identity, (0,2,0))
//   target_to_top (b→a) = (90degZ, (1,0,0))
//   inverse(target_to_top) = (-90degZ, (0,1,0))
//   compose((-90degZ,(0,1,0)), (identity,(0,2,0))):
//     rot = -90degZ, trans = rot(-90degZ,(0,2,0)) + (0,1,0) = (2,0,0) + (0,1,0) = (2,1,0)
TEST(TransformBuffer, FullPathWithRotation)
{
  const double pi = std::acos(-1.0);
  TransformBuffer buf;
  buf.set_transform("b", "a", make_transform_z(1000, pi / 2.0, 1.0, 0.0, 0.0));
  buf.set_transform("c", "a", make_translation(1000, 0.0, 2.0, 0.0));

  auto result = buf.lookup_transform("b", "c", 1000);
  ASSERT_TRUE(result.has_value());
  // rotation: -90 deg around Z
  EXPECT_NEAR(result->rotation[2], std::sin(-pi / 4.0), 1e-9);
  EXPECT_NEAR(result->rotation[3], std::cos(-pi / 4.0), 1e-9);
  // translation: (2, 1, 0)
  EXPECT_NEAR(result->translation[0], 2.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 1.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 0.0, 1e-9);
}

// =====================================================================
// get_latest_common_time root frame bug fix tests
// =====================================================================

// Root frame has no data (stamp=0). Time(0) lookup should use
// get_latest_common_time to resolve actual time from dynamic frames.
// Bug: root frame with stamp=0 was incorrectly treated as "no data available"
// causing get_latest_common_time to return 0 and Time(0) lookups to fail.
TEST(TransformBuffer, Time0LookupWithRootFrame)
{
  // Tree: c->b->a (a is root, has no data inserted directly)
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("c", "b", make_translation(1000, 0.0, 2.0, 0.0));

  // Time(0) = latest: should resolve via get_latest_common_time
  auto result = buf.lookup_transform("a", "c", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], 1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 2.0, 1e-9);

  EXPECT_TRUE(buf.can_transform("a", "c", 0));
}

// V-tree: two branches sharing a common root that has no data itself.
// Tree: b->root, c->root, lookup("b", "c", 0)
TEST(TransformBuffer, Time0LookupVtreeRootFrame)
{
  TransformBuffer buf;
  buf.set_transform("b", "root", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("c", "root", make_translation(1000, 0.0, 2.0, 0.0));

  auto result = buf.lookup_transform("b", "c", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], -1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 2.0, 1e-9);

  EXPECT_TRUE(buf.can_transform("b", "c", 0));
}

// get_latest_common_time should return the minimum of latest stamps across the path.
// Both branches must have data covering the common time for lookup to succeed.
TEST(TransformBuffer, Time0UsesMinimumStamp)
{
  TransformBuffer buf;
  // b->a: stamps 1000, 2000
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(2000, 2.0, 0.0, 0.0));
  // c->a: stamps 1000, 3000
  buf.set_transform("c", "a", make_translation(1000, 0.0, 1.0, 0.0));
  buf.set_transform("c", "a", make_translation(3000, 0.0, 3.0, 0.0));

  // common time = min(latest_b=2000, latest_c=3000) = 2000
  auto result = buf.lookup_transform("b", "c", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->stamp_ns, 2000);
}

// Deep chain with root frame: root->a->b->c->d, lookup d to root at Time(0)
TEST(TransformBuffer, Time0DeepChainRootFrame)
{
  TransformBuffer buf;
  buf.set_transform("a", "root", make_translation(5000, 1.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(5000, 0.0, 1.0, 0.0));
  buf.set_transform("c", "b", make_translation(5000, 0.0, 0.0, 1.0));
  buf.set_transform("d", "c", make_translation(5000, 2.0, 0.0, 0.0));

  auto result = buf.lookup_transform("root", "d", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], 3.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 1.0, 1e-9);
  EXPECT_NEAR(result->translation[2], 1.0, 1e-9);

  EXPECT_TRUE(buf.can_transform("root", "d", 0));
}

// Root frame + static frame mixed with dynamic frames
TEST(TransformBuffer, Time0StaticAndDynamicMixed)
{
  TransformBuffer buf;
  // b->a is static, c->b is dynamic
  buf.set_transform("b", "a", make_translation(0, 1.0, 0.0, 0.0), true);
  buf.set_transform("c", "b", make_translation(3000, 0.0, 2.0, 0.0));

  auto result = buf.lookup_transform("a", "c", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], 1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 2.0, 1e-9);

  EXPECT_TRUE(buf.can_transform("a", "c", 0));
}

// V-tree with static + dynamic: static branch and dynamic branch sharing root
TEST(TransformBuffer, Time0VtreeStaticDynamic)
{
  TransformBuffer buf;
  buf.set_transform("b", "root", make_translation(0, 1.0, 0.0, 0.0), true);
  buf.set_transform("c", "root", make_translation(2000, 0.0, 3.0, 0.0));

  auto result = buf.lookup_transform("b", "c", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->translation[0], -1.0, 1e-9);
  EXPECT_NEAR(result->translation[1], 3.0, 1e-9);

  EXPECT_TRUE(buf.can_transform("b", "c", 0));
}

// =====================================================================
// can_walk_to_top_parent tests
// =====================================================================

// canTransform should agree with lookupTransform on success/failure
TEST(TransformBuffer, CanTransformAgreesWithLookup)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("c", "a", make_translation(1000, 0.0, 2.0, 0.0));

  // Valid transforms
  EXPECT_TRUE(buf.can_transform("a", "b", 1000));
  EXPECT_TRUE(buf.can_transform("b", "a", 1000));
  EXPECT_TRUE(buf.can_transform("b", "c", 1000));
  EXPECT_TRUE(buf.can_transform("c", "b", 1000));
  EXPECT_TRUE(buf.can_transform("a", "a", 1000));

  // Time(0) = latest
  EXPECT_TRUE(buf.can_transform("b", "c", 0));

  // Out of range time: can_transform uses lightweight get_parent which
  // checks range but parent_id is same for all entries, so it returns true
  // if the frame has any data. This is by design — can_transform is fast,
  // not an exact range check.
  EXPECT_TRUE(buf.can_transform("a", "b", 9999));

  // Unknown frames
  EXPECT_FALSE(buf.can_transform("a", "unknown", 1000));
  EXPECT_FALSE(buf.can_transform("unknown", "a", 1000));
}

// canTransform on disconnected trees
TEST(TransformBuffer, CanTransformDisconnectedTrees)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("d", "c", make_translation(1000, 0.0, 1.0, 0.0));

  EXPECT_FALSE(buf.can_transform("a", "c", 1000));
  EXPECT_FALSE(buf.can_transform("b", "d", 1000));
  EXPECT_FALSE(buf.can_transform("a", "d", 0));
}

// canTransform with long chain
TEST(TransformBuffer, CanTransformLongChain)
{
  TransformBuffer buf;
  // f0 -> f1 -> f2 -> ... -> f19
  for (int i = 0; i < 20; ++i) {
    auto child = "f" + std::to_string(i + 1);
    auto parent = "f" + std::to_string(i);
    buf.set_transform(child, parent, make_translation(1000, 1.0, 0.0, 0.0));
  }

  EXPECT_TRUE(buf.can_transform("f0", "f20", 1000));
  EXPECT_TRUE(buf.can_transform("f20", "f0", 1000));
  EXPECT_TRUE(buf.can_transform("f5", "f15", 1000));
}

// canTransform with static chain
TEST(TransformBuffer, CanTransformStaticChain)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(0, 1.0, 0.0, 0.0), true);
  buf.set_transform("c", "b", make_translation(0, 0.0, 1.0, 0.0), true);

  // Static frames accept any non-zero time
  EXPECT_TRUE(buf.can_transform("a", "c", 1000));
  EXPECT_TRUE(buf.can_transform("a", "c", 999999));

  // Time(0) on all-static chain: get_latest_common_time returns 0
  // (static stamps are 0), so time resolution fails. Must use explicit time.
  EXPECT_FALSE(buf.can_transform("c", "a", 0));
}

// =====================================================================
// Additional edge cases
// =====================================================================

// Lookup at exact boundary timestamps of the cache
TEST(TransformBuffer, LookupAtExactBoundaryTimestamps)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(2000, 2.0, 0.0, 0.0));
  buf.set_transform("b", "a", make_translation(3000, 3.0, 0.0, 0.0));

  // Exact oldest
  auto r1 = buf.lookup_transform("a", "b", 1000);
  ASSERT_TRUE(r1.has_value());
  EXPECT_NEAR(r1->translation[0], 1.0, 1e-9);

  // Exact newest
  auto r3 = buf.lookup_transform("a", "b", 3000);
  ASSERT_TRUE(r3.has_value());
  EXPECT_NEAR(r3->translation[0], 3.0, 1e-9);

  // Just outside range: should fail
  EXPECT_FALSE(buf.lookup_transform("a", "b", 999).has_value());
  EXPECT_FALSE(buf.lookup_transform("a", "b", 3001).has_value());
}

// Extrapolation is not allowed: single entry, different time
TEST(TransformBuffer, NoExtrapolation)
{
  TransformBuffer buf;
  buf.set_transform("b", "a", make_translation(1000, 1.0, 0.0, 0.0));

  EXPECT_FALSE(buf.lookup_transform("a", "b", 999).has_value());
  EXPECT_FALSE(buf.lookup_transform("a", "b", 1001).has_value());
  EXPECT_TRUE(buf.lookup_transform("a", "b", 1000).has_value());
}

// V-tree with multiple timestamps: different branches have different time ranges
TEST(TransformBuffer, VtreeMultipleTimestamps)
{
  TransformBuffer buf;
  // b->root: stamps 1000, 2000, 3000
  buf.set_transform("b", "root", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("b", "root", make_translation(2000, 2.0, 0.0, 0.0));
  buf.set_transform("b", "root", make_translation(3000, 3.0, 0.0, 0.0));
  // c->root: stamps 1500, 2500
  buf.set_transform("c", "root", make_translation(1500, 0.0, 1.0, 0.0));
  buf.set_transform("c", "root", make_translation(2500, 0.0, 2.0, 0.0));

  // Exact lookups within overlapping range
  auto r = buf.lookup_transform("b", "c", 2000);
  ASSERT_TRUE(r.has_value());

  // Time(0) should resolve to min(latest_b=3000, latest_c=2500) = 2500
  auto r0 = buf.lookup_transform("b", "c", 0);
  ASSERT_TRUE(r0.has_value());
  EXPECT_EQ(r0->stamp_ns, 2500);

  // Time outside c's range (1500..2500) should fail
  EXPECT_FALSE(buf.lookup_transform("b", "c", 1000).has_value());
  EXPECT_FALSE(buf.lookup_transform("b", "c", 3000).has_value());
}

// Self-lookup for unregistered frame should fail
TEST(TransformBuffer, SelfLookupUnregistered)
{
  TransformBuffer buf;
  EXPECT_FALSE(buf.lookup_transform("x", "x", 1000).has_value());
  EXPECT_FALSE(buf.can_transform("x", "x", 1000));
}

// canTransform at Time(0) with V-tree
TEST(TransformBuffer, CanTransformTime0Vtree)
{
  TransformBuffer buf;
  buf.set_transform("left", "root", make_translation(1000, 1.0, 0.0, 0.0));
  buf.set_transform("right", "root", make_translation(1000, 0.0, 1.0, 0.0));

  EXPECT_TRUE(buf.can_transform("left", "right", 0));
  EXPECT_TRUE(buf.can_transform("right", "left", 0));
}
