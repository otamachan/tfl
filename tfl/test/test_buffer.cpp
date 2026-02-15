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
  // t=1000: identity, t=3000: 90 deg around Z
  buf.set_transform("b", "a", make_rotation_z(1000, 0.0));
  buf.set_transform("b", "a", make_rotation_z(3000, pi / 2.0));

  // nlerp helper: lerp + normalize
  auto nlerp_z = [&](double t) -> std::array<double, 4> {
    // q1 = (0,0,0,1), q2 = (0, 0, sin(pi/4), cos(pi/4))
    double q2z = std::sin(pi / 4.0);
    double q2w = std::cos(pi / 4.0);
    double rz = t * q2z;
    double rw = (1.0 - t) + t * q2w;
    double norm = 1.0 / std::sqrt(rz * rz + rw * rw);
    return {0.0, 0.0, rz * norm, rw * norm};
  };

  // t=0.5 (midpoint): nlerp == slerp exactly
  {
    auto result = buf.lookup_transform("a", "b", 2000);
    ASSERT_TRUE(result.has_value());
    auto expected = nlerp_z(0.5);
    EXPECT_NEAR(result->rotation[2], expected[2], 1e-9);
    EXPECT_NEAR(result->rotation[3], expected[3], 1e-9);
  }

  // t=0.25 (quarter): nlerp differs slightly from slerp
  {
    auto result = buf.lookup_transform("a", "b", 1500);
    ASSERT_TRUE(result.has_value());
    auto expected = nlerp_z(0.25);
    EXPECT_NEAR(result->rotation[0], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[1], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[2], expected[2], 1e-9);
    EXPECT_NEAR(result->rotation[3], expected[3], 1e-9);
  }

  // t=0.75 (three-quarter)
  {
    auto result = buf.lookup_transform("a", "b", 2500);
    ASSERT_TRUE(result.has_value());
    auto expected = nlerp_z(0.75);
    EXPECT_NEAR(result->rotation[0], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[1], 0.0, 1e-9);
    EXPECT_NEAR(result->rotation[2], expected[2], 1e-9);
    EXPECT_NEAR(result->rotation[3], expected[3], 1e-9);
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
