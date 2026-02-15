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

#include <atomic>
#include <chrono>
#include <thread>
#include <vector>

#include "tfl/transform_buffer.hpp"

using tfl::TimeNs;
using tfl::TransformBuffer;
using tfl::TransformData;

// 1 writer continuously inserting transforms,
// N readers continuously looking up transforms.
// No crashes, no hangs, no invalid quaternion norms.
TEST(TransformBuffer, StressSingleWriterMultiReader)
{
  constexpr int kNumReaders = 4;
  constexpr auto kDuration = std::chrono::seconds(2);

  TransformBuffer buf;
  // Pre-register frames so readers can start immediately
  TransformData seed;
  seed.stamp_ns = 1;
  buf.set_transform("b", "a", seed);
  buf.set_transform("c", "b", seed);
  buf.set_transform("d", "a", seed);

  std::atomic<bool> running{true};
  std::atomic<uint64_t> write_count{0};
  std::atomic<uint64_t> read_count{0};
  std::atomic<uint64_t> read_success{0};

  std::atomic<TimeNs> latest_stamp{1};

  // Writer thread: insert transforms with increasing timestamps
  std::thread writer([&]() {
    TimeNs stamp = 1000;
    while (running.load(std::memory_order_relaxed)) {
      TransformData d;
      d.stamp_ns = stamp;
      double v = static_cast<double>(stamp % 1000);
      d.translation = {v, v * 0.5, v * 0.1};
      buf.set_transform("b", "a", d);
      buf.set_transform("c", "b", d);
      buf.set_transform("d", "a", d);
      latest_stamp.store(stamp, std::memory_order_relaxed);
      stamp += 100;
      write_count.fetch_add(1, std::memory_order_relaxed);
    }
  });

  // Reader threads: lookup at a known-written timestamp
  std::vector<std::thread> readers;
  for (int i = 0; i < kNumReaders; ++i) {
    readers.emplace_back([&, i]() {
      while (running.load(std::memory_order_relaxed)) {
        TimeNs t = latest_stamp.load(std::memory_order_relaxed);
        auto r1 = buf.lookup_transform("a", "b", t);
        auto r2 = buf.lookup_transform("a", "c", t);
        auto r3 = buf.lookup_transform("a", "d", t);
        // full_path: b and d share common ancestor a
        auto r4 = buf.lookup_transform("b", "d", t);

        read_count.fetch_add(4, std::memory_order_relaxed);

        // Validate: if we got a result, quaternion norm should be ~1
        for (const auto & r : {r1, r2, r3, r4}) {
          if (r.has_value()) {
            read_success.fetch_add(1, std::memory_order_relaxed);
            const auto & q = r->rotation;
            double norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            EXPECT_NEAR(norm, 1.0, 1e-6) << "Invalid quaternion norm: " << norm;
          }
        }
      }
    });
  }

  std::this_thread::sleep_for(kDuration);
  running.store(false, std::memory_order_relaxed);

  writer.join();
  for (auto & r : readers) {
    r.join();
  }

  // Print stats
  std::cout << "writes: " << write_count.load() << ", reads: " << read_count.load()
            << ", successful: " << read_success.load() << std::endl;

  EXPECT_GT(write_count.load(), 0u);
  EXPECT_GT(read_success.load(), 0u);
}

// Stress test with clear() called periodically by the writer
TEST(TransformBuffer, StressWriterWithClear)
{
  constexpr int kNumReaders = 4;
  constexpr auto kDuration = std::chrono::seconds(2);

  TransformBuffer buf;
  TransformData seed;
  seed.stamp_ns = 1;
  buf.set_transform("b", "a", seed);

  std::atomic<bool> running{true};
  std::atomic<uint64_t> clear_count{0};

  // Writer: insert + periodic clear
  std::thread writer([&]() {
    TimeNs stamp = 1000;
    uint64_t iter = 0;
    while (running.load(std::memory_order_relaxed)) {
      TransformData d;
      d.stamp_ns = stamp;
      d.translation = {1.0, 2.0, 3.0};
      buf.set_transform("b", "a", d);
      stamp += 100;
      iter++;
      if (iter % 100 == 0) {
        buf.clear();
        clear_count.fetch_add(1, std::memory_order_relaxed);
      }
    }
  });

  // Readers: lookup continuously, expect nullopt or valid data (never crash)
  std::vector<std::thread> readers;
  for (int i = 0; i < kNumReaders; ++i) {
    readers.emplace_back([&]() {
      while (running.load(std::memory_order_relaxed)) {
        auto r = buf.lookup_transform("a", "b", 0);
        if (r.has_value()) {
          const auto & q = r->rotation;
          double norm = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
          EXPECT_NEAR(norm, 1.0, 1e-6);
        }
      }
    });
  }

  std::this_thread::sleep_for(kDuration);
  running.store(false, std::memory_order_relaxed);

  writer.join();
  for (auto & r : readers) {
    r.join();
  }

  std::cout << "clears: " << clear_count.load() << std::endl;
  EXPECT_GT(clear_count.load(), 0u);
}
