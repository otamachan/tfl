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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "tfl/transform_buffer.hpp"

using tfl::TimeNs;
using tfl::TransformBuffer;
using tfl::TransformData;

using Clock = std::chrono::high_resolution_clock;

// Build a chain: frame_0 -> frame_1 -> ... -> frame_{depth}
void buildChain(TransformBuffer & buf, int depth, TimeNs stamp_ns, bool is_static)
{
  for (int i = 0; i < depth; ++i) {
    TransformData d;
    d.stamp_ns = stamp_ns;
    d.translation = {1.0, 0.0, 0.0};
    buf.set_transform("frame_" + std::to_string(i + 1), "frame_" + std::to_string(i), d, is_static);
  }
}

// Build a tree: root -> branch_b_0 -> branch_b_1 -> ...
void buildTree(
  TransformBuffer & buf, int num_branches, int branch_depth, TimeNs stamp_ns, bool is_static)
{
  for (int b = 0; b < num_branches; ++b) {
    std::string parent = "root";
    for (int d = 0; d < branch_depth; ++d) {
      std::string child = "branch_" + std::to_string(b) + "_" + std::to_string(d);
      double angle = 0.01 * (b + 1);
      TransformData td;
      td.stamp_ns = stamp_ns;
      td.translation = {0.1 * (d + 1), 0.0, 0.0};
      td.rotation = {0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
      buf.set_transform(child, parent, td, is_static);
      parent = child;
    }
  }
}

struct BenchResult
{
  std::string name;
  int iterations;
  double total_us;
  double per_call_ns;
};

void printResults(const std::vector<BenchResult> & results)
{
  std::printf("\n========== Benchmark Results ==========\n");
  std::printf("%-45s %10s %12s %12s\n", "Test", "Iters", "Total(us)", "Per-call(ns)");
  std::printf("%-45s %10s %12s %12s\n", "----", "-----", "---------", "------------");
  for (auto & r : results) {
    std::printf(
      "%-45s %10d %12.1f %12.1f\n", r.name.c_str(), r.iterations, r.total_us, r.per_call_ns);
  }
  std::printf("=======================================\n");
}

int main()
{
  std::vector<BenchResult> results;
  constexpr TimeNs kOneSecNs = 1000000000LL;

  // --- Bench 1: setTransform throughput ---
  {
    const int N = 100000;
    TransformBuffer buf;
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      TransformData d;
      d.stamp_ns = static_cast<TimeNs>(i) * 10000000LL;
      d.translation = {static_cast<double>(i) * 0.001, 0.0, 0.0};
      buf.set_transform("child", "parent", d);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"setTransform (100k calls)", N, us, us * 1000.0 / N});
  }

  // --- Bench 2: lookupTransform - same frame (identity) ---
  {
    const int N = 1000000;
    TransformBuffer buf;
    buildChain(buf, 1, kOneSecNs, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform("frame_0", "frame_0", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform identity (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 3: lookupTransform - chain depth 5 ---
  {
    const int N = 500000;
    TransformBuffer buf;
    buildChain(buf, 5, kOneSecNs, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform("frame_0", "frame_5", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=5 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 4: lookupTransform - chain depth 10 ---
  {
    const int N = 500000;
    TransformBuffer buf;
    buildChain(buf, 10, kOneSecNs, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform("frame_0", "frame_10", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=10 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 5: lookupTransform - cross branch (tree) ---
  {
    const int N = 500000;
    TransformBuffer buf;
    buildTree(buf, 4, 5, kOneSecNs, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform("branch_0_4", "branch_3_4", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform cross-branch (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 6: lookupTransform with interpolation ---
  {
    const int N = 500000;
    TransformBuffer buf;
    for (int t = 0; t < 100; ++t) {
      TransformData d;
      d.stamp_ns = static_cast<TimeNs>(t) * 100000000LL;
      d.translation = {static_cast<double>(t) * 0.01, 0.0, 0.0};
      double angle = static_cast<double>(t) * 0.01;
      d.rotation = {0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0)};
      buf.set_transform("child", "parent", d);
    }

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      TimeNs query_ns = 50000000LL + static_cast<TimeNs>(i % 98) * 100000000LL;
      buf.lookup_transform("parent", "child", query_ns);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform interp (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 7: canTransform ---
  {
    const int N = 1000000;
    TransformBuffer buf;
    buildChain(buf, 10, kOneSecNs, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.can_transform("frame_0", "frame_10", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"canTransform chain=10 (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 8: static transforms ---
  {
    const int N = 500000;
    TransformBuffer buf;
    buildChain(buf, 10, kOneSecNs, true);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform("frame_0", "frame_10", kOneSecNs);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform static chain=10 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 9-14: V-tree (tf2 speed_test equivalent) ---
  {
    const uint32_t num_levels = 10;
    const int N = 1000000;
    TransformBuffer buf;

    auto setLink = [&](const std::string & parent, const std::string & child, TimeNs stamp) {
      TransformData d;
      d.stamp_ns = stamp;
      d.translation = {1.0, 0.0, 0.0};
      buf.set_transform(child, parent, d);
    };

    // Build left branch: root -> 0 -> 1 -> ... -> num_levels/2-1
    setLink("root", "0", kOneSecNs);
    setLink("root", "0", 2 * kOneSecNs);
    for (uint32_t i = 1; i < num_levels / 2; ++i) {
      setLink(std::to_string(i - 1), std::to_string(i), kOneSecNs);
      setLink(std::to_string(i - 1), std::to_string(i), 2 * kOneSecNs);
    }
    // Build right branch: root -> 5 -> 6 -> ... -> num_levels-1
    std::string mid = std::to_string(num_levels / 2);
    setLink("root", mid, kOneSecNs);
    setLink("root", mid, 2 * kOneSecNs);
    for (uint32_t i = num_levels / 2 + 1; i < num_levels; ++i) {
      setLink(std::to_string(i - 1), std::to_string(i), kOneSecNs);
      setLink(std::to_string(i - 1), std::to_string(i), 2 * kOneSecNs);
    }

    std::string tip_right = std::to_string(num_levels - 1);
    std::string tip_left = std::to_string(num_levels / 2 - 1);

    // Time(0) — latest
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, 0);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(0) latest (1M)", N, us, us * 1000.0 / N});
    }

    // Time(1) — exact match
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, kOneSecNs);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1) exact (1M)", N, us, us * 1000.0 / N});
    }

    // Time(1.5) — interpolation
    {
      TimeNs mid_ns = kOneSecNs + kOneSecNs / 2;
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, mid_ns);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1.5) interp (1M)", N, us, us * 1000.0 / N});
    }

    // Time(2) — exact match (other end)
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, 2 * kOneSecNs);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(2) exact (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(0)
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.can_transform(tip_left, tip_right, 0);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree canTransform Time(0) (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(1.5)
    {
      TimeNs mid_ns = kOneSecNs + kOneSecNs / 2;
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.can_transform(tip_left, tip_right, mid_ns);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree canTransform Time(1.5) (1M)", N, us, us * 1000.0 / N});
    }
  }

  printResults(results);

  // ================================================================
  // Multi-threaded benchmarks
  // ================================================================

  // --- MT Bench 1: Reader scalability (N readers, no writer) ---
  {
    std::printf("\n========== MT: Reader Scalability ==========\n");
    std::printf(
      "%-12s %12s %14s %12s\n", "Readers", "Total(ops/s)", "Per-thr(ops/s)", "Per-call(ns)");
    std::printf(
      "%-12s %12s %14s %12s\n", "-------", "------------", "--------------", "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      TransformBuffer buf;
      buildChain(buf, 10, kOneSecNs, false);

      const int iters_per_thread = 500000;
      std::vector<double> durations_us(num_readers);
      std::atomic<int> barrier{0};

      std::vector<std::thread> threads;
      for (int t = 0; t < num_readers; ++t) {
        threads.emplace_back([&, t]() {
          barrier.fetch_add(1);
          while (barrier.load() < num_readers) {
          }

          auto t0 = Clock::now();
          for (int i = 0; i < iters_per_thread; ++i) {
            buf.lookup_transform("frame_0", "frame_10", kOneSecNs);
          }
          auto t1 = Clock::now();
          durations_us[t] = std::chrono::duration<double, std::micro>(t1 - t0).count();
        });
      }
      for (auto & th : threads) {
        th.join();
      }

      double max_dur = *std::max_element(durations_us.begin(), durations_us.end());
      double total_ops = static_cast<double>(num_readers * iters_per_thread);
      double total_ops_per_sec = total_ops / (max_dur / 1e6);
      double per_thread = total_ops_per_sec / num_readers;
      double per_call_ns = max_dur * 1000.0 / iters_per_thread;

      std::printf(
        "%-12d %12.0f %14.0f %12.1f\n", num_readers, total_ops_per_sec, per_thread, per_call_ns);
    }
    std::printf("=============================================\n");
  }

  // --- MT Bench 2: 1 Writer + N Readers (varying writer freq) ---
  for (int write_hz : {100, 1000, 10000}) {
    int sleep_us = 1000000 / write_hz;
    std::printf("\n========== MT: 1 Writer(%dHz) + N Readers ==========\n", write_hz);
    std::printf(
      "%-12s %12s %14s %12s %12s\n", "Readers", "Read(ops/s)", "Per-thr(ops/s)", "Per-call(ns)",
      "Write(ops/s)");
    std::printf(
      "%-12s %12s %14s %12s %12s\n", "-------", "-----------", "--------------", "------------",
      "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      TransformBuffer buf;
      buildChain(buf, 10, kOneSecNs, false);

      const double test_duration_sec = 1.0;
      std::atomic<bool> running{true};
      std::vector<int64_t> read_counts(num_readers, 0);
      std::vector<double> reader_durations_us(num_readers);
      std::atomic<int64_t> write_count{0};
      std::atomic<int> barrier{0};
      int total_participants = num_readers + 1;

      // Writer thread
      std::thread writer([&]() {
        barrier.fetch_add(1);
        while (barrier.load() < total_participants) {
        }

        int64_t seq = 0;
        while (running.load(std::memory_order_relaxed)) {
          for (int i = 0; i < 10; ++i) {
            TransformData d;
            d.stamp_ns = kOneSecNs + seq * 1000000LL;
            d.translation = {1.0 + 0.001 * static_cast<double>(seq), 0.0, 0.0};
            buf.set_transform("frame_" + std::to_string(i + 1), "frame_" + std::to_string(i), d);
          }
          write_count.fetch_add(10, std::memory_order_relaxed);
          seq++;
          std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
      });

      // Reader threads
      std::vector<std::thread> readers;
      for (int t = 0; t < num_readers; ++t) {
        readers.emplace_back([&, t]() {
          barrier.fetch_add(1);
          while (barrier.load() < total_participants) {
          }

          int64_t count = 0;
          auto t0 = Clock::now();
          while (running.load(std::memory_order_relaxed)) {
            buf.lookup_transform("frame_0", "frame_10", kOneSecNs);
            count++;
          }
          auto t1 = Clock::now();
          read_counts[t] = count;
          reader_durations_us[t] = std::chrono::duration<double, std::micro>(t1 - t0).count();
        });
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_sec));
      running.store(false, std::memory_order_relaxed);

      writer.join();
      for (auto & th : readers) {
        th.join();
      }

      int64_t total_reads = std::accumulate(read_counts.begin(), read_counts.end(), int64_t{0});
      double max_dur = *std::max_element(reader_durations_us.begin(), reader_durations_us.end());
      double total_ops_per_sec = static_cast<double>(total_reads) / (max_dur / 1e6);
      double per_thread = total_ops_per_sec / num_readers;
      double per_call_ns = max_dur * 1000.0 / (static_cast<double>(total_reads) / num_readers);
      double actual_write = static_cast<double>(write_count.load()) / (max_dur / 1e6);

      std::printf(
        "%-12d %12.0f %14.0f %12.1f %12.0f\n", num_readers, total_ops_per_sec, per_thread,
        per_call_ns, actual_write);
    }
    std::printf("====================================================\n");
  }

  return 0;
}
