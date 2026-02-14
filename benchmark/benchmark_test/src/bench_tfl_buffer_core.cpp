#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "tfl/transform_buffer.hpp"

using Clock = std::chrono::high_resolution_clock;
using tfl::TransformBuffer;
using tfl::TimeNs;
using tfl::TransformData;

// ── Helpers ─────────────────────────────────────────────────

// Build a chain: frame_0 -> frame_1 -> ... -> frame_{depth}
// Returns frame names [0..depth]
std::vector<std::string> build_chain(
  TransformBuffer & buf, int depth, TimeNs stamp, bool is_static)
{
  for (int i = 0; i < depth; ++i) {
    std::string child = "frame_" + std::to_string(i + 1);
    std::string parent = "frame_" + std::to_string(i);
    TransformData td;
    td.stamp_ns = stamp;
    td.translation[0] = 1.0;
    td.rotation[3] = 1.0;
    buf.set_transform(child, parent, td, is_static);
  }
  std::vector<std::string> names(depth + 1);
  for (int i = 0; i <= depth; ++i) {
    names[i] = "frame_" + std::to_string(i);
  }
  return names;
}

// Build a tree: root -> branch_b_0 -> ... -> branch_b_{depth-1}
struct TreeInfo
{
  std::string root;
  std::vector<std::vector<std::string>> branch_names;  // [branch][depth]
};

TreeInfo build_tree(
  TransformBuffer & buf, int num_branches, int branch_depth, TimeNs stamp, bool is_static)
{
  TreeInfo info;
  info.root = "root";
  info.branch_names.resize(num_branches);

  for (int b = 0; b < num_branches; ++b) {
    info.branch_names[b].resize(branch_depth);
    std::string parent_name = "root";
    for (int d = 0; d < branch_depth; ++d) {
      std::string child_name = "branch_" + std::to_string(b) + "_" + std::to_string(d);

      TransformData td;
      td.stamp_ns = stamp;
      td.translation[0] = 0.1 * (d + 1);
      double angle = 0.01 * (b + 1);
      td.rotation[2] = std::sin(angle / 2.0);
      td.rotation[3] = std::cos(angle / 2.0);
      buf.set_transform(child_name, parent_name, td, is_static);

      info.branch_names[b][d] = child_name;
      parent_name = child_name;
    }
  }
  return info;
}

struct BenchResult
{
  std::string name;
  int iterations;
  double total_us;
  double per_call_ns;
};

void print_results(const std::vector<BenchResult> & results)
{
  std::cout << "\n========== tfl Benchmark Results ==========\n";
  printf("%-45s %10s %12s %12s\n", "Test", "Iters", "Total(us)", "Per-call(ns)");
  printf("%-45s %10s %12s %12s\n", "----", "-----", "---------", "------------");
  for (auto & r : results) {
    printf("%-45s %10d %12.1f %12.1f\n", r.name.c_str(), r.iterations, r.total_us, r.per_call_ns);
  }
  std::cout << "=============================================\n";
}

int main()
{
  std::vector<BenchResult> results;
  const TimeNs STAMP_1S = 1'000'000'000LL;

  // --- Bench 1: setTransform throughput ---
  {
    const int N = 100000;
    TransformBuffer buf(16, 10'000'000'000LL);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      TransformData td;
      td.stamp_ns = static_cast<int64_t>(i) * 10'000'000LL;
      td.translation[0] = static_cast<double>(i) * 0.001;
      td.rotation[3] = 1.0;
      buf.set_transform("child", "parent", td);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"setTransform (100k calls)", N, us, us * 1000.0 / N});
  }

  // --- Bench 2: lookupTransform identity ---
  {
    const int N = 1000000;
    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 1, STAMP_1S, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform(names[0], names[0], 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform identity (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 3: lookupTransform chain=5 ---
  {
    const int N = 500000;
    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 5, STAMP_1S, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform(names[0], names[5], 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=5 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 4: lookupTransform chain=10 ---
  {
    const int N = 500000;
    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 10, STAMP_1S, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform(names[0], names[10], 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=10 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 5: lookupTransform cross-branch ---
  {
    const int N = 500000;
    TransformBuffer buf(64, 10'000'000'000LL);
    auto info = build_tree(buf, 4, 5, STAMP_1S, false);

    const auto & b0_4 = info.branch_names[0][4];
    const auto & b3_4 = info.branch_names[3][4];
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform(b0_4, b3_4, 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform cross-branch (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 6: lookupTransform with interpolation ---
  {
    const int N = 500000;
    TransformBuffer buf(16, 10'000'000'000LL);

    for (int t = 0; t < 100; ++t) {
      TransformData td;
      td.stamp_ns = static_cast<int64_t>(t) * 100'000'000LL;
      td.translation[0] = static_cast<double>(t) * 0.01;
      double angle = static_cast<double>(t) * 0.01;
      td.rotation[2] = std::sin(angle / 2.0);
      td.rotation[3] = std::cos(angle / 2.0);
      buf.set_transform("child", "parent", td);
    }

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      TimeNs query = 50'000'000LL + static_cast<int64_t>(i % 98) * 100'000'000LL;
      buf.lookup_transform("parent", "child", query);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform interp (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 7: canTransform ---
  {
    const int N = 1000000;
    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 10, STAMP_1S, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.can_transform(names[0], names[10], 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"canTransform chain=10 (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 8: static transforms ---
  {
    const int N = 500000;
    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 10, STAMP_1S, true);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      buf.lookup_transform(names[0], names[10], 0);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform static chain=10 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 9-14: V-tree (tf2 speed_test equivalent) ---
  // V-shaped tree: root splits into 2 branches of depth num_levels/2
  //   root -> 0 -> 1 -> ... -> 4   (left branch)
  //   root -> 5 -> 6 -> ... -> 9   (right branch)
  // Lookup from tip of right branch to tip of left branch (cross-branch)
  {
    const uint32_t num_levels = 10;
    const int N = 1000000;
    TransformBuffer buf(32, 10'000'000'000LL);

    // Helper to set a transform link
    auto set_link = [&](const std::string & parent, const std::string & child, TimeNs stamp) {
      TransformData td;
      td.stamp_ns = stamp;
      td.translation[0] = 1.0;
      td.rotation[3] = 1.0;
      buf.set_transform(child, parent, td);
    };

    const TimeNs T1 = 1'000'000'000LL;
    const TimeNs T2 = 2'000'000'000LL;

    // Build left branch: root -> 0 -> 1 -> ... -> num_levels/2-1
    set_link("root", "0", T1);
    set_link("root", "0", T2);
    for (uint32_t i = 1; i < num_levels / 2; ++i) {
      set_link(std::to_string(i - 1), std::to_string(i), T1);
      set_link(std::to_string(i - 1), std::to_string(i), T2);
    }
    // Build right branch: root -> 5 -> 6 -> ... -> num_levels-1
    set_link("root", std::to_string(num_levels / 2), T1);
    set_link("root", std::to_string(num_levels / 2), T2);
    for (uint32_t i = num_levels / 2 + 1; i < num_levels; ++i) {
      set_link(std::to_string(i - 1), std::to_string(i), T1);
      set_link(std::to_string(i - 1), std::to_string(i), T2);
    }

    std::string tip_right = std::to_string(num_levels - 1);
    std::string tip_left = std::to_string(num_levels / 2 - 1);

    // lookupTransform at Time(0) — latest
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, 0);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(0) latest (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(1) — exact match
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, T1);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1) exact (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(1.5) — interpolation
    {
      const TimeNs T15 = 1'500'000'000LL;
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, T15);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1.5) interp (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(2) — exact match (other end)
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.lookup_transform(tip_left, tip_right, T2);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(2) exact (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(0) — latest
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.can_transform(tip_left, tip_right, 0);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree canTransform Time(0) (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(1.5) — interpolation
    {
      const TimeNs T15 = 1'500'000'000LL;
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        buf.can_transform(tip_left, tip_right, T15);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree canTransform Time(1.5) (1M)", N, us, us * 1000.0 / N});
    }
  }

  print_results(results);

  // ================================================================
  // Multi-threaded benchmarks
  // ================================================================
  std::cout << "\n\n";

  // --- MT Bench 1: Reader scalability ---
  {
    std::cout << "========== tfl MT: Reader Scalability ==========\n";
    printf("%-12s %12s %14s %12s\n", "Readers", "Total(ops/s)", "Per-thr(ops/s)", "Per-call(ns)");
    printf("%-12s %12s %14s %12s\n", "-------", "------------", "--------------", "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      TransformBuffer buf(16, 10'000'000'000LL);
      auto names = build_chain(buf, 10, STAMP_1S, false);
      const auto & f0 = names[0];
      const auto & f10 = names[10];

      const int iters_per_thread = 500000;
      std::vector<std::thread> threads;
      std::vector<double> durations_us(num_readers);

      std::atomic<int> barrier{0};

      for (int t = 0; t < num_readers; ++t) {
        threads.emplace_back([&, t]() {
          barrier.fetch_add(1);
          while (barrier.load() < num_readers) {
          }

          auto t0 = Clock::now();
          for (int i = 0; i < iters_per_thread; ++i) {
            buf.lookup_transform(f0, f10, 0);
          }
          auto t1 = Clock::now();
          durations_us[t] = std::chrono::duration<double, std::micro>(t1 - t0).count();
        });
      }
      for (auto & th : threads) th.join();

      double max_dur_us = *std::max_element(durations_us.begin(), durations_us.end());
      double total_ops = static_cast<double>(num_readers * iters_per_thread);
      double total_ops_s = total_ops / (max_dur_us / 1e6);
      double per_thr_ops_s = total_ops_s / num_readers;
      double per_call_ns = max_dur_us * 1000.0 / iters_per_thread;

      printf("%-12d %12.0f %14.0f %12.1f\n", num_readers, total_ops_s, per_thr_ops_s, per_call_ns);
    }
    std::cout << "==================================================\n";
  }

  // --- MT Bench 2: 1 Writer + N Readers ---
  for (int write_hz : {100, 1000, 10000}) {
    int sleep_us = 1000000 / write_hz;
    std::cout << "\n========== tfl MT: 1 Writer(" << write_hz << "Hz) + N Readers ==========\n";
    printf(
      "%-12s %12s %14s %12s %12s\n", "Readers", "Read(ops/s)", "Per-thr(ops/s)", "Per-call(ns)",
      "Write(ops/s)");
    printf(
      "%-12s %12s %14s %12s %12s\n", "-------", "-----------", "--------------", "------------",
      "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      TransformBuffer buf(16, 10'000'000'000LL);
      auto names = build_chain(buf, 10, STAMP_1S, false);
      const auto & f0 = names[0];
      const auto & f10 = names[10];

      const double test_duration_sec = 1.0;
      std::atomic<bool> running{true};
      std::vector<int64_t> read_counts(num_readers, 0);
      std::vector<double> reader_durations_us(num_readers);
      std::atomic<int64_t> write_count{0};

      std::atomic<int> barrier{0};
      int total_participants = num_readers + 1;

      // Pre-compute frame name pairs for writer
      std::vector<std::pair<std::string, std::string>> frame_pairs;
      for (int i = 0; i < 10; ++i) {
        frame_pairs.emplace_back("frame_" + std::to_string(i + 1), "frame_" + std::to_string(i));
      }

      // Writer
      std::thread writer([&]() {
        barrier.fetch_add(1);
        while (barrier.load() < total_participants) {
        }

        int64_t seq = 0;
        while (running.load(std::memory_order_relaxed)) {
          for (int i = 0; i < 10; ++i) {
            TransformData td;
            td.stamp_ns = STAMP_1S + seq * 1'000'000LL;
            td.translation[0] = 1.0 + 0.001 * static_cast<double>(seq);
            td.rotation[3] = 1.0;
            buf.set_transform(frame_pairs[i].first, frame_pairs[i].second, td);
          }
          write_count.fetch_add(10, std::memory_order_relaxed);
          seq++;
          std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
        }
      });

      // Readers
      std::vector<std::thread> readers;
      for (int t = 0; t < num_readers; ++t) {
        readers.emplace_back([&, t]() {
          barrier.fetch_add(1);
          while (barrier.load() < total_participants) {
          }

          int64_t count = 0;
          auto t0 = Clock::now();
          while (running.load(std::memory_order_relaxed)) {
            buf.lookup_transform(f0, f10, 0);
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
      for (auto & th : readers) th.join();

      int64_t total_reads = std::accumulate(read_counts.begin(), read_counts.end(), int64_t{0});
      double max_dur_us = *std::max_element(reader_durations_us.begin(), reader_durations_us.end());
      double total_ops_s = static_cast<double>(total_reads) / (max_dur_us / 1e6);
      double per_thr_ops_s = total_ops_s / num_readers;
      double per_call_ns = max_dur_us * 1000.0 / (static_cast<double>(total_reads) / num_readers);
      double actual_write_ops = static_cast<double>(write_count.load()) / (max_dur_us / 1e6);

      printf(
        "%-12d %12.0f %14.0f %12.1f %12.0f\n", num_readers, total_ops_s, per_thr_ops_s, per_call_ns,
        actual_write_ops);
    }
    std::cout << "========================================================\n";
  }

  // --- MT Bench 3: Latency distribution ---
  for (int write_hz : {100, 1000, 10000}) {
    int sleep_us = 1000000 / write_hz;
    std::cout << "\n========== tfl MT: Latency Distribution (1W@" << write_hz
              << "Hz+1R) ==========\n";

    TransformBuffer buf(16, 10'000'000'000LL);
    auto names = build_chain(buf, 10, STAMP_1S, false);
    const auto & f0 = names[0];
    const auto & f10 = names[10];

    // Pre-compute frame name pairs for writer
    std::vector<std::pair<std::string, std::string>> frame_pairs;
    for (int i = 0; i < 10; ++i) {
      frame_pairs.emplace_back("frame_" + std::to_string(i + 1), "frame_" + std::to_string(i));
    }

    const double test_duration_sec = 2.0;
    std::atomic<bool> running{true};
    std::vector<int64_t> latencies_ns;
    latencies_ns.reserve(10000000);

    std::atomic<int> barrier{0};

    // Writer
    std::thread writer([&]() {
      barrier.fetch_add(1);
      while (barrier.load() < 2) {
      }

      int64_t seq = 0;
      while (running.load(std::memory_order_relaxed)) {
        for (int i = 0; i < 10; ++i) {
          TransformData td;
          td.stamp_ns = STAMP_1S + seq * 1'000'000LL;
          td.translation[0] = 1.0 + 0.001 * static_cast<double>(seq);
          td.rotation[3] = 1.0;
          buf.set_transform(frame_pairs[i].first, frame_pairs[i].second, td);
        }
        seq++;
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
      }
    });

    // Reader
    std::thread reader([&]() {
      barrier.fetch_add(1);
      while (barrier.load() < 2) {
      }

      while (running.load(std::memory_order_relaxed)) {
        auto t0 = Clock::now();
        buf.lookup_transform(f0, f10, 0);
        auto t1 = Clock::now();
        latencies_ns.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
      }
    });

    std::this_thread::sleep_for(std::chrono::duration<double>(test_duration_sec));
    running.store(false, std::memory_order_relaxed);

    writer.join();
    reader.join();

    std::sort(latencies_ns.begin(), latencies_ns.end());
    size_t n = latencies_ns.size();
    if (n > 0) {
      auto pct = [&](double p) -> int64_t {
        size_t idx = static_cast<size_t>(p * static_cast<double>(n - 1));
        return latencies_ns[idx];
      };
      double avg =
        static_cast<double>(std::accumulate(latencies_ns.begin(), latencies_ns.end(), int64_t{0})) /
        static_cast<double>(n);

      printf("  Samples:  %zu\n", n);
      printf("  Mean:     %.1f ns\n", avg);
      printf("  p50:      %ld ns\n", pct(0.50));
      printf("  p90:      %ld ns\n", pct(0.90));
      printf("  p99:      %ld ns\n", pct(0.99));
      printf("  p99.9:    %ld ns\n", pct(0.999));
      printf("  Max:      %ld ns\n", latencies_ns.back());
    }
    std::cout << "==========================================================\n";
  }

  return 0;
}
