#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "tf2/buffer_core.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using Clock = std::chrono::high_resolution_clock;

// Build a chain: frame_0 -> frame_1 -> ... -> frame_{depth-1}
void buildChain(
  tf2::BufferCore & bc,
  int depth,
  const std::chrono::nanoseconds & stamp_ns,
  bool is_static)
{
  for (int i = 0; i < depth; ++i) {
    geometry_msgs::msg::TransformStamped ts;
    ts.header.frame_id = "frame_" + std::to_string(i);
    ts.child_frame_id = "frame_" + std::to_string(i + 1);
    ts.header.stamp.sec = static_cast<int32_t>(stamp_ns.count() / 1000000000LL);
    ts.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns.count() % 1000000000LL);
    ts.transform.translation.x = 1.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;
    ts.transform.rotation.x = 0.0;
    ts.transform.rotation.y = 0.0;
    ts.transform.rotation.z = 0.0;
    ts.transform.rotation.w = 1.0;
    bc.setTransform(ts, "benchmark", is_static);
  }
}

// Build a tree: root -> branch_b_0 -> branch_b_1 -> ... (num_branches branches, each of given depth)
void buildTree(
  tf2::BufferCore & bc,
  int num_branches,
  int branch_depth,
  const std::chrono::nanoseconds & stamp_ns,
  bool is_static)
{
  for (int b = 0; b < num_branches; ++b) {
    std::string parent = "root";
    for (int d = 0; d < branch_depth; ++d) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.frame_id = parent;
      ts.child_frame_id = "branch_" + std::to_string(b) + "_" + std::to_string(d);
      ts.header.stamp.sec = static_cast<int32_t>(stamp_ns.count() / 1000000000LL);
      ts.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns.count() % 1000000000LL);
      ts.transform.translation.x = 0.1 * (d + 1);
      ts.transform.translation.y = 0.0;
      ts.transform.translation.z = 0.0;
      // Small rotation around Z
      double angle = 0.01 * (b + 1);
      ts.transform.rotation.x = 0.0;
      ts.transform.rotation.y = 0.0;
      ts.transform.rotation.z = std::sin(angle / 2.0);
      ts.transform.rotation.w = std::cos(angle / 2.0);
      bc.setTransform(ts, "benchmark", is_static);
      parent = ts.child_frame_id;
    }
  }
}

struct BenchResult {
  std::string name;
  int iterations;
  double total_us;
  double per_call_ns;
};

void printResults(const std::vector<BenchResult> & results)
{
  std::cout << "\n========== Benchmark Results ==========\n";
  std::cout << std::left;
  printf("%-45s %10s %12s %12s\n", "Test", "Iters", "Total(us)", "Per-call(ns)");
  printf("%-45s %10s %12s %12s\n", "----", "-----", "---------", "------------");
  for (auto & r : results) {
    printf("%-45s %10d %12.1f %12.1f\n",
      r.name.c_str(), r.iterations, r.total_us, r.per_call_ns);
  }
  std::cout << "=======================================\n";
}

int main()
{
  std::vector<BenchResult> results;

  // --- Bench 1: setTransform throughput ---
  {
    const int N = 100000;
    tf2::BufferCore bc;
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.frame_id = "parent";
      ts.child_frame_id = "child";
      auto stamp_ns = std::chrono::nanoseconds(i * 10000000LL);  // 10ms increments
      ts.header.stamp.sec = static_cast<int32_t>(stamp_ns.count() / 1000000000LL);
      ts.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns.count() % 1000000000LL);
      ts.transform.translation.x = static_cast<double>(i) * 0.001;
      ts.transform.translation.y = 0.0;
      ts.transform.translation.z = 0.0;
      ts.transform.rotation.w = 1.0;
      bc.setTransform(ts, "bench");
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"setTransform (100k calls)", N, us, us * 1000.0 / N});
  }

  // --- Bench 2: lookupTransform - same frame (identity) ---
  {
    const int N = 1000000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 1, stamp, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.lookupTransform("frame_0", "frame_0", tf2::TimePointZero);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform identity (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 3: lookupTransform - chain depth 5 ---
  {
    const int N = 500000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 5, stamp, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.lookupTransform("frame_0", "frame_5", tf2::TimePointZero);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=5 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 4: lookupTransform - chain depth 10 ---
  {
    const int N = 500000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 10, stamp, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.lookupTransform("frame_0", "frame_10", tf2::TimePointZero);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform chain=10 (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 5: lookupTransform - cross branch (tree) ---
  {
    const int N = 500000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildTree(bc, 4, 5, stamp, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.lookupTransform("branch_0_4", "branch_3_4", tf2::TimePointZero);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform cross-branch (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 6: lookupTransform with interpolation ---
  {
    const int N = 500000;
    tf2::BufferCore bc;
    // Insert two timestamps for interpolation
    for (int t = 0; t < 100; ++t) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.frame_id = "parent";
      ts.child_frame_id = "child";
      auto stamp_ns = std::chrono::nanoseconds(static_cast<int64_t>(t) * 100000000LL);
      ts.header.stamp.sec = static_cast<int32_t>(stamp_ns.count() / 1000000000LL);
      ts.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns.count() % 1000000000LL);
      ts.transform.translation.x = static_cast<double>(t) * 0.01;
      ts.transform.translation.y = 0.0;
      ts.transform.translation.z = 0.0;
      double angle = static_cast<double>(t) * 0.01;
      ts.transform.rotation.z = std::sin(angle / 2.0);
      ts.transform.rotation.w = std::cos(angle / 2.0);
      bc.setTransform(ts, "bench");
    }

    // Query at midpoints
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      int64_t query_ns = 50000000LL + static_cast<int64_t>(i % 98) * 100000000LL;
      tf2::TimePoint tp{std::chrono::nanoseconds{query_ns}};
      bc.lookupTransform("parent", "child", tp);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"lookupTransform interp (500k)", N, us, us * 1000.0 / N});
  }

  // --- Bench 7: canTransform ---
  {
    const int N = 1000000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 10, stamp, false);

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.canTransform("frame_0", "frame_10", tf2::TimePointZero);
    }
    auto t1 = Clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
    results.push_back({"canTransform chain=10 (1M)", N, us, us * 1000.0 / N});
  }

  // --- Bench 8: static transforms ---
  {
    const int N = 500000;
    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 10, stamp, true);  // static

    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      bc.lookupTransform("frame_0", "frame_10", tf2::TimePointZero);
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
    tf2::BufferCore bc;

    // Helper to set a transform at a given time
    auto setLink = [&](const std::string& parent, const std::string& child,
                       double time_sec) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.frame_id = parent;
      ts.child_frame_id = child;
      auto ns = static_cast<int64_t>(time_sec * 1e9);
      ts.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      ts.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
      ts.transform.translation.x = 1.0;
      ts.transform.rotation.w = 1.0;
      bc.setTransform(ts, "speed_test");
    };

    // Build left branch: root -> 0 -> 1 -> ... -> num_levels/2-1
    setLink("root", "0", 1.0);
    setLink("root", "0", 2.0);
    for (uint32_t i = 1; i < num_levels / 2; ++i) {
      setLink(std::to_string(i - 1), std::to_string(i), 1.0);
      setLink(std::to_string(i - 1), std::to_string(i), 2.0);
    }
    // Build right branch: root -> 5 -> 6 -> ... -> num_levels-1
    std::string mid = std::to_string(num_levels / 2);
    setLink("root", mid, 1.0);
    setLink("root", mid, 2.0);
    for (uint32_t i = num_levels / 2 + 1; i < num_levels; ++i) {
      setLink(std::to_string(i - 1), std::to_string(i), 1.0);
      setLink(std::to_string(i - 1), std::to_string(i), 2.0);
    }

    std::string tip_right = std::to_string(num_levels - 1);
    std::string tip_left = std::to_string(num_levels / 2 - 1);

    // lookupTransform at Time(0) — latest
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.lookupTransform(tip_left, tip_right, tf2::TimePointZero);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(0) latest (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(1) — exact match
    {
      tf2::TimePoint tp{std::chrono::nanoseconds{1000000000LL}};
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.lookupTransform(tip_left, tip_right, tp);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1) exact (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(1.5) — interpolation
    {
      tf2::TimePoint tp{std::chrono::nanoseconds{1500000000LL}};
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.lookupTransform(tip_left, tip_right, tp);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(1.5) interp (1M)", N, us, us * 1000.0 / N});
    }

    // lookupTransform at Time(2) — exact match (other end)
    {
      tf2::TimePoint tp{std::chrono::nanoseconds{2000000000LL}};
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.lookupTransform(tip_left, tip_right, tp);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree lookup Time(2) exact (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(0) — latest
    {
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.canTransform(tip_left, tip_right, tf2::TimePointZero);
      }
      auto t1 = Clock::now();
      double us = std::chrono::duration<double, std::micro>(t1 - t0).count();
      results.push_back({"V-tree canTransform Time(0) (1M)", N, us, us * 1000.0 / N});
    }

    // canTransform at Time(1.5) — interpolation
    {
      tf2::TimePoint tp{std::chrono::nanoseconds{1500000000LL}};
      auto t0 = Clock::now();
      for (int i = 0; i < N; ++i) {
        bc.canTransform(tip_left, tip_right, tp);
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
  std::cout << "\n\n";

  // --- MT Bench 1: Reader scalability (N readers, no writer) ---
  {
    std::cout << "========== MT: Reader Scalability ==========\n";
    printf("%-12s %12s %12s %12s\n", "Readers", "Total(ops/s)", "Per-thr(ops/s)", "Per-call(ns)");
    printf("%-12s %12s %12s %12s\n", "-------", "------------", "--------------", "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      tf2::BufferCore bc;
      auto stamp = std::chrono::nanoseconds(1000000000LL);
      buildChain(bc, 10, stamp, false);

      const int iters_per_thread = 500000;
      std::vector<std::thread> threads;
      std::vector<double> durations_us(num_readers);

      auto barrier = std::atomic<int>{0};

      for (int t = 0; t < num_readers; ++t) {
        threads.emplace_back([&, t]() {
          barrier.fetch_add(1);
          while (barrier.load() < num_readers) {}  // spin-wait for all threads

          auto t0 = Clock::now();
          for (int i = 0; i < iters_per_thread; ++i) {
            bc.lookupTransform("frame_0", "frame_10", tf2::TimePointZero);
          }
          auto t1 = Clock::now();
          durations_us[t] = std::chrono::duration<double, std::micro>(t1 - t0).count();
        });
      }
      for (auto & th : threads) { th.join(); }

      double max_dur_us = *std::max_element(durations_us.begin(), durations_us.end());
      double total_ops = static_cast<double>(num_readers * iters_per_thread);
      double total_ops_per_sec = total_ops / (max_dur_us / 1e6);
      double per_thread_ops_per_sec = total_ops_per_sec / num_readers;
      double per_call_ns = max_dur_us * 1000.0 / iters_per_thread;

      printf("%-12d %12.0f %14.0f %12.1f\n",
        num_readers, total_ops_per_sec, per_thread_ops_per_sec, per_call_ns);
    }
    std::cout << "=============================================\n";
  }

  // --- MT Bench 2: 1 Writer + N Readers (varying writer freq) ---
  for (int write_hz : {100, 1000, 10000}) {
    int sleep_us = 1000000 / write_hz;
    std::cout << "\n========== MT: 1 Writer(" << write_hz << "Hz) + N Readers ==========\n";
    printf("%-12s %12s %14s %12s %12s\n",
      "Readers", "Read(ops/s)", "Per-thr(ops/s)", "Per-call(ns)", "Write(ops/s)");
    printf("%-12s %12s %14s %12s %12s\n",
      "-------", "-----------", "--------------", "------------", "------------");

    for (int num_readers : {1, 2, 4, 8}) {
      tf2::BufferCore bc;
      auto stamp = std::chrono::nanoseconds(1000000000LL);
      buildChain(bc, 10, stamp, false);

      const double test_duration_sec = 1.0;
      std::atomic<bool> running{true};
      std::vector<int64_t> read_counts(num_readers, 0);
      std::vector<double> reader_durations_us(num_readers);
      std::atomic<int64_t> write_count{0};

      auto barrier = std::atomic<int>{0};
      int total_participants = num_readers + 1;

      // Writer thread
      std::thread writer([&]() {
        barrier.fetch_add(1);
        while (barrier.load() < total_participants) {}

        int64_t seq = 0;
        while (running.load(std::memory_order_relaxed)) {
          for (int i = 0; i < 10; ++i) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.frame_id = "frame_" + std::to_string(i);
            ts.child_frame_id = "frame_" + std::to_string(i + 1);
            int64_t ns = 1000000000LL + seq * 1000000LL;
            ts.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
            ts.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
            ts.transform.translation.x = 1.0 + 0.001 * static_cast<double>(seq);
            ts.transform.translation.y = 0.0;
            ts.transform.translation.z = 0.0;
            ts.transform.rotation.w = 1.0;
            bc.setTransform(ts, "bench");
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
          while (barrier.load() < total_participants) {}

          int64_t count = 0;
          auto t0 = Clock::now();
          while (running.load(std::memory_order_relaxed)) {
            bc.lookupTransform("frame_0", "frame_10", tf2::TimePointZero);
            count++;
          }
          auto t1 = Clock::now();
          read_counts[t] = count;
          reader_durations_us[t] = std::chrono::duration<double, std::micro>(t1 - t0).count();
        });
      }

      std::this_thread::sleep_for(
        std::chrono::duration<double>(test_duration_sec));
      running.store(false, std::memory_order_relaxed);

      writer.join();
      for (auto & th : readers) { th.join(); }

      int64_t total_reads = std::accumulate(read_counts.begin(), read_counts.end(), int64_t{0});
      double max_dur_us = *std::max_element(
        reader_durations_us.begin(), reader_durations_us.end());
      double total_ops_per_sec = static_cast<double>(total_reads) / (max_dur_us / 1e6);
      double per_thread_ops_per_sec = total_ops_per_sec / num_readers;
      double per_call_ns = max_dur_us * 1000.0 /
        (static_cast<double>(total_reads) / num_readers);
      double actual_write_ops = static_cast<double>(write_count.load()) / (max_dur_us / 1e6);

      printf("%-12d %12.0f %14.0f %12.1f %12.0f\n",
        num_readers, total_ops_per_sec, per_thread_ops_per_sec, per_call_ns, actual_write_ops);
    }
    std::cout << "====================================================\n";
  }

  // --- MT Bench 3: Latency distribution (1 Writer + 1 Reader, varying writer freq) ---
  for (int write_hz : {100, 1000, 10000}) {
    int sleep_us = 1000000 / write_hz;
    std::cout << "\n========== MT: Latency Distribution (1W@"
              << write_hz << "Hz+1R) ==========\n";

    tf2::BufferCore bc;
    auto stamp = std::chrono::nanoseconds(1000000000LL);
    buildChain(bc, 10, stamp, false);

    const double test_duration_sec = 2.0;
    std::atomic<bool> running{true};
    std::vector<int64_t> latencies_ns;
    latencies_ns.reserve(10000000);

    std::atomic<int> barrier{0};

    // Writer
    std::thread writer([&]() {
      barrier.fetch_add(1);
      while (barrier.load() < 2) {}

      int64_t seq = 0;
      while (running.load(std::memory_order_relaxed)) {
        for (int i = 0; i < 10; ++i) {
          geometry_msgs::msg::TransformStamped ts;
          ts.header.frame_id = "frame_" + std::to_string(i);
          ts.child_frame_id = "frame_" + std::to_string(i + 1);
          int64_t ns = 1000000000LL + seq * 1000000LL;
          ts.header.stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
          ts.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
          ts.transform.translation.x = 1.0 + 0.001 * static_cast<double>(seq);
          ts.transform.translation.y = 0.0;
          ts.transform.translation.z = 0.0;
          ts.transform.rotation.w = 1.0;
          bc.setTransform(ts, "bench");
        }
        seq++;
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
      }
    });

    // Reader
    std::thread reader([&]() {
      barrier.fetch_add(1);
      while (barrier.load() < 2) {}

      while (running.load(std::memory_order_relaxed)) {
        auto t0 = Clock::now();
        bc.lookupTransform("frame_0", "frame_10", tf2::TimePointZero);
        auto t1 = Clock::now();
        latencies_ns.push_back(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
      }
    });

    std::this_thread::sleep_for(
      std::chrono::duration<double>(test_duration_sec));
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
      double avg = static_cast<double>(
        std::accumulate(latencies_ns.begin(), latencies_ns.end(), int64_t{0}))
        / static_cast<double>(n);

      printf("  Samples:  %zu\n", n);
      printf("  Mean:     %.1f ns\n", avg);
      printf("  p50:      %ld ns\n", pct(0.50));
      printf("  p90:      %ld ns\n", pct(0.90));
      printf("  p99:      %ld ns\n", pct(0.99));
      printf("  p99.9:    %ld ns\n", pct(0.999));
      printf("  Max:      %ld ns\n", latencies_ns.back());
    }
    std::cout << "======================================================\n";
  }

  return 0;
}
