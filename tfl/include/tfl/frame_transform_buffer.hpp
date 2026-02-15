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

#pragma once
#include <atomic>
#include <cstring>
#include <memory>
#include <utility>

#include "tfl/types.hpp"

namespace tfl
{

// SeqLock + circular buffer per-frame cache.
// Single writer, mutex-free readers.
class FrameTransformBuffer
{
public:
  static constexpr uint32_t kDefaultCapacity = 2000;

  explicit FrameTransformBuffer(uint32_t capacity = kDefaultCapacity, bool is_static = false);
  FrameTransformBuffer(FrameTransformBuffer && o) noexcept;
  FrameTransformBuffer & operator=(FrameTransformBuffer && o) noexcept;

  void insert(const TransformData & data, TimeNs cache_duration_ns);

  // Returns: 0 = no data, 1 = exact match (in d1), 2 = interpolate (d1=older,
  // d2=newer)
  uint8_t get_data(TimeNs time, TransformData & d1, TransformData & d2) const
  {
    for (int retry = 0; retry < kMaxSeqRetries; ++retry) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1) {
        continue;  // write in progress
      }

      uint8_t result = get_data_unsafe(time, d1, d2);

      std::atomic_thread_fence(std::memory_order_acquire);
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) {
        return result;
      }
      // Sequence changed, retry
    }
    return 0;  // give up after retries
  }

  TimeNs get_latest_stamp() const
  {
    for (int retry = 0; retry < kMaxSeqRetries; ++retry) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1) {
        continue;
      }

      TimeNs result = 0;
      if (is_static_) {
        result = 0;  // static frames have no meaningful timestamp
      } else {
        uint32_t sz = size_.load(std::memory_order_relaxed);
        if (sz > 0) {
          uint32_t h = head_.load(std::memory_order_relaxed);
          result = buf_[h].stamp_ns;
        }
      }

      std::atomic_thread_fence(std::memory_order_acquire);
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) {
        return result;
      }
    }
    return 0;
  }

  FrameID get_parent(TimeNs time) const
  {
    for (int retry = 0; retry < kMaxSeqRetries; ++retry) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1) {
        continue;
      }

      FrameID result = get_parent_unsafe(time);

      std::atomic_thread_fence(std::memory_order_acquire);
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) {
        return result;
      }
    }
    return INVALID_FRAME;
  }

  bool is_static() const { return is_static_; }

  // Reset all cached data. Single-writer only.
  void clear();

private:
  static constexpr int kMaxSeqRetries = 64;

  static constexpr uint32_t round_up_pow2(uint32_t v)
  {
    if (v <= 1) return 1;
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
  }

  // Logical index i (0=newest) to physical buffer index
  uint32_t phys_idx(uint32_t head, uint32_t i) const { return (head + capacity_ - i) & mask_; }

  // Lightweight parent lookup — only reads parent_id, no TransformData copy
  FrameID get_parent_unsafe(TimeNs time) const
  {
    if (is_static_) {
      return buf_[0].parent_id;
    }
    uint32_t sz = size_.load(std::memory_order_relaxed);
    if (sz == 0) {
      return INVALID_FRAME;
    }
    uint32_t h = head_.load(std::memory_order_relaxed);
    if (time == 0 || sz == 1) {
      return buf_[h].parent_id;
    }
    // For non-zero time with multiple entries, check range and return parent
    // (parent_id is same for all entries in a frame)
    TimeNs newest = buf_[h].stamp_ns;
    uint32_t oldest_idx = phys_idx(h, sz - 1);
    TimeNs oldest = buf_[oldest_idx].stamp_ns;
    if (time > newest || time < oldest) {
      return INVALID_FRAME;
    }
    return buf_[h].parent_id;
  }

  // Must be called inside SeqLock read section (no seq_ check)
  uint8_t get_data_unsafe(TimeNs time, TransformData & d1, TransformData & d2) const
  {
    if (is_static_) {
      d1 = buf_[0];
      d1.stamp_ns = time;  // static cache overwrites timestamp (like tf2)
      return 1;
    }

    uint32_t sz = size_.load(std::memory_order_relaxed);
    if (sz == 0) {
      return 0;
    }

    uint32_t h = head_.load(std::memory_order_relaxed);

    // time == 0 means "latest"
    if (time == 0) {
      d1 = buf_[h];
      return 1;
    }

    // Single element
    if (sz == 1) {
      if (buf_[h].stamp_ns == time) {
        d1 = buf_[h];
        return 1;
      }
      return 0;  // extrapolation error
    }

    TimeNs newest = buf_[h].stamp_ns;
    uint32_t oldest_idx = phys_idx(h, sz - 1);
    TimeNs oldest = buf_[oldest_idx].stamp_ns;

    // Exact match with newest
    if (time == newest) {
      d1 = buf_[h];
      return 1;
    }
    // Exact match with oldest
    if (time == oldest) {
      d1 = buf_[oldest_idx];
      return 1;
    }

    // Out of range
    if (time > newest || time < oldest) {
      return 0;
    }

    // Binary search: find first logical index where stamp < time
    // Buffer is logically sorted descending: [newest ... oldest]
    uint32_t lo = 0, hi = sz - 1;
    while (lo < hi) {
      uint32_t mid = (lo + hi) / 2;
      if (buf_[phys_idx(h, mid)].stamp_ns > time) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    // lo = first index where stamp <= time
    if (buf_[phys_idx(h, lo)].stamp_ns == time) {
      d1 = buf_[phys_idx(h, lo)];
      return 1;
    }
    // lo points to older, lo-1 points to newer
    d1 = buf_[phys_idx(h, lo)];      // older
    d2 = buf_[phys_idx(h, lo - 1)];  // newer
    return 2;
  }

  // Sort the logical buffer window (called only on out-of-order insert)
  void sort_buffer();

  alignas(64) std::atomic<uint64_t> seq_{0};
  std::unique_ptr<TransformData[]> buf_;
  uint32_t capacity_;
  uint32_t mask_;
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> size_{0};
  bool is_static_;
};

}  // namespace tfl
