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

  explicit FrameTransformBuffer(uint32_t capacity = kDefaultCapacity, bool is_static = false)
  : buf_(std::make_unique<TransformData[]>(is_static ? 1 : round_up_pow2(capacity))),
    capacity_(is_static ? 1 : round_up_pow2(capacity)),
    mask_(capacity_ - 1),
    is_static_(is_static)
  {
  }

  FrameTransformBuffer(FrameTransformBuffer && o) noexcept
  : seq_(o.seq_.load(std::memory_order_relaxed)),
    buf_(std::move(o.buf_)),
    capacity_(o.capacity_),
    mask_(o.mask_),
    head_(o.head_.load(std::memory_order_relaxed)),
    size_(o.size_.load(std::memory_order_relaxed)),
    is_static_(o.is_static_)
  {
  }

  FrameTransformBuffer & operator=(FrameTransformBuffer && o) noexcept
  {
    if (this != &o) {
      seq_.store(o.seq_.load(std::memory_order_relaxed), std::memory_order_relaxed);
      buf_ = std::move(o.buf_);
      capacity_ = o.capacity_;
      mask_ = o.mask_;
      head_.store(o.head_.load(std::memory_order_relaxed), std::memory_order_relaxed);
      size_.store(o.size_.load(std::memory_order_relaxed), std::memory_order_relaxed);
      is_static_ = o.is_static_;
    }
    return *this;
  }

  void insert(const TransformData & data, TimeNs cache_duration_ns)
  {
    // Begin write
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    std::atomic_thread_fence(std::memory_order_release);

    if (is_static_) {
      buf_[0] = data;
    } else {
      uint32_t h = head_.load(std::memory_order_relaxed);
      uint32_t sz = size_.load(std::memory_order_relaxed);

      // Reject old data
      if (sz > 0) {
        TimeNs latest = buf_[h].stamp_ns;
        if (data.stamp_ns < latest - cache_duration_ns) {
          // Too old, skip — still need to end the SeqLock
          std::atomic_thread_fence(std::memory_order_release);
          seq_.fetch_add(1, std::memory_order_release);
          return;
        }
      }

      // Advance head
      if (sz == 0) {
        h = 0;
        sz = 1;
        buf_[0] = data;
      } else {
        // Find insertion point: data should go in timestamp order
        // Common case: new data is newest
        if (data.stamp_ns >= buf_[h].stamp_ns) {
          h = (h + 1) & mask_;
          if (sz < capacity_) {
            sz++;
          }
          buf_[h] = data;
        } else {
          // Out-of-order insert: find correct position
          h = (h + 1) & mask_;
          if (sz < capacity_) {
            sz++;
          }
          buf_[h] = data;
          // Sort the logical window to maintain descending order
          head_.store(h, std::memory_order_relaxed);
          size_.store(sz, std::memory_order_relaxed);
          sort_buffer();
          // sort_buffer may not change h/sz, but reload for consistency
          h = head_.load(std::memory_order_relaxed);
          sz = size_.load(std::memory_order_relaxed);
        }
      }

      // Prune old entries
      if (sz > 1) {
        TimeNs newest = buf_[h].stamp_ns;
        while (sz > 1) {
          uint32_t tail = phys_idx(h, sz - 1);
          if (buf_[tail].stamp_ns >= newest - cache_duration_ns) {
            break;
          }
          sz--;
        }
      }

      head_.store(h, std::memory_order_relaxed);
      size_.store(sz, std::memory_order_relaxed);
    }

    // End write
    std::atomic_thread_fence(std::memory_order_release);
    seq_.fetch_add(1, std::memory_order_release);  // even = done
  }

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
  void clear()
  {
    seq_.fetch_add(1, std::memory_order_release);
    std::atomic_thread_fence(std::memory_order_release);

    size_.store(0, std::memory_order_relaxed);
    head_.store(0, std::memory_order_relaxed);

    std::atomic_thread_fence(std::memory_order_release);
    seq_.fetch_add(1, std::memory_order_release);
  }

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
  void sort_buffer()
  {
    uint32_t h = head_.load(std::memory_order_relaxed);
    uint32_t sz = size_.load(std::memory_order_relaxed);
    // Simple insertion sort on the logical window (small and rare)
    for (uint32_t i = 1; i < sz; ++i) {
      uint32_t pi = phys_idx(h, i);
      TransformData key = buf_[pi];
      uint32_t j = i;
      while (j > 0 && buf_[phys_idx(h, j - 1)].stamp_ns < key.stamp_ns) {
        buf_[phys_idx(h, j)] = buf_[phys_idx(h, j - 1)];
        j--;
      }
      if (j != i) {
        buf_[phys_idx(h, j)] = key;
      }
    }
  }

  alignas(64) std::atomic<uint64_t> seq_{0};
  std::unique_ptr<TransformData[]> buf_;
  uint32_t capacity_;
  uint32_t mask_;
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> size_{0};
  bool is_static_;
};

}  // namespace tfl
