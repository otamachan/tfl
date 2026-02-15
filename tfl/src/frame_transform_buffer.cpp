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

#include "tfl/frame_transform_buffer.hpp"

namespace tfl
{

FrameTransformBuffer::FrameTransformBuffer(uint32_t capacity, bool is_static)
: buf_(std::make_unique<TransformData[]>(is_static ? 1 : round_up_pow2(capacity))),
  capacity_(is_static ? 1 : round_up_pow2(capacity)),
  mask_(capacity_ - 1),
  is_static_(is_static)
{
}

FrameTransformBuffer::FrameTransformBuffer(FrameTransformBuffer && o) noexcept
: seq_(o.seq_.load(std::memory_order_relaxed)),
  buf_(std::move(o.buf_)),
  capacity_(o.capacity_),
  mask_(o.mask_),
  head_(o.head_.load(std::memory_order_relaxed)),
  size_(o.size_.load(std::memory_order_relaxed)),
  is_static_(o.is_static_)
{
}

FrameTransformBuffer & FrameTransformBuffer::operator=(FrameTransformBuffer && o) noexcept
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

void FrameTransformBuffer::insert(const TransformData & data, TimeNs cache_duration_ns)
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

void FrameTransformBuffer::clear()
{
  seq_.fetch_add(1, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_release);

  size_.store(0, std::memory_order_relaxed);
  head_.store(0, std::memory_order_relaxed);

  std::atomic_thread_fence(std::memory_order_release);
  seq_.fetch_add(1, std::memory_order_release);
}

void FrameTransformBuffer::sort_buffer()
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

}  // namespace tfl
