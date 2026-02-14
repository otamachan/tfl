#pragma once
#include <atomic>
#include <cstring>
#include <memory>
#include "tfl/types.hpp"

namespace tfl {

// SeqLock + circular buffer per-frame cache.
// Single writer, multiple lock-free readers.
class FrameTimeCache {
 public:
  explicit FrameTimeCache(uint32_t capacity = 2000, bool is_static = false)
      : capacity_(is_static ? 1 : capacity),
        is_static_(is_static),
        buf_(std::make_unique<TransformData[]>(is_static ? 1 : capacity)) {}

  FrameTimeCache(FrameTimeCache&& o) noexcept
      : seq_(o.seq_.load(std::memory_order_relaxed)),
        buf_(std::move(o.buf_)),
        capacity_(o.capacity_),
        head_(o.head_),
        size_(o.size_),
        is_static_(o.is_static_) {}

  FrameTimeCache& operator=(FrameTimeCache&& o) noexcept {
    if (this != &o) {
      seq_.store(o.seq_.load(std::memory_order_relaxed), std::memory_order_relaxed);
      buf_ = std::move(o.buf_);
      capacity_ = o.capacity_;
      head_ = o.head_;
      size_ = o.size_;
      is_static_ = o.is_static_;
    }
    return *this;
  }

  // ── Writer (single-threaded) ──────────────────────────────

  void insert(const TransformData& data, TimeNs cache_duration_ns) {
    // Begin write
    seq_.fetch_add(1, std::memory_order_release);  // odd = writing
    std::atomic_thread_fence(std::memory_order_release);

    if (is_static_) {
      buf_[0] = data;
    } else {
      // Reject old data
      if (size_ > 0) {
        TimeNs latest = buf_[head_].stamp_ns;
        if (data.stamp_ns < latest - cache_duration_ns) {
          // Too old, skip — still need to end the SeqLock
          std::atomic_thread_fence(std::memory_order_release);
          seq_.fetch_add(1, std::memory_order_release);
          return;
        }
      }

      // Advance head
      if (size_ == 0) {
        head_ = 0;
        size_ = 1;
        buf_[0] = data;
      } else {
        // Find insertion point: data should go in timestamp order
        // Common case: new data is newest
        if (data.stamp_ns >= buf_[head_].stamp_ns) {
          head_ = (head_ + 1) % capacity_;
          if (size_ < capacity_) size_++;
          buf_[head_] = data;
        } else {
          // Out-of-order insert: find correct position
          // For simplicity, insert at head anyway (SeqLock will protect)
          // In practice, tf2 also just inserts at the right position
          head_ = (head_ + 1) % capacity_;
          if (size_ < capacity_) size_++;
          buf_[head_] = data;
          // Sort the logical window to maintain descending order
          sortBuffer();
        }
      }

      // Prune old entries
      if (size_ > 1) {
        TimeNs newest = buf_[head_].stamp_ns;
        while (size_ > 1) {
          uint32_t tail = physIdx(size_ - 1);
          if (buf_[tail].stamp_ns >= newest - cache_duration_ns) break;
          size_--;
        }
      }
    }

    // End write
    std::atomic_thread_fence(std::memory_order_release);
    seq_.fetch_add(1, std::memory_order_release);  // even = done
  }

  // ── Reader (lock-free, multi-threaded) ────────────────────

  // Returns: 0 = no data, 1 = exact match (in d1), 2 = interpolate (d1=older, d2=newer)
  uint8_t getData(TimeNs time, TransformData& d1, TransformData& d2) const {
    for (int retry = 0; retry < 64; ++retry) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1) continue;  // write in progress

      uint8_t result = getDataUnsafe(time, d1, d2);

      std::atomic_thread_fence(std::memory_order_acquire);
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) return result;
      // Sequence changed, retry
    }
    return 0;  // give up after retries
  }

  // Single-result convenience
  uint8_t getData(TimeNs time, TransformData& out) const {
    TransformData d1, d2;
    uint8_t n = getData(time, d1, d2);
    if (n == 0) return 0;
    if (n == 1) {
      out = d1;
      return 1;
    }
    // Interpolate
    if (d1.parent_id != d2.parent_id) {
      out = d1;  // parent changed, no interpolation
      return 1;
    }
    double total = static_cast<double>(d2.stamp_ns - d1.stamp_ns);
    if (total <= 0.0) {
      out = d2;
      return 1;
    }
    double ratio = static_cast<double>(time - d1.stamp_ns) / total;
    out.rotation    = slerp(d1.rotation, d2.rotation, ratio);
    out.translation = lerp(d1.translation, d2.translation, ratio);
    out.stamp_ns    = time;
    out.parent_id   = d1.parent_id;
    return 1;
  }

  TimeNs getLatestStamp() const {
    for (int retry = 0; retry < 64; ++retry) {
      uint64_t s1 = seq_.load(std::memory_order_acquire);
      if (s1 & 1) continue;

      TimeNs result = 0;
      if (is_static_) {
        result = 0;  // static frames have no meaningful timestamp
      } else if (size_ > 0) {
        result = buf_[head_].stamp_ns;
      }

      std::atomic_thread_fence(std::memory_order_acquire);
      uint64_t s2 = seq_.load(std::memory_order_acquire);
      if (s1 == s2) return result;
    }
    return 0;
  }

  FrameID getParent(TimeNs time) const {
    TransformData d;
    if (getData(time, d) > 0) return d.parent_id;
    return INVALID_FRAME;
  }

  bool empty() const {
    uint64_t s1 = seq_.load(std::memory_order_acquire);
    if (s1 & 1) return true;  // writing, treat as empty
    bool result = (!is_static_ && size_ == 0);
    uint64_t s2 = seq_.load(std::memory_order_acquire);
    if (s1 != s2) return true;
    return result;
  }

  bool isStatic() const { return is_static_; }

 private:
  // Logical index i (0=newest) to physical buffer index
  uint32_t physIdx(uint32_t i) const {
    return (head_ + capacity_ - i) % capacity_;
  }

  // Must be called inside SeqLock read section (no seq_ check)
  uint8_t getDataUnsafe(TimeNs time, TransformData& d1, TransformData& d2) const {
    if (is_static_) {
      d1 = buf_[0];
      d1.stamp_ns = time;  // static cache overwrites timestamp (like tf2)
      return 1;
    }

    if (size_ == 0) return 0;

    // time == 0 means "latest"
    if (time == 0) {
      d1 = buf_[head_];
      return 1;
    }

    // Single element
    if (size_ == 1) {
      if (buf_[head_].stamp_ns == time) {
        d1 = buf_[head_];
        return 1;
      }
      return 0;  // extrapolation error
    }

    TimeNs newest = buf_[head_].stamp_ns;
    uint32_t oldest_idx = physIdx(size_ - 1);
    TimeNs oldest = buf_[oldest_idx].stamp_ns;

    // Exact match with newest
    if (time == newest) { d1 = buf_[head_]; return 1; }
    // Exact match with oldest
    if (time == oldest) { d1 = buf_[oldest_idx]; return 1; }

    // Out of range
    if (time > newest || time < oldest) return 0;

    // Binary search: find first logical index where stamp < time
    // Buffer is logically sorted descending: [newest ... oldest]
    uint32_t lo = 0, hi = size_ - 1;
    while (lo < hi) {
      uint32_t mid = (lo + hi) / 2;
      if (buf_[physIdx(mid)].stamp_ns > time) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    // lo = first index where stamp <= time
    if (buf_[physIdx(lo)].stamp_ns == time) {
      d1 = buf_[physIdx(lo)];
      return 1;
    }
    // lo points to older, lo-1 points to newer
    d1 = buf_[physIdx(lo)];      // older
    d2 = buf_[physIdx(lo - 1)];  // newer
    return 2;
  }

  // Sort the logical buffer window (called only on out-of-order insert)
  void sortBuffer() {
    // Simple insertion sort on the logical window (small and rare)
    for (uint32_t i = 1; i < size_; ++i) {
      uint32_t pi = physIdx(i);
      TransformData key = buf_[pi];
      uint32_t j = i;
      while (j > 0 && buf_[physIdx(j - 1)].stamp_ns < key.stamp_ns) {
        buf_[physIdx(j)] = buf_[physIdx(j - 1)];
        j--;
      }
      if (j != i) buf_[physIdx(j)] = key;
    }
  }

  alignas(64) std::atomic<uint64_t> seq_{0};
  std::unique_ptr<TransformData[]> buf_;
  uint32_t capacity_;
  uint32_t head_ = 0;
  uint32_t size_ = 0;
  bool is_static_;
};

}  // namespace tfl
