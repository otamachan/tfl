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
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "tfl/types.hpp"

namespace tfl
{

// Append-only wait-free hash map: string -> FrameID.
// Insert is single-writer (but safe with concurrent readers).
// Find is wait-free for any number of reader threads.
class FrameMap
{
public:
  explicit FrameMap(uint32_t capacity = 2048)
  : capacity_(capacity), slots_(std::make_unique<Slot[]>(capacity))
  {
  }

  // Insert a name -> id mapping. Returns true if inserted, false if already
  // exists. Single-writer only.
  bool insert(const std::string & name, FrameID id)
  {
    uint64_t h = hash(name);
    if (h == 0) {
      h = 1;  // 0 is reserved for empty
    }

    for (uint32_t i = 0; i < capacity_; ++i) {
      uint32_t idx = (static_cast<uint32_t>(h) + i) % capacity_;
      Slot & slot = slots_[idx];

      if (slot.hash.load(std::memory_order_acquire) == 0) {
        // Empty slot — try to claim it
        // Single-writer so no CAS needed, just store
        slot.key = name;
        slot.value = id;
        std::atomic_thread_fence(std::memory_order_release);
        slot.hash.store(h, std::memory_order_release);
        size_.fetch_add(1, std::memory_order_relaxed);
        return true;
      }

      // Slot occupied — check if same key
      if (slot.hash.load(std::memory_order_acquire) == h && slot.key == name) {
        return false;  // already exists
      }
      // Collision, continue probing
    }
    return false;  // table full
  }

  // Find a name. Lock-free, safe from any thread.
  FrameID find(const std::string & name) const
  {
    uint64_t h = hash(name);
    if (h == 0) {
      h = 1;
    }

    for (uint32_t i = 0; i < capacity_; ++i) {
      uint32_t idx = (static_cast<uint32_t>(h) + i) % capacity_;
      const Slot & slot = slots_[idx];

      uint64_t sh = slot.hash.load(std::memory_order_acquire);
      if (sh == 0) {
        return INVALID_FRAME;  // empty slot = not found
      }

      if (sh == h && slot.key == name) {
        return slot.value;
      }
      // Hash collision, continue probing
    }
    return INVALID_FRAME;  // table full, not found
  }

  uint32_t size() const { return size_.load(std::memory_order_relaxed); }
  uint32_t capacity() const { return capacity_; }

private:
  static uint64_t hash(const std::string & s) { return std::hash<std::string>{}(s); }

  struct Slot
  {
    std::atomic<uint64_t> hash{0};  // 0 = empty, non-zero = occupied
    std::string key;                // immutable after hash becomes non-zero
    FrameID value = INVALID_FRAME;  // immutable after hash becomes non-zero
  };

  uint32_t capacity_;
  std::unique_ptr<Slot[]> slots_;
  std::atomic<uint32_t> size_{0};
};

}  // namespace tfl
