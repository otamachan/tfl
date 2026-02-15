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
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "tfl/frame_map.hpp"
#include "tfl/frame_transform_buffer.hpp"

namespace tfl
{

class TransformBuffer
{
public:
  static constexpr uint32_t kDefaultMaxFrames = 1024;
  static constexpr int64_t kDefaultCacheDurationNs = 10'000'000'000LL;  // 10 seconds

  // max_frames: maximum number of frames (pre-allocated, no reallocation)
  explicit TransformBuffer(
    uint32_t max_frames = kDefaultMaxFrames, int64_t cache_duration_ns = kDefaultCacheDurationNs);

  // Writer (single-threaded). Frames are auto-registered on first use.
  void set_transform(
    const std::string & child, const std::string & parent, const TransformData & data,
    bool is_static = false);

  // Reader (mutex-free, multi-threaded)
  std::optional<TransformData> lookup_transform(
    const std::string & target, const std::string & source, TimeNs time) const;
  bool can_transform(const std::string & target, const std::string & source, TimeNs time) const;

  // Timeout variants: poll every 10ms until success or deadline
  std::optional<TransformData> lookup_transform(
    const std::string & target, const std::string & source, TimeNs time,
    std::chrono::nanoseconds timeout) const;
  bool can_transform(
    const std::string & target, const std::string & source, TimeNs time,
    std::chrono::nanoseconds timeout) const;

  uint32_t frame_count() const { return next_id_.load(std::memory_order_relaxed) - 1; }

  // Reset all cached transform data. Frame registrations are preserved.
  void clear();

private:
  FrameID register_frame(const std::string & name, bool is_static = false);
  FrameID resolve_frame_id(const std::string & name) const;

  std::optional<TransformData> walk_to_top_parent(
    FrameID target_id, FrameID source_id, TimeNs time) const;

  TimeNs get_latest_common_time(FrameID target_id, FrameID source_id) const;

  uint32_t max_frames_;
  std::vector<FrameTransformBuffer> frames_;  // pre-allocated, index = FrameID
  FrameMap name_to_id_;                       // wait-free append-only
  std::atomic<FrameID> next_id_{1};           // 0 = INVALID_FRAME
  int64_t cache_duration_ns_;
};

}  // namespace tfl
