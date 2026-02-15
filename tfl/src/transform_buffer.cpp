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

#include "tfl/transform_buffer.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

namespace tfl
{

namespace
{

using Quat = TransformData::Quat;
using Vec3 = TransformData::Vec3;

void quat_multiply(const Quat & a, const Quat & b, Quat & out)
{
  out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
}

void quat_inverse(const Quat & q, Quat & out)
{
  out[0] = -q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = q[3];
}

void quat_rotate(const Quat & q, const Vec3 & v, Vec3 & out)
{
  const double tx = 2.0 * (q[1] * v[2] - q[2] * v[1]);
  const double ty = 2.0 * (q[2] * v[0] - q[0] * v[2]);
  const double tz = 2.0 * (q[0] * v[1] - q[1] * v[0]);
  out[0] = v[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
  out[1] = v[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
  out[2] = v[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
}

constexpr double kSlerpThreshold = 1e-6;

double quat_dot(const Quat & a, const Quat & b)
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

void slerp(const Quat & q1, const Quat & q2, double t, Quat & out)
{
  double d = quat_dot(q1, q2);
  Quat q2a = q2;
  if (d < 0.0) {
    d = -d;
    q2a[0] = -q2[0];
    q2a[1] = -q2[1];
    q2a[2] = -q2[2];
    q2a[3] = -q2[3];
  }
  if (d > 1.0 - kSlerpThreshold) {
    for (int i = 0; i < 4; ++i) {
      out[i] = q1[i] + t * (q2a[i] - q1[i]);
    }
    return;
  }
  const double theta = std::acos(d);
  const double sin_theta = std::sin(theta);
  const double ra = std::sin((1.0 - t) * theta) / sin_theta;
  const double rb = std::sin(t * theta) / sin_theta;
  for (int i = 0; i < 4; ++i) {
    out[i] = ra * q1[i] + rb * q2a[i];
  }
}

void lerp3(const Vec3 & a, const Vec3 & b, double t, Vec3 & out)
{
  for (int i = 0; i < 3; ++i) {
    out[i] = a[i] + (b[i] - a[i]) * t;
  }
}

void add3(const Vec3 & a, const Vec3 & b, Vec3 & out)
{
  for (int i = 0; i < 3; ++i) {
    out[i] = a[i] + b[i];
  }
}

void neg3(const Vec3 & a, Vec3 & out)
{
  for (int i = 0; i < 3; ++i) {
    out[i] = -a[i];
  }
}

// result = parent * child  (apply child, then parent)
TransformData compose(const TransformData & parent, const TransformData & child)
{
  TransformData result;
  Vec3 rotated;
  quat_rotate(parent.rotation, child.translation, rotated);
  add3(rotated, parent.translation, result.translation);
  quat_multiply(parent.rotation, child.rotation, result.rotation);
  return result;
}

TransformData inverse(const TransformData & t)
{
  TransformData result;
  quat_inverse(t.rotation, result.rotation);
  Vec3 neg_v;
  neg3(t.translation, neg_v);
  quat_rotate(result.rotation, neg_v, result.translation);
  return result;
}

}  // anonymous namespace

TransformBuffer::TransformBuffer(uint32_t max_frames, int64_t cache_duration_ns)
: max_frames_(max_frames),
  name_to_id_(max_frames * 2),  // load factor ~0.5
  cache_duration_ns_(cache_duration_ns)
{
  frames_.reserve(max_frames);
  // Index 0 = INVALID_FRAME (sentinel)
  for (uint32_t i = 0; i < max_frames; ++i) {
    frames_.emplace_back(FrameTransformBuffer::kDefaultCapacity, false);
  }
}

FrameID TransformBuffer::register_frame(const std::string & name, bool is_static)
{
  // Check if already registered
  const FrameID existing = name_to_id_.find(name);
  if (existing != INVALID_FRAME) {
    return existing;
  }

  const FrameID id = next_id_.fetch_add(1, std::memory_order_relaxed);
  if (id >= max_frames_) {
    return INVALID_FRAME;  // full
  }

  if (is_static) {
    frames_[id] = FrameTransformBuffer(1, true);
  }
  name_to_id_.insert(name, id);
  return id;
}

void TransformBuffer::set_transform(
  const std::string & child, const std::string & parent, const TransformData & data, bool is_static)
{
  const FrameID child_id = register_frame(child, is_static);
  const FrameID parent_id = register_frame(parent);
  if (child_id == INVALID_FRAME || parent_id == INVALID_FRAME) {
    return;
  }
  TransformData internal = data;
  internal.parent_id = parent_id;
  frames_[child_id].insert(internal, cache_duration_ns_);
}

FrameID TransformBuffer::resolve_frame_id(const std::string & name) const
{
  return name_to_id_.find(name);
}

void TransformBuffer::clear()
{
  const uint32_t count = next_id_.load(std::memory_order_relaxed);
  for (uint32_t i = 1; i < count; ++i) {
    frames_[i].clear();
  }
}

void interpolate(
  const TransformData & d1, const TransformData & d2, TimeNs time, TransformData & out)
{
  const double range = static_cast<double>(d2.stamp_ns - d1.stamp_ns);
  const double t = (range > 0.0) ? static_cast<double>(time - d1.stamp_ns) / range : 0.0;
  slerp(d1.rotation, d2.rotation, t, out.rotation);
  lerp3(d1.translation, d2.translation, t, out.translation);
  out.stamp_ns = time;
  out.parent_id = d1.parent_id;
}

// Fetch frame data with interpolation. Returns false on error.
bool get_frame_data(const FrameTransformBuffer & cache, TimeNs time, TransformData & st)
{
  TransformData d1, d2;
  const uint8_t n = cache.get_data(time, d1, d2);
  if (n == 0) {
    return false;
  }
  if (n == 1) {
    st = d1;
    return true;
  }
  // n == 2
  interpolate(d1, d2, time, st);
  return true;
}

std::optional<TransformData> TransformBuffer::walk_to_top_parent(
  FrameID target_id, FrameID source_id, TimeNs time) const
{
  if (source_id == target_id) {
    TransformData identity;
    identity.stamp_ns = time;
    return identity;
  }

  if (time == 0) {
    time = get_latest_common_time(target_id, source_id);
    if (time == 0) {
      return std::nullopt;
    }
  }

  // Phase 1: walk source chain up to root, accumulate source_to_top
  TransformData source_to_top;  // identity by default
  FrameID frame = source_id;
  FrameID top_parent = INVALID_FRAME;
  uint32_t depth = 0;

  while (frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= max_frames_) {
      return std::nullopt;
    }

    if (frame == target_id) {
      source_to_top.stamp_ns = time;
      return source_to_top;
    }

    TransformData st;
    if (!get_frame_data(frames_[frame], time, st)) {
      // Reached root (no data in this frame) — stop source walk
      top_parent = frame;
      break;
    }

    source_to_top = compose(st, source_to_top);

    top_parent = frame;
    frame = st.parent_id;
    depth++;

    if (frame == target_id) {
      source_to_top.stamp_ns = time;
      return source_to_top;
    }
  }
  if (depth >= MAX_GRAPH_DEPTH) {
    return std::nullopt;
  }

  // Phase 2: walk target chain up to top_parent, accumulate target_to_top
  TransformData target_to_top;  // identity by default
  frame = target_id;
  depth = 0;

  while (frame != top_parent && frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= max_frames_) {
      return std::nullopt;
    }

    TransformData st;
    if (!get_frame_data(frames_[frame], time, st)) {
      return std::nullopt;
    }

    target_to_top = compose(st, target_to_top);
    frame = st.parent_id;
    depth++;

    if (frame == source_id) {
      auto result = inverse(target_to_top);
      result.stamp_ns = time;
      return result;
    }
  }

  if (frame != top_parent) {
    return std::nullopt;
  }

  // Full path: target_to_top⁻¹ * source_to_top
  auto result = compose(inverse(target_to_top), source_to_top);
  result.stamp_ns = time;
  return result;
}

std::optional<TransformData> TransformBuffer::lookup_transform(
  const std::string & target, const std::string & source, TimeNs time) const
{
  const FrameID t = resolve_frame_id(target);
  const FrameID s = resolve_frame_id(source);
  if (t == INVALID_FRAME || s == INVALID_FRAME) {
    return std::nullopt;
  }
  return walk_to_top_parent(t, s, time);
}

bool TransformBuffer::can_transform(
  const std::string & target, const std::string & source, TimeNs time) const
{
  const FrameID t = resolve_frame_id(target);
  const FrameID s = resolve_frame_id(source);
  if (t == INVALID_FRAME || s == INVALID_FRAME) {
    return false;
  }
  return walk_to_top_parent(t, s, time).has_value();
}

std::optional<TransformData> TransformBuffer::lookup_transform(
  const std::string & target, const std::string & source, TimeNs time,
  std::chrono::nanoseconds timeout) const
{
  auto result = lookup_transform(target, source, time);
  if (result.has_value() || timeout.count() <= 0) {
    return result;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    result = lookup_transform(target, source, time);
    if (result.has_value()) {
      return result;
    }
  }
  return std::nullopt;
}

bool TransformBuffer::can_transform(
  const std::string & target, const std::string & source, TimeNs time,
  std::chrono::nanoseconds timeout) const
{
  if (can_transform(target, source, time)) {
    return true;
  }
  if (timeout.count() <= 0) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (can_transform(target, source, time)) {
      return true;
    }
  }
  return false;
}

TimeNs TransformBuffer::get_latest_common_time(FrameID target_id, FrameID source_id) const
{
  if (source_id == target_id) {
    if (source_id < max_frames_) {
      return frames_[source_id].get_latest_stamp();
    }
    return 0;
  }

  struct StampFrame
  {
    TimeNs stamp;
    FrameID id;
  };
  std::array<StampFrame, MAX_GRAPH_DEPTH> source_chain;
  uint32_t source_len = 0;

  FrameID frame = source_id;
  TimeNs common_time = std::numeric_limits<TimeNs>::max();

  while (frame != INVALID_FRAME && source_len < MAX_GRAPH_DEPTH) {
    if (frame >= max_frames_) {
      break;
    }

    const TimeNs stamp = frames_[frame].get_latest_stamp();
    if (!frames_[frame].is_static()) {
      if (stamp == 0) {
        return 0;
      }
      if (stamp < common_time) {
        common_time = stamp;
      }
    }

    const FrameID parent = frames_[frame].get_parent(0);
    source_chain[source_len++] = {stamp, frame};

    if (frame == target_id) {
      return common_time == std::numeric_limits<TimeNs>::max() ? 0 : common_time;
    }
    frame = parent;
  }

  frame = target_id;
  uint32_t depth = 0;

  while (frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= max_frames_) {
      break;
    }

    const TimeNs stamp = frames_[frame].get_latest_stamp();
    if (!frames_[frame].is_static()) {
      if (stamp == 0) {
        return 0;
      }
      if (stamp < common_time) {
        common_time = stamp;
      }
    }

    const FrameID parent = frames_[frame].get_parent(0);

    for (uint32_t i = 0; i < source_len; ++i) {
      if (source_chain[i].id == frame || parent == source_chain[i].id) {
        TimeNs result = common_time;
        for (uint32_t j = 0; j <= i; ++j) {
          if (!frames_[source_chain[j].id].is_static() && source_chain[j].stamp < result) {
            result = source_chain[j].stamp;
          }
        }
        return result == std::numeric_limits<TimeNs>::max() ? 0 : result;
      }
    }

    frame = parent;
    depth++;
  }

  return 0;
}

}  // namespace tfl
