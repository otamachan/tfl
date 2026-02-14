#include "tfl/buffer_core.hpp"
#include <algorithm>
#include <limits>

namespace tfl {

BufferCore::BufferCore(const std::vector<FrameSpec>& specs, int64_t cache_duration_ns)
    : cache_duration_ns_(cache_duration_ns) {
  // Find max frame ID
  FrameID max_id = 0;
  for (auto& s : specs) {
    if (s.id > max_id) max_id = s.id;
  }

  // Allocate frame vector (index 0 is unused sentinel)
  frames_.reserve(max_id + 1);
  for (FrameID i = 0; i <= max_id; ++i) {
    frames_.emplace_back(2000, false);  // default dynamic
  }
  parent_ids_.resize(max_id + 1, INVALID_FRAME);
  id_to_name_.resize(max_id + 1);

  // Configure each frame
  for (auto& s : specs) {
    frames_[s.id] = FrameTimeCache(s.is_static ? 1 : 2000, s.is_static);
    parent_ids_[s.id] = s.parent_id;
    name_to_id_[s.name] = s.id;
    id_to_name_[s.id] = s.name;
  }
}

void BufferCore::setTransform(FrameID child, TimeNs stamp, Quat rot, Vec3 trans) {
  if (child == INVALID_FRAME || child >= frames_.size()) return;

  TransformData data;
  data.rotation = rot;
  data.translation = trans;
  data.stamp_ns = stamp;
  data.parent_id = parent_ids_[child];

  frames_[child].insert(data, cache_duration_ns_);
}

FrameID BufferCore::resolveFrameId(const std::string& name) const {
  auto it = name_to_id_.find(name);
  if (it == name_to_id_.end()) return INVALID_FRAME;
  return it->second;
}

// ── walkToTopParent ─────────────────────────────────────────

WalkEnding BufferCore::walkToTopParent(
    TransformAccum& accum, FrameID target_id, FrameID source_id, TimeNs time) const {
  if (source_id == target_id) {
    return WalkEnding::Identity;
  }

  // If time == 0, find latest common time
  if (time == 0) {
    time = getLatestCommonTime(target_id, source_id);
    if (time == 0) return WalkEnding::Error;
  }
  accum.time = time;

  // Walk source → root
  FrameID frame = source_id;
  FrameID top_parent = INVALID_FRAME;
  uint32_t depth = 0;

  while (frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= frames_.size()) return WalkEnding::Error;

    TransformData st;
    if (frames_[frame].getData(time, st) == 0) {
      return WalkEnding::Error;
    }

    // Check if we reached target
    if (frame == target_id) {
      // Source is a descendant of target — but we haven't accumulated
      // the current frame yet since it IS the target
      return WalkEnding::TargetParentOfSource;
    }

    if (depth > 0) {
      // First iteration: frame == source_id, st is source→parent transform
      // Don't accumulate the first call's own frame
    }
    accum.accumSource(st);

    top_parent = frame;
    frame = st.parent_id;
    depth++;

    if (frame == target_id) {
      return WalkEnding::TargetParentOfSource;
    }
  }
  if (depth >= MAX_GRAPH_DEPTH) return WalkEnding::Error;

  // Walk target → top_parent
  frame = target_id;
  depth = 0;

  while (frame != top_parent && frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= frames_.size()) return WalkEnding::Error;

    TransformData st;
    if (frames_[frame].getData(time, st) == 0) {
      return WalkEnding::Error;
    }

    if (frame == source_id) {
      return WalkEnding::SourceParentOfTarget;
    }

    accum.accumTarget(st);
    frame = st.parent_id;
    depth++;

    if (frame == source_id) {
      return WalkEnding::SourceParentOfTarget;
    }
  }

  if (frame != top_parent) return WalkEnding::Error;  // disconnected

  return WalkEnding::FullPath;
}

// ── finalize ────────────────────────────────────────────────

TransformResult BufferCore::TransformAccum::finalize(WalkEnding ending) const {
  TransformResult result;
  result.stamp_ns = time;

  switch (ending) {
    case WalkEnding::Identity:
      result.rotation = {0, 0, 0, 1};
      result.translation = {0, 0, 0};
      break;

    case WalkEnding::TargetParentOfSource:
      result.rotation = source_to_top_quat;
      result.translation = source_to_top_vec;
      break;

    case WalkEnding::SourceParentOfTarget: {
      Quat inv_q = quatInverse(target_to_top_quat);
      result.rotation = inv_q;
      result.translation = quatRotate(inv_q, -target_to_top_vec);
      break;
    }

    case WalkEnding::FullPath: {
      Quat inv_q = quatInverse(target_to_top_quat);
      Vec3 inv_v = quatRotate(inv_q, -target_to_top_vec);
      result.rotation = quatMultiply(inv_q, source_to_top_quat);
      result.translation = quatRotate(inv_q, source_to_top_vec) + inv_v;
      break;
    }

    default:
      result.rotation = {0, 0, 0, 1};
      result.translation = {0, 0, 0};
      break;
  }
  return result;
}

// ── lookupTransform ─────────────────────────────────────────

TransformResult BufferCore::lookupTransform(
    FrameID target, FrameID source, TimeNs time) const {
  TransformAccum accum;
  WalkEnding ending = walkToTopParent(accum, target, source, time);
  if (ending == WalkEnding::Error) {
    return TransformResult{{0, 0, 0, 1}, {0, 0, 0}, 0};
  }
  return accum.finalize(ending);
}

TransformResult BufferCore::lookupTransform(
    const std::string& target, const std::string& source, TimeNs time) const {
  FrameID t = resolveFrameId(target);
  FrameID s = resolveFrameId(source);
  if (t == INVALID_FRAME || s == INVALID_FRAME) {
    return TransformResult{{0, 0, 0, 1}, {0, 0, 0}, 0};
  }
  return lookupTransform(t, s, time);
}

bool BufferCore::canTransform(FrameID target, FrameID source, TimeNs time) const {
  TransformAccum accum;
  return walkToTopParent(accum, target, source, time) != WalkEnding::Error;
}

// ── getLatestCommonTime ─────────────────────────────────────

TimeNs BufferCore::getLatestCommonTime(FrameID target_id, FrameID source_id) const {
  if (source_id == target_id) {
    if (source_id < frames_.size()) {
      TimeNs t = frames_[source_id].getLatestStamp();
      return t;
    }
    return 0;
  }

  // Walk source → root, collecting (stamp, frame_id) pairs
  struct StampFrame { TimeNs stamp; FrameID id; };
  StampFrame source_chain[MAX_GRAPH_DEPTH];
  uint32_t source_len = 0;

  FrameID frame = source_id;
  TimeNs common_time = std::numeric_limits<TimeNs>::max();

  while (frame != INVALID_FRAME && source_len < MAX_GRAPH_DEPTH) {
    if (frame >= frames_.size()) break;

    TimeNs stamp = frames_[frame].getLatestStamp();
    if (frames_[frame].isStatic()) {
      // Static frames don't contribute to common time
    } else {
      if (stamp == 0) return 0;  // no data
      if (stamp < common_time) common_time = stamp;
    }

    FrameID parent = frames_[frame].getParent(0);  // get latest parent
    source_chain[source_len++] = {stamp, frame};

    if (frame == target_id) {
      return common_time == std::numeric_limits<TimeNs>::max() ? 0 : common_time;
    }
    frame = parent;
  }

  // Walk target → root, look for intersection with source chain
  frame = target_id;
  uint32_t depth = 0;

  while (frame != INVALID_FRAME && depth < MAX_GRAPH_DEPTH) {
    if (frame >= frames_.size()) break;

    TimeNs stamp = frames_[frame].getLatestStamp();
    if (!frames_[frame].isStatic()) {
      if (stamp == 0) return 0;
      if (stamp < common_time) common_time = stamp;
    }

    FrameID parent = frames_[frame].getParent(0);

    // Check if this frame is in the source chain
    for (uint32_t i = 0; i < source_len; ++i) {
      if (source_chain[i].id == frame || parent == source_chain[i].id) {
        // Found common ancestor
        // Walk source chain up to this point, taking min timestamp
        TimeNs result = common_time;
        for (uint32_t j = 0; j <= i; ++j) {
          if (!frames_[source_chain[j].id].isStatic() && source_chain[j].stamp < result) {
            result = source_chain[j].stamp;
          }
        }
        return result == std::numeric_limits<TimeNs>::max() ? 0 : result;
      }
    }

    frame = parent;
    depth++;
  }

  return 0;  // disconnected
}

}  // namespace tfl
