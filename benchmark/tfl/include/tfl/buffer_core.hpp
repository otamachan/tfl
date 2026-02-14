#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include "tfl/frame_time_cache.hpp"

namespace tfl {

struct FrameSpec {
  std::string name;
  FrameID id;
  FrameID parent_id = INVALID_FRAME;
  bool is_static = false;
};

enum class WalkEnding {
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
  Error
};

class BufferCore {
 public:
  // frames: must include all frames. id=0 is reserved (INVALID_FRAME).
  // Frame IDs must be contiguous from 1..max_id.
  BufferCore(const std::vector<FrameSpec>& specs,
             int64_t cache_duration_ns = 10'000'000'000LL);

  // Writer (single-threaded)
  void setTransform(FrameID child, TimeNs stamp, Quat rot, Vec3 trans);

  // Reader (lock-free, multi-threaded)
  TransformResult lookupTransform(FrameID target, FrameID source, TimeNs time) const;
  TransformResult lookupTransform(const std::string& target,
                                  const std::string& source, TimeNs time) const;
  bool canTransform(FrameID target, FrameID source, TimeNs time) const;

  FrameID resolveFrameId(const std::string& name) const;
  size_t frameCount() const { return frames_.size(); }

 private:
  struct TransformAccum {
    Quat source_to_top_quat{0, 0, 0, 1};
    Vec3 source_to_top_vec{0, 0, 0};
    Quat target_to_top_quat{0, 0, 0, 1};
    Vec3 target_to_top_vec{0, 0, 0};
    TimeNs time = 0;

    void accumSource(const TransformData& st) {
      source_to_top_vec = quatRotate(st.rotation, source_to_top_vec) + st.translation;
      source_to_top_quat = quatMultiply(st.rotation, source_to_top_quat);
    }

    void accumTarget(const TransformData& st) {
      target_to_top_vec = quatRotate(st.rotation, target_to_top_vec) + st.translation;
      target_to_top_quat = quatMultiply(st.rotation, target_to_top_quat);
    }

    TransformResult finalize(WalkEnding ending) const;
  };

  WalkEnding walkToTopParent(TransformAccum& accum,
                             FrameID target_id, FrameID source_id,
                             TimeNs time) const;

  TimeNs getLatestCommonTime(FrameID target_id, FrameID source_id) const;

  std::vector<FrameTimeCache> frames_;
  std::vector<FrameID> parent_ids_;
  std::unordered_map<std::string, FrameID> name_to_id_;
  std::vector<std::string> id_to_name_;
  int64_t cache_duration_ns_;
};

}  // namespace tfl
