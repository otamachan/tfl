#pragma once
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace tfl {

using TimeNs  = int64_t;
using FrameID = uint32_t;

constexpr FrameID INVALID_FRAME = 0;
constexpr uint32_t MAX_GRAPH_DEPTH = 1000;

// ── POD math types ──────────────────────────────────────────────

struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

struct Quat {
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};

struct TransformData {
  Quat     rotation;
  Vec3     translation;
  TimeNs   stamp_ns   = 0;
  FrameID  parent_id  = INVALID_FRAME;
  uint32_t _pad       = 0;
};

static_assert(std::is_trivially_copyable_v<TransformData>);

struct TransformResult {
  Quat   rotation;
  Vec3   translation;
  TimeNs stamp_ns = 0;
};

// ── Inline math ─────────────────────────────────────────────────

inline double dot(Quat a, Quat b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline Quat quatMultiply(Quat a, Quat b) {
  return {
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
  };
}

inline Quat quatInverse(Quat q) {
  // unit quaternion: conjugate == inverse
  return {-q.x, -q.y, -q.z, q.w};
}

inline Vec3 quatRotate(Quat q, Vec3 v) {
  // q * (v,0) * q^-1  — expanded for efficiency
  // t = 2 * cross(q.xyz, v)
  double tx = 2.0 * (q.y * v.z - q.z * v.y);
  double ty = 2.0 * (q.z * v.x - q.x * v.z);
  double tz = 2.0 * (q.x * v.y - q.y * v.x);
  return {
    v.x + q.w * tx + (q.y * tz - q.z * ty),
    v.y + q.w * ty + (q.z * tx - q.x * tz),
    v.z + q.w * tz + (q.x * ty - q.y * tx)
  };
}

inline Quat slerp(Quat q1, Quat q2, double t) {
  double d = dot(q1, q2);
  // Take shortest path
  Quat q2a = q2;
  if (d < 0.0) {
    d = -d;
    q2a = {-q2.x, -q2.y, -q2.z, -q2.w};
  }
  if (d > 1.0 - 1e-6) {
    // Very close — linear interpolation
    return {
      q1.x + t * (q2a.x - q1.x),
      q1.y + t * (q2a.y - q1.y),
      q1.z + t * (q2a.z - q1.z),
      q1.w + t * (q2a.w - q1.w)
    };
  }
  double theta = std::acos(d);
  double sin_theta = std::sin(theta);
  double ra = std::sin((1.0 - t) * theta) / sin_theta;
  double rb = std::sin(t * theta) / sin_theta;
  return {
    ra * q1.x + rb * q2a.x,
    ra * q1.y + rb * q2a.y,
    ra * q1.z + rb * q2a.z,
    ra * q1.w + rb * q2a.w
  };
}

inline Vec3 lerp(Vec3 a, Vec3 b, double t) {
  return {
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t,
    a.z + (b.z - a.z) * t
  };
}

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }

}  // namespace tfl
