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
#include <array>
#include <cstdint>
#include <type_traits>

namespace tfl
{

using TimeNs = int64_t;
using FrameID = uint32_t;

constexpr FrameID INVALID_FRAME = 0;
constexpr uint32_t MAX_GRAPH_DEPTH = 128;

struct TransformData
{
  using Quat = std::array<double, 4>;
  using Vec3 = std::array<double, 3>;

  Quat rotation = {0.0, 0.0, 0.0, 1.0};  // x, y, z, w
  Vec3 translation = {0.0, 0.0, 0.0};
  TimeNs stamp_ns = 0;
  FrameID parent_id = INVALID_FRAME;
};

static_assert(std::is_trivially_copyable_v<TransformData>);

}  // namespace tfl
