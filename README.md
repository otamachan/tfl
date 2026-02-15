# tfl (tf light)

Mutex-free tf2 alternative for ROS 2. An experimental rewrite of tf2's `BufferCore` that eliminates all mutexes.

## Motivation

tf2's `BufferCore` uses `std::mutex` to protect shared data. This means all concurrent access — including reads — is serialized. tfl is an experimental attempt to remove mutex locks entirely, using SeqLock + wait-free hash map to allow concurrent reads without blocking.

## Packages

| Package | Description |
|---------|-------------|
| `tfl` | Core library. No ROS dependencies. Mutex-free transform buffer. |
| `tfl_ros` | ROS 2 integration. Subscribes to `/tf` and `/tf_static`, feeds into `tfl::TransformBuffer`. |

## Prerequisites

- Docker

`run.sh` wraps `docker run` with `--user $(id -u):$(id -g)` so that build artifacts are owned by your user, not root. On first run it builds the Docker image automatically.

## Setup

```bash
# Create colcon workspace with symlinks
mkdir -p ws/src
ln -s ../../tfl ws/src/tfl
ln -s ../../tfl_ros ws/src/tfl_ros  # optional
```

## Build

```bash
# tfl only
./run.sh bash -c \
  "source /opt/ros/\$ROS_DISTRO/setup.bash \
   && cd ws \
   && colcon build --packages-select tfl"

# tfl + tfl_ros
./run.sh bash -c \
  "source /opt/ros/\$ROS_DISTRO/setup.bash \
   && cd ws \
   && colcon build --packages-select tfl tfl_ros"
```

## Test

```bash
./run.sh bash -c \
  "source /opt/ros/\$ROS_DISTRO/setup.bash \
   && cd ws \
   && colcon test --packages-select tfl \
   && colcon test-result --verbose"
```

## Benchmark

```bash
./run.sh bash -c \
  "source /opt/ros/\$ROS_DISTRO/setup.bash \
   && cd ws \
   && source install/setup.bash \
   && ./build/tfl/bench_lookup"
```

For tf2 comparison benchmarks, see [benchmark/README.md](benchmark/README.md).

## Design

### Threading model

```
Writer thread (single)          Reader threads (many)
  TransformListener               lookup_transform()
    /tf callback ──┐               can_transform()
    /tf_static ────┤                    │
                   ▼                    ▼
              set_transform()     SeqLock read (retry on conflict)
                   │                    │
                   ▼                    ▼
              FrameTransformBuffer (circular buffer per frame)
              FrameMap (wait-free hash map)
```

### Components

| Class | Role | Read | Write | Synchronization |
|-------|------|------|-------|-----------------|
| `FrameMap` | Frame name → ID mapping | Wait-free | Single-writer | Atomic hash slots, open addressing |
| `FrameTransformBuffer` | Per-frame transform history | Obstruction-free (SeqLock) | Single-writer | SeqLock (sequence counter) |
| `TransformBuffer` | Frame tree, LCA traversal | Mutex-free (delegates to above) | Single-writer | Composes FrameMap + FrameTransformBuffer |
| `TransformListener` | ROS 2 `/tf` subscriber | — | Dedicated thread | SingleThreadedExecutor + MutuallyExclusive callback group |

- **Wait-free**: Reads complete in bounded steps regardless of other threads.
- **Obstruction-free**: Reads complete in bounded steps if no write is in progress. Under contention, retries up to 64 times.
- **Single-writer**: All writes (`set_transform`) must come from one thread. `TransformListener` enforces this with a dedicated executor thread.

## Requirements

- Docker (for building via `run.sh`)
- Or: C++17, CMake 3.14+, ROS 2 Jazzy or Rolling

## License

Apache License 2.0
