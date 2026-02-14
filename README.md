# tfl (tf light)

Mutex-free tf2 alternative for ROS 2. An experimental rewrite of tf2's `BufferCore` that eliminates all mutexes.

## Motivation

tf2's `BufferCore` uses `std::mutex` to protect shared data. This means all concurrent access — including reads — is serialized. tfl is an experimental attempt to remove mutex locks entirely, using SeqLock + wait-free hash map to allow concurrent reads without blocking.

## Benchmark

See [benchmark/README.md](benchmark/README.md).

## Packages

| Package | Description |
|---------|-------------|
| `tfl` | Core library. No ROS dependencies. Mutex-free transform buffer. |
| `tfl_ros` | ROS 2 integration. Subscribes to `/tf` and `/tf_static`, feeds into `tfl::TransformBuffer`. |

## Build

```bash
ROS_DISTRO=jazzy ./run.sh bash -c \
  "cd benchmark/ws \
   && source /opt/ros/\$ROS_DISTRO/setup.bash \
   && colcon build --packages-select tfl tfl_ros"
```

## Example

Coming soon.

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

- C++17
- CMake 3.14+
- ROS 2 Jazzy or Rolling (for tfl_ros; tfl itself has no ROS dependency)

## License

Apache License 2.0
