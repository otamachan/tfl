# tf2 / tfl Benchmark

Performance benchmark comparing ROS 2 tf2 `BufferCore` and tfl `TransformBuffer`.
Measures single-thread performance and multi-thread contention.

## How to Run

### Prerequisites

- Docker installed

### Build

```bash
./run.sh bash -c "source /opt/ros/jazzy/setup.bash && colcon build \
  --build-base docker-build --install-base docker-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --packages-up-to benchmark_test"
```

### Run

```bash
# tf2
./run.sh bash -c "source /opt/ros/jazzy/setup.bash && \
  source docker-install/setup.bash && \
  docker-install/benchmark_test/lib/benchmark_test/bench_tf2_buffer_core"

# tfl
./run.sh bash -c "source /opt/ros/jazzy/setup.bash && \
  source docker-install/setup.bash && \
  docker-install/benchmark_test/lib/benchmark_test/bench_tfl_buffer_core"
```

### Clean Build

```bash
rm -rf benchmark/ws/docker-build benchmark/ws/docker-install
```

## Test Cases

### Single-thread

| Test | Description |
|------|-------------|
| setTransform | Write throughput over 100k calls |
| lookupTransform identity | Same frame (shortcut path) |
| lookupTransform chain=5/10 | Sequential 5/10-hop tree traversal |
| lookupTransform cross-branch | Cross-branch lookup in a 4-branch tree |
| lookupTransform interp | With time interpolation (SLERP) |
| canTransform chain=10 | Transform availability check |
| lookupTransform static | Static transforms only |

### Multi-thread

| Test | Description |
|------|-------------|
| Reader Scalability | Throughput with 1/2/4/8 readers (no writer) |
| 1 Writer + N Readers | Writer at 100/1k/10kHz x 1/2/4/8 readers |
| Latency Distribution | 1 Writer + 1 Reader measuring p50/p90/p99/p99.9/max |

## Results (tf2)

### Environment

| Item | Value |
|------|-------|
| CPU | Intel Core i9-14900K (24C/32T) |
| RAM | 64 GB |
| OS (host) | Ubuntu 24.04.1 LTS / kernel 6.14.0-37-generic |
| OS (container) | Ubuntu 24.04.3 LTS |
| ROS | Jazzy |
| tf2 | geometry2 0.45.6 (submodule, source build) |
| Build | Release (-O3) |

### Single-thread

```
Test                                               Iters    Total(us) Per-call(ns)
----                                               -----    --------- ------------
setTransform (100k calls)                         100000      15084.6        150.8
lookupTransform identity (1M)                    1000000      35737.7         35.7
lookupTransform chain=5 (500k)                    500000     102345.3        204.7
lookupTransform chain=10 (500k)                   500000     158872.4        317.7
lookupTransform cross-branch (500k)               500000     165545.4        331.1
lookupTransform interp (500k)                     500000      95114.8        190.2
canTransform chain=10 (1M)                       1000000     211918.8        211.9
lookupTransform static chain=10 (500k)            500000     162956.3        325.9
V-tree lookup Time(0) latest (1M)                1000000     322561.5        322.6
V-tree lookup Time(1) exact (1M)                 1000000     195274.6        195.3
V-tree lookup Time(1.5) interp (1M)              1000000     275989.4        276.0
V-tree lookup Time(2) exact (1M)                 1000000     193896.1        193.9
V-tree canTransform Time(0) (1M)                 1000000     222334.8        222.3
V-tree canTransform Time(1.5) (1M)               1000000     102421.8        102.4
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                 1813527        1813527        551.4
2                 1141904         570952       1751.5
4                  827098         206775       4836.2
8                  421974          52747      18958.5
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1705530        1705530        586.3          990
2                 1155967         577983       1730.2          990
4                  825281         206320       4846.8          990
8                  555678          69460      14396.8          960
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1774553        1774553        563.5         9040
2                 1096664         548332       1823.7         8930
4                  800859         200215       4994.6         8600
8                  277623          34703      28816.1         6039
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1559969        1559969        641.0        55698
2                 1005731         502865       1988.6        48784
4                  784963         196241       5095.8        42510
8                  612154          76519      13068.6        22018
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:          3392904    3292607    3105749
Mean:             572.5 ns   590.2 ns   626.7 ns
p50:              557 ns     560 ns     560 ns
p90:              594 ns     609 ns     612 ns
p99:              766 ns     881 ns    1879 ns
p99.9:           1324 ns    4184 ns    6725 ns
Max:           132337 ns   152704 ns   51836 ns
```

## Results (tfl)

### Single-thread

```
Test                                               Iters    Total(us) Per-call(ns)
----                                               -----    --------- ------------
setTransform (100k calls)                         100000       5710.2         57.1
lookupTransform identity (1M)                    1000000      20875.0         20.9
lookupTransform chain=5 (500k)                    500000      23458.1         46.9
lookupTransform chain=10 (500k)                   500000      38599.3         77.2
lookupTransform cross-branch (500k)               500000      21700.5         43.4
lookupTransform interp (500k)                     500000      50513.7        101.0
canTransform chain=10 (1M)                       1000000      82484.4         82.5
lookupTransform static chain=10 (500k)            500000      34911.4         69.8
V-tree lookup Time(0) latest (1M)                1000000      40886.5         40.9
V-tree lookup Time(1) exact (1M)                 1000000     235956.4        236.0
V-tree lookup Time(1.5) interp (1M)              1000000     304238.8        304.2
V-tree lookup Time(2) exact (1M)                 1000000     226451.6        226.5
V-tree canTransform Time(0) (1M)                 1000000      41042.5         41.0
V-tree canTransform Time(1.5) (1M)               1000000     298100.2        298.1
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                12227045       12227045         81.8
2                24493677       12246839         81.7
4                47182093       11795523         84.8
8                94099950       11762494         85.0
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                12903366       12903366         77.5         1000
2                25876646       12938323         77.3          990
4                51646934       12911733         77.4          990
8               102486149       12810769         78.1         1000
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                12338933       12338933         81.0         9459
2                24644257       12322129         81.2         9430
4                49295531       12323883         81.1         9440
8                99652599       12456575         80.3         9459
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                11646225       11646225         85.9        65195
2                23522620       11761310         85.0        65376
4                47237938       11809484         84.7        65150
8                96666792       12083349         82.8        64897
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:         17126620   17086898   16885721
Mean:              98.3 ns    98.7 ns    99.8 ns
p50:              100 ns     100 ns     101 ns
p90:              102 ns     102 ns     103 ns
p99:              104 ns     109 ns     115 ns
p99.9:            115 ns     192 ns     334 ns
Max:            50564 ns    29345 ns   34314 ns
```

## Comparison (tf2 vs tfl)

### Single-thread

| Test | tf2 (ns) | tfl (ns) | Speedup |
|------|----------|----------|---------|
| setTransform | 150.8 | 57.1 | 2.6x |
| lookupTransform identity | 35.7 | 20.9 | 1.7x |
| lookupTransform chain=5 | 204.7 | 46.9 | 4.4x |
| lookupTransform chain=10 | 317.7 | 77.2 | 4.1x |
| lookupTransform cross-branch | 331.1 | 43.4 | 7.6x |
| lookupTransform interp | 190.2 | 101.0 | 1.9x |
| canTransform chain=10 | 211.9 | 82.5 | 2.6x |
| lookupTransform static chain=10 | 325.9 | 69.8 | 4.7x |

### Single-thread: V-tree (cross-branch, 10 hops)

| Test | tf2 (ns) | tfl (ns) | Speedup |
|------|----------|----------|---------|
| V-tree Time(0) latest | 322.6 | 40.9 | 7.9x |
| V-tree Time(1) exact | 195.3 | 236.0 | 0.8x |
| V-tree Time(1.5) interp | 276.0 | 304.2 | 0.9x |
| V-tree Time(2) exact | 193.9 | 226.5 | 0.9x |
| V-tree canTransform Time(0) | 222.3 | 41.0 | 5.4x |
| V-tree canTransform Time(1.5) | 102.4 | 298.1 | 0.3x |

### Multi-thread: Reader Scalability (per-thread ops/s, no writer)

| Readers | tf2 | tfl | Speedup |
|---------|-----|-----|---------|
| 1 | 1.8M | 12.2M | 6.7x |
| 2 | 571k | 12.2M | 21x |
| 4 | 207k | 11.8M | 57x |
| 8 | 53k | 11.8M | 223x |

### Multi-thread: Latency p50 (1 Writer + 1 Reader)

| Writer freq | tf2 | tfl | Speedup |
|-------------|-----|-----|---------|
| 100Hz | 557 ns | 100 ns | 5.6x |
| 1kHz | 560 ns | 100 ns | 5.6x |
| 10kHz | 560 ns | 101 ns | 5.5x |
