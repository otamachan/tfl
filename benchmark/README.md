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
setTransform (100k calls)                         100000      18866.6        188.7
lookupTransform identity (1M)                    1000000      38069.1         38.1
lookupTransform chain=5 (500k)                    500000     101472.8        202.9
lookupTransform chain=10 (500k)                   500000     162521.8        325.0
lookupTransform cross-branch (500k)               500000     165925.6        331.9
lookupTransform interp (500k)                     500000      94890.8        189.8
canTransform chain=10 (1M)                       1000000     217480.9        217.5
lookupTransform static chain=10 (500k)            500000     155966.6        311.9
V-tree lookup Time(0) latest (1M)                1000000     324230.9        324.2
V-tree lookup Time(1) exact (1M)                 1000000     197718.9        197.7
V-tree lookup Time(1.5) interp (1M)              1000000     282431.2        282.4
V-tree lookup Time(2) exact (1M)                 1000000     194623.8        194.6
V-tree canTransform Time(0) (1M)                 1000000     222324.2        222.3
V-tree canTransform Time(1.5) (1M)               1000000     102002.6        102.0
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                 1676801        1676801        596.4
2                 1176801         588400       1699.5
4                  805070         201267       4968.5
8                  442510          55314      18078.7
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1797084        1797084        556.5          990
2                 1121869         560934       1782.7          990
4                  835330         208833       4788.5          990
8                  338211          42276      23653.9          940
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1738308        1738308        575.3         9120
2                 1111755         555878       1799.0         8841
4                  815506         203877       4904.9         8660
8                  291660          36458      27429.2         5909
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1640181        1640181        609.7        55552
2                  995752         497876       2008.5        46751
4                  795321         198830       5029.4        41311
8                  273988          34249      29198.3        13048
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:          3368331    3333288    3057323
Mean:             576.7 ns   583.1 ns   636.7 ns
p50:              557 ns     558 ns     572 ns
p90:              598 ns     599 ns     627 ns
p99:              764 ns     832 ns    1876 ns
p99.9:           1998 ns    4187 ns    7196 ns
Max:           123208 ns   128754 ns  584838 ns
```

## Results (tfl)

### Single-thread

```
Test                                               Iters    Total(us) Per-call(ns)
----                                               -----    --------- ------------
setTransform (100k calls)                         100000       3792.9         37.9
lookupTransform identity (1M)                    1000000      20948.2         20.9
lookupTransform chain=5 (500k)                    500000      60717.4        121.4
lookupTransform chain=10 (500k)                   500000     113714.5        227.4
lookupTransform cross-branch (500k)               500000     125488.1        251.0
lookupTransform interp (500k)                     500000      37394.8         74.8
canTransform chain=10 (1M)                       1000000     115500.5        115.5
lookupTransform static chain=10 (500k)            500000      45116.0         90.2
V-tree lookup Time(0) latest (1M)                1000000     235997.8        236.0
V-tree lookup Time(1) exact (1M)                 1000000     185819.9        185.8
V-tree lookup Time(1.5) interp (1M)              1000000     267801.6        267.8
V-tree lookup Time(2) exact (1M)                 1000000     184817.0        184.8
V-tree canTransform Time(0) (1M)                 1000000     109480.8        109.5
V-tree canTransform Time(1.5) (1M)               1000000      49193.8         49.2
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                 4392685        4392685        227.7
2                 8692769        4346385        230.1
4                16944556        4236139        236.1
8                30148234        3768529        265.4
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 4361729        4361729        229.3          990
2                 8638286        4319143        231.5          990
4                17329075        4332269        230.8          990
8                34667508        4333439        230.8         1000
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 4294572        4294572        232.9         9449
2                 8618758        4309379        232.1         9459
4                17372733        4343183        230.2         9440
8                34629014        4328627        231.0         9450
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 4294471        4294471        232.9        65570
2                 8654597        4327298        231.1        65072
4                17234558        4308639        232.1        64947
8                34425937        4303242        232.4        64940
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:          7045956    7077089    7008793
Mean:             267.0 ns   265.8 ns   268.5 ns
p50:              264 ns     263 ns     265 ns
p90:              270 ns     267 ns     271 ns
p99:              305 ns     324 ns     334 ns
p99.9:            436 ns     511 ns     937 ns
Max:          1047840 ns    31200 ns   53519 ns
```

## Comparison (tf2 vs tfl)

### Single-thread

| Test | tf2 (ns) | tfl (ns) | Speedup |
|------|----------|----------|---------|
| setTransform | 188.7 | 37.9 | 5.0x |
| lookupTransform identity | 38.1 | 20.9 | 1.8x |
| lookupTransform chain=5 | 202.9 | 121.4 | 1.7x |
| lookupTransform chain=10 | 325.0 | 227.4 | 1.4x |
| lookupTransform cross-branch | 331.9 | 251.0 | 1.3x |
| lookupTransform interp | 189.8 | 74.8 | 2.5x |
| canTransform chain=10 | 217.5 | 115.5 | 1.9x |
| lookupTransform static chain=10 | 311.9 | 90.2 | 3.5x |

### Single-thread: V-tree (cross-branch, 10 hops)

| Test | tf2 (ns) | tfl (ns) | Speedup |
|------|----------|----------|---------|
| V-tree Time(0) latest | 324.2 | 236.0 | 1.4x |
| V-tree Time(1) exact | 197.7 | 185.8 | 1.1x |
| V-tree Time(1.5) interp | 282.4 | 267.8 | 1.1x |
| V-tree Time(2) exact | 194.6 | 184.8 | 1.1x |
| V-tree canTransform Time(0) | 222.3 | 109.5 | 2.0x |
| V-tree canTransform Time(1.5) | 102.0 | 49.2 | 2.1x |

### Multi-thread: Reader Scalability (per-thread ops/s, no writer)

| Readers | tf2 | tfl | Speedup |
|---------|-----|-----|---------|
| 1 | 1.7M | 4.4M | 2.6x |
| 2 | 588k | 4.3M | 7.4x |
| 4 | 201k | 4.2M | 21.1x |
| 8 | 55k | 3.8M | 68.1x |

### Multi-thread: Latency p50 (1 Writer + 1 Reader)

| Writer freq | tf2 | tfl | Speedup |
|-------------|-----|-----|---------|
| 100Hz | 557 ns | 264 ns | 2.1x |
| 1kHz | 558 ns | 263 ns | 2.1x |
| 10kHz | 572 ns | 265 ns | 2.2x |
