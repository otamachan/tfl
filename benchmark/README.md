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
setTransform (100k calls)                         100000      17342.3        173.4
lookupTransform identity (1M)                    1000000      34377.1         34.4
lookupTransform chain=5 (500k)                    500000     102359.2        204.7
lookupTransform chain=10 (500k)                   500000     166270.3        332.5
lookupTransform cross-branch (500k)               500000     166470.8        332.9
lookupTransform interp (500k)                     500000      95845.2        191.7
canTransform chain=10 (1M)                       1000000     216442.7        216.4
lookupTransform static chain=10 (500k)            500000     168445.1        336.9
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                 1824174        1824174        548.2
2                 1205336         602668       1659.3
4                  819328         204832       4882.1
8                  490176          61272      16320.7
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1857854        1857854        538.3          980
2                 1109729         554864       1802.2          980
4                  748046         187012       5347.3          990
8                  335760          41970      23826.6          930
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1738550        1738550        575.2         8800
2                 1100405         550203       1817.5         8890
4                  772775         193194       5176.2         8579
8                  349021          43628      22921.2         6329
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1661070        1661070        602.0        55887
2                 1009407         504703       1981.4        50489
4                  753979         188495       5305.2        40049
8                  273823          34228      29216.0        12579
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:          3454550    3365419    3212606
Mean:             561.9 ns   577.3 ns   605.2 ns
p50:              545 ns     557 ns     545 ns
p90:              572 ns     586 ns     581 ns
p99:              860 ns     830 ns    1831 ns
p99.9:           1863 ns    4363 ns    6438 ns
Max:           121417 ns   135041 ns   33658 ns
```

## Results (std::shared_mutex)

Replaced `frame_mutex_` with `std::shared_mutex` and used `shared_lock` for read-only operations.

### Single-thread

```
Test                                               Iters    Total(us) Per-call(ns)
----                                               -----    --------- ------------
setTransform (100k calls)                         100000      12421.8        124.2
lookupTransform identity (1M)                    1000000      36021.7         36.0
lookupTransform chain=5 (500k)                    500000     104041.6        208.1
lookupTransform chain=10 (500k)                   500000     162139.5        324.3
lookupTransform cross-branch (500k)               500000     162872.6        325.7
lookupTransform interp (500k)                     500000      96514.6        193.0
canTransform chain=10 (1M)                       1000000     214071.7        214.1
lookupTransform static chain=10 (500k)            500000     165587.0        331.2
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                 1836893        1836893        544.4
2                 1757257         878628       1138.1
4                 1781743         445436       2245.0
8                 2063316         257915       3877.3
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1843679        1843679        542.4          990
2                 1781190         890595       1122.8          720
4                 1923894         480974       2079.1           80
8                 2202534         275317       3632.2           10
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1714483        1714483        583.3         8970
2                 1781920         890960       1122.4          440
4                 1866633         466658       2142.9          110
8                 2081105         260138       3844.1           10
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                 1414291        1414291        707.1        50098
2                 1520448         760224       1315.4        13190
4                 1963969         490992       2036.7           70
8                 2194991         274374       3644.7           10
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:          3485996    3270014    2637771
Mean:             556.9 ns   594.6 ns   740.5 ns
p50:              544 ns     557 ns     560 ns
p90:              565 ns     585 ns     607 ns
p99:              695 ns     781 ns    5338 ns
p99.9:           2782 ns    6876 ns    7352 ns
Max:           128282 ns   139314 ns  536796 ns
```

## Results (tfl)

### Single-thread

```
Test                                               Iters    Total(us) Per-call(ns)
----                                               -----    --------- ------------
setTransform (100k calls)                         100000       4162.6         41.6
lookupTransform identity (1M)                    1000000      25538.2         25.5
lookupTransform chain=5 (500k)                    500000      23328.4         46.7
lookupTransform chain=10 (500k)                   500000      38792.0         77.6
lookupTransform cross-branch (500k)               500000      21914.6         43.8
lookupTransform interp (500k)                     500000      47172.3         94.3
canTransform chain=10 (1M)                       1000000      89350.6         89.4
lookupTransform static chain=10 (500k)            500000      37377.2         74.8
V-tree lookup Time(0) latest (1M)                1000000      41768.1         41.8
V-tree lookup Time(1) exact (1M)                 1000000     102723.2        102.7
V-tree lookup Time(1.5) interp (1M)              1000000     127092.7        127.1
V-tree lookup Time(2) exact (1M)                 1000000     105220.5        105.2
V-tree canTransform Time(0) (1M)                 1000000      41520.9         41.5
V-tree canTransform Time(1.5) (1M)               1000000     127176.8        127.2
```

### Multi-thread: Reader Scalability (no writer)

```
Readers      Total(ops/s) Per-thr(ops/s) Per-call(ns)
-------      ------------ -------------- ------------
1                12962416       12962416         77.1
2                21644080       10822040         92.4
4                39741062        9935266        100.7
8                82554134       10319267         96.9
```

### Multi-thread: 1 Writer + N Readers

**Writer 100Hz (typical robot odometry)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                12540499       12540499         79.7          990
2                25245534       12622767         79.2          990
4                48247053       12061763         82.9         1000
8                97327655       12165957         82.2         1000
```

**Writer 1kHz**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                12206674       12206674         81.9         9450
2                23382747       11691373         85.5         9459
4                43973839       10993460         91.0         9479
8                94685935       11835742         84.5         9460
```

**Writer 10kHz (high-frequency sensor)**
```
Readers       Read(ops/s) Per-thr(ops/s) Per-call(ns) Write(ops/s)
-------       ----------- -------------- ------------ ------------
1                11492802       11492802         87.0        65206
2                23620057       11810028         84.7        65017
4                43807519       10951880         91.3        64996
8                94380486       11797561         84.8        64976
```

### Multi-thread: Latency Distribution (1 Writer + 1 Reader)

```
                    100Hz       1kHz      10kHz
Samples:         18029849   17987105   17656756
Mean:              92.7 ns    92.9 ns    94.7 ns
p50:               91 ns      91 ns      91 ns
p90:              101 ns     101 ns     102 ns
p99:              104 ns     104 ns     120 ns
p99.9:            128 ns     186 ns     333 ns
Max:           135483 ns   91113 ns  243284 ns
```
