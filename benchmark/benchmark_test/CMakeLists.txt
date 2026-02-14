cmake_minimum_required(VERSION 3.14)
project(benchmark_test)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tfl REQUIRED)

add_executable(bench_tf2_buffer_core src/bench_tf2_buffer_core.cpp)
ament_target_dependencies(bench_tf2_buffer_core
  tf2
  geometry_msgs
)

add_executable(bench_tfl_buffer_core src/bench_tfl_buffer_core.cpp)
ament_target_dependencies(bench_tfl_buffer_core
  tfl
)

install(TARGETS
  bench_tf2_buffer_core
  bench_tfl_buffer_core
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
