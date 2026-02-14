cmake_minimum_required(VERSION 3.14)
project(tfl)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

add_library(tfl STATIC src/buffer_core.cpp)
target_include_directories(tfl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS tfl
  EXPORT tflTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(tflTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
