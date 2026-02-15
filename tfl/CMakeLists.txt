cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0079 NEW)
project(tfl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

add_library(tfl STATIC src/transform_buffer.cpp)
set_target_properties(tfl PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(tfl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS tfl
  EXPORT tflTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(tflTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frame_cache test/test_frame_cache.cpp)
  target_link_libraries(test_frame_cache ${PROJECT_NAME})

  ament_add_gtest(test_frame_map test/test_frame_map.cpp)
  target_link_libraries(test_frame_map ${PROJECT_NAME})

  ament_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${PROJECT_NAME})

  ament_add_gtest(test_stress test/test_stress.cpp TIMEOUT 30)
  target_link_libraries(test_stress ${PROJECT_NAME})

  add_executable(bench_lookup test/bench_lookup.cpp)
  target_link_libraries(bench_lookup ${PROJECT_NAME})
endif()

ament_package()
