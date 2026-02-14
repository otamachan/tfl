cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0079 NEW)
project(tfl_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tfl REQUIRED)

add_library(tfl_ros SHARED src/transform_listener.cpp)
target_include_directories(tfl_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tfl_ros PUBLIC
  ${tf2_msgs_TARGETS}
  rclcpp::rclcpp
  tfl::tfl
)

install(TARGETS tfl_ros
  EXPORT tfl_rosTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(tfl_rosTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2_msgs tfl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_transform_listener test/test_transform_listener.cpp)
  target_link_libraries(test_transform_listener
    tfl_ros
    ${tf2_msgs_TARGETS}
    rclcpp::rclcpp
    tfl::tfl
  )

  ament_add_gtest(test_construct_and_destruct test/test_construct_and_destruct.cpp)
  target_link_libraries(test_construct_and_destruct
    tfl_ros
    ${tf2_msgs_TARGETS}
    rclcpp::rclcpp
    tfl::tfl
  )
  # Disabled: spin()+cancel() may deadlock on SingleThreadedExecutor teardown
  # when subscriptions are present (observed on jazzy and rolling).
  set_tests_properties(test_construct_and_destruct PROPERTIES DISABLED true)
endif()

ament_package()
