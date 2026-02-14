ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-ament-cmake-ros \
    ros-${ROS_DISTRO}-ament-cmake-google-benchmark \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-builtin-interfaces \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-rosidl-default-runtime \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rcutils \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    clang-tidy \
    clang-format \
  && rm -rf /var/lib/apt/lists/*
