cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

# Create executable
add_executable(fast_calib src/main.cpp)

# Link libraries
ament_target_dependencies(fast_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
  cv_bridge
)

target_link_libraries(fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Multi-scene calibration executable
add_executable(multi_fast_calib src/multi_scene.cpp)

ament_target_dependencies(multi_fast_calib
  rclcpp
)

target_link_libraries(multi_fast_calib
  Eigen3::Eigen
)

# Live calibration executable
add_executable(live_fast_calib src/live_calib.cpp)

ament_target_dependencies(live_fast_calib
  rclcpp
  sensor_msgs
  pcl_conversions
  cv_bridge
  message_filters
)

target_link_libraries(live_fast_calib
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

# Install
install(TARGETS
  fast_calib
  multi_fast_calib
  live_fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()