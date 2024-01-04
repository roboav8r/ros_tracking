cmake_minimum_required(VERSION 3.8)
project(ros_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tracking_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(include) # needed for include "ros_tracking/tracking_datatypes.hpp"
include_directories(${GTSAM_INCLUDE_DIR})
# include_directories(${EIGEN3_INCLUDE_DIR})

# add_executable(ros_tracker src/ros_tracker.cpp)
# ament_target_dependencies(ros_tracker rclcpp foxglove_msgs tracking_msgs GTSAM Eigen3) # GTSAM Eigen3 gtsam
# target_link_libraries(ros_tracker gtsam)

add_executable(det_converter src/det_converter.cpp)
ament_target_dependencies(det_converter rclcpp foxglove_msgs tracking_msgs)
add_executable(depthai_converter src/depthai_converter.cpp)
ament_target_dependencies(depthai_converter rclcpp vision_msgs geometry_msgs tracking_msgs tf2 tf2_ros tf2_geometry_msgs)


# target_compile_features(ros_tracker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Install CPP
install(TARGETS 
  # ros_tracker
  det_converter
  depthai_converter
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install Python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/py_tracker.py
  scripts/nuscenes_to_mcap.py
  scripts/compute_nuscenes_results.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
