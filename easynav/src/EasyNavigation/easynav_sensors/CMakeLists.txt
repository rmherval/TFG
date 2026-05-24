cmake_minimum_required(VERSION 3.20)
project(easynav_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(easynav_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
)

set(package_dependencies
  rclcpp
  rclcpp_lifecycle
  easynav_common
)

add_library(${PROJECT_NAME} SHARED
  src/easynav_sensors/SensorsNode.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  easynav_common::easynav_common
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(PCL REQUIRED COMPONENTS common io)

  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  easynav_common
  tf2_ros
  sensor_msgs
)
ament_package()
