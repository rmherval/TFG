cmake_minimum_required(VERSION 3.20)
project(easynav_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(EASYNAV_TRACING "Enable Yaets tracing with TRACE_EVENT" ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(yaets REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/easynav_common/types/ImagePerception.cpp
  src/easynav_common/types/PointPerception.cpp
  src/easynav_common/types/IMUPerception.cpp
  src/easynav_common/types/GNSSPerception.cpp
  src/easynav_common/types/DetectionsPerception.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  cv_bridge::cv_bridge
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${vision_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  yaets::yaets
  ${PCL_LIBRARIES}
)

if(EASYNAV_TRACING)
  message(STATUS "Yaets tracing is ENABLED")
  target_compile_definitions(easynav_common PUBLIC EASYNAV_DEBUG_WITH_YAETS)
else()
  message(STATUS "Yaets tracing is DISABLED")
endif()

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
  add_subdirectory(tests)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pcl_conversions
  tf2_ros
  geometry_msgs
  sensor_msgs
  vision_msgs
  nav_msgs
  tf2_geometry_msgs
  yaets
  PCL
  cv_bridge
)
ament_package()
