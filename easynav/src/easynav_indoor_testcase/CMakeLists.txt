cmake_minimum_required(VERSION 3.20)
project(easynav_indoor_testcase)

find_package(ament_cmake REQUIRED)

install(DIRECTORY maps robots_params launch DESTINATION share/${PROJECT_NAME})

ament_package()
