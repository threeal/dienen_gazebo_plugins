cmake_minimum_required(VERSION 3.5)

project(tosshin_sim)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tosshin_interfaces REQUIRED)

add_library(navigation_plugin SHARED
  "src/tosshin_sim/navigation_plugin.cpp")

target_include_directories(navigation_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(navigation_plugin
  gazebo_ros
  keisan
  rclcpp
  tosshin_interfaces)

install(TARGETS navigation_plugin
  DESTINATION "share/${PROJECT_NAME}/plugins/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
