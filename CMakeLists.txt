cmake_minimum_required(VERSION 3.5)

project(dienen_gazebo_plugins)

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

set(DEPENDENCIES gazebo_ros keisan rclcpp tosshin)

foreach(DEPENDENCY ${DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

add_library(dienen_navigation_plugin SHARED
  "src/navigation_plugin.cpp")

target_include_directories(dienen_navigation_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dienen_navigation_plugin ${DEPENDENCIES})

install(TARGETS dienen_navigation_plugin
  DESTINATION "share/${PROJECT_NAME}/plugins")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
