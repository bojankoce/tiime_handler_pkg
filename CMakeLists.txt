cmake_minimum_required(VERSION 3.5)
#cmake_policy(SET CMP0076 NEW)
project(time_handler_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#file(GLOB SOURCES "src/*.cpp" "src/rtc_node/*.cpp")
# TODO(bojankoce): It's not a good practice to USE GLOB
# Research better ways to organize and include source code - https://crascit.com/2016/01/31/enhanced-source-file-handling-with-target_sources/
# https://cliutils.gitlab.io/modern-cmake/

# TODO(bojankoce): Research if there is a better way than to manually add all the source files 
set(RTC_NODE_SOURCES 
  src/rtc_node_wrapper.cpp
  src/rtc_node/time_handler.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces_pkg REQUIRED)

add_executable(rtc_node ${RTC_NODE_SOURCES})
target_include_directories(rtc_node PRIVATE src/rtc_node)
ament_target_dependencies(rtc_node rclcpp custom_interfaces_pkg)


# This part is needed so ros2 run can find our executable
install(TARGETS 
  rtc_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
