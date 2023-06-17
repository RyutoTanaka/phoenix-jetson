cmake_minimum_required(VERSION 3.10)
project(phoenix_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include_directories  (src/.)
include_directories  (include/.)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wno-unused-parameter)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

if($ENV{ROS_DISTRO} MATCHES "dashing")
  add_definitions(-DKIKS_ROS_DISTRO_DASHING)
elseif($ENV{ROS_DISTRO} MATCHES "foxy")
  add_definitions(-DKIKS_ROS_DISTRO_FOXY)
elseif($ENV{ROS_DISTRO} MATCHES "humble")
  add_definitions(-DKIKS_ROS_DISTRO_HUMBLE)
else()
  message(FATAL_ERROR "unsupported ros distro")
endif()

add_executable(send_command
  src/send_command.cpp)
ament_target_dependencies(send_command rclcpp geometry_msgs)

install(TARGETS
  send_command
  DESTINATION lib/${PROJECT_NAME})

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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