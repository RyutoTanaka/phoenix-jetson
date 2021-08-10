cmake_minimum_required(VERSION 3.5)
project(phoenix)

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

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add include path
include_directories(${PROJECT_SOURCE_DIR}/../../phoenix-firmware/FPGA/App/software/controller/include)

# battery
set(battery_SOURCES
  src/battery_node.cpp
  src/i2c.cpp
  src/ads1015.cpp
)
ament_auto_add_executable(battery ${battery_SOURCES})
target_compile_definitions(battery PUBLIC PHOENIX_BUILD_STANDALONE)

# stream
set(stream_SOURCES
  src/stream_node.cpp
  src/diagnostics.cpp
  src/uart.cpp
  src/avalon_st.cpp
)
ament_auto_add_executable(stream ${stream_SOURCES})
target_compile_definitions(stream PUBLIC PHOENIX_BUILD_STANDALONE)

# stream (library)
#ament_auto_add_library(stream SHARED ${stream_SOURCES})
#target_compile_definitions(stream PUBLIC PHOENIX_BUILD_LIBRARY)
#rclcpp_components_register_nodes(stream "phoenix::StreamPublisherNode")

# command
set(command_SOURCES
  src/command_node.cpp
  src/diagnostics.cpp
  src/gpio.cpp
  src/spi.cpp
  src/avalon_st.cpp
  src/avalon_mm.cpp
  src/epcq.cpp
)
ament_auto_add_executable(command ${command_SOURCES})
target_compile_definitions(command PUBLIC PHOENIX_BUILD_STANDALONE)

# Launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
