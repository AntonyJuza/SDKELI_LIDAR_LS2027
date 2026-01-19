cmake_minimum_required(VERSION 3.8)
project(sdkeli_ls_udp)

# -----------------------------
# Compiler settings
# -----------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# -----------------------------
# Find dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# System dependencies
find_package(Boost REQUIRED)

# -----------------------------
# Include directories
# -----------------------------
include_directories(
  include/sdkeli_ls_udp
)

# -----------------------------
# Driver library (core logic)
# -----------------------------
add_library(sdkeli_ls_core
  src/sdkeli_ls_common.cpp
  src/sdkeli_ls_common_udp.cpp
  src/sdkeli_ls1207de_parser.cpp
  src/sdkeli_ls_sensor_frame.cpp
)

ament_target_dependencies(sdkeli_ls_core
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(sdkeli_ls_core
  Boost::boost
)

# -----------------------------
# Executable node
# -----------------------------
add_executable(sdkeli_ls1207de
  src/sdkeli_ls1207de.cpp
)

target_link_libraries(sdkeli_ls1207de
  sdkeli_ls_core
)

ament_target_dependencies(sdkeli_ls1207de
  rclcpp
  sensor_msgs
  std_msgs
)

# -----------------------------
# Install
# -----------------------------
install(
  TARGETS sdkeli_ls_core sdkeli_ls1207de
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# -----------------------------
# Export package
# -----------------------------
ament_export_include_directories(include)
ament_export_libraries(sdkeli_ls_core)
ament_export_dependencies(rclcpp sensor_msgs std_msgs)

ament_package()
