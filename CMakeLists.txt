cmake_minimum_required(VERSION 3.5)
project(synchronizer)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

# ament_export_dependencies(rosidl_default_generators)
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)

set(dependencies
  rclcpp
  message_filters
  sensor_msgs
)

add_executable(synchronizer_node src/synchronizer_node.cpp)
target_include_directories(synchronizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(synchronizer_node
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(test_node src/test_node.cpp)
target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(test_node
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(verification_node src/verification_node.cpp)
target_include_directories(verification_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(verification_node
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

install(TARGETS 
  synchronizer_node
  test_node
  verification_node
  DESTINATION lib/${PROJECT_NAME})


# if(BUILD_TESTING)
#   find_package(rclcpp REQUIRED)
#   find_package(sensor_msgs REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()

# Install launch files.
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
