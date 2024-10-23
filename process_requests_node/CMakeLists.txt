cmake_minimum_required(VERSION 3.8)
project(process_requests_node)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIRD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty)

include_directories(
  include
  ${THIRD_PATH}/threadpool
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(process_requests_node src/main.cpp)
ament_target_dependencies(process_requests_node rclcpp std_msgs example_interfaces)

add_executable(add_ints_server_one src/add_ints_server_one.cpp)
ament_target_dependencies(add_ints_server_one rclcpp example_interfaces)

add_executable(add_ints_server_two src/add_ints_server_two.cpp)
ament_target_dependencies(add_ints_server_two rclcpp example_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS 
  process_requests_node
  add_ints_server_one
  add_ints_server_two
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
