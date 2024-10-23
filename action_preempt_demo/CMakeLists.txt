cmake_minimum_required(VERSION 3.8)
project(action_preempt_demo)

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
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(example_interfaces REQUIRED)

include_directories(
  include
  process_requests_node
)

set(dependencies
  rclcpp
  std_msgs
  rclcpp_action
  nav2_util
  example_interfaces
)

add_executable(action_preempt_server src/action_preempt_server.cpp)
ament_target_dependencies(action_preempt_server
  ${dependencies}
)

add_executable(action_preempt_client src/action_preempt_client.cpp)
ament_target_dependencies(action_preempt_client
  ${dependencies}
)

install(TARGETS
  action_preempt_client
  action_preempt_server
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
