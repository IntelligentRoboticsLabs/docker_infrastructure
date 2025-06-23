cmake_minimum_required(VERSION 3.8)
project(subscriber_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(subscriber_example src/subscriber_node.cpp)
ament_target_dependencies(subscriber_example
  rclcpp
  std_msgs
)

install(TARGETS subscriber_example
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
