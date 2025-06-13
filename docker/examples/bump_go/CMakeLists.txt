cmake_minimum_required(VERSION 3.8)
project(bump_go)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(bump_go src/bump_go.cpp)
ament_target_dependencies(bump_go
  rclcpp
  sensor_msgs
  geometry_msgs
)

install(TARGETS bump_go
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
