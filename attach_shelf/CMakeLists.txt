cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs
)

add_executable(pre_approach src/pre_approach.cpp)
ament_target_dependencies(pre_approach rclcpp sensor_msgs geometry_msgs nav_msgs tf2)

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_eigen)
rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2)
rosidl_target_interfaces(pre_approach_v2
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  TARGETS pre_approach approach_service_server pre_approach_v2
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
