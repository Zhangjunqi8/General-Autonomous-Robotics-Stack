cmake_minimum_required(VERSION 3.16)
project(hanmole_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hanmole_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
set(navigation_share_dirs
  launch
  config
  maps
  rviz
)

# 当前包作为 Nav2 资产与 launch 装配包使用，暂不构建本地可执行节点。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  list(APPEND navigation_share_dirs behavior_trees)
endif()


add_executable(route_follower_node src/route_follower_node.cpp)
ament_target_dependencies(route_follower_node
  rclcpp
  rclcpp_action
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  hanmole_msgs
)
target_link_libraries(route_follower_node yaml-cpp)


add_executable(route_recorder_node src/route_recorder_node.cpp)
ament_target_dependencies(route_recorder_node
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  hanmole_msgs
)
target_link_libraries(route_recorder_node yaml-cpp)

install(
  TARGETS route_follower_node route_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${navigation_share_dirs}
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/wheel_odom_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(
    test_bringup_launch
    test/test_bringup_launch.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )
  ament_add_pytest_test(
    test_nav2_config
    test/test_nav2_config.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )
endif()

ament_package()
