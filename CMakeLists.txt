cmake_minimum_required(VERSION 3.16)
project(hanmole_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hanmole_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(target_repository src/target_repository.cpp)
target_include_directories(target_repository PUBLIC include)
target_link_libraries(target_repository yaml-cpp)

set(navigation_node_dependencies
  action_msgs
  geometry_msgs
  hanmole_msgs
  lifecycle_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

add_executable(target_catalog_publisher_node src/target_catalog_publisher_node.cpp)
target_include_directories(target_catalog_publisher_node PRIVATE include)
target_link_libraries(target_catalog_publisher_node target_repository)
ament_target_dependencies(target_catalog_publisher_node rclcpp std_msgs)

add_executable(nav2_target_gateway_node src/nav2_target_gateway_node.cpp)
target_include_directories(nav2_target_gateway_node PRIVATE include)
target_link_libraries(nav2_target_gateway_node target_repository)
ament_target_dependencies(nav2_target_gateway_node ${navigation_node_dependencies})

set(navigation_share_dirs
  launch
  config
  maps
)

# behavior_trees 目录存在时作为 Nav2 行为树资产一起安装。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  list(APPEND navigation_share_dirs behavior_trees)
endif()

install(
  DIRECTORY ${navigation_share_dirs}
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/random_target_nav_loop.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    target_repository
    target_catalog_publisher_node
    nav2_target_gateway_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
  ament_add_pytest_test(
    test_target_gateway_contract
    test/test_target_gateway_contract.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
  )
endif()

ament_package()
