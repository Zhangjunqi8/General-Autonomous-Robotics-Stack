cmake_minimum_required(VERSION 3.16)
project(hanmole_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
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
