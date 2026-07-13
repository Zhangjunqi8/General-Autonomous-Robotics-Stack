cmake_minimum_required(VERSION 3.16)
project(hanmole_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED IMPORTED_TARGET gstreamer-video-1.0)
pkg_check_modules(ALSA REQUIRED IMPORTED_TARGET alsa)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hanmole_msgs REQUIRED)
find_package(hanmole_sdk REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(target_repository src/target_repository.cpp)
target_compile_features(target_repository PUBLIC cxx_std_17)
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

add_executable(sdk_stereo_calib_recorder src/sdk_stereo_calib_recorder.cpp)
target_compile_features(sdk_stereo_calib_recorder PRIVATE cxx_std_17)
target_link_libraries(sdk_stereo_calib_recorder
  hanmole_sdk_core
  PkgConfig::GSTREAMER_APP
  PkgConfig::GSTREAMER_VIDEO
)
ament_target_dependencies(sdk_stereo_calib_recorder hanmole_sdk)

set(navigation_install_targets
  target_repository
  target_catalog_publisher_node
  nav2_target_gateway_node
  sdk_stereo_calib_recorder
)

set(HANMOLE_THIRD_PARTY_ROOT "/home/hanmole/third_party" CACHE PATH
  "Root directory for local third-party algorithm libraries")
set(ORBSLAM3_ROOT "${HANMOLE_THIRD_PARTY_ROOT}/ORB_SLAM3" CACHE PATH
  "Local ORB-SLAM3 source/build directory")
set(PANGOLIN_ROOT "${HANMOLE_THIRD_PARTY_ROOT}/Pangolin" CACHE PATH
  "Local Pangolin source directory used by ORB-SLAM3")
set(PANGOLIN_BUILD_ROOT "${HANMOLE_THIRD_PARTY_ROOT}/.build/Pangolin" CACHE PATH
  "Local Pangolin build directory used by ORB-SLAM3")
set(PANGOLIN_INSTALL_ROOT "${HANMOLE_THIRD_PARTY_ROOT}/.install/Pangolin" CACHE PATH
  "Local Pangolin install directory used by ORB-SLAM3")
set(PANGOLIN_LIBRARY_DIRS
  "${PANGOLIN_BUILD_ROOT}/src"
  "${PANGOLIN_INSTALL_ROOT}/lib"
)
set(ORBSLAM3_LIBRARY "${ORBSLAM3_ROOT}/lib/libORB_SLAM3.so")
set(ORBSLAM3_DBOW2_LIBRARY "${ORBSLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so")
set(ORBSLAM3_G2O_LIBRARY "${ORBSLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so")

if(EXISTS "${ORBSLAM3_LIBRARY}" AND
    EXISTS "${ORBSLAM3_DBOW2_LIBRARY}" AND
    EXISTS "${ORBSLAM3_G2O_LIBRARY}")
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS serialization)
  find_package(OpenSSL REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(GLEW REQUIRED)

  find_library(PANGOLIN_LIBRARY
    NAMES pangolin
    PATHS ${PANGOLIN_LIBRARY_DIRS}
    NO_DEFAULT_PATH
  )

  add_executable(orbslam3_stereo_inertial_node src/orbslam3_stereo_inertial_node.cpp)
  target_compile_features(orbslam3_stereo_inertial_node PRIVATE cxx_std_23)
  target_include_directories(orbslam3_stereo_inertial_node PRIVATE
    include
    "${ORBSLAM3_ROOT}"
    "${ORBSLAM3_ROOT}/include"
    "${ORBSLAM3_ROOT}/include/CameraModels"
    "${ORBSLAM3_ROOT}/Thirdparty/Sophus"
    "${PANGOLIN_ROOT}/include"
    "${PANGOLIN_BUILD_ROOT}/src/include"
    "${PANGOLIN_INSTALL_ROOT}/include"
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(orbslam3_stereo_inertial_node
    yaml-cpp
    "${ORBSLAM3_LIBRARY}"
    "${ORBSLAM3_DBOW2_LIBRARY}"
    "${ORBSLAM3_G2O_LIBRARY}"
    hanmole_sdk_core
    PkgConfig::GSTREAMER_APP
    PkgConfig::GSTREAMER_VIDEO
    ${OpenCV_LIBS}
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    Eigen3::Eigen
    Boost::serialization
    OpenSSL::Crypto
  )
  if(PANGOLIN_LIBRARY)
    target_link_libraries(orbslam3_stereo_inertial_node ${PANGOLIN_LIBRARY})
  endif()
  ament_target_dependencies(orbslam3_stereo_inertial_node
    geometry_msgs
    hanmole_sdk
    nav_msgs
    rclcpp
    sensor_msgs
    tf2_ros
  )
  set(ORBSLAM3_VIO_RPATH
    "${ORBSLAM3_ROOT}/lib"
    "${ORBSLAM3_ROOT}/Thirdparty/DBoW2/lib"
    "${ORBSLAM3_ROOT}/Thirdparty/g2o/lib"
    ${PANGOLIN_LIBRARY_DIRS}
  )
  list(JOIN ORBSLAM3_VIO_RPATH ";" ORBSLAM3_VIO_RPATH_VALUE)
  set_target_properties(orbslam3_stereo_inertial_node PROPERTIES
    BUILD_RPATH "${ORBSLAM3_VIO_RPATH_VALUE}"
    INSTALL_RPATH "${ORBSLAM3_VIO_RPATH_VALUE}"
  )
  list(APPEND navigation_install_targets orbslam3_stereo_inertial_node)
else()
  message(WARNING
    "ORB-SLAM3 VIO node is not built because local ORB-SLAM3 libraries were not found under "
    "${ORBSLAM3_ROOT}. Build ORB-SLAM3 in /home/hanmole/third_party without installing it system-wide.")
endif()

set(navigation_share_dirs
  launch
  config
  maps
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
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS
    scripts/random_target_nav_loop.py
    scripts/sdk_stereo_calib_export_kalibr.py
    scripts/vo_quality_check.py
    scripts/vision_shm_stereo_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${navigation_install_targets}
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
