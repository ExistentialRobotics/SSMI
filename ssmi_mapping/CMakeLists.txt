cmake_minimum_required(VERSION 3.8)
project(ssmi_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_rviz_plugins REQUIRED)
find_package(ssmi_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)

ament_export_include_directories(include)

add_library(semantic_octree
    src/semantic_octree/Semantics.cpp
    src/semantic_octomap_node/octomap_generator.cpp
)

target_include_directories(semantic_octree
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  semantic_octree
  rclcpp
  pcl_ros
  pcl_conversions
  octomap
  octomap_msgs
  ssmi_interface
  geometry_msgs
  tf2_eigen
  tf2
)

add_executable(octomap_generator_ros 
    src/semantic_octomap_node/octomap_generator_ros.cpp
)

ament_target_dependencies(
  octomap_generator_ros
  rclcpp
  sensor_msgs
  nav_msgs
  std_srvs
  pcl_ros
  pcl_conversions
  octomap
  octomap_msgs
  octomap_rviz_plugins
  ssmi_interface
  geometry_msgs
  tf2_eigen
  tf2
)

target_link_libraries(octomap_generator_ros
  semantic_octree
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  ${PROJECT_NAME}/dynamic_tf.py
  ${PROJECT_NAME}/semantic_sensor_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS
  octomap_generator_ros
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY
  sdf
  DESTINATION share/${PROJECT_NAME}/
  )
install(
  DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
