cmake_minimum_required(VERSION 2.8)
project(semantic_octomap)

set(CMAKE_CXX_STANDARD 17)

# SET OCTOMAP_OMP to disable/enable OPENMP (experimental for octomap, default disabled)
SET(OCTOMAP_OMP FALSE CACHE BOOL "Enable/disable OpenMP parallelization")
IF(DEFINED ENV{OCTOMAP_OMP})
  SET(OCTOMAP_OMP $ENV{OCTOMAP_OMP})
ENDIF(DEFINED ENV{OCTOMAP_OMP})
IF(OCTOMAP_OMP)
  FIND_PACKAGE( OpenMP REQUIRED)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
ENDIF(OCTOMAP_OMP)

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  octomap_msgs
)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

find_package(octomap REQUIRED)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES} message_generation)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES LE.msg RayRLE.msg
)

add_service_files(
  DIRECTORY srv
  FILES GetRLE.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(semantic_octomap SHARED
            src/semantic_octomap_node/octomap_generator.cpp
            src/semantic_octree/Semantics.cpp)

target_link_libraries(semantic_octomap ${LINK_LIBS})

add_executable(semantic_octomap_node
 src/semantic_octomap_node/octomap_generator_ros.cpp
)

target_link_libraries(semantic_octomap_node semantic_octomap ${LINK_LIBS})

add_dependencies(semantic_octomap_node semantic_octomap_generate_messages_cpp)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  )

install(DIRECTORY
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
  )

catkin_install_python(PROGRAMS
  scripts/semantic_sensor_node.py
  scripts/semantic_sensor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS semantic_octomap_node semantic_octomap
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  include/semantic_octomap_node
  include/semantic_octree
  include/semantic_point_cloud
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}/.."
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hxx"
  )
