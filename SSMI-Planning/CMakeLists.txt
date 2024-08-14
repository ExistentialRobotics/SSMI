cmake_minimum_required(VERSION 2.8)
project(semantic_info_gathering)

set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -O3 -Wall -fpic")

set(PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_srvs
)
# Set your python version here if using python2, e.g. python2.7
execute_process(
        COMMAND "python" -c "from distutils import sysconfig; print(sysconfig.get_python_inc())"
        OUTPUT_VARIABLE PYTHON_INCLUDE_DIRS
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

catkin_package(
  INCLUDE_DIRS src/cpp/inc deps/pybind11/include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/src/cpp/inc"
  "${PROJECT_SOURCE_DIR}/deps/pybind11/include"
  )

set(SOURCE_FILES
        src/cpp/src/exploration/astar.cpp
        src/cpp/src/exploration/collision.cpp
        src/cpp/src/exploration/util.cpp
        )

set(EXPLORATION_LIB_DESTINATION "${CATKIN_DEVEL_PREFIX}/lib/python3/dist-packages")
add_library(exploration_cpp SHARED ${PROJECT_SOURCE_DIR} ${SOURCE_FILES} src/cpp/src/exploration/python.cpp)
target_link_libraries(exploration_cpp ${PYTHON_LIBRARIES})
set_target_properties(exploration_cpp PROPERTIES SUFFIX ".so" PREFIX "_"
                      LIBRARY_OUTPUT_DIRECTORY ${EXPLORATION_LIB_DESTINATION})


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
  
install(DIRECTORY
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.rviz"
  )

catkin_python_setup()
catkin_install_python(PROGRAMS
  src/exploration/semantic_exploration_node.py
  src/navigation/path_navigation_node_position.py
  src/footprints/collision_cpp.py
  src/footprints/footprint_points.py
  src/footprints/footprints.py
  src/mapping/costmap.py
  src/planners/astar_cpp.py
  src/utilities/utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS exploration_cpp
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/semantic_info_gathering"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/semantic_info_gathering"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}/semantic_info_gathering"
)

install(DIRECTORY
  src/cpp/inc
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
